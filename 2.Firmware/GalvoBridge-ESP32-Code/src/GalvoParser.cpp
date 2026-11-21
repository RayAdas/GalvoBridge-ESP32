#include "GalvoParser.h"

#include "DAC8562.h"
#include "LaserCtrl.h"
#include <algorithm>
#include <esp_system.h>
#include <esp_timer.h>

namespace
{
constexpr uint32_t Slots = 1000;
constexpr uint32_t Capacity = 4000;
constexpr uint32_t Target = 1300;
constexpr uint32_t LeaseUs = 250000;
constexpr uint32_t FrameTimeoutUs = 200000;
constexpr uint32_t TimerHz = 10000000;
constexpr size_t MaxBody = 4151;
constexpr size_t MaxEncoded = 4168;

enum Message : uint8_t { Hello = 1, Points, SetLight, Keepalive, GetStatus,
                        HelloReply = 0x81, Ack, FrameStarted, Status, ScanStats };
enum Result : uint8_t { Ok, BadRequest, Busy, RateUnsupported, Inhibited, Stale };

struct Sample
{
    uint32_t frame;
    uint16_t x, y;
    bool blank;
};

struct Started
{
    uint64_t session, at;
    uint32_t frame;
};

static portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t starts;
static Sample samples[Capacity];
static Sample lastFrame[Slots];
static uint32_t front, back, remaining, sampled, generation;
static uint32_t cachedPoints, replayIndex;
static bool replaying;
static uint32_t baseRate = 25000, minRate = 24500, maxRate = 25500;
static uint32_t requestedPeriod = 400, appliedPeriod = 400;
static uint32_t lastRequest, accepted, active, underruns, permit;
static uint64_t session, nonce, activeAt, receivedAt, leaseUntil, startAt, lastScan;
enum ScanFault : uint16_t { WatchdogGap = 1 << 4, BeforeDacGap = 1 << 5,
                            DacUpdateGap = 1 << 6, TimerConfiguration = 1 << 7 };
#if defined(GALVO_SCAN_DIAGNOSTIC)
constexpr uint16_t DiagnosticMode = 1 << 8;
#else
constexpr uint16_t DiagnosticMode = 0;
#endif
static uint16_t scanFault;
static bool output, writingDac;
struct ScanMetrics
{
    uint32_t points, coalesced, maxDacUs, slowDac, maxGapUs, busy;
};
static ScanMetrics metrics;
static uint64_t metricsSince;
static uint8_t input[MaxEncoded], body[MaxBody];

uint64_t nowUs() { return esp_timer_get_time(); }

uint32_t scanGapLimitUs()
{
    // Tolerate coalesced ticks, with headroom for the slowest negotiated rate.
    return std::max<uint32_t>(1000, 3 * (appliedPeriod + 9) / 10);
}

uint64_t readNumber(const uint8_t *src, size_t size)
{
    uint64_t value = 0;
    while (size--)
        value = (value << 8) | *src++;
    return value;
}

void appendNumber(uint8_t *&dst, uint64_t value, size_t size)
{
    for (size_t i = size; i > 0; --i)
        *dst++ = value >> ((i - 1) * 8);
}

uint32_t crc32(const uint8_t *data, size_t size)
{
    uint32_t crc = 0xffffffff;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
    return ~crc;
}

void setOutput(bool enabled)
{
    if (output != enabled)
    {
        laserCtrl_setOutput(enabled);
        output = enabled;
    }
}

void revoke()
{
    permit = 0;
    leaseUntil = 0;
    setOutput(false);
}

uint16_t flags(uint64_t now)
{
    return (permit && now < leaseUntil ? 0 : 2) |
           (active && now - receivedAt < FrameTimeoutUs ? 0 : 4) |
           (scanFault ? 8 | scanFault : 0) | DiagnosticMode;
}

bool scanBlocked()
{
    return !DiagnosticMode && scanFault;
}

void checkLight(uint64_t now)
{
    if (flags(now))
        revoke();
}

uint32_t periodForRate(uint32_t rate)
{
    const uint32_t shortest = (TimerHz + maxRate - 1) / maxRate;
    const uint32_t longest = TimerHz / minRate;
    return constrain((TimerHz + rate / 2) / rate, shortest, longest);
}

void resetPlayback()
{
    revoke();
    front = back;
    remaining = 0;
    active = 0;
    activeAt = 0;
    startAt = 0;
    cachedPoints = replayIndex = 0;
    replaying = false;
    requestedPeriod = periodForRate(baseRate);
    ++generation;
}

void latchScanFault(ScanFault reason)
{
    if (!scanFault)
    {
        scanFault = reason;
        if (scanBlocked())
            resetPlayback();
    }
}

void send(Message type, uint64_t id, uint32_t request, const uint8_t *payload, size_t size)
{
    uint8_t raw[96];
    uint8_t wire[99] = {};
    uint8_t *dst = raw;
    appendNumber(dst, 2, 1);
    appendNumber(dst, type, 1);
    appendNumber(dst, id, 8);
    appendNumber(dst, request, 4);
    appendNumber(dst, size, 2);
    memcpy(dst, payload, size);
    dst += size;
    const uint32_t crc = crc32(raw, dst - raw);
    appendNumber(dst, crc, 4);

    // Replies are smaller than a single maximum-length COBS block.
    uint8_t *code = wire + 1;
    uint8_t *out = wire + 2;
    for (const uint8_t *p = raw; p != dst; ++p)
    {
        if (*p)
            *out++ = *p;
        else
        {
            *code = out - code;
            code = out++;
        }
    }
    *code = out - code;
    *out++ = 0;
    Serial.write(wire, out - wire);
}

void report(Message type, uint32_t request, Result result = Ok, bool handshakeError = false)
{
    uint8_t data[48] = {};
    uint8_t *dst = data;
    if (type == Ack)
        *dst++ = result;
    portENTER_CRITICAL(&stateLock);
    const uint64_t id = handshakeError ? 0 : session;
    if (!handshakeError)
    {
        const uint64_t now = nowUs();
        appendNumber(dst, now, 8);
        appendNumber(dst, active, 4);
        appendNumber(dst, activeAt, 8);
        appendNumber(dst, accepted, 4);
        appendNumber(dst, remaining, 4);
        appendNumber(dst, sampled, 4);
        appendNumber(dst, TimerHz / appliedPeriod, 4);
        appendNumber(dst, underruns, 4);
        appendNumber(dst, permit, 4);
        appendNumber(dst, flags(now), 2);
        appendNumber(dst, output, 1);
    }
    else
        dst += 47;
    portEXIT_CRITICAL(&stateLock);
    send(type, id, request, data, dst - data);
}

void reportScanMetrics()
{
    if (!DiagnosticMode)
        return;
    portENTER_CRITICAL(&stateLock);
    const uint64_t now = nowUs();
    const uint64_t elapsed = now - metricsSince;
    const uint64_t id = session;
    if (!id || elapsed < 1000000)
    {
        portEXIT_CRITICAL(&stateLock);
        return;
    }
    const ScanMetrics snapshot = metrics;
    const uint32_t configuredRate = TimerHz / appliedPeriod;
    metrics = {};
    metricsSince = now;
    portEXIT_CRITICAL(&stateLock);
    uint8_t data[32];
    uint8_t *dst = data;
    appendNumber(dst, elapsed, 4);
    appendNumber(dst, snapshot.points, 4);
    appendNumber(dst, configuredRate, 4);
    appendNumber(dst, snapshot.coalesced, 4);
    appendNumber(dst, snapshot.maxDacUs, 4);
    appendNumber(dst, snapshot.slowDac, 4);
    appendNumber(dst, snapshot.maxGapUs, 4);
    appendNumber(dst, snapshot.busy, 4);
    send(ScanStats, id, 0, data, sizeof(data));
}

void hello(const uint8_t *data, size_t length, uint32_t request)
{
    if (length != 12 || request != 1 || !readNumber(data, 8))
    {
        report(Ack, request, BadRequest, true);
        return;
    }
    const uint64_t newNonce = readNumber(data, 8);
    const uint32_t rate = readNumber(data + 8, 4);
    if (rate < 1000 || rate > 30000)
    {
        report(Ack, request, RateUnsupported, true);
        return;
    }

    const uint64_t newSession = (uint64_t(esp_random()) << 32) | esp_random();
    uint8_t reply[36];
    uint8_t *dst = reply;
    portENTER_CRITICAL(&stateLock);
    if (session && nonce == newNonce && rate != baseRate)
    {
        portEXIT_CRITICAL(&stateLock);
        report(Ack, request, BadRequest, true);
        return;
    }
    if (!session || nonce != newNonce)
    {
        session = newSession ? newSession : 1;
        nonce = newNonce;
        baseRate = rate;
        minRate = baseRate - baseRate / 50;
        maxRate = std::min<uint32_t>(30000, baseRate + baseRate / 50);
        accepted = sampled = lastRequest = underruns = 0;
        scanFault = 0;
        metrics = {};
        metricsSince = nowUs();
        resetPlayback();
    }
    const uint64_t id = session;
    appendNumber(dst, nonce, 8);
    appendNumber(dst, baseRate, 4);
    appendNumber(dst, minRate, 4);
    appendNumber(dst, maxRate, 4);
    appendNumber(dst, Target, 4);
    appendNumber(dst, Capacity, 4);
    appendNumber(dst, LeaseUs, 4);
    appendNumber(dst, FrameTimeoutUs, 4);
    portEXIT_CRITICAL(&stateLock);
    send(HelloReply, id, request, reply, dst - reply);
}

Result enqueue(const uint8_t *data, size_t length)
{
    if (length < 6)
        return BadRequest;
    const uint32_t frame = readNumber(data, 4);
    const uint32_t count = readNumber(data + 4, 2);
    if (!frame || !count || count > Slots || length != 6 + 4 * count + (count + 7) / 8)
        return BadRequest;
    const uint8_t *mask = data + 6 + count * 4;
    if (!(mask[0] & 1) || (count % 8 && mask[count / 8] >> (count % 8)))
        return BadRequest;

    portENTER_CRITICAL(&stateLock);
    const bool stale = frame <= accepted;
    const bool busy = remaining > Capacity - Slots;
    if (DiagnosticMode && busy && !stale)
        ++metrics.busy;
    const bool fault = scanBlocked();
    const uint32_t offset = back;
    const uint32_t epoch = generation;
    portEXIT_CRITICAL(&stateLock);
    if (stale || busy || fault)
        return stale ? Stale : (fault ? Inhibited : Busy);

    const uint32_t expanded = (Slots / count) * count;
    for (uint32_t j = 0; j < Slots; ++j)
    {
        const uint32_t i = (j * expanded / Slots) % count;
        Sample &point = samples[(offset + j) % Capacity];
        point = {j ? 0 : frame, uint16_t(readNumber(data + 6 + i * 4, 2)),
                 uint16_t(readNumber(data + 8 + i * 4, 2)), bool(mask[i / 8] & (1 << (i % 8)))};
    }

    portENTER_CRITICAL(&stateLock);
    if (generation != epoch && scanBlocked())
    {
        portEXIT_CRITICAL(&stateLock);
        return Inhibited;
    }
    const uint64_t now = nowUs();
    checkLight(now);
    if (!remaining && !active)
        startAt = now + uint64_t(Target - Slots) * 1000000 / baseRate;
    back = (back + Slots) % Capacity;
    remaining += Slots;
    sampled = remaining;
    accepted = frame;
    receivedAt = now;
    if (active)
    {
        // Kp = 2 Hz/point. Limit the step to 0.5% of the baseline.
        const int goal = constrain(int(baseRate) + 2 * (int(sampled) - int(Target)),
                                   int(minRate), int(maxRate));
        const int current = TimerHz / appliedPeriod;
        const int step = baseRate / 200;
        const int rate = constrain(constrain(goal, current - step, current + step),
                                   int(minRate), int(maxRate));
        requestedPeriod = periodForRate(rate);
    }
    portEXIT_CRITICAL(&stateLock);
    return Ok;
}

void handle(size_t length)
{
    if (length < 20 || body[0] != 2 ||
        readNumber(body + 14, 2) != length - 20 ||
        readNumber(body + length - 4, 4) != crc32(body, length - 4))
        return;
    const uint64_t id = readNumber(body + 2, 8);
    const uint32_t request = readNumber(body + 10, 4);
    const uint8_t type = body[1];
    const uint8_t *data = body + 16;
    const size_t size = length - 20;
    if (type == Hello && !id)
    {
        hello(data, size, request);
        return;
    }
    if (!id || id != session || !request)
        return;

    const bool off = type == SetLight && size == 1 && *data == 0;
    if (request <= lastRequest && !off)
    {
        report(Ack, request, Stale);
        return;
    }
    lastRequest = max(lastRequest, request);

    if (type == GetStatus && !size)
    {
        report(Status, request);
        return;
    }
    if (type == Points)
    {
        report(Ack, request, enqueue(data, size));
        return;
    }
    Result result = BadRequest;
    portENTER_CRITICAL(&stateLock);
    const uint64_t now = nowUs();
    checkLight(now);
    if (off)
    {
        revoke();
        result = Ok;
    }
    else if (type == SetLight && size == 1 && *data == 1)
    {
        if (flags(now) == 2)
        {
            permit = request;
            leaseUntil = now + LeaseUs;
            result = Ok;
        }
        else
            result = Inhibited;
    }
    else if (type == Keepalive && size == 4)
    {
        if (permit && permit == readNumber(data, 4))
        {
            leaseUntil = now + LeaseUs;
            result = Ok;
        }
        else
            result = Inhibited;
    }
    portEXIT_CRITICAL(&stateLock);
    report(Ack, request, result);
}

void decode(size_t size)
{
    size_t written = 0;
    for (size_t pos = 0; pos < size;)
    {
        const uint8_t code = input[pos++];
        if (!code || pos + code - 1 > size || written + code - 1 > MaxBody)
            return;
        memcpy(body + written, input + pos, code - 1);
        pos += code - 1;
        written += code - 1;
        if (code != 255 && pos < size)
        {
            if (written == MaxBody)
                return;
            body[written++] = 0;
        }
    }
    handle(written);
}

void receive(void *)
{
    size_t size = 0;
    bool overflow = false;
    uint16_t previousFlags = 0xffff;
    uint32_t previousUnderruns = 0;
    for (;;)
    {
        const bool connected = bool(Serial);
        portENTER_CRITICAL(&stateLock);
        if (!connected && session)
        {
            resetPlayback();
            session = 0;
            size = 0;
            overflow = false;
        }
        const uint64_t now = nowUs();
        if (active && now - lastScan > scanGapLimitUs())
        {
            latchScanFault(writingDac ? DacUpdateGap : WatchdogGap);
        }
        checkLight(now);
        const uint16_t currentFlags = flags(now);
        const uint32_t currentUnderruns = underruns;
        portEXIT_CRITICAL(&stateLock);

        if (session && (currentFlags != previousFlags || currentUnderruns != previousUnderruns))
            report(Status, 0);
        previousFlags = currentFlags;
        previousUnderruns = currentUnderruns;
        reportScanMetrics();

        Started event;
        while (xQueueReceive(starts, &event, 0) == pdTRUE)
        {
            if (!session || session != event.session)
                continue;
            uint8_t payload[12];
            uint8_t *dst = payload;
            appendNumber(dst, event.frame, 4);
            appendNumber(dst, event.at, 8);
            send(FrameStarted, event.session, 0, payload, sizeof(payload));
        }

        uint8_t chunk[1024];
        const size_t bytes = connected ? Serial.read(chunk, sizeof(chunk)) : 0;
        for (size_t i = 0; bytes <= sizeof(chunk) && i < bytes; ++i)
        {
            const uint8_t byte = chunk[i];
            if (!byte)
            {
                if (!overflow && size)
                    decode(size);
                size = 0;
                overflow = false;
            }
            else if (size < sizeof(input))
                input[size++] = byte;
            else
                overflow = true;
        }
        vTaskDelay(1);
    }
}
} // namespace

void galvoParser_init()
{
    starts = xQueueCreate(16, sizeof(Started));
    if (starts)
        xTaskCreatePinnedToCore(receive, "GalvoSerial", 6144, nullptr, 5, nullptr, 0);
}

uint32_t galvoParser_scan(DAC8562 &dac, bool timerFault, uint32_t coalescedTicks)
{
    Sample point = {};
    bool available;
    uint32_t epoch;
    uint64_t id;
    portENTER_CRITICAL(&stateLock);
    const uint64_t now = nowUs();
    if (DiagnosticMode && active)
        metrics.coalesced += coalescedTicks;
    if (timerFault)
        latchScanFault(TimerConfiguration);
    else if (active && now - lastScan > scanGapLimitUs())
        latchScanFault(BeforeDacGap);
    if (active && !remaining && now - receivedAt >= FrameTimeoutUs)
    {
        resetPlayback();
    }
    available = false;
    if (!scanBlocked() && remaining && now >= startAt)
    {
        point = samples[front];
        front = (front + 1) % Capacity;
        --remaining;
        if (point.frame)
            cachedPoints = 0;
        lastFrame[cachedPoints] = {0, point.x, point.y, point.blank};
        ++cachedPoints;
        replaying = false;
        available = true;
    }
    else if (!scanBlocked() && active && !remaining && cachedPoints == Slots)
    {
        if (!replaying)
        {
            ++underruns;
            replayIndex = 0;
            replaying = true;
        }
        point = lastFrame[replayIndex];
        replayIndex = (replayIndex + 1) % Slots;
        available = true;
    }
    if (available)
    {
        writingDac = true;
        if (point.blank)
            setOutput(false);
    }
    checkLight(now);
    epoch = generation;
    id = session;
    portEXIT_CRITICAL(&stateLock);

    if (available)
    {
        const uint64_t at = nowUs();
        dac.sync_outputs(point.x, point.y);
        portENTER_CRITICAL(&stateLock);
        const uint64_t after = nowUs();
        writingDac = false;
        if (DiagnosticMode && epoch == generation)
        {
            ++metrics.points;
            metrics.maxDacUs = std::max(metrics.maxDacUs, uint32_t(after - at));
            metrics.slowDac += after - at > scanGapLimitUs();
            if (active)
                metrics.maxGapUs = std::max(metrics.maxGapUs, uint32_t(after - lastScan));
        }
        if (epoch == generation && after - at > scanGapLimitUs())
            latchScanFault(DacUpdateGap);
        else if (epoch == generation && active && after - lastScan > scanGapLimitUs())
            latchScanFault(BeforeDacGap);
        const bool valid = epoch == generation;
        if (valid)
            lastScan = after;
        if (valid && point.frame)
        {
            active = point.frame;
            activeAt = at;
        }
        checkLight(after);
        setOutput(valid && !point.blank && !flags(after));
        portEXIT_CRITICAL(&stateLock);
        if (valid && point.frame && starts)
        {
            const Started event = {id, at, point.frame};
            xQueueSend(starts, &event, 0);
        }
    }

    portENTER_CRITICAL(&stateLock);
    const uint32_t period = requestedPeriod;
    portEXIT_CRITICAL(&stateLock);
    return period;
}

void galvoParser_setAppliedPeriod(uint32_t period)
{
    portENTER_CRITICAL(&stateLock);
    appliedPeriod = period;
    portEXIT_CRITICAL(&stateLock);
}
