#include "GalvoParser.h"

#include "DAC8562.h"
#include "LaserCtrl.h"
#include <algorithm>
#include <esp_system.h>
#include <esp_timer.h>

namespace
{
using GalvoPlayback::BaseRate;
using GalvoPlayback::TimerHz;
constexpr uint32_t Capacity = 4000;
constexpr uint32_t MaxTarget = 3000;
static_assert(MaxTarget < Capacity, "Playback target must fit the queue");
constexpr uint32_t MinRate = BaseRate - BaseRate / 50;
constexpr uint32_t MaxRate = BaseRate + BaseRate / 50;
constexpr uint8_t ProtocolVersion = 3;
constexpr size_t MaxBody = 20 + UINT16_MAX;  // Header and CRC plus the full wire payload range.
constexpr size_t MaxEncoded = MaxBody + MaxBody / 254 + 1;

enum Message : uint8_t { Hello = 1, Points, GetStatus = 5,
                        HelloReply = 0x81, Ack, FrameStarted, Status, ScanStats };
enum Result : uint8_t { BadRequest = 1, RateUnsupported = 3, Stale = 5 };

struct Sample
{
    uint32_t frame;  // Nonzero on the first point of a packet, for reporting only.
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
static uint32_t front, back, remaining, sampled, generation;
static bool starved;
static uint32_t requestedPeriod = GalvoPlayback::InitialPeriod;
static uint32_t appliedPeriod = GalvoPlayback::InitialPeriod;
static uint32_t lastRequest, accepted, active, underruns;
static uint64_t session, nonce, activeAt, receivedAt, startAt, lastScan;
static uint32_t targetPoints;
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
    uint32_t enqueued, started, maxEnqueueGapUs, maxStartGapUs, minPostEnqueue;
    uint32_t underrunEvents;
};
static ScanMetrics metrics;
static uint64_t metricsSince;
static uint8_t input[MaxEncoded];

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

uint16_t scanFlags()
{
    return (scanFault ? 8 | scanFault : 0) | DiagnosticMode;
}

bool scanBlocked()
{
    return !DiagnosticMode && scanFault;
}

uint32_t periodForRate(uint32_t rate)
{
    const uint32_t shortest = (TimerHz + MaxRate - 1) / MaxRate;
    const uint32_t longest = TimerHz / MinRate;
    return constrain((TimerHz + rate / 2) / rate, shortest, longest);
}

void resetPlayback()
{
    setOutput(false);
    front = back;
    remaining = 0;
    active = 0;
    activeAt = 0;
    startAt = 0;
    starved = false;
    requestedPeriod = GalvoPlayback::InitialPeriod;
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
    appendNumber(dst, ProtocolVersion, 1);
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

void report(Message type, uint32_t request, uint8_t result = 0, bool handshakeError = false)
{
    uint8_t data[44] = {};
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
        appendNumber(dst, scanFlags(), 2);
        appendNumber(dst, output, 1);
    }
    else
        dst += 43;
    portEXIT_CRITICAL(&stateLock);
    send(type, id, request, data, dst - data);
}

void reportScanMetrics()
{
    uint8_t data[38];
    const uint64_t now = nowUs();
    const uint64_t elapsed = now - metricsSince;
    const uint64_t id = session;
    if (!id || elapsed < 1000000)
        return;
    // Fit the whole wire packet into the 64-byte CDC TX FIFO without waiting for space.
    if (Serial.availableForWrite() < int(sizeof(data) + 23))
        return;

    portENTER_CRITICAL(&stateLock);
    const ScanMetrics snapshot = metrics;
    const uint32_t configuredRate = TimerHz / appliedPeriod;
    const uint32_t postEnqueue = sampled;
    metrics = {};
    metricsSince = now;
    portEXIT_CRITICAL(&stateLock);
    uint8_t *dst = data;
    const auto appendShort = [&dst](uint32_t value) {
        appendNumber(dst, std::min<uint32_t>(value, UINT16_MAX), 2);
    };
    appendNumber(dst, std::min<uint64_t>(elapsed, UINT32_MAX), 4);
    appendShort(snapshot.points);
    appendShort(configuredRate);
    appendShort(snapshot.coalesced);
    appendShort(snapshot.maxDacUs);
    appendShort(snapshot.slowDac);
    appendShort(snapshot.maxGapUs);
    appendShort(snapshot.busy);
    appendShort(snapshot.enqueued);
    appendShort(snapshot.started);
    appendNumber(dst, snapshot.maxEnqueueGapUs, 4);
    appendNumber(dst, snapshot.maxStartGapUs, 4);
    appendShort(snapshot.minPostEnqueue);
    appendShort(postEnqueue);
    appendShort(0);  // Reserved former replay_points field.
    appendShort(snapshot.underrunEvents);
    send(ScanStats, id, 0, data, dst - data);
}

void hello(const uint8_t *data, size_t length, uint32_t request)
{
    if (length != 16 || request != 1 || !readNumber(data, 8))
    {
        report(Ack, request, BadRequest, true);
        return;
    }
    const uint64_t newNonce = readNumber(data, 8);
    const uint32_t rate = readNumber(data + 8, 4);
    if (rate != BaseRate)
    {
        report(Ack, request, RateUnsupported, true);
        return;
    }

    const uint32_t target = readNumber(data + 12, 4);
    if (target > MaxTarget || (session && nonce == newNonce && target != targetPoints))
    {
        report(Ack, request, BadRequest, true);
        return;
    }

    const uint64_t newSession = (uint64_t(esp_random()) << 32) | esp_random();
    uint8_t reply[24];
    uint8_t *dst = reply;
    portENTER_CRITICAL(&stateLock);
    if (!session || nonce != newNonce)
    {
        session = newSession ? newSession : 1;
        nonce = newNonce;
        targetPoints = target;
        accepted = sampled = lastRequest = underruns = 0;
        scanFault = 0;
        metrics = {};
        metricsSince = nowUs();
        resetPlayback();
    }
    const uint64_t id = session;
    appendNumber(dst, nonce, 8);
    appendNumber(dst, BaseRate, 4);
    appendNumber(dst, MinRate, 4);
    appendNumber(dst, MaxRate, 4);
    appendNumber(dst, MaxTarget, 4);
    portEXIT_CRITICAL(&stateLock);
    send(HelloReply, id, request, reply, dst - reply);
}

void enqueue(const uint8_t *data, size_t length)
{
    if (length < 6)
        return;
    const uint32_t frame = readNumber(data, 4);
    const uint32_t count = readNumber(data + 4, 2);
    if (!frame || !count || length != 6 + 4 * count + (count + 7) / 8)
        return;
    const uint8_t *mask = data + 6 + count * 4;

    portENTER_CRITICAL(&stateLock);
    const bool stale = frame <= accepted;
    const bool busy = count > Capacity - remaining;
    const bool fault = scanBlocked();
    if (busy && !stale && !fault)
        ++metrics.busy;
    const uint32_t offset = back;
    const uint32_t epoch = generation;
    portEXIT_CRITICAL(&stateLock);
    if (stale || busy || fault)
        return;

    for (uint32_t i = 0; i < count; ++i)
    {
        Sample &point = samples[(offset + i) % Capacity];
        point = {i ? 0 : frame, uint16_t(readNumber(data + 6 + i * 4, 2)),
                 uint16_t(readNumber(data + 8 + i * 4, 2)), bool(mask[i / 8] & (1 << (i % 8)))};
    }

    portENTER_CRITICAL(&stateLock);
    if (generation != epoch && scanBlocked())
    {
        portEXIT_CRITICAL(&stateLock);
        return;
    }
    const uint64_t now = nowUs();
    if (!remaining && !active)
    {
        const uint32_t headroom = targetPoints > count ? targetPoints - count : 0;
        startAt = now + uint64_t(headroom) * 1000000 / BaseRate;
    }
    back = (back + count) % Capacity;
    remaining += count;
    sampled = remaining;
    if (!metrics.enqueued || sampled < metrics.minPostEnqueue)
        metrics.minPostEnqueue = sampled;
    ++metrics.enqueued;
    if (accepted)
        metrics.maxEnqueueGapUs = std::max(metrics.maxEnqueueGapUs,
            uint32_t(std::min<uint64_t>(now - receivedAt, UINT32_MAX)));
    accepted = frame;
    receivedAt = now;
    if (active)
    {
        // Kp = 2 Hz/point. Limit the step to 0.5% of the baseline.
        const int goal = constrain(int(BaseRate) + 2 * (int(sampled) - int(targetPoints)),
                                   int(MinRate), int(MaxRate));
        const int current = TimerHz / appliedPeriod;
        const int step = BaseRate / 200;
        const int rate = constrain(constrain(goal, current - step, current + step),
                                   int(MinRate), int(MaxRate));
        requestedPeriod = periodForRate(rate);
    }
    portEXIT_CRITICAL(&stateLock);
}

void handle(const uint8_t *body, size_t length)
{
    if (length < 20 || body[0] != ProtocolVersion ||
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

    if (request <= lastRequest)
    {
        if (type != Points)
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
        enqueue(data, size);
        return;
    }
    report(Ack, request, BadRequest);
}

void decode(size_t size)
{
    size_t written = 0;
    for (size_t pos = 0; pos < size;)
    {
        const uint8_t code = input[pos++];
        if (!code || pos + code - 1 > size || written + code - 1 > MaxBody)
            return;
        // Decoded bytes trail the read cursor, so one buffer suffices.
        memmove(input + written, input + pos, code - 1);
        pos += code - 1;
        written += code - 1;
        if (code != 255 && pos < size)
        {
            if (written == MaxBody)
                return;
            input[written++] = 0;
        }
    }
    handle(input, written);
}

void receive(void *)
{
    size_t size = 0;
    bool overflow = false;
    uint16_t previousFlags = 0xffff;
    uint32_t previousUnderruns = 0;
    uint64_t previousSession = 0, lastStatusAt = 0;
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
        const uint16_t currentFlags = scanFlags();
        const uint32_t currentUnderruns = underruns;
        portEXIT_CRITICAL(&stateLock);

        if (session && (session != previousSession || currentFlags != previousFlags
                        || currentUnderruns != previousUnderruns || now - lastStatusAt >= 1000000))
        {
            report(Status, 0);
            lastStatusAt = now;
        }
        previousSession = session;
        previousFlags = currentFlags;
        previousUnderruns = currentUnderruns;

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
        reportScanMetrics();
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
    if (active)
        metrics.coalesced += coalescedTicks;
    if (timerFault)
        latchScanFault(TimerConfiguration);
    else if (active && now - lastScan > scanGapLimitUs())
        latchScanFault(BeforeDacGap);
    available = false;
    if (!scanBlocked() && remaining && now >= startAt)
    {
        point = samples[front];
        front = (front + 1) % Capacity;
        --remaining;
        starved = false;
        available = true;
    }
    else if (!scanBlocked() && active && !remaining)
    {
        if (!starved)
        {
            ++underruns;
            ++metrics.underrunEvents;
            starved = true;
            setOutput(false);
        }
        lastScan = now;  // The scan task is healthy even when the producer has no points.
    }
    if (available)
    {
        writingDac = true;
        if (point.blank)
            setOutput(false);
    }
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
        const uint32_t gapLimit = scanGapLimitUs();
        if (epoch == generation)
        {
            ++metrics.points;
            metrics.maxDacUs = std::max(metrics.maxDacUs, uint32_t(after - at));
            metrics.slowDac += after - at > gapLimit;
            if (active)
                metrics.maxGapUs = std::max(metrics.maxGapUs, uint32_t(after - lastScan));
        }
        if (epoch == generation && after - at > gapLimit)
            latchScanFault(DacUpdateGap);
        else if (epoch == generation && active && after - lastScan > gapLimit)
            latchScanFault(BeforeDacGap);
        const bool valid = epoch == generation;
        if (valid)
            lastScan = after;
        if (valid && point.frame)
        {
            ++metrics.started;
            if (active)
                metrics.maxStartGapUs = std::max(metrics.maxStartGapUs, uint32_t(at - activeAt));
            active = point.frame;
            activeAt = at;
        }
        setOutput(valid && !point.blank && !DiagnosticMode && !scanBlocked());
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
