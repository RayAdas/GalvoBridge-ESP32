#include <Arduino.h>
#include <atomic>
#include <driver/gptimer.h>
#include "DAC8562.h"
#include "GalvoParser.h"
#include "LaserCtrl.h"

#if defined(GALVO_SCAN_DIAGNOSTIC)
static DAC8562 dac(10000000);
#else
static DAC8562 dac;
#endif
static gptimer_handle_t timer;
static TaskHandle_t scanTask;
static constexpr UBaseType_t ScanPriority = 20;
using GalvoPlayback::InitialPeriod;
static std::atomic<uint32_t> requestedPeriod{InitialPeriod};
static std::atomic<uint32_t> programmedPeriod{InitialPeriod};
static std::atomic<bool> timerUpdateFailed{false};

static bool ARDUINO_ISR_ATTR notifyScan(gptimer_handle_t handle,
                                       const gptimer_alarm_event_data_t *, void *)
{
    const uint32_t period = requestedPeriod.load(std::memory_order_relaxed);
    if (period != programmedPeriod.load(std::memory_order_relaxed))
    {
        gptimer_alarm_config_t alarm = {};
        alarm.alarm_count = period;
        alarm.flags.auto_reload_on_alarm = true;
        if (gptimer_set_alarm_action(handle, &alarm) == ESP_OK)
            programmedPeriod.store(period, std::memory_order_relaxed);
        else
            timerUpdateFailed.store(true, std::memory_order_relaxed);
    }
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(scanTask, &wake);
    return wake == pdTRUE;
}

static void scan(void *)
{
    uint32_t applied = InitialPeriod;
    for (;;)
    {
        const uint32_t ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const uint32_t period = programmedPeriod.load(std::memory_order_relaxed);
        if (applied != period)
        {
            applied = period;
            galvoParser_setAppliedPeriod(applied);
        }
        requestedPeriod.store(galvoParser_scan(dac, timerUpdateFailed.load(std::memory_order_relaxed),
                                               ticks - 1),
                              std::memory_order_relaxed);
    }
}

void setup()
{
    laserCtrl_init();
    if (Serial.setRxBufferSize(8192) != 8192)
        return;
    Serial.begin(921600);
    Serial.setTxTimeoutMs(5);

    // Arduino-ESP32 starts the unpinned TinyUSB device task before setup().
    // Let the timer wake the scan task even while CDC is receiving a packet.
    if (TaskHandle_t usbTask = xTaskGetHandle("usbd"))
    {
        if (uxTaskPriorityGet(usbTask) >= ScanPriority)
            vTaskPrioritySet(usbTask, ScanPriority - 1);
    }
    // Allow the analog rail to settle before sending the DAC's reset and reference commands.
    delay(500);
    dac.begin();
    dac.sync_outputs(32768, 32768);

    gptimer_config_t config = {};
    config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    config.direction = GPTIMER_COUNT_UP;
    config.resolution_hz = GalvoPlayback::TimerHz;
    gptimer_event_callbacks_t callbacks = {};
    callbacks.on_alarm = notifyScan;
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = InitialPeriod;
    alarm.flags.auto_reload_on_alarm = true;
    if (gptimer_new_timer(&config, &timer) != ESP_OK ||
        gptimer_register_event_callbacks(timer, &callbacks, nullptr) != ESP_OK ||
        gptimer_set_alarm_action(timer, &alarm) != ESP_OK ||
        gptimer_enable(timer) != ESP_OK ||
        xTaskCreatePinnedToCore(scan, "GalvoScan", 4096, nullptr, ScanPriority, &scanTask, 1) != pdPASS ||
        gptimer_start(timer) != ESP_OK)
        return;
    galvoParser_init();
}

void loop()
{
    delay(1000);
}
