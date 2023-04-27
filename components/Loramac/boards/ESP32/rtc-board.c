#include "rtc-board.h"
#include "board-config.h"
#include "board.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio.h"
#include "systime.h"
#include "timer.h"




#define RTC_DEBUG_ENABLE 1
#define RTC_DEBUG_DISABLE 0

#define RTC_DEBUG_GPIO_STATE RTC_DEBUG_DISABLE
#define RTC_DEBUG_PRINTF_STATE RTC_DEBUG_DISABLE

#define MIN_ALARM_DELAY 3   // in ticks

#define TIMER_FREQUENCY configTICK_RATE_HZ   

static bool          RtcInitialized             = false;
static volatile bool RtcTimeoutPendingInterrupt = false;
static volatile bool RtcTimeoutPendingPolling   = false;

typedef enum AlarmStates_e {
    ALARM_STOPPED = 0,
    ALARM_RUNNING = !ALARM_STOPPED
} AlarmStates_t;

typedef struct {
    uint32_t Time;
    uint32_t Delay;
    uint32_t AlarmState;
} RtcTimerContext_t;

static RtcTimerContext_t  RtcTimerContext;
static esp_timer_handle_t rtc_timer;

#if (RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE)
Gpio_t DbgRtcPin0;
Gpio_t DbgRtcPin1;
#endif

static void IRAM_ATTR RtcAlarmIrq(void *arg)
{
    RtcTimerContext.AlarmState = ALARM_STOPPED;
    // Because of one shot the task will be removed after the callback
    RtcTimeoutPendingInterrupt = false;
    RtcTimeoutPendingPolling   = true;
    // NOTE: The handler should take less then 1 ms otherwise the clock shifts
    TimerIrqHandler();
}

void RtcInit(void)
{
    if (RtcInitialized == false) {
        esp_timer_create_args_t rtc_timer_args = {
            .callback = &RtcAlarmIrq, .arg = NULL, .name = "rtc_timer"};

        esp_timer_create(&rtc_timer_args, &rtc_timer);

        RtcTimerContext.AlarmState = ALARM_STOPPED;
        RtcSetTimerContext();
        RtcInitialized = true;
    }
}

uint32_t RtcSetTimerContext(void)
{
    RtcTimerContext.Time = RtcMs2Tick(((uint32_t)esp_timer_get_time()) / 1000);
    return (uint32_t)RtcTimerContext.Time;
}

uint32_t RtcGetTimerContext(void)
{
    return RtcTimerContext.Time;
}

uint32_t RtcGetMinimumTimeout(void)
{
    return (MIN_ALARM_DELAY);
}

uint32_t RtcMs2Tick(TimerTime_t milliseconds)
{
    return (uint32_t)(milliseconds * TIMER_FREQUENCY) / 1000;
}

TimerTime_t RtcTick2Ms(uint32_t tick)
{
    return ((tick * 1000) / TIMER_FREQUENCY);
}

void RtcDelayMs(TimerTime_t milliseconds)
{
    uint32_t delayTicks = 0;
    uint32_t refTicks   = RtcGetTimerValue();

    delayTicks = RtcMs2Tick(milliseconds);

    // Wait delay ms
    while (((RtcGetTimerValue() - refTicks)) < delayTicks) {
    }
}

void RtcSetAlarm(uint32_t timeout)
{
    RtcStartAlarm(timeout);
}

void RtcStopAlarm(void)
{
    esp_timer_stop(rtc_timer);
    RtcTimerContext.AlarmState = ALARM_STOPPED;
}

void RtcStartAlarm(uint32_t timeout)
{
    CRITICAL_SECTION_BEGIN();

    RtcStopAlarm();

    RtcTimerContext.Delay = timeout;

    RtcTimeoutPendingInterrupt = true;
    RtcTimeoutPendingPolling   = false;

    RtcTimerContext.AlarmState = ALARM_RUNNING;

    esp_timer_start_once(rtc_timer, (uint64_t)RtcTick2Ms(RtcTimerContext.Time + RtcTimerContext.Delay)*1000);
    CRITICAL_SECTION_END();

}

uint32_t RtcGetTimerValue(void)
{
    return (uint32_t)esp_timer_get_time() / 1000;
}

uint32_t RtcGetTimerElapsedTime(void)
{
    return (uint32_t)(RtcMs2Tick(RtcGetTimerValue()) - RtcTimerContext.Time);
}

uint32_t RtcGetCalendarTime(uint16_t *milliseconds)
{
    uint32_t calendarValue = RtcGetTimerValue();

    uint32_t seconds = (uint32_t)calendarValue / 1000;

    return seconds;
}

void RtcBkupWrite(uint32_t data0, uint32_t data1)
{}

void RtcBkupRead(uint32_t *data0, uint32_t *data1)
{}

void RtcProcess(void)
{
    CRITICAL_SECTION_BEGIN();

    if ((RtcTimerContext.AlarmState == ALARM_RUNNING)
        && (RtcTimeoutPendingPolling == true)) {
        if (RtcGetTimerElapsedTime() >= RtcTimerContext.Delay) {
            RtcTimerContext.AlarmState = ALARM_STOPPED;

            // Because of one shot the task will be removed after the callback
            RtcTimeoutPendingPolling = false;

            // NOTE: The handler should take less then 1 ms otherwise the clock shifts
            TimerIrqHandler();
        }
    }
    CRITICAL_SECTION_END();
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature)
{
    return period;
}
