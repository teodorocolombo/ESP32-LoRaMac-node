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

uint32_t RtcBkupRegisters[] = {0, 0};

static void IRAM_ATTR RtcAlarmIrq(void *arg);

void RtcInit(void)
{
    if (RtcInitialized == false) {
#if (RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE)
        GpioInit(&DbgRtcPin0, RTC_DBG_PIN_0, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
        GpioInit(&DbgRtcPin1, RTC_DBG_PIN_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
#endif
        esp_timer_create_args_t rtc_timer_args = {
            .callback = &RtcAlarmIrq, .name = "rtc_timer"};

        esp_timer_create(&rtc_timer_args, &rtc_timer);

        RtcTimerContext.AlarmState = ALARM_STOPPED;
        RtcSetTimerContext();
        RtcInitialized = true;
    }
}

uint32_t RtcSetTimerContext(void)
{
    RtcTimerContext.Time = (uint32_t)esp_timer_get_time();
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
    return (uint32_t)((((uint64_t)milliseconds) << 10) / 1000);
}

TimerTime_t RtcTick2Ms(uint32_t tick)
{
    uint32_t seconds = tick >> 10;

    tick = tick & 0x3FF;
    return ((seconds * 1000) + ((tick * 1000) >> 10));
}

void RtcDelayMs(TimerTime_t milliseconds)
{
    uint32_t delayTicks = 0;
    uint32_t refTicks   = RtcGetTimerValue();

    delayTicks = RtcMs2Tick(milliseconds);

    // Wait delay ms
    while (((RtcGetTimerValue() - refTicks)) < delayTicks) {
        __NOP();
    }
}

void RtcSetAlarm(uint32_t timeout)
{
    RtcStartAlarm(timeout);
}

void RtcStopAlarm(void)
{
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
    esp_timer_start_once(rtc_timer, RtcTimerContext.Time + RtcTimerContext.Delay);
    CRITICAL_SECTION_END();
}

uint32_t RtcGetTimerValue(void)
{
    return (uint32_t)HwTimerGetTime();
}

uint32_t RtcGetTimerElapsedTime(void)
{
    return (uint32_t)(HwTimerGetTime() - RtcTimerContext.Time);
}

uint32_t RtcGetCalendarTime(uint16_t *milliseconds)
{
    uint32_t ticks = 0;

    uint32_t calendarValue = HwTimerGetTime();

    uint32_t seconds = (uint32_t)calendarValue >> 10;

    ticks = (uint32_t)calendarValue & 0x3FF;

    *milliseconds = RtcTick2Ms(ticks);

    return seconds;
}

void RtcBkupWrite(uint32_t data0, uint32_t data1)
{
    CRITICAL_SECTION_BEGIN();
    RtcBkupRegisters[0] = data0;
    RtcBkupRegisters[1] = data1;
    CRITICAL_SECTION_END();
}

void RtcBkupRead(uint32_t *data0, uint32_t *data1)
{
    CRITICAL_SECTION_BEGIN();
    *data0 = RtcBkupRegisters[0];
    *data1 = RtcBkupRegisters[1];
    CRITICAL_SECTION_END();
}

void RtcProcess(void)
{
    CRITICAL_SECTION_BEGIN();

    if ((RtcTimerContext.AlarmState == ALARM_RUNNING)
        && (RtcTimeoutPendingPolling == true)) {
        if (RtcGetTimerElapsedTime() >= RtcTimerContext.Delay) {
            RtcTimerContext.AlarmState = ALARM_STOPPED;

            // Because of one shot the task will be removed after the callback
            RtcTimeoutPendingPolling = false;
#if (RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE)
            GpioWrite(&DbgRtcPin0, 0);
            GpioWrite(&DbgRtcPin1, 1);
#endif
            // NOTE: The handler should take less then 1 ms otherwise the clock shifts
            TimerIrqHandler();
#if (RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE)
            GpioWrite(&DbgRtcPin1, 0);
#endif
        }
    }
    CRITICAL_SECTION_END();
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature)
{
    return period;
}

static void IRAM_ATTR RtcAlarmIrq(void *arg)
{
    RtcTimerContext.AlarmState = ALARM_STOPPED;
    // Because of one shot the task will be removed after the callback
    RtcTimeoutPendingInterrupt = false;
#if (RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE)
    GpioWrite(&DbgRtcPin1, 1);
#endif
    // NOTE: The handler should take less then 1 ms otherwise the clock shifts
    TimerIrqHandler();
#if (RTC_DEBUG_GPIO_STATE == RTC_DEBUG_ENABLE)
    GpioWrite(&DbgRtcPin1, 0);
#endif
}

static void RtcOverflowIrq(void)
{
    //RtcTimerContext.Time += ( uint64_t )( 1 << 32 );
}
