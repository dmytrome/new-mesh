#ifndef RTC_MANAGER_H
#define RTC_MANAGER_H

#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"
#include <time.h>

// RTC Wake-up sources
#define RTC_TIMER_WAKEUP        ESP_SLEEP_WAKEUP_TIMER
#define RTC_MANUAL_WAKEUP       ESP_SLEEP_WAKEUP_UNDEFINED

// Sleep durations (seconds)
#define DEFAULT_SLEEP_DURATION  300    // 5 minutes
#define SHORT_SLEEP_DURATION    60     // 1 minute
#define LONG_SLEEP_DURATION     3600   // 1 hour

typedef struct {
    esp_sleep_wakeup_cause_t last_wakeup_cause;
    time_t last_sleep_time;
    time_t last_wake_time;
    uint32_t total_sleep_cycles;
    bool rtc_initialized;
} rtc_status_t;

// RTC Manager Functions
esp_err_t rtc_manager_init(void);
esp_err_t rtc_set_sleep_timer(uint32_t sleep_duration_sec);
esp_err_t rtc_enter_deep_sleep(uint32_t duration_sec);
esp_sleep_wakeup_cause_t rtc_get_wakeup_cause(void);
bool rtc_is_timer_wakeup(void);
void rtc_log_wakeup_info(void);
time_t rtc_get_current_time(void);
esp_err_t rtc_set_current_time(time_t unix_time);

#endif // RTC_MANAGER_H 