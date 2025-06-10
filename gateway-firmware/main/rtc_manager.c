#include "include/rtc_manager.h"
#include "esp_timer.h"
#include "sys/time.h"

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *TAG = "RTC_MANAGER";
static rtc_status_t rtc_status = {0};

/*******************************************************
 *                RTC Manager Functions
 *******************************************************/

esp_err_t rtc_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing RTC Manager...");
    
    // Initialize RTC status
    rtc_status.last_wakeup_cause = esp_sleep_get_wakeup_cause();
    rtc_status.last_wake_time = time(NULL);
    rtc_status.rtc_initialized = true;
    
    // Log wake-up information
    rtc_log_wakeup_info();
    
    ESP_LOGI(TAG, "✅ RTC Manager initialized successfully");
    return ESP_OK;
}

esp_err_t rtc_set_sleep_timer(uint32_t sleep_duration_sec)
{
    if (!rtc_status.rtc_initialized) {
        ESP_LOGE(TAG, "RTC not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint64_t sleep_duration_us = (uint64_t)sleep_duration_sec * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_duration_us);
    
    ESP_LOGI(TAG, "⏰ Sleep timer set for %lu seconds", sleep_duration_sec);
    return ESP_OK;
}

esp_err_t rtc_enter_deep_sleep(uint32_t duration_sec)
{
    if (!rtc_status.rtc_initialized) {
        ESP_LOGE(TAG, "RTC not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Set sleep timer
    esp_err_t ret = rtc_set_sleep_timer(duration_sec);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Update status
    rtc_status.last_sleep_time = time(NULL);
    rtc_status.total_sleep_cycles++;
    
    ESP_LOGI(TAG, "💤 Entering deep sleep for %lu seconds (cycle #%lu)", 
             duration_sec, rtc_status.total_sleep_cycles);
    
    // Enter deep sleep (device will reset when waking up)
    esp_deep_sleep_start();
    
    // Should never reach here
    return ESP_OK;
}

esp_sleep_wakeup_cause_t rtc_get_wakeup_cause(void)
{
    return esp_sleep_get_wakeup_cause();
}

bool rtc_is_timer_wakeup(void)
{
    return (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER);
}

void rtc_log_wakeup_info(void)
{
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "🔔 Wake-up from TIMER");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(TAG, "🔔 Wake-up from external signal (RTC_IO)");
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            ESP_LOGI(TAG, "🔔 Wake-up from external signal (RTC_CNTL)");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            ESP_LOGI(TAG, "🔔 Wake-up from touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            ESP_LOGI(TAG, "🔔 Wake-up from ULP program");
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(TAG, "🔔 Wake-up from POWER-ON or RESET");
            break;
    }
    
    // Log sleep statistics
    if (rtc_status.total_sleep_cycles > 0) {
        ESP_LOGI(TAG, "📊 Sleep cycles completed: %lu", rtc_status.total_sleep_cycles);
    }
}

time_t rtc_get_current_time(void)
{
    return time(NULL);
}

esp_err_t rtc_set_current_time(time_t unix_time)
{
    struct timeval tv = {
        .tv_sec = unix_time,
        .tv_usec = 0
    };
    
    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to set system time");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "✅ System time updated to: %lld", (long long)unix_time);
    return ESP_OK;
} 