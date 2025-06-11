#include "include/time_sync_manager.h"
#include "include/message_protocol.h"
#include "include/rtc_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *TAG = "SENSOR_TIME_SYNC";
static sensor_time_sync_status_t sync_status = {0};

/*******************************************************
 *                Time Sync Manager Functions
 *******************************************************/

esp_err_t sensor_time_sync_init(void)
{
    ESP_LOGI(TAG, "Initializing Sensor Time Sync Manager...");
    
    // Initialize status
    memset(&sync_status, 0, sizeof(sensor_time_sync_status_t));
    
    // Set initial state
    sync_status.time_is_synchronized = false;
    sync_status.waiting_for_sync = true;
    sync_status.in_collection_window = false;
    sync_status.sync_messages_received = 0;
    
    ESP_LOGI(TAG, "✅ Sensor Time Sync Manager initialized - waiting for gateway sync");
    
    return ESP_OK;
}

esp_err_t handle_time_sync_message(const time_sync_message_t *sync_msg)
{
    if (!sync_msg) {
        ESP_LOGE(TAG, "Invalid sync message pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate message
    if (!is_sync_message_valid(sync_msg)) {
        ESP_LOGW(TAG, "Invalid or stale sync message - ignoring");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "📥 Received time sync from gateway - Next wake: %lld, Sleep: %d sec", 
             (long long)sync_msg->next_wake_time, sync_msg->sleep_duration_sec);
    
    // Update system time with gateway's time
    time_t gateway_time = sync_msg->current_unix_time;
    if (rtc_set_current_time(gateway_time) == ESP_OK) {
        ESP_LOGI(TAG, "⏰ System time synchronized with gateway");
    }
    
    // Update sync status
    sync_status.current_sync_time = gateway_time;
    sync_status.next_wake_time = sync_msg->next_wake_time;
    sync_status.sleep_duration_sec = sync_msg->sleep_duration_sec;
    sync_status.sync_source = sync_msg->sync_source;
    sync_status.last_sync_received = time(NULL);
    sync_status.time_is_synchronized = true;
    sync_status.waiting_for_sync = false;
    sync_status.sync_messages_received++;
    
    // **IMMEDIATE DATA RESPONSE**: Set collection window immediately upon time sync
    time_t now = time(NULL);
    sync_status.in_collection_window = true;
    sync_status.collection_start_time = now;
    sync_status.collection_end_time = now + 30; // 30-second immediate collection window
    ESP_LOGI(TAG, "📊 Immediate data collection window opened - 30 seconds to respond");
    
    ESP_LOGI(TAG, "✅ Time sync processed (msg #%lu) - Source: %s", 
             sync_status.sync_messages_received,
             sync_msg->sync_source == 0 ? "Hardcoded" : 
             sync_msg->sync_source == 1 ? "GPRS" : "Unknown");
    
    return ESP_OK;
}

esp_err_t wait_for_time_sync(uint32_t timeout_sec)
{
    ESP_LOGI(TAG, "⏳ Waiting for time sync from gateway (timeout: %lu sec)...", timeout_sec);
    
    sync_status.waiting_for_sync = true;
    uint32_t start_time = esp_timer_get_time() / 1000000;
    uint32_t current_time;
    
    while (sync_status.waiting_for_sync) {
        current_time = esp_timer_get_time() / 1000000;
        
        if ((current_time - start_time) >= timeout_sec) {
            ESP_LOGW(TAG, "⚠️ Time sync timeout after %lu seconds", timeout_sec);
            sync_status.waiting_for_sync = false;
            return ESP_ERR_TIMEOUT;
        }
        
        // Check every 100ms
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "✅ Time sync received successfully");
    return ESP_OK;
}

/*******************************************************
 *                Helper Functions
 *******************************************************/

time_t get_time_until_next_wake(void)
{
    if (!sync_status.time_is_synchronized) {
        return 0;
    }
    
    time_t now = time(NULL);
    time_t time_remaining = sync_status.next_wake_time - now;
    return (time_remaining > 0) ? time_remaining : 0;
}

time_t get_current_synchronized_time(void)
{
    return time(NULL);
}

bool is_collection_window_active(void)
{
    if (!sync_status.time_is_synchronized) {
        return false;
    }
    
    time_t now = time(NULL);
    
    // Update collection window status based on current time
    if (sync_status.in_collection_window) {
        if (now > sync_status.collection_end_time) {
            sync_status.in_collection_window = false;
            ESP_LOGI(TAG, "📊 Data collection window closed");
        }
    }
    
    return sync_status.in_collection_window;
}

bool should_send_sensor_data(void)
{
    return is_collection_window_active() && sync_status.time_is_synchronized;
}

bool is_sync_message_valid(const time_sync_message_t *sync_msg)
{
    if (!sync_msg) {
        return false;
    }
    
    // Check message header - trust the authenticated mesh network for time values
    if (sync_msg->header.message_type != MSG_TYPE_TIME_SYNC) {
        ESP_LOGW(TAG, "Wrong message type: %d", sync_msg->header.message_type);
        return false;
    }
    
    if (sync_msg->header.node_type != NODE_TYPE_GATEWAY) {
        ESP_LOGW(TAG, "Time sync not from gateway - node type: %d", sync_msg->header.node_type);
        return false;
    }
    
    // Trust the gateway time - no validation needed in authenticated mesh
    ESP_LOGI(TAG, "Valid time sync from gateway - accepting time: %lu", sync_msg->current_unix_time);
    return true;
}

/*******************************************************
 *                Sleep Coordination Functions
 *******************************************************/

esp_err_t calculate_sleep_duration(uint32_t *sleep_sec)
{
    if (!sleep_sec || !sync_status.time_is_synchronized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    time_t time_to_next_wake = get_time_until_next_wake();
    
    if (time_to_next_wake <= 0) {
        // If we're past the wake time, use the configured sleep duration
        *sleep_sec = sync_status.sleep_duration_sec;
    } else {
        // Sleep until next wake time
        *sleep_sec = (uint32_t)time_to_next_wake;
    }
    
    ESP_LOGI(TAG, "💤 Calculated sleep duration: %lu seconds", *sleep_sec);
    return ESP_OK;
}

esp_err_t prepare_for_coordinated_sleep(void)
{
    if (!sync_status.time_is_synchronized) {
        ESP_LOGW(TAG, "Cannot coordinate sleep - not time synchronized");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t sleep_duration;
    esp_err_t ret = calculate_sleep_duration(&sleep_duration);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "📅 Preparing coordinated sleep - wake at: %lld", 
             (long long)sync_status.next_wake_time);
    
    // Set up sleep timer
    ret = rtc_set_sleep_timer(sleep_duration);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sleep timer");
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ Ready for coordinated sleep - %lu seconds", sleep_duration);
    return ESP_OK;
}

/*******************************************************
 *                Status Functions
 *******************************************************/

sensor_time_sync_status_t* get_sensor_time_sync_status(void)
{
    return &sync_status;
}

bool is_sensor_time_synchronized(void)
{
    return sync_status.time_is_synchronized;
} 