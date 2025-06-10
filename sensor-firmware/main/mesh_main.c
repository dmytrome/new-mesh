#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "mesh_network.h"
#include "message_handler.h"
#include "rtc_manager.h"
#include "time_sync_manager.h"

/*******************************************************
 *                Constants & Macros
 *******************************************************/
// Ultra-Low Power Coordination Constants
#define DATA_COLLECTION_WINDOW_SEC    60    // 60 seconds for all data collection
#define MESH_CONNECTION_TIMEOUT_SEC   10    // 10 seconds to connect to mesh
#define SENSOR_READ_TIMEOUT_SEC       5     // 5 seconds to read all sensors
#define DATA_SEND_TIMEOUT_SEC         10    // 10 seconds to send data
#define EMERGENCY_SLEEP_SEC           300   // 5 minutes emergency sleep

/*******************************************************
 *                Type Definitions
 *******************************************************/
// Coordinated operation states
typedef enum {
    COORD_STATE_WAKE_UP,
    COORD_STATE_WAIT_TIME_SYNC,
    COORD_STATE_CONNECT_MESH,
    COORD_STATE_COLLECT_DATA,
    COORD_STATE_SEND_DATA,
    COORD_STATE_PREPARE_SLEEP,
    COORD_STATE_DEEP_SLEEP
} coordinated_state_t;

/*******************************************************
 *                Global Variables
 *******************************************************/
static const char *MESH_TAG = "mesh_main";

// Coordination variables
static coordinated_state_t current_state = COORD_STATE_WAKE_UP;
static bool time_synchronized = false;
static bool data_collection_done = false;
static uint32_t wake_start_time = 0;

/*******************************************************
 *                Ultra-Low Power Functions
 *******************************************************/
static esp_err_t check_wake_timeout(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000000; // Convert to seconds
    uint32_t elapsed = current_time - wake_start_time;
    
    if (elapsed > DATA_COLLECTION_WINDOW_SEC) {
        ESP_LOGW(MESH_TAG, "⚠️ Data collection timeout (%lu sec) - entering emergency sleep", elapsed);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t wait_for_time_sync_signal(uint32_t timeout_ms)
{
    ESP_LOGI(MESH_TAG, "⏰ Waiting for time sync from gateway...");
    
    // Use the time sync manager to wait for sync
    esp_err_t result = wait_for_time_sync(timeout_ms / 1000);
    if (result == ESP_OK) {
        ESP_LOGI(MESH_TAG, "✅ Time sync received successfully");
        return ESP_OK;
    } else {
        ESP_LOGW(MESH_TAG, "⚠️ Time sync timeout - proceeding with emergency operations");
        return ESP_ERR_TIMEOUT;
    }
}

static esp_err_t read_and_send_sensor_data_quick(void)
{
    ESP_LOGI(MESH_TAG, "📊 Reading sensor data quickly...");
    
    // Use the message handler to send sensor data
    if (should_send_sensor_data()) {
        ESP_LOGI(MESH_TAG, "📤 Sensor data collection triggered by time sync window");
        // Data will be sent by the sensor TX task automatically when in collection window
        return ESP_OK;
    } else {
        ESP_LOGW(MESH_TAG, "⚠️ Not in data collection window");
        return ESP_ERR_INVALID_STATE;
    }
}

static void enter_coordinated_deep_sleep(uint32_t sleep_duration_sec)
{
    ESP_LOGI(MESH_TAG, "💤 Entering coordinated deep sleep for %lu seconds", sleep_duration_sec);
    
    // Use RTC manager for coordinated sleep
    rtc_enter_deep_sleep(sleep_duration_sec);
}

static esp_err_t coordinated_sensor_cycle(void)
{
    ESP_LOGI(MESH_TAG, "🔋 Starting coordinated ultra-low power cycle");
    wake_start_time = esp_timer_get_time() / 1000000;
    
    // Phase 1: Wait for time synchronization (with timeout)
    current_state = COORD_STATE_WAIT_TIME_SYNC;
    if (wait_for_time_sync_signal(10000) == ESP_OK) {  // 10 second timeout
        time_synchronized = true;
        ESP_LOGI(MESH_TAG, "✅ Time synchronized with gateway");
    } else {
        ESP_LOGW(MESH_TAG, "⚠️ Time sync timeout - proceeding anyway");
    }
    
    // Phase 2: Quick mesh connection (with timeout)
    current_state = COORD_STATE_CONNECT_MESH;
    uint32_t mesh_wait_start = esp_timer_get_time() / 1000;
    while (!mesh_network_is_mesh_connected() && (esp_timer_get_time() / 1000 - mesh_wait_start) < (MESH_CONNECTION_TIMEOUT_SEC * 1000)) {
        if (check_wake_timeout() != ESP_OK) {
            enter_coordinated_deep_sleep(EMERGENCY_SLEEP_SEC);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (mesh_network_is_mesh_connected()) {
        ESP_LOGI(MESH_TAG, "✅ Mesh connected quickly");
        
        // Phase 3: Read and send sensor data
        current_state = COORD_STATE_COLLECT_DATA;
        if (read_and_send_sensor_data_quick() == ESP_OK) {
            data_collection_done = true;
            ESP_LOGI(MESH_TAG, "✅ Sensor data collection complete");
        }
    } else {
        ESP_LOGW(MESH_TAG, "⚠️ Mesh connection failed - emergency sleep");
    }
    
    // Phase 4: Prepare for coordinated sleep
    current_state = COORD_STATE_PREPARE_SLEEP;
    
    if (is_sensor_time_synchronized()) {
        // Calculate coordinated sleep duration
        uint32_t sleep_duration;
        if (calculate_sleep_duration(&sleep_duration) == ESP_OK) {
            ESP_LOGI(MESH_TAG, "💤 Entering coordinated sleep for %lu seconds", sleep_duration);
            enter_coordinated_deep_sleep(sleep_duration);
        } else {
            ESP_LOGW(MESH_TAG, "💤 Failed to calculate sleep duration - using emergency sleep");
            enter_coordinated_deep_sleep(EMERGENCY_SLEEP_SEC);
        }
    } else {
        ESP_LOGI(MESH_TAG, "💤 No time sync available - entering emergency sleep");
        enter_coordinated_deep_sleep(EMERGENCY_SLEEP_SEC);
    }
    
    return ESP_OK;
}

/*******************************************************
 *                Main Application
 *******************************************************/
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Initialize RTC Manager for time synchronization and sleep coordination
    ESP_ERROR_CHECK(rtc_manager_init());
    
    // Initialize Time Sync Manager for coordination with gateway
    ESP_ERROR_CHECK(sensor_time_sync_init());
    
    // Check wake-up reason for ultra-low power coordination
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool is_coordinated_wake = (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER);
    
    if (is_coordinated_wake) {
        ESP_LOGI(MESH_TAG, "⏰ Coordinated wake-up detected - entering ultra-low power mode");
        
        // Fast initialization for coordinated operation
        ESP_ERROR_CHECK(mesh_network_init_full_system(true));
        
        // Start message handler for time sync reception
        ESP_ERROR_CHECK(esp_mesh_comm_p2p_start());
        
        // Execute coordinated ultra-low power cycle
        coordinated_sensor_cycle();
        
        // Should not reach here (will deep sleep)
        ESP_LOGW(MESH_TAG, "⚠️ Coordinated cycle returned unexpectedly");
        return;
    }
    
    // Normal initialization for standard operation or first boot
    ESP_LOGI(MESH_TAG, "🔄 Normal boot - initializing standard mesh operation");
    
    // Full initialization for normal operation
    ESP_ERROR_CHECK(mesh_network_init_full_system(false));
    
    // Start message handler (includes time sync reception)
    ESP_ERROR_CHECK(esp_mesh_comm_p2p_start());

    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  
             esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", 
             esp_mesh_is_ps_enabled());
             
    // Provide hints for ultra-low power mode
    ESP_LOGI(MESH_TAG, "💡 Ready to receive time sync from gateway for coordinated operation");
    ESP_LOGI(MESH_TAG, "🔄 Running in standard continuous mode - monitoring for time sync");
}