#include "include/time_sync_manager.h"
#include "include/message_protocol.h"
#include "include/mesh_network.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include <string.h>

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *TAG = "TIME_SYNC";
static time_sync_status_t sync_status = {0};
static uint8_t reported_sensors[MAX_EXPECTED_SENSORS][6]; // MAC addresses of sensors that reported
static time_t last_sync_broadcast = 0;

/*******************************************************
 *                Time Sync Manager Functions
 *******************************************************/

esp_err_t time_sync_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing Time Sync Manager...");
    
    // Initialize status
    memset(&sync_status, 0, sizeof(time_sync_status_t));
    memset(reported_sensors, 0, sizeof(reported_sensors));
    
    // Set hardcoded starting time (TODO: Replace with GPRS)
    esp_err_t ret = sync_time_from_hardcoded();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize time sync");
        return ret;
    }
    
    // Initialize first cycle
    schedule_next_wake_cycle();
    
    ESP_LOGI(TAG, "✅ Time Sync Manager initialized successfully");
    ESP_LOGI(TAG, "📅 Next data collection cycle: %lld", (long long)sync_status.next_wake_time);
    
    return ESP_OK;
}

esp_err_t sync_time_from_hardcoded(void)
{
    ESP_LOGI(TAG, "🕐 Setting time from hardcoded value (TODO: Replace with GPRS)");
    
    // Set system time to hardcoded value
    struct timeval tv = {
        .tv_sec = HARDCODED_START_TIME,
        .tv_usec = 0
    };
    
    if (settimeofday(&tv, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to set system time");
        return ESP_FAIL;
    }
    
    sync_status.current_sync_time = HARDCODED_START_TIME;
    sync_status.time_is_synchronized = true;
    sync_status.sync_source = 0; // Hardcoded
    
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    ESP_LOGI(TAG, "✅ Time synchronized: %s", asctime(&timeinfo));
    ESP_LOGI(TAG, "TODO: Replace hardcoded time with GPRS sync when hardware available");
    
    return ESP_OK;
}

esp_err_t broadcast_time_sync_to_sensors(void)
{
    if (!sync_status.time_is_synchronized) {
        ESP_LOGW(TAG, "Time not synchronized, skipping broadcast");
        return ESP_ERR_INVALID_STATE;
    }
    
    time_sync_message_t sync_msg = {0};
    
    // Fill header
    esp_wifi_get_mac(WIFI_IF_STA, sync_msg.header.node_mac);
    sync_msg.header.node_type = NODE_TYPE_GATEWAY;
    sync_msg.header.message_type = MSG_TYPE_TIME_SYNC;
    sync_msg.header.timestamp = time(NULL);
    sync_msg.header.sequence_number = 0; // TODO: Implement sequence tracking
    
    // Fill time sync data
    sync_msg.current_unix_time = time(NULL);
    sync_msg.next_wake_time = sync_status.next_wake_time;
    sync_msg.sleep_duration_sec = sync_status.sleep_duration_sec;
    sync_msg.sync_source = sync_status.sync_source;
    
    // Broadcast to all mesh nodes
    mesh_data_t data = {
        .data = (uint8_t*)&sync_msg,
        .size = sizeof(time_sync_message_t),
        .proto = MESH_PROTO_BIN,
        .tos = MESH_TOS_P2P
    };
    
    // Broadcast address (all nodes)
    mesh_addr_t broadcast_addr = {{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
    esp_err_t err = esp_mesh_send(&broadcast_addr, &data, MESH_DATA_P2P, NULL, 0);
    
    if (err == ESP_OK) {
        last_sync_broadcast = time(NULL);
        ESP_LOGI(TAG, "📡 Time sync broadcast sent - Next wake: %lld, Sleep: %d sec", 
                 (long long)sync_msg.next_wake_time, sync_msg.sleep_duration_sec);
    } else {
        ESP_LOGE(TAG, "Failed to broadcast time sync: %s", esp_err_to_name(err));
    }
    
    return err;
}

esp_err_t handle_sensor_data_received(const uint8_t *sensor_mac)
{
    if (!is_data_collection_window_active()) {
        ESP_LOGD(TAG, "Sensor data received outside collection window");
        return ESP_OK;
    }
    
    // TODO: FUTURE ENHANCEMENT - Dynamic sensor discovery
    // TODO: When sensor first reports, auto-add to known sensors list
    // TODO: Implement sensor heartbeat/keepalive to detect disconnections
    // TODO: Support hot-plug sensor addition/removal during operation
    
    // Check if this sensor already reported this cycle
    for (int i = 0; i < sync_status.sensors_reported_count; i++) {
        if (memcmp(reported_sensors[i], sensor_mac, 6) == 0) {
            ESP_LOGD(TAG, "Sensor already reported this cycle");
            return ESP_OK;
        }
    }
    
    // Add sensor to reported list
    if (sync_status.sensors_reported_count < MAX_EXPECTED_SENSORS) {
        memcpy(reported_sensors[sync_status.sensors_reported_count], sensor_mac, 6);
        sync_status.sensors_reported_count++;
        
        ESP_LOGI(TAG, "📊 Sensor reported data (%d/%d) - MAC: " MACSTR, 
                 sync_status.sensors_reported_count, sync_status.expected_sensor_count,
                 MAC2STR(sensor_mac));
        
        // Check if all expected sensors have reported
        if (sync_status.sensors_reported_count >= sync_status.expected_sensor_count) {
            sync_status.data_collection_complete = true;
            ESP_LOGI(TAG, "✅ All sensors reported! Data collection cycle complete");
            
            // Schedule next cycle immediately
            schedule_next_wake_cycle();
            broadcast_time_sync_to_sensors();
        }
    } else {
        // TODO: Handle sensor count overflow - expand array or implement dynamic allocation
        ESP_LOGW(TAG, "⚠️ Sensor count exceeded MAX_EXPECTED_SENSORS (%d), ignoring additional sensors", 
                 MAX_EXPECTED_SENSORS);
    }
    
    return ESP_OK;
}

esp_err_t schedule_next_wake_cycle(void)
{
    time_t now = time(NULL);
    
    // Calculate next wake time (next 5-minute interval)
    time_t next_interval = ((now / DATA_COLLECTION_INTERVAL_SEC) + 1) * DATA_COLLECTION_INTERVAL_SEC;
    
    sync_status.next_wake_time = next_interval;
    sync_status.sleep_duration_sec = DATA_COLLECTION_INTERVAL_SEC;
    sync_status.cycle_start_time = now;
    sync_status.collection_deadline = now + DATA_COLLECTION_WINDOW_SEC;
    
    // Reset data collection tracking for next cycle
    reset_data_collection_cycle();
    
    ESP_LOGI(TAG, "📅 Next cycle scheduled - Wake: %lld (in %lld sec), Duration: %d sec", 
             (long long)sync_status.next_wake_time, 
             (long long)(sync_status.next_wake_time - now),
             sync_status.sleep_duration_sec);
    
    return ESP_OK;
}

/*******************************************************
 *                Helper Functions
 *******************************************************/

time_t get_next_collection_time(void)
{
    return sync_status.next_wake_time;
}

time_t get_current_synchronized_time(void)
{
    return time(NULL);
}

bool is_data_collection_window_active(void)
{
    time_t now = time(NULL);
    return (now >= sync_status.cycle_start_time && now <= sync_status.collection_deadline);
}

bool should_broadcast_sync(void)
{
    time_t now = time(NULL);
    return (now - last_sync_broadcast) >= SYNC_BROADCAST_INTERVAL_SEC;
}

void reset_data_collection_cycle(void)
{
    sync_status.sensors_reported_count = 0;
    sync_status.data_collection_complete = false;
    
    // TODO: DYNAMIC SENSOR DISCOVERY - Replace hardcoded sensor count
    // TODO: Implement auto-discovery: sensors announce themselves on mesh join
    // TODO: Track sensor disconnections and update expected count dynamically  
    // TODO: Add configuration file support for manual sensor count override
    // TODO: For 60+ sensors, consider longer collection windows (120-180 sec)
    sync_status.expected_sensor_count = 2; // TEMPORARY: Currently hardcoded for 2 sensors
    
    memset(reported_sensors, 0, sizeof(reported_sensors));
    
    ESP_LOGD(TAG, "🔄 Data collection cycle reset - Expecting %d sensors", 
             sync_status.expected_sensor_count);
}

/*******************************************************
 *                Status Functions
 *******************************************************/

time_sync_status_t* get_time_sync_status(void)
{
    return &sync_status;
}

bool is_time_sync_ready(void)
{
    return sync_status.time_is_synchronized;
} 