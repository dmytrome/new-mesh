#include "message_handler.h"
#include "mesh_network.h"
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "message_protocol.h"
#include "mqtt_handler.h"

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *MESH_TAG = "message_handler";

// Forward declarations
void send_time_sync_message(const mesh_addr_t* dest_addr);
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;

// Global wake-up coordination
static time_t gateway_start_time = 0;  // Time when gateway first synced with WiFi
static time_t unified_wake_time = 0;   // Unified wake-up time for all devices
static bool time_coordination_initialized = false;

// Sensor data storage for combined payload
typedef struct {
    bool has_data;
    sensor_message_t data;
    char sensor_id[8]; // "1:0", "1:1", etc.
} sensor_data_entry_t;

#define MAX_SENSORS 10
static sensor_data_entry_t sensor_data[MAX_SENSORS];
static int sensor_count = 0;
static time_t last_publish_time = 0;
static time_t last_data_received_time = 0;

/*******************************************************
 *                Sensor Data Management Functions
 *******************************************************/

// Find or create sensor entry by MAC address
static int find_sensor_by_mac(uint8_t *mac) {
    for (int i = 0; i < sensor_count; i++) {
        if (memcmp(sensor_data[i].data.header.node_mac, mac, 6) == 0) {
            return i;
        }
    }
    
    // Create new sensor entry
    if (sensor_count < MAX_SENSORS) {
        int idx = sensor_count++;
        memcpy(sensor_data[idx].data.header.node_mac, mac, 6);
        sensor_data[idx].has_data = false;
        // Generate sensor ID based on MAC
        int sensor_number = (mac[4] + mac[5]) % 10;
        snprintf(sensor_data[idx].sensor_id, sizeof(sensor_data[idx].sensor_id), "1:%d", sensor_number);
        return idx;
    }
    
    return -1; // No space available
}

// Create combined JSON payload with all sensor data
static void create_combined_payload(char *json_buffer, size_t buffer_size) {
    time_t now;
    time(&now);
    
    // Start JSON structure with proper Unix timestamp
    int offset = snprintf(json_buffer, buffer_size, 
        "{\"timestamp\":\"%lld\",\"nodes\":[", (long long)now);
    
    // Add each sensor's data
    bool first_sensor = true;
    for (int i = 0; i < sensor_count; i++) {
        if (!sensor_data[i].has_data) continue;
        
        if (!first_sensor) {
            offset += snprintf(json_buffer + offset, buffer_size - offset, ",");
        }
        first_sensor = false;
        
        sensor_message_t *msg = &sensor_data[i].data;
        offset += snprintf(json_buffer + offset, buffer_size - offset,
            "{\"sensor_id\":\"%s\",\"data\":{"
            "\"lux\":%d,"
            "\"temp_air\":%.1f,"
            "\"hum_air\":%.1f,"
            "\"temp_ground\":%.1f,"
            "\"soil_temp\":%.1f,"
            "\"soil_hum\":%.1f,"
            "\"soil_ec\":%.2f,"
            "\"soil_ph\":%.1f,"
            "\"soil_n\":%.1f,"
            "\"soil_p\":%.1f,"
            "\"soil_k\":%.1f,"
            "\"soil_salinity\":%.2f,"
            "\"soil_tds_npk\":%.2f,"
            "\"bat_lvl\":%.1f,"
            "\"bat_vol\":%.0f"
            "}}",
            sensor_data[i].sensor_id,
            msg->data.lux,
            msg->data.temp_air,
            msg->data.hum_air,
            msg->data.soil_temp, // temp_ground same as soil_temp
            msg->data.soil_temp,
            msg->data.soil_hum,
            (float)msg->data.soil_ec / 1000.0,
            msg->data.soil_ph,
            (float)msg->data.soil_n,
            (float)msg->data.soil_p,
            (float)msg->data.soil_k,
            0.0, // soil_salinity (not available)
            0.0, // soil_tds_npk (not available)
            msg->data.bat_lvl,
            (float)msg->data.bat_vol
        );
    }
    
    // Close JSON structure
    snprintf(json_buffer + offset, buffer_size - offset, "]}");
}

// Check if we should publish (only when all sensors are disconnected/sleeping)
static bool should_publish(void) {
    time_t now;
    time(&now);
    
    // Count active sensors with data
    int active_sensors = 0;
    for (int i = 0; i < sensor_count; i++) {
        if (sensor_data[i].has_data) {
            active_sensors++;
        }
    }
    
    // Get current connected nodes count
    int connected_nodes = esp_mesh_get_routing_table_size();
    
    // Publish if we have data from sensors AND no nodes are connected (all sensors are sleeping)
    if (active_sensors >= 1 && connected_nodes == 0) {
        ESP_LOGI(MESH_TAG, "📤 Publishing trigger: %d sensors with data, %d nodes connected", 
                 active_sensors, connected_nodes);
        return true;
    }
    
    return false;
}

/*******************************************************
 *                Time Coordination Functions
 *******************************************************/

// Initialize unified wake-up time when gateway first connects to WiFi
void initialize_time_coordination(void) {
    if (!time_coordination_initialized) {
        gateway_start_time = time(NULL);
        int sleep_period_minutes = CONFIG_SENSOR_SLEEP_PERIOD_MINUTES;
        
        // Calculate unified wake time as: current_time + sleep_period + buffer_time
        // This ensures sensors have time to connect, send data, and receive time sync
        int buffer_minutes = 5; // 5 minutes buffer for sensor operations
        unified_wake_time = gateway_start_time + ((sleep_period_minutes + buffer_minutes) * 60);
        time_coordination_initialized = true;
        
        ESP_LOGI(MESH_TAG, "🕐 Time coordination initialized:");
        ESP_LOGI(MESH_TAG, "   Gateway start time: %lld", (long long)gateway_start_time);
        ESP_LOGI(MESH_TAG, "   Unified wake time: %lld", (long long)unified_wake_time);
        ESP_LOGI(MESH_TAG, "   Sleep period: %d minutes + %d buffer", sleep_period_minutes, buffer_minutes);
    }
}

// Calculate individual sleep duration for current time
uint32_t calculate_sleep_duration(time_t current_time) {
    if (current_time >= unified_wake_time) {
        ESP_LOGW(MESH_TAG, "⚠️ Current time (%lld) >= unified wake time (%lld), using minimum sleep", 
                 (long long)current_time, (long long)unified_wake_time);
        return 10; // Minimum 10 seconds
    }
    return (uint32_t)(unified_wake_time - current_time);
}

// Gateway deep sleep coordination
void gateway_enter_deep_sleep(void) {
    time_t now = time(NULL);
    uint32_t sleep_duration = calculate_sleep_duration(now);
    
    ESP_LOGI(MESH_TAG, "🌙 Gateway entering deep sleep:");
    ESP_LOGI(MESH_TAG, "   Current time: %lld", (long long)now);
    ESP_LOGI(MESH_TAG, "   Wake time: %lld", (long long)unified_wake_time);
    ESP_LOGI(MESH_TAG, "   Sleep duration: %" PRIu32 " seconds", sleep_duration);
    
    // Enable timer wake-up
    esp_sleep_enable_timer_wakeup(sleep_duration * 1000000ULL);
    
    // Short delay to allow logging
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(MESH_TAG, "💤 Gateway deep sleep started");
    esp_deep_sleep_start();
}

/*******************************************************
 *                P2P Communication Functions
 *******************************************************/

void esp_mesh_p2p_tx_main(void *arg)
{
    int send_count = 0;
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    int previous_connected_nodes = -1; // Track changes in connected nodes
    is_running = true;

    while (is_running) {
        /* Gateway: Check routing table and log sensor network status */
        if (!esp_mesh_is_root()) {
            ESP_LOGI(MESH_TAG, "layer:%d, rtableSize:%d, %s", get_mesh_layer(),
                     esp_mesh_get_routing_table_size(),
                     is_mesh_connected_status() ? "NODE" : "DISCONNECT");
            vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);
        send_count++;
        
        // Track connected nodes changes
        int current_connected_nodes = esp_mesh_get_routing_table_size();
        if (current_connected_nodes != previous_connected_nodes) {
            ESP_LOGI(MESH_TAG, "🔄 Connected nodes changed: %d -> %d", previous_connected_nodes, current_connected_nodes);
            previous_connected_nodes = current_connected_nodes;
        }
        
        if (!(send_count % 10)) {
            ESP_LOGI(MESH_TAG, "🌐 GATEWAY STATUS: Connected sensors:%d/%d, cycle:%d", 
                     route_table_size, current_connected_nodes, send_count);
        }
        
        // Check if we should publish when sensors disconnect
        if (should_publish()) {
            // Start MQTT client now that we're ready to publish
            extern esp_err_t mqtt_handler_start(void);
            esp_err_t mqtt_err = mqtt_handler_start();
            if (mqtt_err != ESP_OK) {
                ESP_LOGE(MESH_TAG, "❌ Failed to start MQTT client: %s", esp_err_to_name(mqtt_err));
                vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
                continue;
            }
            
            // Wait a moment for MQTT to connect
            ESP_LOGI(MESH_TAG, "⏳ Waiting for MQTT connection before publishing...");
            int mqtt_wait_count = 0;
            while (!mqtt_handler_is_connected() && mqtt_wait_count < 30) { // Wait up to 30 seconds
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                mqtt_wait_count++;
            }
            
            if (!mqtt_handler_is_connected()) {
                ESP_LOGW(MESH_TAG, "⚠️ MQTT connection timeout, skipping publish");
                vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
                continue;
            }
            
            static char combined_json[2048];
            create_combined_payload(combined_json, sizeof(combined_json));
            
            ESP_LOGI(MESH_TAG, "📤 Publishing combined payload with %d sensors (all sensors disconnected)", sensor_count);
            ESP_LOGI(MESH_TAG, "📄 JSON: %s", combined_json);
            
            // Publish to AWS IoT Core via MQTT
            mqtt_err = mqtt_handler_publish_sensor_data("combined_sensors", combined_json);
            if (mqtt_err == ESP_OK) {
                ESP_LOGI(MESH_TAG, "☁️ Combined data published to AWS IoT Core successfully");
                last_publish_time = time(NULL);
                
                // Mark all sensors as published
                for (int i = 0; i < sensor_count; i++) {
                    sensor_data[i].has_data = false;
                }
                
                // Stop MQTT client after publishing
                extern esp_err_t mqtt_handler_stop(void);
                mqtt_handler_stop();
                ESP_LOGI(MESH_TAG, "🛑 MQTT client stopped after publishing");
                
                // After publishing, gateway should also go to deep sleep
                ESP_LOGI(MESH_TAG, "🌙 All sensors sleeping, data published. Gateway preparing for deep sleep...");
                vTaskDelay(5000 / portTICK_PERIOD_MS); // 5 second delay for cleanup
                gateway_enter_deep_sleep();
            } else {
                ESP_LOGW(MESH_TAG, "⚠️ Failed to publish combined data to AWS IoT Core: %s", esp_err_to_name(mqtt_err));
            }
        }
        
        // Initialize time coordination if not done yet
        if (!time_coordination_initialized) {
            initialize_time_coordination();
        }
        
        // Send periodic time sync with unified wake-up time
        static time_t last_time_sync = 0;
        time_t now = time(NULL);
        if (now - last_time_sync >= 30) { // Every 30 seconds
            if (current_connected_nodes > 0) {
                ESP_LOGI(MESH_TAG, "🕐 Periodic unified time sync broadcast to %d connected sensors", current_connected_nodes);
                
                // Send time sync to all connected sensors
                for (int i = 0; i < route_table_size && i < current_connected_nodes; i++) {
                    send_time_sync_message(&route_table[i]);
                    vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay between sends
                }
                
                // Also broadcast to all (in case routing table is not complete)
                send_time_sync_message(NULL);
                last_time_sync = now;
            }
        }
        
        // Gateway doesn't need to send test data frequently - focus on receiving sensor data
        vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void esp_mesh_p2p_rx_main(void *arg)
{
    int recv_count = 0;
    esp_err_t err;
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    data.size = RX_SIZE;
    is_running = true;

    while (is_running) {
        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK || !data.size) {
            ESP_LOGE(MESH_TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }
        
        recv_count++;
        
        // Check if this is a new agricultural sensor message
        if (data.size == sizeof(sensor_message_t)) {
            sensor_message_t *sensor_msg = (sensor_message_t*)data.data;
            
            // Validate checksum
            uint16_t expected_checksum = calculate_checksum(sensor_msg, sizeof(sensor_message_t) - sizeof(sensor_msg->checksum));
            if (sensor_msg->checksum != expected_checksum) {
                ESP_LOGW(MESH_TAG, "❌ Invalid checksum from "MACSTR" - expected 0x%04X, got 0x%04X", 
                         MAC2STR(from.addr), expected_checksum, sensor_msg->checksum);
                continue;
            }
            
            // First, show what we received in the same format as sensor sends
            ESP_LOGI(MESH_TAG, "📥 RECEIVED agricultural data: Air %.1f°C, Soil %.1f°C, %.1f%% RH, %d lux, pH %.1f (Layer %d, Seq %d)", 
                     sensor_msg->data.temp_air, sensor_msg->data.soil_temp, sensor_msg->data.hum_air, 
                     sensor_msg->data.lux, sensor_msg->data.soil_ph,
                     sensor_msg->header.mesh_layer, sensor_msg->header.sequence_number);
            
            // Process agricultural sensor data
            if (sensor_msg->header.message_type == MSG_TYPE_SENSOR_DATA) {
                // Use MAC address directly and make layer prominent
                ESP_LOGI(MESH_TAG, "🌱 AGRICULTURAL DATA ["MACSTR" L%d]: %.1f°C, %.1f%% RH, %d lux, pH %.1f", 
                         MAC2STR(sensor_msg->header.node_mac),
                         sensor_msg->header.mesh_layer,
                         sensor_msg->data.temp_air, 
                         sensor_msg->data.hum_air, 
                         sensor_msg->data.lux, 
                         sensor_msg->data.soil_ph);
                         
                ESP_LOGI(MESH_TAG, "🌱 SOIL DATA ["MACSTR" L%d]: Temp %.1f°C, Hum %.1f%%, EC %d µS/cm, NPK(%d,%d,%d)", 
                         MAC2STR(sensor_msg->header.node_mac),
                         sensor_msg->header.mesh_layer,
                         sensor_msg->data.soil_temp,
                         sensor_msg->data.soil_hum, 
                         sensor_msg->data.soil_ec,
                         sensor_msg->data.soil_n,
                         sensor_msg->data.soil_p, 
                         sensor_msg->data.soil_k);
                         
                ESP_LOGI(MESH_TAG, "🔋 STATUS ["MACSTR" L%d]: Battery %.2fV (%dmV), Seq %d, Quality %d%%", 
                         MAC2STR(sensor_msg->header.node_mac),
                         sensor_msg->header.mesh_layer,
                         sensor_msg->data.bat_lvl,
                         sensor_msg->data.bat_vol,
                         sensor_msg->header.sequence_number,
                         sensor_msg->data.reading_quality);
                
                // Store sensor data for combined payload
                int sensor_idx = find_sensor_by_mac(sensor_msg->header.node_mac);
                if (sensor_idx >= 0) {
                    memcpy(&sensor_data[sensor_idx].data, sensor_msg, sizeof(sensor_message_t));
                    sensor_data[sensor_idx].has_data = true;
                    
                    // Update last data received time
                    last_data_received_time = time(NULL);
                    
                    ESP_LOGI(MESH_TAG, "💾 Stored data for sensor %s (total sensors: %d)", 
                             sensor_data[sensor_idx].sensor_id, sensor_count);
                    
                    // Don't send time sync immediately - let all sensors connect and send data first
                    // Time sync will be sent when sensors disconnect (in the TX task)
                } else {
                    ESP_LOGW(MESH_TAG, "❌ No space available for new sensor "MACSTR"", MAC2STR(sensor_msg->header.node_mac));
                }
                
                continue; // Don't process as regular message
            }
        }
        
        // Log other messages (less frequently to avoid spam)
        if (!(recv_count % 10)) {
            ESP_LOGW(MESH_TAG,
                     "[#RX:%d][L:%d] receive from "MACSTR", size:%d, heap:%" PRId32 ", flag:%d[err:0x%x, proto:%d, tos:%d]",
                     recv_count, get_mesh_layer(), MAC2STR(from.addr),
                     data.size, esp_get_minimum_free_heap_size(), flag, err, data.proto,
                     data.tos);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        
        // Gateway: Enable both TX and RX
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 8192, NULL, 5, NULL);
        ESP_LOGI(MESH_TAG, "GATEWAY: P2P TX and RX enabled");
    }
    return ESP_OK;
}

void send_time_sync_message(const mesh_addr_t* dest_addr) {
    // Ensure time coordination is initialized
    if (!time_coordination_initialized) {
        initialize_time_coordination();
    }
    
    time_sync_message_t msg = {0};
    time_t now = time(NULL);
    uint32_t sleep_duration = calculate_sleep_duration(now);
    
    uint8_t gateway_mac[6];
    esp_read_mac(gateway_mac, ESP_MAC_WIFI_STA);
    memcpy(msg.header.node_mac, gateway_mac, 6);
    msg.header.node_type = NODE_TYPE_GATEWAY;
    msg.header.message_type = MSG_TYPE_TIME_SYNC;
    msg.header.timestamp = (uint32_t)now;
    msg.header.sequence_number = 0;
    msg.header.mesh_layer = 0;
    msg.header.signal_strength = 0;
    msg.current_unix_time = (uint32_t)now;
    msg.next_wake_time = (uint32_t)unified_wake_time;  // Use unified wake time
    msg.sleep_duration_sec = (uint16_t)sleep_duration;  // Individual sleep duration
    msg.sync_source = 2; // NTP
    msg.collection_window_sec = 10; // 10 sec window
    
    mesh_data_t data = {
        .data = (uint8_t*)&msg,
        .size = sizeof(msg)
    };
    
    esp_err_t err;
    if (dest_addr == NULL) {
        // Broadcast to all nodes in the mesh network
        err = esp_mesh_send(NULL, &data, MESH_DATA_P2P, NULL, 0);
        if (err == ESP_OK) {
            ESP_LOGI(MESH_TAG, "🕐 Broadcast unified time sync (now: %lld, unified wake: %lld, sleep: %" PRIu32 " sec)", 
                     (long long)now, (long long)unified_wake_time, sleep_duration);
        } else {
            ESP_LOGW(MESH_TAG, "Failed to broadcast time sync: %s", esp_err_to_name(err));
        }
    } else {
        // Send to specific node
        err = esp_mesh_send(dest_addr, &data, MESH_DATA_P2P, NULL, 0);
        if (err == ESP_OK) {
            ESP_LOGI(MESH_TAG, "🕐 Sent unified time sync to "MACSTR" (now: %lld, unified wake: %lld, sleep: %" PRIu32 " sec)", 
                     MAC2STR(dest_addr->addr), (long long)now, (long long)unified_wake_time, sleep_duration);
        } else {
            ESP_LOGW(MESH_TAG, "Failed to send time sync to "MACSTR": %s", MAC2STR(dest_addr->addr), esp_err_to_name(err));
        }
    }
} 