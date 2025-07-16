#include "message_handler.h"
#include "mesh_network.h"
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
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

// Global wake-up coordination - calculate once, use for all sensors
static time_t unified_wake_time = 0;   // Unified wake-up time for all devices (calculated once)
static bool wake_time_set = false;     // Flag to ensure wake time is calculated only once

// Sensor data storage for combined payload
typedef struct {
    bool has_data;
    sensor_message_t data;
    char sensor_id[18]; // MAC address format: "fc:01:2c:ca:a0:ac"
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
        // Generate sensor ID using full MAC address
        snprintf(sensor_data[idx].sensor_id, sizeof(sensor_data[idx].sensor_id), 
                "%02x:%02x:%02x:%02x:%02x:%02x", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return idx;
    }
    
    return -1; // No space available
}

// Create combined JSON payload with all sensor data
static void create_combined_payload(char *json_buffer, size_t buffer_size) {
    time_t now;
    time(&now);
    
    int offset = snprintf(json_buffer, buffer_size, 
        "{\"timestamp\":%lld,\"nodes\":[", (long long)now);
    
    // Add each sensor's data
    bool first_sensor = true;
    for (int i = 0; i < sensor_count; i++) {
        if (!sensor_data[i].has_data) continue;
        
        if (!first_sensor) {
            offset += snprintf(json_buffer + offset, buffer_size - offset, ",");
        }
        first_sensor = false;
        
        sensor_message_t *msg = &sensor_data[i].data;
        
        // Convert sensor data to JSON format (null for NAN values)  
        char temp_air_str[16], hum_air_str[16], pressure_air_str[16], temp_ground_str[16], soil_temp_str[16], soil_hum_str[16];
        char soil_ph_str[16], lux_str[16], soil_ec_str[16], soil_n_str[16], soil_p_str[16], soil_k_str[16];
        char bat_lvl_str[16];
        
        // Float values (null if NAN)
        if (isnan(msg->data.temp_air)) {
            strcpy(temp_air_str, "null");
        } else {
            snprintf(temp_air_str, sizeof(temp_air_str), "%.1f", msg->data.temp_air);
        }
        
        if (isnan(msg->data.hum_air)) {
            strcpy(hum_air_str, "null");
        } else {
            snprintf(hum_air_str, sizeof(hum_air_str), "%.1f", msg->data.hum_air);
        }
        
        if (isnan(msg->data.pressure_air)) {
            strcpy(pressure_air_str, "null");
        } else {
            snprintf(pressure_air_str, sizeof(pressure_air_str), "%.1f", msg->data.pressure_air);
        }
        
        if (isnan(msg->data.temp_ground)) {
            strcpy(temp_ground_str, "null");
        } else {
            snprintf(temp_ground_str, sizeof(temp_ground_str), "%.1f", msg->data.temp_ground);
        }
        
        if (isnan(msg->data.soil_temp)) {
            strcpy(soil_temp_str, "null");
        } else {
            snprintf(soil_temp_str, sizeof(soil_temp_str), "%.1f", msg->data.soil_temp);
        }
        
        if (isnan(msg->data.soil_hum)) {
            strcpy(soil_hum_str, "null");
        } else {
            snprintf(soil_hum_str, sizeof(soil_hum_str), "%.1f", msg->data.soil_hum);
        }
        
        if (isnan(msg->data.soil_ph)) {
            strcpy(soil_ph_str, "null");
        } else {
            snprintf(soil_ph_str, sizeof(soil_ph_str), "%.1f", msg->data.soil_ph);
        }
        
        if (isnan(msg->data.bat_lvl)) {
            strcpy(bat_lvl_str, "null");
        } else {
            snprintf(bat_lvl_str, sizeof(bat_lvl_str), "%.2f", msg->data.bat_lvl);
        }
        
        // Integer values (just output as numbers, they don't have null indicators)
        snprintf(lux_str, sizeof(lux_str), "%d", msg->data.lux);
        snprintf(soil_ec_str, sizeof(soil_ec_str), "%d", msg->data.soil_ec);
        snprintf(soil_n_str, sizeof(soil_n_str), "%d", msg->data.soil_n);
        snprintf(soil_p_str, sizeof(soil_p_str), "%d", msg->data.soil_p);
        snprintf(soil_k_str, sizeof(soil_k_str), "%d", msg->data.soil_k);
        
        offset += snprintf(json_buffer + offset, buffer_size - offset,
            "{\"sensor_id\":\"%s\",\"data\":{"
            "\"lux\":%s,"
            "\"temp_air\":%s,"
            "\"hum_air\":%s,"
            "\"pressure\":%s,"
            "\"temp_ground\":%s,"
            "\"soil_temp\":%s,"
            "\"soil_hum\":%s,"
            "\"soil_ec\":%s,"
            "\"soil_ph\":%s,"
            "\"soil_n\":%s,"
            "\"soil_p\":%s,"
            "\"soil_k\":%s,"
            "\"soil_salinity\":null,"
            "\"soil_tds_npk\":null,"
            "\"bat_lvl\":%s,"
            "\"bat_vol\":%d"
            "}}",
            sensor_data[i].sensor_id,
            lux_str,
            temp_air_str,
            hum_air_str,
            pressure_air_str,
            temp_ground_str,
            soil_temp_str,
            soil_hum_str,
            soil_ec_str,
            soil_ph_str,
            soil_n_str,
            soil_p_str,
            soil_k_str,
            bat_lvl_str,
            msg->data.bat_vol             // mV
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
    
    // Get current connected nodes count - for gateway, this includes itself
    int connected_nodes = esp_mesh_get_routing_table_size();
    
    // Minimal debug tracking
    static int last_active_sensors = -1;
    if (active_sensors != last_active_sensors) {
        last_active_sensors = active_sensors;
    }
    
    // For gateway (root), routing table includes itself, so connected_nodes will be at least 1
    // Publish if we have data from sensors AND routing table size is minimal (only gateway/router)
    // In tree topology, routing table size should be close to 1 when all sensors are sleeping
    if (active_sensors >= 1 && connected_nodes <= 1) {
        return true;
    }
    
    // Emergency timeout trigger: ONLY if we have data and a very long time has passed
    // This prevents stuck data but should rarely be used - sensors should disconnect properly
    if (active_sensors >= 1 && (now - last_data_received_time) > 120) { // 2 minutes emergency timeout
        ESP_LOGW(MESH_TAG, "⚠️ Emergency publishing trigger (timeout): %d sensors with data, %d seconds since last data", 
                 active_sensors, (int)(now - last_data_received_time));
        ESP_LOGW(MESH_TAG, "⚠️ This suggests sensors are not disconnecting properly after sending data");
        return true;
    }
    
    return false;
}

/*******************************************************
 *                Time Coordination Functions
 *******************************************************/

// Initialize unified wake time (called once when first sensor sends data)
void initialize_unified_wake_time(void) {
    if (!wake_time_set) {
        time_t current_time = time(NULL);
        int sleep_period_minutes = CONFIG_MESH_SLEEP_CYCLE_MINUTES;
        
        // Calculate unified wake time as: current_real_time + sleep_period
        // This wake time will be used for ALL sensors to ensure synchronized wake-up
        unified_wake_time = current_time + (sleep_period_minutes * 60);
        wake_time_set = true;
    }
}

// Calculate individual sleep duration for given wake time
uint32_t calculate_sleep_duration(time_t current_time, time_t wake_time) {
    if (current_time >= wake_time) {
        ESP_LOGW(MESH_TAG, "⚠️ Current time (%lld) >= wake time (%lld), using minimum sleep", 
                 (long long)current_time, (long long)wake_time);
        return 10; // Minimum 10 seconds
    }
    return (uint32_t)(wake_time - current_time);
}

// Gateway deep sleep coordination
void gateway_enter_deep_sleep(void) {
    time_t now = time(NULL);
    
    // Gateway wakes up early to be ready when sensors wake up
    int early_wake_seconds = CONFIG_GATEWAY_EARLY_WAKE_SECONDS;
    time_t gateway_wake_time = unified_wake_time - early_wake_seconds;
    uint32_t sleep_duration = calculate_sleep_duration(now, gateway_wake_time);
        
    // Enable timer wake-up
    esp_sleep_enable_timer_wakeup(sleep_duration * 1000000ULL);
    
    // Short delay to allow logging
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
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
            previous_connected_nodes = current_connected_nodes;
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
            
            // Publish to AWS IoT Core via MQTT
            mqtt_err = mqtt_handler_publish_sensor_data("combined_sensors", combined_json);
            if (mqtt_err == ESP_OK) {
                last_publish_time = time(NULL);
                
                // Mark all sensors as published
                for (int i = 0; i < sensor_count; i++) {
                    sensor_data[i].has_data = false;
                }
                
                // Stop MQTT client after publishing
                extern esp_err_t mqtt_handler_stop(void);
                mqtt_handler_stop();
                
                // Preparing for sleep
                vTaskDelay(5000 / portTICK_PERIOD_MS); // 5 second delay for cleanup
                gateway_enter_deep_sleep();
            } else {
                ESP_LOGW(MESH_TAG, "⚠️ Failed to publish combined data to AWS IoT Core: %s", esp_err_to_name(mqtt_err));
            }
        }
        
        // Remove periodic time sync broadcast - only send time sync to sensors that sent data
        // This is now handled in the RX task when receiving sensor data
        
        // Check for publishing more frequently when we have sensor data
        int active_sensors = 0;
        for (int i = 0; i < sensor_count; i++) {
            if (sensor_data[i].has_data) {
                active_sensors++;
            }
        }
        
        // If we have sensor data, check more frequently for publishing opportunities
        if (active_sensors > 0) {
            vTaskDelay(5 * 1000 / portTICK_PERIOD_MS); // Check every 5 seconds when we have data
        } else {
            vTaskDelay(30 * 1000 / portTICK_PERIOD_MS); // Normal 30 second interval when idle
        }
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
            
            // Process agricultural sensor data
            if (sensor_msg->header.message_type == MSG_TYPE_SENSOR_DATA) {
                
                // Store sensor data for combined payload
                int sensor_idx = find_sensor_by_mac(sensor_msg->header.node_mac);
                if (sensor_idx >= 0) {
                    memcpy(&sensor_data[sensor_idx].data, sensor_msg, sizeof(sensor_message_t));
                    sensor_data[sensor_idx].has_data = true;
                    
                                        // Update last data received time
                    last_data_received_time = time(NULL);
                    
                    // Send time sync to the specific sensor that sent data
                    mesh_addr_t sender_addr;
                    memcpy(sender_addr.addr, from.addr, 6);
                    send_time_sync_message(&sender_addr);
                    
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
    // Initialize wake time once when first sensor sends data
    initialize_unified_wake_time();
    
    time_sync_message_t msg = {0};
    time_t now = time(NULL);
    uint32_t sleep_duration = calculate_sleep_duration(now, unified_wake_time);
    
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
    msg.next_wake_time = (uint32_t)unified_wake_time;  // Use SAME wake time for ALL sensors
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
        if (err != ESP_OK) {
            ESP_LOGW(MESH_TAG, "Failed to broadcast time sync: %s", esp_err_to_name(err));
        }
    } else {
        // Send to specific node
        err = esp_mesh_send(dest_addr, &data, MESH_DATA_P2P, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGW(MESH_TAG, "Failed to send time sync to "MACSTR": %s", MAC2STR(dest_addr->addr), esp_err_to_name(err));
        }
    }
} 