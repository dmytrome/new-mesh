#include "message_handler.h"
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "message_protocol.h"
#include "mqtt_handler.h"

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *MESH_TAG = "message_handler";
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;

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
static const int PUBLISH_INTERVAL = 30; // Publish every 30 seconds

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

// Check if we should publish (time-based or data-based)
static bool should_publish(void) {
    time_t now;
    time(&now);
    
    // Publish if enough time has passed
    if (now - last_publish_time >= PUBLISH_INTERVAL) {
        return true;
    }
    
    // Publish if we have data from all sensors
    int active_sensors = 0;
    for (int i = 0; i < sensor_count; i++) {
        if (sensor_data[i].has_data) {
            active_sensors++;
        }
    }
    
    return active_sensors >= 2; // Publish when we have at least 2 sensors
}

/*******************************************************
 *                P2P Communication Functions (extracted from mesh_main.c)
 *******************************************************/

void esp_mesh_p2p_tx_main(void *arg)
{
    int send_count = 0;
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
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
        
        if (!(send_count % 10)) {
            ESP_LOGI(MESH_TAG, "🌐 GATEWAY STATUS: Connected sensors:%d/%d, cycle:%d", 
                     route_table_size, esp_mesh_get_routing_table_size(), send_count);
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
                    
                    ESP_LOGI(MESH_TAG, "💾 Stored data for sensor %s (total sensors: %d)", 
                             sensor_data[sensor_idx].sensor_id, sensor_count);
                    
                    // Check if we should publish combined payload
                    if (should_publish()) {
                        static char combined_json[2048];
                        create_combined_payload(combined_json, sizeof(combined_json));
                        
                        ESP_LOGI(MESH_TAG, "📤 Publishing combined payload with %d sensors", sensor_count);
                        ESP_LOGI(MESH_TAG, "📄 JSON: %s", combined_json);
                        
                        // Publish to AWS IoT Core via MQTT
                        esp_err_t mqtt_err = mqtt_handler_publish_sensor_data("combined_sensors", combined_json);
                        if (mqtt_err == ESP_OK) {
                            ESP_LOGI(MESH_TAG, "☁️ Combined data published to AWS IoT Core successfully");
                            last_publish_time = time(NULL);
                            
                            // Mark all sensors as published
                            for (int i = 0; i < sensor_count; i++) {
                                sensor_data[i].has_data = false;
                            }
                        } else {
                            ESP_LOGW(MESH_TAG, "⚠️ Failed to publish combined data to AWS IoT Core: %s", esp_err_to_name(mqtt_err));
                        }
                    }
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