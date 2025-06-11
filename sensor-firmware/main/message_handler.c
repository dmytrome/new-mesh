#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "message_handler.h"
#include "mesh_network.h"
#include "time_sync_manager.h"

/*******************************************************
 *                Constants & Variables
 *******************************************************/
static const char *MSG_TAG = "message_handler";

// Communication buffers (extracted from mesh_main.c)
static uint8_t rx_buf[RX_SIZE] = { 0, };

// Message state variables (extracted from mesh_main.c)
static bool is_running = true;
static uint16_t sequence_number = 0;

/*******************************************************
 *                Sensor Data Collection
 *******************************************************/
esp_err_t collect_sensor_readings(sensor_data_t* data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    
    // Simulate agricultural sensor readings (replace with real sensors later)
    data->lux = 450 + (esp_timer_get_time() % 200);  // 450-650 lux
    
    // Air measurements
    data->temp_air = 22.5f + ((esp_timer_get_time() % 100) / 50.0f);  // 22.5-24.5°C
    data->hum_air = 65.0f + ((esp_timer_get_time() % 200) / 10.0f);   // 65-85%
    
    // Ground temperature
    data->temp_ground = 18.0f + ((esp_timer_get_time() % 80) / 40.0f); // 18-20°C
    
    // Soil measurements
    data->soil_temp = 19.0f + ((esp_timer_get_time() % 60) / 30.0f);  // 19-21°C  
    data->soil_hum = 45.0f + ((esp_timer_get_time() % 300) / 10.0f);  // 45-75%
    data->soil_ec = 800 + (esp_timer_get_time() % 400);               // 800-1200 µS/cm
    data->soil_ph = 6.2f + ((esp_timer_get_time() % 120) / 100.0f);   // 6.2-7.4 pH
    data->soil_n = 25 + (esp_timer_get_time() % 15);                  // 25-40 mg/kg
    data->soil_p = 12 + (esp_timer_get_time() % 8);                   // 12-20 mg/kg
    data->soil_k = 180 + (esp_timer_get_time() % 40);                 // 180-220 mg/kg
    data->soil_salinity = 300 + (esp_timer_get_time() % 200);         // 300-500 mg/L
    data->soil_tds_npk = 450 + (esp_timer_get_time() % 150);          // 450-600 ppm
    
    // Battery status (simulated)
    data->bat_lvl = 3.7f - ((esp_timer_get_time() % 1000) / 10000.0f); // 3.6-3.7V
    data->bat_vol = (uint16_t)(data->bat_lvl * 1000);                   // mV
    
    // Metadata
    data->reading_timestamp = esp_timer_get_time() / 1000000;  // Unix timestamp
    data->sensor_status = 0x00;  // No errors
    data->reading_quality = 95;  // 95% quality
    
    ESP_LOGD(MSG_TAG, "📊 Collected sensor data: %.1f°C, %.1f%% RH, %d lux, pH %.1f", 
             data->temp_air, data->hum_air, data->lux, data->soil_ph);
    
    return ESP_OK;
}

/*******************************************************
 *                Task Functions (updated to use rich protocol)
 *******************************************************/
void esp_mesh_sensor_mac_tx_main(void *arg)
{
    mesh_data_t data;
    uint8_t mac[6];
    mesh_addr_t gateway_addr;
    sensor_message_t sensor_msg;
    esp_err_t err;
    is_running = true;
    
    // Get our MAC address
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    
    // Gateway address (root address)
    memset(&gateway_addr, 0, sizeof(gateway_addr));
    
    ESP_LOGI(MSG_TAG, "🌱 Agricultural sensor TX task started - MAC: "MACSTR"", MAC2STR(mac));
    
    while (is_running) {
        // Only send if we're connected to mesh and in collection window
        if (mesh_network_is_mesh_connected() && !esp_mesh_is_root()) {
            // Check if we should send data based on time sync
            if (is_sensor_time_synchronized() && should_send_sensor_data()) {
                // Prepare message header
                memcpy(sensor_msg.header.node_mac, mac, 6);
                sensor_msg.header.node_type = NODE_TYPE_SENSOR;
                sensor_msg.header.message_type = MSG_TYPE_SENSOR_DATA;
                sensor_msg.header.timestamp = esp_timer_get_time() / 1000000;
                sensor_msg.header.sequence_number = sequence_number++;
                sensor_msg.header.mesh_layer = mesh_network_get_current_layer();
                sensor_msg.header.signal_strength = 70; // Simulated signal strength
                
                // Collect sensor readings
                if (collect_sensor_readings(&sensor_msg.data) == ESP_OK) {
                    // Calculate checksum
                    sensor_msg.checksum = calculate_checksum(&sensor_msg, sizeof(sensor_msg) - sizeof(sensor_msg.checksum));
                    
                    // Prepare mesh data
                    data.data = (uint8_t*)&sensor_msg;
                    data.size = sizeof(sensor_msg);
                    data.proto = MESH_PROTO_BIN;
                    data.tos = MESH_TOS_P2P;
                    
                    // Send to root (gateway)
                    err = esp_mesh_send(&gateway_addr, &data, MESH_DATA_P2P, NULL, 0);
                    if (err == ESP_OK) {
                        ESP_LOGI(MSG_TAG, "🌱 Sent agricultural data: Air %.1f°C, Soil %.1f°C, %.1f%% RH, %d lux, pH %.1f (Layer %d, Seq %d)", 
                                 sensor_msg.data.temp_air, sensor_msg.data.soil_temp, sensor_msg.data.hum_air, 
                                 sensor_msg.data.lux, sensor_msg.data.soil_ph,
                                 sensor_msg.header.mesh_layer, sensor_msg.header.sequence_number);
                    } else {
                        ESP_LOGW(MSG_TAG, "❌ Failed to send sensor data to gateway: 0x%x", err);
                    }
                } else {
                    ESP_LOGW(MSG_TAG, "❌ Failed to collect sensor readings");
                }
            } else if (is_sensor_time_synchronized()) {
                ESP_LOGD(MSG_TAG, "⏳ Outside data collection window - waiting for next cycle");
            } else {
                ESP_LOGD(MSG_TAG, "⏳ Waiting for time synchronization before sending data");
            }
        } else {
            ESP_LOGD(MSG_TAG, "⏳ Waiting for mesh connection before sending data...");
        }
        
        // Send every 2 seconds for faster response to time sync (was 10 seconds)
        vTaskDelay(2 * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void esp_mesh_p2p_tx_main(void *arg)
{
    is_running = true;

    while (is_running) {
        /* Sensors primarily use dedicated sensor data TX task for communication */
        if (!esp_mesh_is_root()) {
            ESP_LOGD(MSG_TAG, "Sensor in layer:%d, %s", mesh_network_get_current_layer(),
                     mesh_network_is_mesh_connected() ? "CONNECTED" : "DISCONNECTED");
            vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        /* Sensors should never become root, but handle gracefully */
        ESP_LOGW(MSG_TAG, "⚠️ Sensor unexpectedly became root - check configuration");
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
            ESP_LOGE(MSG_TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }
        
        recv_count++;
        
        // Check if this is a time sync message
        if (data.size == sizeof(time_sync_message_t)) {
            time_sync_message_t *sync_msg = (time_sync_message_t*)data.data;
            
            if (sync_msg->header.message_type == MSG_TYPE_TIME_SYNC && 
                sync_msg->header.node_type == NODE_TYPE_GATEWAY) {
                
                ESP_LOGI(MSG_TAG, "⏰ Received time sync message from gateway ["MACSTR"]", 
                         MAC2STR(from.addr));
                
                // Process time sync message
                esp_err_t sync_result = handle_time_sync_message(sync_msg);
                if (sync_result == ESP_OK) {
                    ESP_LOGI(MSG_TAG, "✅ Time sync processed successfully");
                } else {
                    ESP_LOGW(MSG_TAG, "⚠️ Failed to process time sync: %s", esp_err_to_name(sync_result));
                }
                continue;
            }
        }
        
        // Log other sensor RX activity (simplified for sensors)
        if (!(recv_count % 10)) {
            ESP_LOGD(MSG_TAG,
                     "[#RX:%d][L:%d] from "MACSTR", size:%d, heap:%" PRId32,
                     recv_count, mesh_network_get_current_layer(), MAC2STR(from.addr),
                     data.size, esp_get_minimum_free_heap_size());
        }
    }
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        
        // Sensor: Enable RX and agricultural data TX for communication
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_sensor_mac_tx_main, "SENSOR_DATA_TX", 3072, NULL, 5, NULL);
        ESP_LOGI(MSG_TAG, "🌱 SENSOR: P2P RX enabled + Agricultural data TX enabled");
    }
    return ESP_OK;
}

/*******************************************************
 *                Public Interface Functions
 *******************************************************/
esp_err_t message_handler_init_sensor(void)
{
    ESP_LOGI(MSG_TAG, "🌱 Initializing agricultural sensor message handler");
    is_running = true;
    sequence_number = 0;
    return ESP_OK;
}

void message_handler_start_mac_tx_task(void)
{
    ESP_LOGI(MSG_TAG, "Starting agricultural data TX task");
    xTaskCreate(esp_mesh_sensor_mac_tx_main, "SENSOR_DATA_TX", 3072, NULL, 5, NULL);
}

void message_handler_start_p2p_tasks(void)
{
    ESP_LOGI(MSG_TAG, "Starting P2P RX and TX tasks");
    xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
    xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
}

esp_err_t message_handler_send_mac_data(void)
{
    // Legacy function - now sends full sensor data
    return message_handler_send_sensor_data();
}

esp_err_t message_handler_send_sensor_data(void)
{
    ESP_LOGI(MSG_TAG, "One-shot agricultural sensor data send requested");
    // Could implement immediate sensor reading and send here
    return ESP_OK;
}

void message_handler_stop(void)
{
    ESP_LOGI(MSG_TAG, "Stopping agricultural sensor message handler");
    is_running = false;
} 