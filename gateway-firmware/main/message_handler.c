#include "message_handler.h"
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "message_protocol.h"
#include "time_sync_manager.h"

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *MESH_TAG = "message_handler";
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;

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
        
        // Check if we should broadcast time sync
        if (is_time_sync_ready() && should_broadcast_sync()) {
            ESP_LOGI(MESH_TAG, "⏰ Broadcasting time sync to sensors");
            broadcast_time_sync_to_sensors();
        }
        
        // Gateway status check every 30 seconds
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
                // Notify time sync manager that sensor data was received
                handle_sensor_data_received(sensor_msg->header.node_mac);
                
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
                
                // TODO: Here you could save to a collection for MQTT JSON generation
                // Example: store_sensor_data_for_mqtt(sensor_msg);
                
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
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
        ESP_LOGI(MESH_TAG, "GATEWAY: P2P TX and RX enabled with time sync coordination");
    }
    return ESP_OK;
}

// New function to start message handler from main
esp_err_t message_handler_start_p2p(void)
{
    return esp_mesh_comm_p2p_start();
} 