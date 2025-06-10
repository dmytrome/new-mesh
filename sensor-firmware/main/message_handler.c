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

/*******************************************************
 *                Constants & Variables
 *******************************************************/
static const char *MSG_TAG = "message_handler";

// Communication buffers (extracted from mesh_main.c)
static uint8_t rx_buf[RX_SIZE] = { 0, };

// Message state variables (extracted from mesh_main.c)
static bool is_running = true;

/*******************************************************
 *                Task Functions (extracted from mesh_main.c)
 *******************************************************/
void esp_mesh_sensor_mac_tx_main(void *arg)
{
    mesh_data_t data;
    uint8_t mac[6];
    mesh_addr_t gateway_addr;
    sensor_mac_msg_t mac_msg;
    esp_err_t err;
    is_running = true;
    
    // Get our MAC address
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    
    // Prepare MAC message
    mac_msg.msg_type = MSG_TYPE_SENSOR_MAC;
    memcpy(mac_msg.mac_addr, mac, 6);
    mac_msg.layer = 0; // Will be updated when we know our layer
    mac_msg.timestamp = 0; // Will be updated each send
    
    data.data = (uint8_t*)&mac_msg;
    data.size = sizeof(mac_msg);
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;
    
    // Gateway address (root address)
    memset(&gateway_addr, 0, sizeof(gateway_addr));
    
    ESP_LOGI(MSG_TAG, "📡 Sensor MAC TX task started - MAC: "MACSTR"", MAC2STR(mac));
    
    while (is_running) {
        // Only send if we're connected to mesh
        if (mesh_network_is_mesh_connected() && !esp_mesh_is_root()) {
            // Update current info
            mac_msg.layer = mesh_network_get_current_layer();
            mac_msg.timestamp = esp_timer_get_time() / 1000000; // seconds
            
            // Send to root (gateway)
            err = esp_mesh_send(&gateway_addr, &data, MESH_DATA_P2P, NULL, 0);
            if (err == ESP_OK) {
                ESP_LOGI(MSG_TAG, "📤 Sent MAC to gateway: "MACSTR" (Layer %d)", 
                         MAC2STR(mac), mac_msg.layer);
            } else {
                ESP_LOGW(MSG_TAG, "❌ Failed to send MAC to gateway: 0x%x", err);
            }
        } else {
            ESP_LOGD(MSG_TAG, "⏳ Waiting for mesh connection before sending MAC...");
        }
        
        // Send every 10 seconds
        vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void esp_mesh_p2p_tx_main(void *arg)
{
    is_running = true;

    while (is_running) {
        /* Sensors primarily use dedicated MAC TX task for communication */
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
        
        // Log sensor RX activity (simplified for sensors)
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
        
        // Sensor: Enable RX and MAC TX for communication testing
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_sensor_mac_tx_main, "SENSOR_MAC_TX", 3072, NULL, 5, NULL);
        ESP_LOGI(MSG_TAG, "SENSOR: P2P RX enabled + MAC TX enabled for testing");
    }
    return ESP_OK;
}

/*******************************************************
 *                Public Interface Functions
 *******************************************************/
esp_err_t message_handler_init_sensor(void)
{
    ESP_LOGI(MSG_TAG, "Initializing sensor message handler module");
    is_running = true;
    return ESP_OK;
}

void message_handler_start_mac_tx_task(void)
{
    ESP_LOGI(MSG_TAG, "Starting MAC TX task");
    xTaskCreate(esp_mesh_sensor_mac_tx_main, "SENSOR_MAC_TX", 3072, NULL, 5, NULL);
}

void message_handler_start_p2p_tasks(void)
{
    ESP_LOGI(MSG_TAG, "Starting P2P RX and TX tasks");
    xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
    xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
}

esp_err_t message_handler_send_mac_data(void)
{
    // This would be called for one-shot MAC data sending
    ESP_LOGI(MSG_TAG, "One-shot MAC data send requested");
    return ESP_OK;
}

void message_handler_stop(void)
{
    ESP_LOGI(MSG_TAG, "Stopping message handler");
    is_running = false;
} 