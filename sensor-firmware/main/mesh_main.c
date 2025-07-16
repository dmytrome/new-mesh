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

/*******************************************************
 *                Global Variables
 *******************************************************/
static const char *MESH_TAG = "mesh_main";

/*******************************************************
 *                Main Application
 *******************************************************/
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Add random startup delay to prevent all sensors from connecting simultaneously
    // This helps avoid timing conflicts during mesh formation
    uint32_t mac_last_bytes;
    esp_read_mac((uint8_t*)&mac_last_bytes, ESP_MAC_WIFI_STA);
    uint32_t delay_ms = (mac_last_bytes % 5000) + 1000; // 1-6 second random delay
    ESP_LOGI(MESH_TAG, "🔄 Adding startup delay: %lu ms to avoid connection conflicts", delay_ms);
    vTaskDelay(delay_ms / portTICK_PERIOD_MS);

    ESP_LOGI(MESH_TAG, "🔄 Normal boot - initializing standard mesh operation");
    
    // Initialize mesh network system
    ESP_ERROR_CHECK(mesh_network_init_full_system(false));
    
    // Initialize sensor-specific components (BME280, etc.)
    ESP_ERROR_CHECK(message_handler_init_sensor());
    
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%lu, root not fixed<0>(tree), ps:0",
             esp_get_minimum_free_heap_size());
    ESP_LOGI(MESH_TAG, "💡 Hint: To enable ultra-low power mode, trigger a timer wake-up");
    ESP_LOGI(MESH_TAG, "🔄 Running in standard continuous mode for now");
}