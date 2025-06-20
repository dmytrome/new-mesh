#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "mesh_network.h"
#include "message_protocol.h"
#include "message_handler.h"
#include "sim800l.h"

/*******************************************************
 *                Variable Definitions
 *******************************************************/
static const char *MESH_TAG = "mesh_main";

/*******************************************************
 *                Main Application Function
 *******************************************************/

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Initialize mesh network system
    ESP_ERROR_CHECK(init_full_mesh_system());
    
    // Initialize SIM800L module if enabled
#ifdef CONFIG_SIM800L_ENABLE
    ESP_LOGI(MESH_TAG, "About to initialize SIM800L...");
    
    sim800l_config_t sim_config = {
        .uart_port = CONFIG_SIM800L_UART_PORT,
        .tx_pin = CONFIG_SIM800L_TX_PIN,
        .rx_pin = CONFIG_SIM800L_RX_PIN,
        .baud_rate = CONFIG_SIM800L_BAUD_RATE,
        .apn = CONFIG_SIM800L_APN,
        .apn_username = CONFIG_SIM800L_APN_USERNAME,
        .apn_password = CONFIG_SIM800L_APN_PASSWORD
    };
    
    ESP_LOGI(MESH_TAG, "SIM800L config: UART%d, TX:%d, RX:%d, Baud:%d", 
             sim_config.uart_port, sim_config.tx_pin, sim_config.rx_pin, sim_config.baud_rate);
    
    esp_err_t sim_err = sim800l_init(&sim_config);
    if (sim_err == ESP_OK) {
        ESP_LOGI(MESH_TAG, "✅ SIM800L module initialized successfully");
    } else {
        ESP_LOGW(MESH_TAG, "❌ SIM800L initialization failed: %s", esp_err_to_name(sim_err));
    }
    
    ESP_LOGI(MESH_TAG, "SIM800L initialization attempt completed");
#endif
    
    ESP_LOGI(MESH_TAG, "=== ✅ Gateway Ready ===");
    
    /* Keep the gateway running - prevent app_main() from returning */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // Check every 10 seconds
        
#ifdef CONFIG_SIM800L_ENABLE
        // Check SIM800L status periodically
        sim800l_status_t status = sim800l_get_status();
        ESP_LOGI(MESH_TAG, "SIM800L Status: %d", status);
        
        if (status >= SIM800L_STATUS_NETWORK_REGISTERED) {
            int signal = sim800l_get_signal_strength();
            ESP_LOGI(MESH_TAG, "Signal Strength: %d (0-31 range, 99=unknown)", signal);
            
            // Try GPRS connection if not connected
            if (status == SIM800L_STATUS_NETWORK_REGISTERED) {
                ESP_LOGI(MESH_TAG, "Attempting GPRS connection...");
                esp_err_t gprs_err = sim800l_connect_gprs();
                if (gprs_err == ESP_OK) {
                    ESP_LOGI(MESH_TAG, "GPRS connection successful!");
                } else {
                    ESP_LOGW(MESH_TAG, "GPRS connection failed: %s", esp_err_to_name(gprs_err));
                }
            }
            
            // Test internet connectivity if GPRS is connected
            if (status == SIM800L_STATUS_GPRS_CONNECTED) {
                ESP_LOGI(MESH_TAG, "Testing internet connectivity...");
                esp_err_t http_err = sim800l_http_post("http://httpbin.org/post", 
                                                      "{\"test\":\"connectivity\"}", 21, 
                                                      NULL, 0);
                if (http_err == ESP_OK) {
                    ESP_LOGI(MESH_TAG, "✅ Internet connectivity test PASSED!");
                } else {
                    ESP_LOGW(MESH_TAG, "❌ Internet connectivity test FAILED: %s", esp_err_to_name(http_err));
                }
            }
        }
#endif
    }
}