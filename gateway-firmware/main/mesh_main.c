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
    sim800l_config_t sim_config = {
        .uart_port = CONFIG_SIM800L_UART_PORT,
        .tx_pin = CONFIG_SIM800L_TX_PIN,
        .rx_pin = CONFIG_SIM800L_RX_PIN,
        .baud_rate = CONFIG_SIM800L_BAUD_RATE,
        .apn = CONFIG_SIM800L_APN,
        .apn_username = CONFIG_SIM800L_APN_USERNAME,
        .apn_password = CONFIG_SIM800L_APN_PASSWORD
    };
    
    esp_err_t sim_err = sim800l_init(&sim_config);
    if (sim_err == ESP_OK) {
        ESP_LOGI(MESH_TAG, "SIM800L module initialized successfully");
    } else {
        ESP_LOGW(MESH_TAG, "SIM800L initialization failed: %s", esp_err_to_name(sim_err));
    }
#endif
    
    ESP_LOGI(MESH_TAG, "=== ✅ Gateway Ready ===");
    
    /* Keep the gateway running - prevent app_main() from returning */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep for 1 second
    }
}