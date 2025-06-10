#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "mesh_network.h"
#include "message_protocol.h"
#include "message_handler.h"

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
    
    ESP_LOGI(MESH_TAG, "=== ✅ Gateway Ready ===");
    
    /* Keep the gateway running - prevent app_main() from returning */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep for 1 second
    }
}