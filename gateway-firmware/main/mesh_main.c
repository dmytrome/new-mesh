#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "mesh_network.h"
#include "message_protocol.h"
#include "message_handler.h"
#include "rtc_manager.h"
#include "time_sync_manager.h"

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
    
    // Initialize RTC Manager for time synchronization and sleep coordination
    ESP_ERROR_CHECK(rtc_manager_init());
    
    // Initialize Time Sync Manager for coordinated cycles
    ESP_ERROR_CHECK(time_sync_manager_init());
    
    // Initialize mesh network system
    ESP_ERROR_CHECK(init_full_mesh_system());
    
    // Start message handler (P2P communication)
    ESP_ERROR_CHECK(message_handler_start_p2p());
    
    ESP_LOGI(MESH_TAG, "=== ✅ Gateway Ready with Time Sync Coordination ===");
    
    /* Keep the gateway running - prevent app_main() from returning */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep for 1 second
    }
}