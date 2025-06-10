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

/*******************************************************
 *                Constants & Macros
 *******************************************************/
// Ultra-Low Power Coordination Constants
#define DATA_COLLECTION_WINDOW_SEC    60    // 60 seconds for all data collection
#define MESH_CONNECTION_TIMEOUT_SEC   10    // 10 seconds to connect to mesh
#define SENSOR_READ_TIMEOUT_SEC       5     // 5 seconds to read all sensors
#define DATA_SEND_TIMEOUT_SEC         10    // 10 seconds to send data
#define EMERGENCY_SLEEP_SEC           300   // 5 minutes emergency sleep

// Communication Constants
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

/*******************************************************
 *                Type Definitions
 *******************************************************/
// Coordinated operation states
typedef enum {
    COORD_STATE_WAKE_UP,
    COORD_STATE_WAIT_TIME_SYNC,
    COORD_STATE_CONNECT_MESH,
    COORD_STATE_COLLECT_DATA,
    COORD_STATE_SEND_DATA,
    COORD_STATE_PREPARE_SLEEP,
    COORD_STATE_DEEP_SLEEP
} coordinated_state_t;

/*******************************************************
 *                Global Variables
 *******************************************************/
static const char *MESH_TAG = "mesh_main";

// Communication buffers
static uint8_t rx_buf[RX_SIZE] = { 0, };

// Mesh state variables (now accessed via mesh_network module)
static bool is_running = true;

// Coordination variables
static coordinated_state_t current_state = COORD_STATE_WAKE_UP;
static bool time_synchronized = false;
static bool data_collection_done = false;
static uint32_t wake_start_time = 0;

/*******************************************************
 *                Message Types
 *******************************************************/
typedef struct {
    uint8_t msg_type;           // Message type identifier
    uint8_t mac_addr[6];        // MAC address
    uint8_t layer;              // Mesh layer
    uint32_t timestamp;         // Timestamp
} sensor_mac_msg_t;

#define MSG_TYPE_SENSOR_MAC 0x01

/*******************************************************
 *                Ultra-Low Power Functions
 *******************************************************/
static esp_err_t check_wake_timeout(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000000; // Convert to seconds
    uint32_t elapsed = current_time - wake_start_time;
    
    if (elapsed > DATA_COLLECTION_WINDOW_SEC) {
        ESP_LOGW(MESH_TAG, "⚠️ Data collection timeout (%lu sec) - entering emergency sleep", elapsed);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t wait_for_time_sync_signal(uint32_t timeout_ms)
{
    // TODO: Implement time sync reception from gateway
    // For now, simulate immediate sync for basic operation
    ESP_LOGI(MESH_TAG, "⏰ Simulating time sync (TODO: implement real time sync)");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Simulate sync delay
    return ESP_OK;
}

static esp_err_t read_and_send_sensor_data_quick(void)
{
    ESP_LOGI(MESH_TAG, "📊 Reading sensor data quickly...");
    
    // TODO: Replace with actual sensor reading
    // For now, simulate sensor data
    uint8_t sensor_data[32];
    memset(sensor_data, 0xAA, sizeof(sensor_data)); // Dummy data
    
    // TODO: Send data via mesh efficiently
    ESP_LOGI(MESH_TAG, "📤 Sensor data collected and ready to send");
    
    return ESP_OK;
}

static void enter_coordinated_deep_sleep(uint32_t sleep_duration_sec)
{
    ESP_LOGI(MESH_TAG, "💤 Entering coordinated deep sleep for %lu seconds", sleep_duration_sec);
    
    // Configure wake-up timer
    esp_sleep_enable_timer_wakeup(sleep_duration_sec * 1000000ULL); // Convert to microseconds
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

static esp_err_t coordinated_sensor_cycle(void)
{
    ESP_LOGI(MESH_TAG, "🔋 Starting coordinated ultra-low power cycle");
    wake_start_time = esp_timer_get_time() / 1000000;
    
    // Phase 1: Wait for time synchronization (with timeout)
    current_state = COORD_STATE_WAIT_TIME_SYNC;
    if (wait_for_time_sync_signal(5000) == ESP_OK) {
        time_synchronized = true;
        ESP_LOGI(MESH_TAG, "✅ Time synchronized with gateway");
    } else {
        ESP_LOGW(MESH_TAG, "⚠️ Time sync timeout - proceeding anyway");
    }
    
    // Phase 2: Quick mesh connection (with timeout)
    current_state = COORD_STATE_CONNECT_MESH;
    uint32_t mesh_wait_start = esp_timer_get_time() / 1000;
    while (!mesh_network_is_mesh_connected() && (esp_timer_get_time() / 1000 - mesh_wait_start) < (MESH_CONNECTION_TIMEOUT_SEC * 1000)) {
        if (check_wake_timeout() != ESP_OK) {
            enter_coordinated_deep_sleep(EMERGENCY_SLEEP_SEC);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (mesh_network_is_mesh_connected()) {
        ESP_LOGI(MESH_TAG, "✅ Mesh connected quickly");
        
        // Phase 3: Read and send sensor data
        current_state = COORD_STATE_COLLECT_DATA;
        if (read_and_send_sensor_data_quick() == ESP_OK) {
            data_collection_done = true;
            ESP_LOGI(MESH_TAG, "✅ Sensor data collection complete");
        }
    } else {
        ESP_LOGW(MESH_TAG, "⚠️ Mesh connection failed - emergency sleep");
    }
    
    // Phase 4: Prepare for coordinated sleep
    current_state = COORD_STATE_PREPARE_SLEEP;
    
    // TODO: Wait for coordinated sleep signal from gateway
    ESP_LOGI(MESH_TAG, "💤 Entering emergency sleep (TODO: implement coordinated sleep)");
    enter_coordinated_deep_sleep(EMERGENCY_SLEEP_SEC);
    
    return ESP_OK;
}

/*******************************************************
 *                Communication Functions
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
    
    ESP_LOGI(MESH_TAG, "📡 Sensor MAC TX task started - MAC: "MACSTR"", MAC2STR(mac));
    
    while (is_running) {
        // Only send if we're connected to mesh
        if (mesh_network_is_mesh_connected() && !esp_mesh_is_root()) {
            // Update current info
            mac_msg.layer = mesh_network_get_current_layer();
            mac_msg.timestamp = esp_timer_get_time() / 1000000; // seconds
            
            // Send to root (gateway)
            err = esp_mesh_send(&gateway_addr, &data, MESH_DATA_P2P, NULL, 0);
            if (err == ESP_OK) {
                ESP_LOGI(MESH_TAG, "📤 Sent MAC to gateway: "MACSTR" (Layer %d)", 
                         MAC2STR(mac), mac_msg.layer);
            } else {
                ESP_LOGW(MESH_TAG, "❌ Failed to send MAC to gateway: 0x%x", err);
            }
        } else {
            ESP_LOGD(MESH_TAG, "⏳ Waiting for mesh connection before sending MAC...");
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
            ESP_LOGD(MESH_TAG, "Sensor in layer:%d, %s", mesh_network_get_current_layer(),
                     mesh_network_is_mesh_connected() ? "CONNECTED" : "DISCONNECTED");
            vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        /* Sensors should never become root, but handle gracefully */
        ESP_LOGW(MESH_TAG, "⚠️ Sensor unexpectedly became root - check configuration");
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
        
        // Log sensor RX activity (simplified for sensors)
        if (!(recv_count % 10)) {
            ESP_LOGD(MESH_TAG,
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
        ESP_LOGI(MESH_TAG, "SENSOR: P2P RX enabled + MAC TX enabled for testing");
    }
    return ESP_OK;
}

/*******************************************************
 *                Main Application
 *******************************************************/
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Check wake-up reason for ultra-low power coordination
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool is_coordinated_wake = (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER);
    
    if (is_coordinated_wake) {
        ESP_LOGI(MESH_TAG, "⏰ Coordinated wake-up detected - entering ultra-low power mode");
        
        // Fast initialization for coordinated operation
        ESP_ERROR_CHECK(mesh_network_init_full_system(true));
        
        // Execute coordinated ultra-low power cycle
        coordinated_sensor_cycle();
        
        // Should not reach here (will deep sleep)
        ESP_LOGW(MESH_TAG, "⚠️ Coordinated cycle returned unexpectedly");
        return;
    }
    
    // Normal initialization for standard operation or first boot
    ESP_LOGI(MESH_TAG, "🔄 Normal boot - initializing standard mesh operation");
    
    // Full initialization for normal operation
    ESP_ERROR_CHECK(mesh_network_init_full_system(false));

    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  
             esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", 
             esp_mesh_is_ps_enabled());
             
    // Provide hints for ultra-low power mode
    ESP_LOGI(MESH_TAG, "💡 Hint: To enable ultra-low power mode, trigger a timer wake-up");
    ESP_LOGI(MESH_TAG, "🔄 Running in standard continuous mode for now");
}