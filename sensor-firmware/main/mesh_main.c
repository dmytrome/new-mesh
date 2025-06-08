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

/*******************************************************
 *                Constants & Macros
 *******************************************************/
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};

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

// Mesh state variables
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

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
 *                Function Declarations
 *******************************************************/
void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

/*******************************************************
 *                Common Initialization Functions
 *******************************************************/
static esp_err_t init_wifi_and_netif(void)
{
    ESP_LOGI(MESH_TAG, "Initializing WiFi and network interfaces...");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    return ESP_OK;
}

static esp_err_t init_mesh_base(void)
{
    ESP_LOGI(MESH_TAG, "Initializing mesh base configuration...");
    
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_fix_root(false));
    ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false));
    
    return ESP_OK;
}

static esp_err_t configure_and_start_mesh(void)
{
    ESP_LOGI(MESH_TAG, "Configuring and starting mesh...");
    
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    
    // Mesh ID and channel
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    cfg.channel = CONFIG_MESH_CHANNEL;
    
    // Router-less mesh: Provide minimal valid router credentials for validation
    // These credentials satisfy ESP-IDF validation but won't be used in router-less operation
    strcpy((char*)cfg.router.ssid, "dummy");
    cfg.router.ssid_len = strlen("dummy");
    strcpy((char*)cfg.router.password, "dummy_password");
    memset(cfg.router.bssid, 0, sizeof(cfg.router.bssid));
    
    // Mesh AP configuration
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));
    ESP_ERROR_CHECK(esp_mesh_start());
    
    return ESP_OK;
}

static esp_err_t init_full_mesh_system(bool is_fast_init)
{
    ESP_LOGI(MESH_TAG, "Initializing %s mesh system...", is_fast_init ? "fast" : "full");
    
    // Initialize WiFi and network interfaces
    ESP_ERROR_CHECK(init_wifi_and_netif());
    
    // Register IP event handler for normal operation
    if (!is_fast_init) {
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    }
    
    // Initialize mesh base configuration
    ESP_ERROR_CHECK(init_mesh_base());
    
    // Additional configuration for normal operation
    if (!is_fast_init) {
        ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
        ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
        
#ifdef CONFIG_MESH_ENABLE_PS
        ESP_ERROR_CHECK(esp_mesh_enable_ps());
        ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
#else
        ESP_ERROR_CHECK(esp_mesh_disable_ps());
        ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
#endif
    }
    
    // Configure and start mesh
    ESP_ERROR_CHECK(configure_and_start_mesh());
    
    return ESP_OK;
}

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
    while (!is_mesh_connected && (esp_timer_get_time() / 1000 - mesh_wait_start) < (MESH_CONNECTION_TIMEOUT_SEC * 1000)) {
        if (check_wake_timeout() != ESP_OK) {
            enter_coordinated_deep_sleep(EMERGENCY_SLEEP_SEC);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    if (is_mesh_connected) {
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
    ESP_LOGI(MESH_TAG, "🔄 Coordinated cycle complete - preparing for sleep");
    
    // Calculate next wake time (TODO: get from gateway time sync)
    uint32_t next_sleep_duration = 300; // 5 minutes default
    
    // Enter coordinated deep sleep
    current_state = COORD_STATE_DEEP_SLEEP;
    enter_coordinated_deep_sleep(next_sleep_duration);
    
    return ESP_OK;
}

/*******************************************************
 *                Communication Functions
 *******************************************************/
void esp_mesh_sensor_mac_tx_main(void *arg)
{
    esp_err_t err;
    mesh_addr_t gateway_addr;
    mesh_data_t data;
    sensor_mac_msg_t mac_msg;
    uint8_t mac[6];
    
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
        if (is_mesh_connected && !esp_mesh_is_root()) {
            // Update current info
            mac_msg.layer = mesh_layer;
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
            ESP_LOGD(MESH_TAG, "Sensor in layer:%d, %s", mesh_layer,
                     is_mesh_connected ? "CONNECTED" : "DISCONNECTED");
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
                     recv_count, mesh_layer, MAC2STR(from.addr),
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
 *                Event Handlers
 *******************************************************/
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        ESP_LOGD(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>");
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        ESP_LOGD(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>");
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    /* TODO handler for the failure */
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%" PRId32 "", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));
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
        ESP_ERROR_CHECK(init_full_mesh_system(true));
        
        // Execute coordinated ultra-low power cycle
        coordinated_sensor_cycle();
        
        // Should not reach here (will deep sleep)
        ESP_LOGW(MESH_TAG, "⚠️ Coordinated cycle returned unexpectedly");
        return;
    }
    
    // Normal initialization for standard operation or first boot
    ESP_LOGI(MESH_TAG, "🔄 Normal boot - initializing standard mesh operation");
    
    // Full initialization for normal operation
    ESP_ERROR_CHECK(init_full_mesh_system(false));

    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  
             esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", 
             esp_mesh_is_ps_enabled());
             
    // Provide hints for ultra-low power mode
    ESP_LOGI(MESH_TAG, "💡 Hint: To enable ultra-low power mode, trigger a timer wake-up");
    ESP_LOGI(MESH_TAG, "🔄 Running in standard continuous mode for now");
}