#include "mesh_network.h"
#include "message_handler.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_mesh_internal.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

/*******************************************************
 *                Constants and Variables
 *******************************************************/
static const char *MESH_TAG = "mesh_network";

// SNTP initialization flag
static bool sntp_initialized = false;

// Mesh constants
const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};

// Network state variables
static bool is_mesh_connected = false;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

/*******************************************************
 *                SNTP Functions
 *******************************************************/

static void initialize_sntp(void) {
    if (sntp_initialized) {
        ESP_LOGW(MESH_TAG, "⚠️ SNTP already initialized");
        return;
    }
    
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    sntp_initialized = true;
    
    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

/*******************************************************
 *                Public Interface Functions
 *******************************************************/

bool is_mesh_connected_status(void) {
    return is_mesh_connected;
}

int get_mesh_layer(void) {
    return mesh_layer;
}

esp_netif_t* get_netif_sta(void) {
    return netif_sta;
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
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
        
        ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
        ESP_ERROR_CHECK(esp_mesh_connect());
        
        /* GATEWAY: Start P2P communication as root */
        is_mesh_connected = true;  // Set as connected since we're the root
        extern esp_err_t esp_mesh_comm_p2p_start(void);  // Forward declaration for message handler
        esp_mesh_comm_p2p_start();
        // P2P started
        
        // Add a small delay to ensure mesh AP is fully ready before accepting connections
        // Waiting for mesh AP
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        // Child connected
        (void)event_data; // Suppress unused parameter warning
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        // Child disconnected
        (void)event_data; // Suppress unused parameter warning
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        extern esp_err_t esp_mesh_comm_p2p_start(void);  // Forward declaration for message handler
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
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d", mesh_layer);
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
    
    // Initialize SNTP for time sync after getting IP address
    initialize_sntp();
}

/*******************************************************
 *                Mesh Initialization
 *******************************************************/

esp_err_t init_full_mesh_system(void) {
    ESP_LOGI(MESH_TAG, "Initializing full mesh system...");
    
    /* tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /* event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* create network interfaces for mesh */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    /* wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
    
    /* GATEWAY: Configure as self-organizing root (not fixed root) */
    ESP_ERROR_CHECK(esp_mesh_fix_root(false));  // All devices must use same fixed root setting
    ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, true));  // Enable self-organizing + become root
    ESP_LOGI(MESH_TAG, "GATEWAY: Configured as self-organizing root");

#ifdef CONFIG_MESH_ENABLE_PS
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
#else
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
#endif

    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* All credentials are taken from sdkconfig */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    cfg.channel = CONFIG_MESH_CHANNEL;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));
    
    /* GATEWAY: Connect to real WiFi router using fast scan approach */
    strcpy((char*)cfg.router.ssid, CONFIG_MESH_ROUTER_SSID);
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    strcpy((char*)cfg.router.password, CONFIG_MESH_ROUTER_PASSWD);
    memset(cfg.router.bssid, 0, sizeof(cfg.router.bssid));

    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    /* Mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
    
    ESP_LOGI(MESH_TAG, "GATEWAY: Root node started and connecting to router");

    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());

    return ESP_OK;
} 