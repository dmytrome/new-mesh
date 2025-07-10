#include <string.h>
#include <inttypes.h>
#include <sys/time.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "message_handler.h"
#include "mesh_network.h"
#include "message_protocol.h"


/*******************************************************
 *                Constants & Variables
 *******************************************************/
static const char *MSG_TAG = "message_handler";

// Communication buffers (extracted from mesh_main.c)
static uint8_t rx_buf[RX_SIZE] = { 0, };

// Message state variables (extracted from mesh_main.c)
static bool is_running = true;
static uint16_t sequence_number = 0;

// Sleep coordination variables
static time_t stored_unified_wake_time = 0;  // Store wake time from gateway
static bool time_sync_received = false;      // Flag to track if we got time sync
static time_t data_sent_time = 0;            // Track when we sent data

// Forward declarations
static void enter_coordinated_deep_sleep(void);

/*******************************************************
 *                Sensor Data Collection
 *******************************************************/
esp_err_t collect_sensor_readings(sensor_data_t* data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    
    // Simulate agricultural sensor readings (replace with real sensors later)
    data->lux = 450 + (esp_timer_get_time() % 200);  // 450-650 lux
    
    // Air measurements
    data->temp_air = 22.5f + ((esp_timer_get_time() % 100) / 50.0f);  // 22.5-24.5°C
    data->hum_air = 65.0f + ((esp_timer_get_time() % 200) / 10.0f);   // 65-85%
    
    // Ground temperature
    data->temp_ground = 18.0f + ((esp_timer_get_time() % 80) / 40.0f); // 18-20°C
    
    // Soil measurements
    data->soil_temp = 19.0f + ((esp_timer_get_time() % 60) / 30.0f);  // 19-21°C  
    data->soil_hum = 45.0f + ((esp_timer_get_time() % 300) / 10.0f);  // 45-75%
    data->soil_ec = 800 + (esp_timer_get_time() % 400);               // 800-1200 µS/cm
    data->soil_ph = 6.2f + ((esp_timer_get_time() % 120) / 100.0f);   // 6.2-7.4 pH
    data->soil_n = 25 + (esp_timer_get_time() % 15);                  // 25-40 mg/kg
    data->soil_p = 12 + (esp_timer_get_time() % 8);                   // 12-20 mg/kg
    data->soil_k = 180 + (esp_timer_get_time() % 40);                 // 180-220 mg/kg
    data->soil_salinity = 300 + (esp_timer_get_time() % 200);         // 300-500 mg/L
    data->soil_tds_npk = 450 + (esp_timer_get_time() % 150);          // 450-600 ppm
    
    // Battery status (simulated)
    data->bat_lvl = 3.7f - ((esp_timer_get_time() % 1000) / 10000.0f); // 3.6-3.7V
    data->bat_vol = (uint16_t)(data->bat_lvl * 1000);                   // mV
    
    // Metadata
    data->reading_timestamp = esp_timer_get_time() / 1000000;  // Unix timestamp
    data->sensor_status = 0x00;  // No errors
    data->reading_quality = 95;  // 95% quality
    
    ESP_LOGD(MSG_TAG, "📊 Collected sensor data: %.1f°C, %.1f%% RH, %d lux, pH %.1f", 
             data->temp_air, data->hum_air, data->lux, data->soil_ph);
    
    return ESP_OK;
}

/*******************************************************
 *                Task Functions (updated to use rich protocol)
 *******************************************************/
void esp_mesh_sensor_mac_tx_main(void *arg)
{
    ESP_LOGI(MSG_TAG, "🌱 Agricultural sensor TX task started - MAC: " MACSTR, MAC2STR(mesh_network_get_parent_addr().addr));
    
    // Wait for mesh connection before sending data
    const int wait_time_sec = CONFIG_SENSOR_DATA_WAIT_TIME_SEC;
    ESP_LOGI(MSG_TAG, "⏳ Waiting %d seconds before sending data to allow mesh connections...", wait_time_sec);
    vTaskDelay(wait_time_sec * 1000 / portTICK_PERIOD_MS);
    
    // Send data only once
    if (mesh_network_is_mesh_connected()) {
        ESP_LOGI(MSG_TAG, "📤 Sending agricultural sensor data to gateway...");
        
        // Prepare sensor message
        sensor_message_t sensor_msg;
        memset(&sensor_msg, 0, sizeof(sensor_msg));
        
        // Set message header
        uint8_t sensor_mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, sensor_mac);
        memcpy(sensor_msg.header.node_mac, sensor_mac, 6);
        sensor_msg.header.node_type = NODE_TYPE_SENSOR;
        sensor_msg.header.message_type = MSG_TYPE_SENSOR_DATA;
        sensor_msg.header.sequence_number = sequence_number++;
        sensor_msg.header.mesh_layer = mesh_network_get_current_layer();
        sensor_msg.header.timestamp = time(NULL);
        
        // Prepare mesh data for sending to root (gateway)
        mesh_data_t data;
        data.data = (uint8_t*)&sensor_msg;
        data.size = sizeof(sensor_msg);
        data.proto = MESH_PROTO_BIN;
        data.tos = MESH_TOS_P2P;
        
        // Collect sensor readings
        if (collect_sensor_readings(&sensor_msg.data) == ESP_OK) {
            // Calculate checksum
            sensor_msg.checksum = calculate_checksum(&sensor_msg, sizeof(sensor_msg) - sizeof(sensor_msg.checksum));
            
            // Prepare mesh data
            data.data = (uint8_t*)&sensor_msg;
            data.size = sizeof(sensor_msg);
            data.proto = MESH_PROTO_BIN;
            data.tos = MESH_TOS_P2P;
            
            // Send to root (gateway) - ESP-MESH will handle routing through intermediate nodes
            esp_err_t err = esp_mesh_send(NULL, &data, MESH_DATA_TODS, NULL, 0);
            if (err == ESP_OK) {
                ESP_LOGI(MSG_TAG, "🌱 Sent agricultural data: Air %.1f°C, Soil %.1f°C, %.1f%% RH, %d lux, pH %.1f (Layer %d, Seq %d)", 
                         sensor_msg.data.temp_air, sensor_msg.data.soil_temp, sensor_msg.data.hum_air, 
                         sensor_msg.data.lux, sensor_msg.data.soil_ph,
                         sensor_msg.header.mesh_layer, sensor_msg.header.sequence_number);
                
                // Record when we sent data for timeout tracking
                data_sent_time = esp_timer_get_time() / 1000000;
            } else {
                ESP_LOGW(MSG_TAG, "❌ Failed to send sensor data to gateway: 0x%x", err);
            }
        } else {
            ESP_LOGW(MSG_TAG, "❌ Failed to collect sensor readings");
        }
    } else {
        ESP_LOGD(MSG_TAG, "⏳ Waiting for mesh connection before sending data...");
    }
    
    // Wait after sending to allow other sensors to send and connect
    ESP_LOGI(MSG_TAG, "⏳ Waiting %d seconds after sending data before next cycle...", wait_time_sec);
    vTaskDelay(wait_time_sec * 1000 / portTICK_PERIOD_MS);

    // After sending and waiting, check if we are a leaf node (no children)
    int check_count = 0;
    const int max_wait_time = CONFIG_SENSOR_CHILDREN_WAIT_TIME_SEC;
    const int check_interval = CONFIG_SENSOR_CHILDREN_CHECK_INTERVAL_SEC;
    const int max_checks = max_wait_time / check_interval;
    
    ESP_LOGI(MSG_TAG, "🌿 Checking for mesh children (max wait: %ds, check interval: %ds)...", max_wait_time, check_interval);
    
    while (check_count < max_checks) {
        // Better way to check if we're a leaf node: check if we have any child connections
        int child_count = esp_mesh_get_routing_table_size() - 1; // Subtract parent connection
        
        // For debugging, also check the actual routing table
        mesh_addr_t route_table[10];
        int route_table_size = 10;
        esp_mesh_get_routing_table(route_table, 10 * sizeof(mesh_addr_t), &route_table_size);
        
        // A sensor at layer 2+ should be able to sleep after sending data if no real children
        // Check if we're at leaf position (no downstream sensors depend on us)
        bool is_leaf_node = (child_count <= 0) || (mesh_network_get_current_layer() >= 3);
        
        if (is_leaf_node) {
            break;
        } else {
            ESP_LOGI(MSG_TAG, "🌳 Still have mesh children (layer %d, children: %d). Waiting before sleep... (%d/%d checks)", 
                     mesh_network_get_current_layer(), child_count, check_count + 1, max_checks);
            vTaskDelay(check_interval * 1000 / portTICK_PERIOD_MS);
            check_count++;
        }
    }
    
    if (check_count >= max_checks) {
        ESP_LOGW(MSG_TAG, "⚠️ Still have children after %ds, forcing sleep for safety.", max_wait_time);
    }
        
    // Wait for time sync (the RX task will handle this and trigger deep sleep)
    // This task will continue running until time sync is received and deep sleep is triggered
    while (is_running) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Check every second
    }
    vTaskDelete(NULL);
}

void esp_mesh_p2p_tx_main(void *arg)
{
    is_running = true;

    while (is_running) {
        /* Sensors primarily use dedicated sensor data TX task for communication */
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
        
        // Use timeout instead of portMAX_DELAY to enable timeout checking
        err = esp_mesh_recv(&from, &data, pdMS_TO_TICKS(5000), &flag, NULL, 0); // 5 second timeout
        
        // Check for timeout - if we've been waiting too long for time sync, enter emergency sleep
        if (data_sent_time > 0 && !time_sync_received) {
            time_t current_time = esp_timer_get_time() / 1000000;
            time_t wait_time = current_time - data_sent_time;
            
            if (wait_time > 30) { // 30 second timeout
                ESP_LOGW(MSG_TAG, "⏰ No time sync received within 30 seconds after sending data - entering emergency sleep");
                ESP_LOGI(MSG_TAG, "💤 Entering emergency deep sleep for 10 seconds...");
                esp_sleep_enable_timer_wakeup(10 * 1000000ULL); // 10 seconds
                esp_deep_sleep_start();
            }
        }
        
        if (err != ESP_OK || !data.size) {
            // Suppress timeout errors (0x400a) as they are expected during normal operation
            if (err != ESP_ERR_TIMEOUT && err != 0x400a) {
                ESP_LOGE(MSG_TAG, "mesh_recv error: 0x%x, size:%d", err, data.size);
            }
            
            // IMPORTANT: Check if we already have time sync and can now sleep (after timeout)
            // This handles the case where sensor received time sync but had children,
            // and now children have disconnected so it can sleep
            if (time_sync_received && stored_unified_wake_time > 0) {
                int child_count = esp_mesh_get_routing_table_size() - 1; // Subtract parent connection
                bool is_leaf_node = (child_count <= 0) || (mesh_network_get_current_layer() >= 3);
                
                if (is_leaf_node) {
                    break; // Exit the receive loop to start coordination cycle
                }
            }
            
            continue;
        }
        
        recv_count++;
        
        // Debug: Log all received messages to understand what's coming through
        if (recv_count <= 5 || (recv_count % 10) == 0) {
            ESP_LOGI(MSG_TAG, "📨 RX #%d: from "MACSTR", size:%d, expected time_sync size:%d", 
                     recv_count, MAC2STR(from.addr), data.size, sizeof(time_sync_message_t));
        }
        
        // Handle sensor data from children that needs to be forwarded to gateway
        if (data.size == sizeof(sensor_message_t)) {
            sensor_message_t *sensor_msg = (sensor_message_t*)data.data;
            
            if (sensor_msg->header.message_type == MSG_TYPE_SENSOR_DATA) {
                ESP_LOGI(MSG_TAG, "📤 Forwarding child sensor data to gateway: Air %.1f°C, Soil %.1f°C, %.1f%% RH, %d lux, pH %.1f (Layer %d)", 
                         sensor_msg->data.temp_air, sensor_msg->data.soil_temp, sensor_msg->data.hum_air, 
                         sensor_msg->data.lux, sensor_msg->data.soil_ph, sensor_msg->header.mesh_layer);
                
                // Forward the data to the gateway/root using TODS
                mesh_data_t forward_data = {
                    .data = (uint8_t*)sensor_msg,
                    .size = sizeof(sensor_message_t)
                };
                
                esp_err_t err = esp_mesh_send(NULL, &forward_data, MESH_DATA_TODS, NULL, 0);
                if (err == ESP_OK) {
                    ESP_LOGI(MSG_TAG, "✅ Forwarded child sensor data to gateway successfully");
                } else {
                    ESP_LOGW(MSG_TAG, "❌ Failed to forward child sensor data: %s", esp_err_to_name(err));
                }
                continue; // Continue processing other messages
            }
        }
        
        // Handle time sync messages from gateway
        if (data.size == sizeof(time_sync_message_t)) {
            time_sync_message_t *time_msg = (time_sync_message_t*)data.data;
            
            if (time_msg->header.message_type == MSG_TYPE_TIME_SYNC) {
                time_t current_time = time(NULL);
                
                ESP_LOGI(MSG_TAG, "🕐 Received unified time sync from gateway:");
                ESP_LOGI(MSG_TAG, "   Gateway current time: %lu", time_msg->current_unix_time);
                ESP_LOGI(MSG_TAG, "   Unified wake time: %lld", (long long)time_msg->next_wake_time);
                ESP_LOGI(MSG_TAG, "   My current time: %lld", (long long)current_time);
                
                // Set system time to gateway time for synchronization
                struct timeval tv;
                tv.tv_sec = time_msg->current_unix_time;
                tv.tv_usec = 0;
                settimeofday(&tv, NULL);
                ESP_LOGI(MSG_TAG, "🕐 Updated system time to gateway time: %lu", time_msg->current_unix_time);
                
                // Recalculate current time after synchronization
                current_time = time(NULL);
                ESP_LOGI(MSG_TAG, "🕐 Synchronized current time: %lld", (long long)current_time);
                
                // Store unified wake time for later calculation
                stored_unified_wake_time = time_msg->next_wake_time;
                time_sync_received = true;
                
                // CRITICAL: Check for mesh children before going to sleep
                // Don't sleep if we have other sensors connected through us
                int child_count = esp_mesh_get_routing_table_size() - 1; // Subtract parent connection
                bool is_leaf_node = (child_count <= 0) || (mesh_network_get_current_layer() >= 3);
                
                if (!is_leaf_node && child_count > 0) {
                    ESP_LOGW(MSG_TAG, "🌳 Cannot sleep yet - still have mesh children (layer %d, children: %d)", 
                             mesh_network_get_current_layer(), child_count);
                    continue; // Don't sleep, continue receiving messages
                }
                
                // Wait 10 seconds after receiving time sync to ensure all sensors receive it
                ESP_LOGI(MSG_TAG, "⏳ Waiting 10 seconds after time sync to ensure all sensors receive it...");
                vTaskDelay(10000 / portTICK_PERIOD_MS); // 10 second delay
                
                // Now start the mesh traffic monitoring cycle
                ESP_LOGI(MSG_TAG, "🔍 Starting mesh traffic monitoring cycle...");
                break; // Exit the receive loop to start coordination cycle
            } else {
                ESP_LOGW(MSG_TAG, "⚠️ Received message with time sync size but wrong message type: 0x%02X", 
                         time_msg->header.message_type);
            }
        }
        
        // Log sensor RX activity (simplified for sensors)
        if (recv_count <= 5 || (recv_count % 10) == 0) {
            ESP_LOGI(MSG_TAG,
                     "[#RX:%d][L:%d] from "MACSTR", size:%d, heap:%" PRId32,
                     recv_count, mesh_network_get_current_layer(), MAC2STR(from.addr),
                     data.size, esp_get_minimum_free_heap_size());
        }
    }
    
    // Mesh traffic coordination cycle - only reached if we received time sync and broke out of loop
    if (time_sync_received && stored_unified_wake_time > 0) {
        while (true) {
            bool mesh_traffic_detected = false;
            
            // Monitor for mesh traffic for 10 seconds
            for (int i = 0; i < 100; i++) { // 100 x 100ms = 10 seconds
                data.size = RX_SIZE;
                
                // Check for mesh traffic with short timeout
                esp_err_t err = esp_mesh_recv(&from, &data, pdMS_TO_TICKS(100), &flag, NULL, 0);
                
                if (err == ESP_OK && data.size > 0) {
                    mesh_traffic_detected = true;
                    break;
                }
            }
            
            if (!mesh_traffic_detected) {
                // No mesh traffic detected for 10 seconds - safe to sleep
                
                // Final check for children before sleeping
                int child_count = esp_mesh_get_routing_table_size() - 1;
                if (child_count > 0) {
                    ESP_LOGW(MSG_TAG, "⚠️ Still have %d children - continuing to monitor", child_count);
                    continue; // Restart coordination cycle
                }
                
                // Calculate and enter deep sleep
                enter_coordinated_deep_sleep();
                break; // Should never reach here as device sleeps
            }
            
            // Mesh traffic detected - restart the cycle
        }
    }
    
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        
        // Sensor: Enable RX and agricultural data TX for communication
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_sensor_mac_tx_main, "SENSOR_DATA_TX", 3072, NULL, 5, NULL);
        ESP_LOGI(MSG_TAG, "🌱 SENSOR: P2P RX enabled + Agricultural data TX enabled");
    }
    return ESP_OK;
}

/*******************************************************
 *                Public Interface Functions
 *******************************************************/
esp_err_t message_handler_init_sensor(void)
{
    ESP_LOGI(MSG_TAG, "🌱 Initializing agricultural sensor message handler");
    is_running = true;
    sequence_number = 0;
    return ESP_OK;
}

void message_handler_start_mac_tx_task(void)
{
    ESP_LOGI(MSG_TAG, "Starting agricultural data TX task");
    xTaskCreate(esp_mesh_sensor_mac_tx_main, "SENSOR_DATA_TX", 3072, NULL, 5, NULL);
}

void message_handler_start_p2p_tasks(void)
{
    ESP_LOGI(MSG_TAG, "Starting P2P RX and TX tasks");
    xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
    xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
}

esp_err_t message_handler_send_mac_data(void)
{
    // Legacy function - now sends full sensor data
    return message_handler_send_sensor_data();
}

esp_err_t message_handler_send_sensor_data(void)
{
    ESP_LOGI(MSG_TAG, "One-shot agricultural sensor data send requested");
    // Could implement immediate sensor reading and send here
    return ESP_OK;
}

void message_handler_stop(void)
{
    ESP_LOGI(MSG_TAG, "Stopping agricultural sensor message handler");
    is_running = false;
}

// Broadcast time sync to new sensors when they connect
void broadcast_time_sync_to_new_sensors(void) {
    time_sync_message_t msg = {0};
    time_t now = time(NULL);
    
    // Note: Sensors don't know the unified wake time, but can share their current time
    // The gateway is responsible for coordinating the unified wake time
    uint8_t sensor_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, sensor_mac);
    
    memcpy(msg.header.node_mac, sensor_mac, 6);
    msg.header.node_type = NODE_TYPE_SENSOR;
    msg.header.message_type = MSG_TYPE_TIME_SYNC;
    msg.header.timestamp = (uint32_t)now;
    msg.header.sequence_number = 0;
    msg.header.mesh_layer = mesh_network_get_current_layer();
    msg.header.signal_strength = 0;
    msg.current_unix_time = (uint32_t)now;
    msg.next_wake_time = (uint32_t)now + 3600; // Default 1 hour if no coordination
    msg.sleep_duration_sec = 3600; // Default 1 hour
    msg.sync_source = 1; // RTC (sensor's current time)
    msg.collection_window_sec = 10; // 10 sec window
    
    mesh_data_t data = {
        .data = (uint8_t*)&msg,
        .size = sizeof(msg)
    };
    
    esp_err_t err = esp_mesh_send(NULL, &data, MESH_DATA_P2P, NULL, 0);
    if (err == ESP_OK) {
        ESP_LOGI(MSG_TAG, "🕐 Broadcast time sync to new sensors (current time: %lld)", (long long)now);
    } else {
        ESP_LOGW(MSG_TAG, "Failed to broadcast time sync: %s", esp_err_to_name(err));
    }
}

// Function to calculate exact sleep duration right before sleeping
static void enter_coordinated_deep_sleep(void) {
    if (!time_sync_received || stored_unified_wake_time == 0) {
        ESP_LOGW(MSG_TAG, "⚠️ No time sync received, entering emergency sleep");
        esp_sleep_enable_timer_wakeup(300 * 1000000ULL); // 5 minutes emergency sleep
        esp_deep_sleep_start();
        return;
    }
    
    // Calculate sleep duration RIGHT BEFORE entering sleep for maximum accuracy
    time_t current_time = time(NULL);
    time_t sleep_duration;
    
    if (current_time >= stored_unified_wake_time) {
        sleep_duration = 10; // Minimum 10 seconds if wake time has passed
        ESP_LOGW(MSG_TAG, "⚠️ Wake time has passed! Current: %lld, Wake: %lld", 
                 (long long)current_time, (long long)stored_unified_wake_time);
    } else {
        // CRITICAL: All sensors wake at SAME time regardless of layer
        sleep_duration = stored_unified_wake_time - current_time;
    }
    
    // NO layer delays added - all sensors wake at unified wake time
    int layer = mesh_network_get_current_layer();
    
    ESP_LOGI(MSG_TAG, "⏰ Final coordinated sleep calculation (calculated right before sleep):");
    ESP_LOGI(MSG_TAG, "   Current time: %lld", (long long)current_time);
    ESP_LOGI(MSG_TAG, "   Unified wake time: %lld", (long long)stored_unified_wake_time);
    ESP_LOGI(MSG_TAG, "   My layer: %d (no layer delay applied)", layer);
    ESP_LOGI(MSG_TAG, "   Exact sleep duration: %lld seconds", (long long)sleep_duration);
    ESP_LOGI(MSG_TAG, "   Expected wake: %lld", (long long)stored_unified_wake_time);
    
    // Schedule wake-up for unified time
    uint64_t sleep_duration_us = (uint64_t)sleep_duration * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_duration_us);
    
    ESP_LOGI(MSG_TAG, "💤 Entering coordinated deep sleep for %lld seconds until unified wake time", (long long)sleep_duration);
    esp_deep_sleep_start();
} 