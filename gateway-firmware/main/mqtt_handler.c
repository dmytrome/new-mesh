#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_handler.h"
#include "certs.h"

/*******************************************************
 *                Constants & Variables
 *******************************************************/
static const char *MQTT_TAG = "mqtt_handler";

// MQTT client handle
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Connection state
static bool mqtt_connected = false;
static bool mqtt_initialized = false;

// Reconnection parameters
static int reconnect_attempts = 0;

/*******************************************************
 *                Internal Functions
 *******************************************************/
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    (void)client; // Suppress unused variable warning
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "✅ MQTT Connected to AWS IoT Core");
            mqtt_connected = true;
            reconnect_attempts = 0;
            
            // Subscribe to topics if needed
            // int msg_id = esp_mqtt_client_subscribe(client, "gateway/1/control", 0);
            // ESP_LOGI(MQTT_TAG, "Subscribed to control topic, msg_id=%d", msg_id);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "❌ MQTT Disconnected from AWS IoT Core");
            mqtt_connected = false;
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "✅ MQTT Subscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "✅ MQTT Unsubscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(MQTT_TAG, "✅ MQTT Published, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(MQTT_TAG, "📥 MQTT Data received:");
            ESP_LOGI(MQTT_TAG, "  Topic: %.*s", event->topic_len, event->topic);
            ESP_LOGI(MQTT_TAG, "  Data: %.*s", event->data_len, event->data);
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(MQTT_TAG, "❌ MQTT Error");
            mqtt_connected = false;
            break;
            
        default:
            ESP_LOGI(MQTT_TAG, "ℹ️ MQTT Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static esp_err_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtts://" AWS_ENDPOINT ":8883",
        .credentials.client_id = MQTT_CLIENT_ID,
        .credentials.username = NULL,
        .credentials.authentication.password = NULL,
        .session.keepalive = MQTT_KEEPALIVE,
        .session.disable_clean_session = MQTT_DISABLE_CLEAN_SESSION,
        .session.last_will.topic = NULL,
        .session.last_will.msg = NULL,
        .session.last_will.qos = 1,
        .session.last_will.retain = 0,
        .network.disable_auto_reconnect = false,
        .network.timeout_ms = 10000,
        .task.stack_size = 8192,
        .task.priority = 5,
        .buffer.size = 1024,
        .buffer.out_size = 1024,
        .broker.verification.certificate = AWS_CA_CERT,
        .credentials.authentication.certificate = DEVICE_CERT,
        .credentials.authentication.key = DEVICE_KEY,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(MQTT_TAG, "❌ Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(MQTT_TAG, "❌ Failed to register MQTT event handler: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(MQTT_TAG, "❌ Failed to start MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(MQTT_TAG, "🚀 MQTT client started");
    return ESP_OK;
}

/*******************************************************
 *                Public Interface Functions
 *******************************************************/
esp_err_t mqtt_handler_init(void)
{
    if (mqtt_initialized) {
        ESP_LOGW(MQTT_TAG, "⚠️ MQTT handler already initialized");
        return ESP_OK;
    }

    ESP_LOGI(MQTT_TAG, "🔧 Initializing MQTT handler for AWS IoT Core");
    // No need to register an event handler with esp_event_handler_register for MQTT
    mqtt_initialized = true;
    ESP_LOGI(MQTT_TAG, "✅ MQTT handler initialized");
    return ESP_OK;
}

esp_err_t mqtt_handler_start(void)
{
    if (!mqtt_initialized) {
        ESP_LOGE(MQTT_TAG, "❌ MQTT handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (mqtt_client != NULL) {
        ESP_LOGW(MQTT_TAG, "⚠️ MQTT client already started");
        return ESP_OK;
    }

    ESP_LOGI(MQTT_TAG, "🚀 Starting MQTT client");
    return mqtt_app_start();
}

esp_err_t mqtt_handler_stop(void)
{
    if (mqtt_client == NULL) {
        ESP_LOGW(MQTT_TAG, "⚠️ MQTT client not running");
        return ESP_OK;
    }

    ESP_LOGI(MQTT_TAG, "🛑 Stopping MQTT client");
    esp_err_t err = esp_mqtt_client_stop(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(MQTT_TAG, "❌ Failed to stop MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_mqtt_client_destroy(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(MQTT_TAG, "❌ Failed to destroy MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    mqtt_client = NULL;
    mqtt_connected = false;
    ESP_LOGI(MQTT_TAG, "✅ MQTT client stopped");
    return ESP_OK;
}

esp_err_t mqtt_handler_publish_sensor_data(const char* sensor_mac, const char* data)
{
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(MQTT_TAG, "⚠️ MQTT not connected, cannot publish data");
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor_mac == NULL || data == NULL) {
        ESP_LOGE(MQTT_TAG, "❌ Invalid parameters for MQTT publish");
        return ESP_ERR_INVALID_ARG;
    }

    // Send the JSON data directly without wrapper
    ESP_LOGI(MQTT_TAG, "📤 Publishing sensor data to AWS IoT Core:");
    ESP_LOGI(MQTT_TAG, "  Topic: %s", TOPIC_METRICS);
    ESP_LOGI(MQTT_TAG, "  Payload: %s", data);

    int msg_id = esp_mqtt_client_publish(mqtt_client, TOPIC_METRICS, data, strlen(data), 1, 0);
    if (msg_id == -1) {
        ESP_LOGE(MQTT_TAG, "❌ Failed to publish MQTT message");
        return ESP_FAIL;
    }

    ESP_LOGI(MQTT_TAG, "✅ MQTT message published with ID: %d", msg_id);
    return ESP_OK;
}

bool mqtt_handler_is_connected(void)
{
    return mqtt_connected && mqtt_client != NULL;
}

/*******************************************************
 *                Event Handler (Public Interface)
 *******************************************************/
void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGD(MQTT_TAG, "MQTT Event dispatched from event loop base=%s, event_id=%ld", event_base, event_id);
    mqtt_event_handler_cb(event_data);
} 