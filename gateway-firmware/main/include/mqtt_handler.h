#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "esp_err.h"
#include "mqtt_client.h"
#include "esp_event.h"

/*******************************************************
 *                MQTT Configuration
 *******************************************************/
// AWS IoT Core Configuration
#define AWS_ENDPOINT "a1w68ba53cj3wf-ats.iot.eu-central-1.amazonaws.com"
#define AWS_PORT 8883
#define THING_NAME "ag_thing"
#define TOPIC_METRICS "gateway/1/metrics"

// MQTT Client Configuration
#define MQTT_CLIENT_ID "ag_thing"
#define MQTT_KEEPALIVE 60
#define MQTT_DISABLE_CLEAN_SESSION 0

/*******************************************************
 *                Interface Functions
 *******************************************************/
esp_err_t mqtt_handler_init(void);
esp_err_t mqtt_handler_start(void);
esp_err_t mqtt_handler_stop(void);
esp_err_t mqtt_handler_publish_sensor_data(const char* sensor_mac, const char* data);
bool mqtt_handler_is_connected(void);

/*******************************************************
 *                Event Handlers
 *******************************************************/
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

#endif // MQTT_HANDLER_H 