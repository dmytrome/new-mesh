#ifndef MESSAGE_PROTOCOL_H
#define MESSAGE_PROTOCOL_H

#include <stdint.h>
#include <time.h>

/*******************************************************
 *                Message Types
 *******************************************************/
#define MSG_TYPE_SENSOR_DATA    0x01  // Full sensor reading
#define MSG_TYPE_HEARTBEAT      0x02  // Node health check
#define MSG_TYPE_BATTERY_LOW    0x03  // Low battery alert
#define MSG_TYPE_ERROR          0x04  // Error reporting
#define MSG_TYPE_TIME_SYNC      0x05  // Time synchronization
#define MSG_TYPE_WAKE_SCHEDULE  0x06  // Wake-up scheduling
#define MSG_TYPE_SLEEP_COORD    0x07  // Sleep coordination

/*******************************************************
 *                Node Types
 *******************************************************/
#define NODE_TYPE_GATEWAY       0x01
#define NODE_TYPE_SENSOR        0x02

/*******************************************************
 *                Error Codes
 *******************************************************/
#define ERROR_SENSOR_READ       0x01
#define ERROR_MESH_DISCONNECT   0x02
#define ERROR_BATTERY_LOW       0x03
#define ERROR_TIME_SYNC         0x04
#define ERROR_SOIL_SENSOR       0x05
#define ERROR_AIR_SENSOR        0x06

/*******************************************************
 *                Core Sensor Data Structure
 *                (Maps directly to JSON "data" object)
 *******************************************************/
typedef struct __attribute__((packed)) {
    // Light sensor
    uint16_t lux;                    // Light level in lux
    
    // Air measurements  
    float temp_air;                  // Air temperature (°C), NAN = null
    float hum_air;                   // Air humidity (%), NAN = null
    float pressure_air;              // Air pressure (hPa, UK sea level), NAN = null
    
    // Ground temperature
    float temp_ground;               // Ground temperature (°C), NAN = null
    
    // Soil measurements
    float soil_temp;                 // Soil temperature (°C), NAN = null
    float soil_hum;                  // Soil humidity (%), NAN = null
    uint16_t soil_ec;                // Soil electrical conductivity (µS/cm)
    float soil_ph;                   // Soil pH level, NAN = null
    uint16_t soil_n;                 // Soil nitrogen (mg/kg)
    uint16_t soil_p;                 // Soil phosphorus (mg/kg) 
    uint16_t soil_k;                 // Soil potassium (mg/kg)
    uint16_t soil_salinity;          // Soil salinity (mg/L)
    uint16_t soil_tds_npk;          // Total dissolved solids NPK (ppm)
    
    // Battery status
    float bat_lvl;                   // Battery level (V), NAN = null
    uint16_t bat_vol;                // Battery voltage (mV)
    
    // Internal metadata (not in JSON)
    uint32_t reading_timestamp;      // When this reading was taken
    uint8_t sensor_status;           // Bitmask of sensor errors
    uint8_t reading_quality;         // Data quality indicator (0-100)
} sensor_data_t;

/*******************************************************
 *                Message Header
 *******************************************************/
typedef struct __attribute__((packed)) {
    uint8_t node_mac[6];             // MAC address (used as sensor_id)
    uint8_t node_type;               // NODE_TYPE_*
    uint8_t message_type;            // MSG_TYPE_*
    uint32_t timestamp;              // Unix timestamp
    uint16_t sequence_number;        // For duplicate detection
    uint8_t mesh_layer;              // Current mesh layer
    uint8_t signal_strength;         // WiFi signal strength (dBm + 100)
} message_header_t;

/*******************************************************
 *                Complete Sensor Message
 *******************************************************/
typedef struct __attribute__((packed)) {
    message_header_t header;
    sensor_data_t data;              // Complete sensor reading
    uint16_t checksum;               // Data integrity check
} sensor_message_t;

/*******************************************************
 *                Gateway Collection Structures
 *                (For MQTT JSON generation)
 *******************************************************/
#define MAX_SENSOR_NODES 50

// Single node data for MQTT JSON
typedef struct {
    char sensor_id[16];              // Format: "1:0", "2:0", etc.
    sensor_data_t data;              // Sensor readings
    uint32_t last_seen;              // When last data received
    bool data_valid;                 // Is this data fresh and valid
} node_data_t;

// Complete MQTT payload structure
typedef struct {
    uint32_t timestamp;              // Collection timestamp
    uint8_t node_count;              // Number of nodes with data
    node_data_t nodes[MAX_SENSOR_NODES]; // Array of sensor data
} mqtt_payload_t;

/*******************************************************
 *                Time Synchronization Messages
 *******************************************************/
typedef struct __attribute__((packed)) {
    message_header_t header;
    uint32_t current_unix_time;      // Current accurate time from GPRS
    uint32_t next_wake_time;         // When all nodes should wake up
    uint16_t sleep_duration_sec;     // How long to sleep
    uint8_t sync_source;             // 0=GPRS, 1=RTC, 2=NTP
    uint8_t collection_window_sec;   // Data collection window duration
} time_sync_message_t;

/*******************************************************
 *                Sleep Coordination Messages
 *******************************************************/
typedef struct __attribute__((packed)) {
    message_header_t header;
    uint32_t sleep_start_time;       // When to start sleeping (coordinated)
    uint32_t wake_up_time;           // When to wake up (synchronized)
    uint16_t collection_window;      // How long data collection window
    uint8_t mesh_layer_order;        // Order of wake-up by layer
} sleep_coord_message_t;

/*******************************************************
 *                Heartbeat Messages
 *******************************************************/
typedef struct __attribute__((packed)) {
    message_header_t header;
    uint32_t uptime_seconds;         // Device uptime
    uint16_t free_heap;              // Available memory
    uint8_t error_count;             // Number of recent errors
    uint8_t last_error_code;         // Most recent error
} heartbeat_message_t;

/*******************************************************
 *                Utility Functions
 *******************************************************/

// Convert MAC address to sensor_id string (e.g., "1:0")
static inline void mac_to_sensor_id(const uint8_t mac[6], char* sensor_id) {
    // Use last two bytes of MAC for simple ID
    snprintf(sensor_id, 16, "%d:%d", mac[4], mac[5]);
}

// Convert sensor_message_t to node_data_t for JSON
static inline void sensor_msg_to_node_data(const sensor_message_t* msg, node_data_t* node) {
    mac_to_sensor_id(msg->header.node_mac, node->sensor_id);
    node->data = msg->data;
    node->last_seen = msg->header.timestamp;
    node->data_valid = true;
}

// Calculate message checksum
static inline uint16_t calculate_checksum(const void* data, size_t len) {
    const uint8_t* bytes = (const uint8_t*)data;
    uint16_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += bytes[i];
    }
    return checksum;
}

#endif // MESSAGE_PROTOCOL_H 