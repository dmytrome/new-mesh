#ifndef MESSAGE_PROTOCOL_H
#define MESSAGE_PROTOCOL_H

#include <stdint.h>
#include <time.h>

// Message Types
#define MSG_TYPE_SENSOR_DATA    0x01
#define MSG_TYPE_HEARTBEAT      0x02
#define MSG_TYPE_BATTERY_LOW    0x03
#define MSG_TYPE_ERROR          0x04
#define MSG_TYPE_TIME_SYNC      0x05  // 🆕 Time synchronization
#define MSG_TYPE_WAKE_SCHEDULE  0x06  // 🆕 Wake-up scheduling
#define MSG_TYPE_SLEEP_COORD    0x07  // 🆕 Sleep coordination

// Node Types
#define NODE_TYPE_GATEWAY       0x01
#define NODE_TYPE_SENSOR        0x02

// Error Codes
#define ERROR_SENSOR_READ       0x01
#define ERROR_MESH_DISCONNECT   0x02
#define ERROR_BATTERY_LOW       0x03
#define ERROR_TIME_SYNC         0x04  // 🆕

typedef struct __attribute__((packed)) {
    uint8_t node_id[6];           // MAC address
    uint8_t node_type;            // NODE_TYPE_*
    uint8_t message_type;         // MSG_TYPE_*
    uint32_t timestamp;           // Unix timestamp  
    uint16_t sequence_number;     // For duplicate detection
    uint8_t battery_level;        // 0-100%
    uint16_t battery_voltage_mv;  // Battery voltage in mV
} message_header_t;

typedef struct __attribute__((packed)) {
    message_header_t header;
    float temperature;            // Celsius
    float humidity;              // Percentage (0-100)
    float pressure;              // hPa
    uint16_t light_level;        // Lux
    uint8_t error_code;          // Error status
    uint32_t uptime_seconds;     // Device uptime
} sensor_data_message_t;

// 🆕 Time synchronization message
typedef struct __attribute__((packed)) {
    message_header_t header;
    uint32_t current_unix_time;   // Current accurate time from GPRS
    uint32_t next_wake_time;      // When all nodes should wake up
    uint16_t sleep_duration_sec;  // How long to sleep
    uint8_t sync_source;          // 0=GPRS, 1=RTC, 2=NTP
} time_sync_message_t;

// 🆕 Sleep coordination message
typedef struct __attribute__((packed)) {
    message_header_t header;
    uint32_t sleep_start_time;    // When to start sleeping (coordinated)
    uint32_t wake_up_time;        // When to wake up (synchronized)
    uint16_t collection_window;   // How long data collection window
    uint8_t mesh_layer_order;     // Order of wake-up by layer
} sleep_coord_message_t;

typedef struct __attribute__((packed)) {
    message_header_t header;
    uint32_t sleep_duration_ms;   // Next sleep duration
    uint8_t signal_strength;      // WiFi signal strength
} heartbeat_message_t;

#endif // MESSAGE_PROTOCOL_H 