#ifndef TIME_SYNC_MANAGER_H
#define TIME_SYNC_MANAGER_H

#include "esp_err.h"
#include <time.h>
#include <stdbool.h>
#include "message_protocol.h"

// Configuration constants
#define TIME_SYNC_TIMEOUT_SEC          10    // Max time to wait for sync message
#define SYNC_MESSAGE_VALIDITY_SEC      120   // How long sync message is valid
#define MAX_CLOCK_DRIFT_SEC            5     // Max acceptable clock drift

typedef struct {
    time_t current_sync_time;         // Last synchronized time from gateway
    time_t next_wake_time;            // Next coordinated wake-up time
    time_t collection_start_time;     // When data collection window opens
    time_t collection_end_time;       // When data collection window closes
    uint16_t sleep_duration_sec;      // How long to sleep between cycles
    bool time_is_synchronized;        // Is time system ready
    uint8_t sync_source;              // Source of sync (0=hardcoded, 1=GPRS)
    
    // Sync tracking
    time_t last_sync_received;        // When last sync message was received
    bool waiting_for_sync;            // Currently waiting for time sync
    bool in_collection_window;        // Currently in data collection period
    uint32_t sync_messages_received;  // Total sync messages received
} sensor_time_sync_status_t;

// Time sync manager functions
esp_err_t sensor_time_sync_init(void);
esp_err_t handle_time_sync_message(const time_sync_message_t *sync_msg);
esp_err_t wait_for_time_sync(uint32_t timeout_sec);

// Time calculation helpers
time_t get_time_until_next_wake(void);
time_t get_current_synchronized_time(void);
bool is_collection_window_active(void);
bool should_send_sensor_data(void);
bool is_sync_message_valid(const time_sync_message_t *sync_msg);

// Sleep coordination
esp_err_t calculate_sleep_duration(uint32_t *sleep_sec);
esp_err_t prepare_for_coordinated_sleep(void);

// Status getters
sensor_time_sync_status_t* get_sensor_time_sync_status(void);
bool is_sensor_time_synchronized(void);

#endif // TIME_SYNC_MANAGER_H 