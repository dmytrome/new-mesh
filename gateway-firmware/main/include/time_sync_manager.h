#ifndef TIME_SYNC_MANAGER_H
#define TIME_SYNC_MANAGER_H

#include "esp_err.h"
#include <time.h>
#include <stdbool.h>
#include "message_protocol.h"

// Configuration constants
#define DATA_COLLECTION_INTERVAL_SEC   300   // 5 minutes between cycles
#define DATA_COLLECTION_WINDOW_SEC     60    // 60 seconds for all data collection
// TODO: Make sensor discovery dynamic instead of fixed limit
// TODO: Implement sensor auto-discovery protocol (announce on join, track disconnects)
// TODO: For large networks (60+ sensors), consider dynamic memory allocation
#define MAX_EXPECTED_SENSORS           100   // Current limit - increase as needed
#define SYNC_BROADCAST_INTERVAL_SEC    30    // Broadcast sync every 30 seconds

// TODO: Replace with GPRS time sync when hardware available
#define HARDCODED_START_TIME           1749600000  // 2025-06-10 00:00:00 UTC

typedef struct {
    time_t current_sync_time;         // Current synchronized time
    time_t next_wake_time;            // Next coordinated wake-up
    time_t cycle_start_time;          // When current cycle started
    uint16_t sleep_duration_sec;      // How long to sleep between cycles
    bool time_is_synchronized;        // Is time system ready
    uint8_t sync_source;              // 0=Hardcoded, 1=GPRS (TODO), 2=NTP
    
    // Data collection tracking
    uint8_t sensors_reported_count;   // How many sensors reported this cycle
    uint8_t expected_sensor_count;    // How many sensors we expect
    bool data_collection_complete;    // All sensors reported this cycle
    time_t collection_deadline;       // When to give up waiting for sensors
} time_sync_status_t;

// Time sync manager functions
esp_err_t time_sync_manager_init(void);
esp_err_t sync_time_from_hardcoded(void);  // TODO: Replace with sync_time_from_gprs()
esp_err_t broadcast_time_sync_to_sensors(void);
esp_err_t handle_sensor_data_received(const uint8_t *sensor_mac);
esp_err_t schedule_next_wake_cycle(void);

// Time calculation helpers
time_t get_next_collection_time(void);
time_t get_current_synchronized_time(void);
bool is_data_collection_window_active(void);
bool should_broadcast_sync(void);
void reset_data_collection_cycle(void);

// Status getters
time_sync_status_t* get_time_sync_status(void);
bool is_time_sync_ready(void);

#endif // TIME_SYNC_MANAGER_H 