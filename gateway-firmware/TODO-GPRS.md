# GPRS Time Synchronization TODO

## Current Implementation Status

✅ **COMPLETED:**
- Time sync manager with hardcoded starting time (2025-01-01 00:00:00 UTC)
- Coordination logic for sensor data collection cycles
- Mesh broadcasting of synchronized time to all sensors
- Data collection window tracking
- Sensor reporting confirmation

⚠️ **TEMPORARY SOLUTION:**
- Using hardcoded time: `HARDCODED_START_TIME = 1735689600` (2025-01-01 00:00:00 UTC)
- Function `sync_time_from_hardcoded()` sets system time to fixed value

## GPRS Integration TODO

### Hardware Requirements
- [ ] GPRS/LTE module (e.g., SIM800L, SIM7600, or similar)
- [ ] SIM card with data plan
- [ ] UART connection to ESP32-S3

### Code Changes Required

#### 1. Replace `sync_time_from_hardcoded()` with `sync_time_from_gprs()`

**Current location:** `gateway-firmware/main/time_sync_manager.c:44`

```c
// TODO: Replace this function
esp_err_t sync_time_from_hardcoded(void)

// With this implementation:
esp_err_t sync_time_from_gprs(void)
{
    ESP_LOGI(TAG, "🔌 Enabling GPRS for time synchronization...");
    
    // 1. Power on GPRS module
    gprs_power_on();
    
    // 2. Initialize GPRS connection
    if (gprs_connect() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to GPRS");
        gprs_power_off();
        return ESP_FAIL;
    }
    
    // 3. Get time via NTP over GPRS
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    
    // 4. Wait for time sync (with timeout)
    int retry = 0;
    time_t now = 0;
    struct tm timeinfo = { 0 };
    
    while (timeinfo.tm_year < (2025 - 1900) && ++retry < 10) {
        ESP_LOGI(TAG, "Waiting for SNTP time sync... (%d/10)", retry);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    
    if (timeinfo.tm_year < (2025 - 1900)) {
        ESP_LOGE(TAG, "Failed to get time from GPRS");
        gprs_power_off();
        return ESP_FAIL;
    }
    
    // 5. Update sync status
    sync_status.last_sync_time = now;
    sync_status.time_is_synchronized = true;
    sync_status.sync_source = 1; // GPRS
    
    ESP_LOGI(TAG, "✅ Time synchronized via GPRS: %s", asctime(&timeinfo));
    
    // 6. Power off GPRS to save energy
    esp_sntp_stop();
    gprs_power_off();
    ESP_LOGI(TAG, "🔌 GPRS powered down to save energy");
    
    return ESP_OK;
}
```

#### 2. Add GPRS Driver Module

**Create:** `gateway-firmware/main/include/gprs_driver.h`
```c
#ifndef GPRS_DRIVER_H
#define GPRS_DRIVER_H

#include "esp_err.h"

esp_err_t gprs_init(void);
esp_err_t gprs_power_on(void);
esp_err_t gprs_power_off(void);
esp_err_t gprs_connect(void);
esp_err_t gprs_disconnect(void);
bool gprs_is_connected(void);

#endif // GPRS_DRIVER_H
```

#### 3. Update CMakeLists.txt

**Add to SRCS:** `"gprs_driver.c"`

#### 4. Configuration Updates

**Update constants in `time_sync_manager.h`:**
```c
#define TIME_SYNC_INTERVAL_SEC     3600  // Sync every hour via GPRS
#define GPRS_TIMEOUT_SEC           30    // GPRS operation timeout
```

### Testing Strategy

1. **Phase 1:** Test GPRS connectivity independently
2. **Phase 2:** Test NTP time sync over GPRS
3. **Phase 3:** Integrate with existing coordination logic
4. **Phase 4:** Test full coordinated sleep cycles with real time

### Power Optimization

- GPRS module should only be powered during time sync operations
- Typical power-on time: ~30 seconds for full sync
- Sync frequency: Every 1-3 hours depending on accuracy requirements

### Fallback Strategy

If GPRS sync fails:
1. Use internal RTC for short-term coordination
2. Retry GPRS sync at next interval
3. Broadcast last known good time with error flag 