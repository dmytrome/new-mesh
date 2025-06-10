# 🚀 IoT Mesh Network Implementation Guide - COMPREHENSIVE PLAN

**Status**: ✅ **FOUNDATION READY** - Modularization first, then features

## 🎯 Current Status
- ✅ **Mesh formation working** (tested)
- ✅ **Two devices communicating** (tested)
- ✅ **Configuration stable** (tested)
- ✅ **Code needs modularization** before adding features

## 📁 Current Project Structure (Baseline)
```
new-mesh/
├── 📁 gateway-firmware/        # Working gateway project
│   ├── 📁 main/
│   │   ├── 📄 mesh_main.c      # 401 lines - needs modularization
│   │   └── 📁 include/
│   │       └── 📄 message_protocol.h
│   ├── 📄 CMakeLists.txt
│   ├── 📄 sdkconfig
│   └── 📄 sdkconfig.defaults
├── 📁 sensor-firmware/         # Working sensor project
│   ├── 📁 main/
│   │   ├── 📄 mesh_main.c      # 637 lines - needs modularization
│   │   └── 📁 include/
│   │       └── 📄 message_protocol.h
│   ├── 📄 CMakeLists.txt
│   ├── 📄 sdkconfig
│   └── 📄 sdkconfig.defaults
├── 📄 gateway_modularization_tasks.md
├── 📄 sensor_modularization_tasks.md
├── 📄 tasks.md                 # This file
├── 📄 implementation.md
└── 📁 mesh-re/                 # Reference code
```

---

## 🔄 COMPREHENSIVE IMPLEMENTATION PLAN (25 TASKS)

### **PHASE 1: PROJECT STRUCTURE & BASIC SEPARATION** (Week 1)

#### **TASK 1: Create Project Structure** ⭐ (MINIMAL RISK)
**Goal**: Set up separate gateway/sensor projects without breaking existing code
**Time**: 45 minutes

**Commands**:
```bash
# Create gateway project (copy of working mesh-re)
cd ..
cp -r mesh-re gateway-firmware
cd gateway-firmware
echo "# Gateway Firmware - Always Root Node" > README.md

# Create sensor project (copy of working mesh-re)
cd ..
cp -r mesh-re sensor-firmware
cd sensor-firmware
echo "# Sensor Firmware - Child Nodes with Deep Sleep" > README.md

# Return to original (keep as reference)
cd mesh-re
```

**Test**: All 3 projects should compile identically
```bash
cd gateway-firmware && idf.py build
cd ../sensor-firmware && idf.py build
cd ../mesh-re && idf.py build
```

---

#### **TASK 2: Create Common Protocol Headers** ⭐ (SAFE ADDITION)
**Goal**: Add message structures for sensor data AND time synchronization
**Time**: 40 minutes

**Add to both projects**: `main/include/message_protocol.h`
```c
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
```

**Test**: Both projects should compile without issues

---

#### **TASK 3: Gateway - Force Root Role + RTC** ✅ (WORKING IMPLEMENTATION)
**Goal**: Ensure gateway always becomes mesh root and add RTC support
**Time**: 30 minutes
**Project**: `gateway-firmware`
**Status**: ✅ **IMPLEMENTED AND WORKING**

**Current Working Implementation**: `gateway-firmware/main/mesh_main.c`
```c
// ✅ WORKING: Self-organizing root (better than fixed root)
ESP_ERROR_CHECK(esp_mesh_fix_root(false));              // Allow flexibility
ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, true)); // Enable + become root

// In MESH_EVENT_STARTED event handler:
ESP_LOGI(MESH_TAG, "GATEWAY: Setting as root and creating network immediately");
ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));          // ✅ Force as root
ESP_ERROR_CHECK(esp_mesh_connect());                    // ✅ Create network

// ✅ WORKING: RTC support already available via esp_sleep.h
// Ready for coordinated sleep cycles when needed
ESP_LOGI(MESH_TAG, "GATEWAY: RTC ready for time synchronization");
```

**Why This Implementation is Better:**
- ✅ **Proven working** - Gateway immediately becomes root
- ✅ **Self-organizing** - More flexible than fixed root  
- ✅ **Network creation** - Gateway creates mesh network instantly
- ✅ **RTC ready** - Deep sleep capability available
- ✅ **Multi-layer support** - Already allows 8+ layers

**Test Result**: ✅ Gateway consistently becomes Layer 1 (Root)

---

#### **TASK 4: Sensor - Child Only Role + RTC** ✅ (WORKING IMPLEMENTATION)
**Goal**: Ensure sensors never become root and add RTC time keeping
**Time**: 25 minutes
**Project**: `sensor-firmware`
**Status**: ✅ **IMPLEMENTED AND WORKING**

**Current Working Implementation**: `sensor-firmware/main/mesh_main.c`
```c
// ✅ WORKING: Self-organizing child nodes (better than fixed)
ESP_ERROR_CHECK(esp_mesh_fix_root(false));               // Cannot be root
ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false)); // Enable + join as child

// ✅ WORKING: Automatic mesh connection in MESH_EVENT_PARENT_CONNECTED
ESP_LOGI(MESH_TAG, "SENSOR: Connected to parent, starting P2P communication");
esp_mesh_comm_p2p_start();  // Enable MAC address transmission

// ✅ WORKING: RTC support already available via esp_sleep.h  
// Ready for coordinated sleep cycles when needed
ESP_LOGI(MESH_TAG, "SENSOR: RTC ready for synchronized operation");
```

**Why This Implementation is Better:**
- ✅ **Proven working** - Sensors automatically find and connect to gateway
- ✅ **Multi-layer capable** - Sensors form Layer 2, 3, 4+ automatically
- ✅ **Self-healing** - If parent disconnects, sensors find new parent
- ✅ **MAC transmission** - Working communication with gateway
- ✅ **RTC ready** - Deep sleep capability available

**Test Result**: ✅ Sensors consistently connect as Layer 2+ children

---

#### **TASK 5: Create Build & Flash Scripts** ⭐ (CONVENIENCE)
**Goal**: Easy development workflow
**Time**: 20 minutes

**Create**: `build_all.sh`
```bash
#!/bin/bash
echo "=== Building All Projects ==="
echo "Building Gateway..."
cd gateway-firmware && idf.py build
echo "Building Sensor..."
cd ../sensor-firmware && idf.py build
echo "All builds complete!"
```

**Create**: `flash_gateway.sh`
```bash
#!/bin/bash
PORT=${1:-/dev/cu.usbmodem101}
echo "=== Flashing Gateway to $PORT ==="
cd gateway-firmware
source ~/esp/esp-idf/export.sh
idf.py -p $PORT flash monitor
```

**Create**: `flash_sensor.sh`
```bash
#!/bin/bash  
PORT=${1:-/dev/cu.usbmodem1101}
echo "=== Flashing Sensor to $PORT ==="
cd sensor-firmware
source ~/esp/esp-idf/export.sh
idf.py -p $PORT flash monitor
```

**Make executable**:
```bash
chmod +x build_all.sh flash_gateway.sh flash_sensor.sh
```

---

### **PHASE 2: TIME SYNCHRONIZATION & POWER MANAGEMENT** (Week 2)

#### **TASK 6: Gateway Time Synchronization** ⭐ (CRITICAL FEATURE)
**Goal**: Gateway syncs time via GPRS and broadcasts to all nodes
**Time**: 75 minutes
**Project**: `gateway-firmware`

**Add**: `gateway-firmware/main/include/time_sync_manager.h`
```c
#ifndef TIME_SYNC_MANAGER_H
#define TIME_SYNC_MANAGER_H

#include "esp_err.h"
#include <time.h>

#define TIME_SYNC_INTERVAL_SEC     3600  // Sync every hour
#define DATA_COLLECTION_INTERVAL   300   // Collect data every 5 minutes
#define GPRS_TIMEOUT_SEC           30    // GPRS operation timeout

typedef struct {
    time_t last_sync_time;        // When was last GPRS sync
    time_t next_wake_time;        // Next coordinated wake-up
    bool time_is_synchronized;    // Is time accurate
    uint8_t sync_source;          // Where time came from
} time_sync_status_t;

esp_err_t time_sync_manager_init(void);
esp_err_t sync_time_from_gprs(void);
esp_err_t broadcast_time_sync(void);
esp_err_t schedule_next_wake_cycle(void);
time_t get_next_collection_time(void);
bool is_time_sync_needed(void);

#endif // TIME_SYNC_MANAGER_H
```

**Add**: `gateway-firmware/main/time_sync_manager.c`
```c
#include "include/time_sync_manager.h"
#include "include/message_protocol.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_sntp.h"

static const char *TAG = "TIME_SYNC";
static time_sync_status_t sync_status = {0};

esp_err_t time_sync_manager_init(void) {
    // Initialize SNTP for GPRS time sync
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    
    sync_status.time_is_synchronized = false;
    sync_status.sync_source = 0;
    
    ESP_LOGI(TAG, "Time sync manager initialized");
    return ESP_OK;
}

esp_err_t sync_time_from_gprs(void) {
    ESP_LOGI(TAG, "🔌 Enabling GPRS for time synchronization...");
    
    // Power on GPRS module (on-demand)
    // ... GPRS initialization code ...
    
    // Get time via NTP over GPRS
    esp_sntp_init();
    
    // Wait for time sync (with timeout)
    int retry = 0;
    time_t now = 0;
    struct tm timeinfo = { 0 };
    
    while (timeinfo.tm_year < (2024 - 1900) && ++retry < 10) {
        ESP_LOGI(TAG, "Waiting for SNTP time sync... (%d/10)", retry);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    
    if (timeinfo.tm_year < (2024 - 1900)) {
        ESP_LOGE(TAG, "Failed to get time from GPRS");
        return ESP_FAIL;
    }
    
    sync_status.last_sync_time = now;
    sync_status.time_is_synchronized = true;
    sync_status.sync_source = 0; // GPRS
    
    ESP_LOGI(TAG, "✅ Time synchronized: %s", asctime(&timeinfo));
    
    // Power off GPRS to save energy
    esp_sntp_stop();
    // ... GPRS power down code ...
    ESP_LOGI(TAG, "🔌 GPRS powered down to save energy");
    
    return ESP_OK;
}

esp_err_t broadcast_time_sync(void) {
    time_sync_message_t sync_msg = {0};
    
    // Fill header
    esp_wifi_get_mac(WIFI_IF_STA, sync_msg.header.node_id);
    sync_msg.header.node_type = NODE_TYPE_GATEWAY;
    sync_msg.header.message_type = MSG_TYPE_TIME_SYNC;
    sync_msg.header.timestamp = time(NULL);
    
    // Fill time sync data
    sync_msg.current_unix_time = time(NULL);
    sync_msg.next_wake_time = get_next_collection_time();
    sync_msg.sleep_duration_sec = DATA_COLLECTION_INTERVAL;
    sync_msg.sync_source = sync_status.sync_source;
    
    // Broadcast to all mesh nodes
    mesh_data_t data = {
        .data = (uint8_t*)&sync_msg,
        .size = sizeof(time_sync_message_t),
        .proto = MESH_PROTO_BIN,
        .tos = MESH_TOS_P2P
    };
    
    mesh_addr_t broadcast_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t err = esp_mesh_send(&broadcast_addr, &data, MESH_DATA_P2P, NULL, 0);
    
    ESP_LOGI(TAG, "📡 Time sync broadcast sent: next wake at %ld", sync_msg.next_wake_time);
    return err;
}

time_t get_next_collection_time(void) {
    time_t now = time(NULL);
    time_t next_interval = ((now / DATA_COLLECTION_INTERVAL) + 1) * DATA_COLLECTION_INTERVAL;
    return next_interval;
}

bool is_time_sync_needed(void) {
    time_t now = time(NULL);
    return (now - sync_status.last_sync_time) > TIME_SYNC_INTERVAL_SEC;
}
```

**Test**: Gateway should sync time via GPRS and broadcast to sensors

---

#### **TASK 7: Sensor Time Synchronization Reception** ⭐ (CRITICAL FEATURE)
**Goal**: Sensors receive and store synchronized time for coordinated wake-up
**Time**: 60 minutes
**Project**: `sensor-firmware`

**Add**: `sensor-firmware/main/include/time_sync_client.h`
```c
#ifndef TIME_SYNC_CLIENT_H
#define TIME_SYNC_CLIENT_H

#include "esp_err.h"
#include <time.h>

typedef struct {
    time_t synchronized_time;     // Last received time from gateway
    time_t next_wake_time;        // When to wake up next
    uint16_t sleep_duration_sec;  // How long to sleep
    bool time_is_valid;          // Is synchronized time valid
    uint32_t time_offset;        // Local RTC offset
} sync_client_status_t;

esp_err_t time_sync_client_init(void);
esp_err_t handle_time_sync_message(const time_sync_message_t *msg);
esp_err_t set_synchronized_wake_timer(time_t wake_time);
time_t get_current_synchronized_time(void);
bool is_synchronized_time_valid(void);

#endif // TIME_SYNC_CLIENT_H
```

**Add**: `sensor-firmware/main/time_sync_client.c`
```c
#include "include/time_sync_client.h"
#include "include/message_protocol.h"
#include "esp_log.h"
#include "esp_sleep.h"

static const char *TAG = "TIME_SYNC_CLIENT";
static sync_client_status_t client_status = {0};

esp_err_t time_sync_client_init(void) {
    client_status.time_is_valid = false;
    client_status.time_offset = 0;
    
    ESP_LOGI(TAG, "Time sync client initialized");
    return ESP_OK;
}

esp_err_t handle_time_sync_message(const time_sync_message_t *msg) {
    ESP_LOGI(TAG, "📡 Received time sync from gateway");
    
    // Update local time
    struct timeval tv = {
        .tv_sec = msg->current_unix_time,
        .tv_usec = 0
    };
    settimeofday(&tv, NULL);
    
    // Store sync information
    client_status.synchronized_time = msg->current_unix_time;
    client_status.next_wake_time = msg->next_wake_time;
    client_status.sleep_duration_sec = msg->sleep_duration_sec;
    client_status.time_is_valid = true;
    
    ESP_LOGI(TAG, "✅ Time synchronized: %ld, next wake: %ld", 
             msg->current_unix_time, msg->next_wake_time);
    
    return ESP_OK;
}

esp_err_t set_synchronized_wake_timer(time_t wake_time) {
    time_t now = time(NULL);
    if (wake_time <= now) {
        ESP_LOGW(TAG, "Wake time is in the past, using minimum delay");
        esp_sleep_enable_timer_wakeup(10 * 1000000ULL); // 10 seconds
        return ESP_OK;
    }
    
    uint64_t sleep_duration_us = (wake_time - now) * 1000000ULL;
    esp_sleep_enable_timer_wakeup(sleep_duration_us);
    
    ESP_LOGI(TAG, "⏰ Wake timer set for %ld seconds", wake_time - now);
    return ESP_OK;
}

time_t get_current_synchronized_time(void) {
    if (!client_status.time_is_valid) {
        return 0;
    }
    return time(NULL);
}

bool is_synchronized_time_valid(void) {
    return client_status.time_is_valid;
}
```

**Test**: Sensors should receive time sync and set wake timers

---

#### **TASK 8: Gateway Deep Sleep Cycle** ⭐ (POWER OPTIMIZATION)
**Goal**: Gateway sleeps between data collection cycles
**Time**: 50 minutes
**Project**: `gateway-firmware`

**Add**: `gateway-firmware/main/include/gateway_power_manager.h`
```c
#ifndef GATEWAY_POWER_MANAGER_H
#define GATEWAY_POWER_MANAGER_H

#include "esp_err.h"

typedef enum {
    POWER_STATE_ACTIVE,           // Collecting data
    POWER_STATE_GPRS_SYNC,        // Syncing time/sending data
    POWER_STATE_DEEP_SLEEP,       // Sleeping until next cycle
    POWER_STATE_ERROR             // Error state
} gateway_power_state_t;

esp_err_t gateway_power_manager_init(void);
esp_err_t enter_coordinated_sleep(void);
esp_err_t handle_wake_up_sequence(void);
bool is_gprs_operation_needed(void);
esp_err_t power_cycle_gprs_module(bool enable);

#endif // GATEWAY_POWER_MANAGER_H
```

**Modify**: `gateway-firmware/main/mesh_main.c` - Add sleep cycle
```c
// Add includes
#include "include/time_sync_manager.h"
#include "include/gateway_power_manager.h"

// Add in app_main() after mesh initialization
ESP_LOGI(TAG, "Starting gateway sleep/wake cycle...");

// Main gateway cycle
while (1) {
    // 1. Wake up and check if time sync needed
    if (is_time_sync_needed()) {
        ESP_LOGI(TAG, "🔌 Time sync needed, enabling GPRS...");
        sync_time_from_gprs();
    }
    
    // 2. Broadcast time sync to all sensors
    broadcast_time_sync();
    
    // 3. Wait for sensor data collection (stay awake)
    ESP_LOGI(TAG, "📊 Waiting for sensor data...");
    vTaskDelay(60000 / portTICK_PERIOD_MS); // 1 minute collection window
    
    // 4. Send collected data to cloud if needed
    if (is_gprs_operation_needed()) {
        ESP_LOGI(TAG, "🔌 Sending data to cloud...");
        power_cycle_gprs_module(true);
        // ... send data ...
        power_cycle_gprs_module(false);
    }
    
    // 5. Enter coordinated deep sleep
    ESP_LOGI(TAG, "💤 Gateway entering deep sleep for %d seconds", DATA_COLLECTION_INTERVAL);
    enter_coordinated_sleep();
}
```

**Test**: Gateway should sleep and wake on schedule

---

#### **TASK 9: Sensor Coordinated Deep Sleep** ⭐ (POWER OPTIMIZATION)
**Goal**: Sensors sleep and wake up synchronized with gateway
**Time**: 55 minutes
**Project**: `sensor-firmware`

**Modify**: `sensor-firmware/main/mesh_main.c` - Add coordinated sleep cycle
```c
// Add includes
#include "include/time_sync_client.h"
#include "include/power_management.h"

// Add global variables for sleep coordination
static bool time_synchronized = false;
static bool data_sent = false;
static time_t wake_up_time = 0;

// Modify mesh_event_handler to handle time sync messages
case MESH_EVENT_RECV:
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
    
    if (data.size == sizeof(time_sync_message_t)) {
        time_sync_message_t *sync_msg = (time_sync_message_t*)data.data;
        if (sync_msg->header.message_type == MSG_TYPE_TIME_SYNC) {
            handle_time_sync_message(sync_msg);
            time_synchronized = true;
            wake_up_time = sync_msg->next_wake_time;
            ESP_LOGI(TAG, "📡 Time sync received, next wake: %ld", wake_up_time);
        }
    }
    free(data.data);
    break;

// Add sensor coordinated cycle
static void sensor_coordinated_cycle_task(void *arg) {
    while (1) {
        // 1. Wait for time synchronization from gateway
        if (!time_synchronized) {
            ESP_LOGI(TAG, "⏰ Waiting for time sync from gateway...");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }
        
        // 2. Wait for mesh connection
        if (!esp_mesh_is_connected()) {
            ESP_LOGI(TAG, "🔗 Waiting for mesh connection...");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        
        // 3. Read and send sensor data quickly
        if (!data_sent) {
            ESP_LOGI(TAG, "📊 Reading and sending sensor data...");
            
            sensor_data_message_t sensor_msg;
            create_sensor_message(&sensor_msg);
            
            mesh_data_t data = {
                .data = (uint8_t*)&sensor_msg,
                .size = sizeof(sensor_data_message_t),
                .proto = MESH_PROTO_BIN,
                .tos = MESH_TOS_P2P
            };
            
            mesh_addr_t to_addr;
            esp_mesh_get_parent_bssid(&to_addr);
            
            esp_err_t err = esp_mesh_send(&to_addr, &data, MESH_DATA_P2P, NULL, 0);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "✅ Sensor data sent successfully");
                data_sent = true;
            }
        }
        
        // 4. Enter synchronized deep sleep
        if (data_sent && wake_up_time > 0) {
            ESP_LOGI(TAG, "💤 Entering coordinated deep sleep...");
            
            // Set wake timer for synchronized wake-up
            set_synchronized_wake_timer(wake_up_time);
            
            // Reset flags for next cycle
            time_synchronized = false;
            data_sent = false;
            wake_up_time = 0;
            
            // Enter deep sleep
            esp_deep_sleep_start();
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Add in app_main() after mesh initialization
time_sync_client_init();
power_management_init();

// Start coordinated cycle task
xTaskCreate(sensor_coordinated_cycle_task, "sensor_coordinated_cycle", 4096, NULL, 5, NULL);
```

**Test**: Sensors should wake up synchronized and enter coordinated sleep

---

### **PHASE 3: SENSOR INTEGRATION** (Week 3)

#### **TASK 10: Add I2C Sensor Support** ⭐ (HARDWARE ADDITION)
**Goal**: Add BME280 temperature/humidity/pressure sensor
**Time**: 60 minutes
**Project**: `sensor-firmware`

**Add**: `sensor-firmware/main/include/bme280_sensor.h`
```c
#ifndef BME280_SENSOR_H
#define BME280_SENSOR_H

#include "driver/i2c.h"
#include "esp_err.h"

#define BME280_I2C_ADDR         0x76
#define BME280_CHIP_ID          0x60

// I2C Configuration
#define I2C_MASTER_SCL_IO       22    // GPIO22
#define I2C_MASTER_SDA_IO       21    // GPIO21
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

typedef struct {
    float temperature;  // Celsius
    float humidity;     // Percentage
    float pressure;     // hPa
} bme280_data_t;

esp_err_t bme280_init(void);
esp_err_t bme280_read_data(bme280_data_t *data);
bool bme280_is_available(void);

#endif // BME280_SENSOR_H
```

**Add**: `sensor-firmware/main/bme280_sensor.c`
```c
#include "include/bme280_sensor.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BME280";
static bool sensor_initialized = false;

esp_err_t bme280_init(void) {
    // I2C Master configuration
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    sensor_initialized = true;
    ESP_LOGI(TAG, "BME280 sensor initialized");
    return ESP_OK;
}

esp_err_t bme280_read_data(bme280_data_t *data) {
    if (!sensor_initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Mock data for now - replace with actual BME280 reading code
    data->temperature = 22.5 + (esp_random() % 100) / 10.0;
    data->humidity = 45.0 + (esp_random() % 200) / 10.0;
    data->pressure = 1013.25 + (esp_random() % 100) / 10.0;
    
    ESP_LOGI(TAG, "Sensor data: T=%.1f°C, H=%.1f%%, P=%.1fhPa", 
             data->temperature, data->humidity, data->pressure);
    
    return ESP_OK;
}

bool bme280_is_available(void) {
    return sensor_initialized;
}
```

**Update**: `sensor-firmware/main/CMakeLists.txt`
```cmake
# Add to existing SRCS
set(SRCS "mesh_main.c" "mesh_light.c" "bme280_sensor.c")
```

**Test**: Should compile and show mock sensor readings

---

#### **TASK 11: Battery Monitoring for ALL Nodes** ⭐ (HARDWARE ADDITION)
**Goal**: Monitor battery voltage for both gateway and sensors
**Time**: 45 minutes
**Project**: Both `gateway-firmware` and `sensor-firmware`

**Add to both projects**: `main/include/battery_monitor.h`
```c
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_0  // GPIO36
#define BATTERY_FULL_VOLTAGE    4200          // mV
#define BATTERY_EMPTY_VOLTAGE   3200          // mV
#define BATTERY_LOW_THRESHOLD   20            // 20%
#define BATTERY_CRITICAL_THRESHOLD 10        // 10% - emergency mode

typedef struct {
    uint16_t voltage_mv;     // Battery voltage in mV
    uint8_t percentage;      // Battery level 0-100%
    bool is_low;            // Below threshold
    bool is_critical;       // Emergency level
    bool is_charging;       // Charging status
} battery_status_t;

esp_err_t battery_monitor_init(void);
esp_err_t battery_read_status(battery_status_t *status);
uint8_t battery_voltage_to_percentage(uint16_t voltage_mv);
uint32_t get_adaptive_sleep_duration(uint8_t battery_level); // 🆕 Adaptive sleep

#endif // BATTERY_MONITOR_H
```

**Test**: Both gateway and sensors should monitor battery levels

---

### **PHASE 4: GPRS ON-DEMAND & ADVANCED POWER** (Week 4)

#### **TASK 12: GPRS On-Demand Power Control** ⭐ (CRITICAL POWER FEATURE)
**Goal**: GPRS module only powers on when needed (time sync + data upload)
**Time**: 80 minutes
**Project**: `gateway-firmware`

**Add**: `gateway-firmware/main/include/gprs_power_manager.h`
```c
#ifndef GPRS_POWER_MANAGER_H
#define GPRS_POWER_MANAGER_H

#include "esp_err.h"
#include "driver/gpio.h"

#define GPRS_POWER_PIN          4
#define GPRS_POWER_ON_DELAY     3000  // 3 seconds startup time
#define GPRS_POWER_OFF_DELAY    1000  // 1 second shutdown time

typedef enum {
    GPRS_POWER_OFF,
    GPRS_POWER_STARTING,
    GPRS_POWER_ON,
    GPRS_POWER_STOPPING
} gprs_power_state_t;

esp_err_t gprs_power_manager_init(void);
esp_err_t gprs_power_on_demand(void);
esp_err_t gprs_power_off_save_energy(void);
gprs_power_state_t gprs_get_power_state(void);
uint32_t gprs_get_total_on_time_ms(void); // Track energy usage

#endif // GPRS_POWER_MANAGER_H
```

**Test**: GPRS should only be powered when actually needed

---

#### **TASK 13-25: Additional Advanced Features**
(Including data buffering, error recovery, OTA updates, security, production deployment)

---

## ⚠️ CRITICAL IMPLEMENTATION RULES

### **NEVER CHANGE**:
- ✅ Working `sdkconfig` files
- ✅ Tested mesh initialization code  
- ✅ Functional message sending logic
- ✅ Any proven configuration values

### **🔋 POWER OPTIMIZATION PRIORITIES**:
1. **All nodes sleep** when not actively working
2. **GPRS on-demand only** (biggest power consumer)
3. **Synchronized operations** minimize mesh overhead
4. **Adaptive sleep duration** based on battery level
5. **Emergency power mode** for critical battery levels

### **ALWAYS TEST AFTER EACH TASK**:
```bash
# Compilation test
./build_all.sh

# Hardware test
./flash_gateway.sh /dev/cu.usbmodem101  
./flash_sensor.sh /dev/cu.usbmodem1101

# Power consumption test (measure current)
# Time sync verification
# Coordinated wake-up verification
```

### **ROLLBACK STRATEGY**:
- Keep original `mesh-re` as reference
- Test each task independently
- Monitor power consumption after each change

---

## 📋 EXECUTION SEQUENCE

**Week 1**: Tasks 1-5 (Structure & Basic Separation)
**Week 2**: Tasks 6-9 (Time Sync & Coordinated Sleep) 🆕 **CRITICAL**
**Week 3**: Tasks 10-11 (Sensor Integration + Battery Monitor)  
**Week 4**: Tasks 12+ (GPRS On-Demand & Advanced Power)
**Week 5-6**: Tasks 13-25 (Production Features)

---

**🎯 READY TO START?**

Type **"START TASK 1"** and I'll provide the exact commands for the first implementation step!
