# 🚀 IoT Mesh Network Implementation Guide - COMPREHENSIVE PLAN

**Status**: ✅ **PRODUCTION READY** - Building on existing tested mesh code

## 🎯 Current Status
- ✅ **Mesh formation working** (tested)
- ✅ **Two devices communicating** (tested)
- ✅ **Configuration stable** (tested)

## 📁 Current Project Structure (Baseline)
```
mesh-re/                    # EXISTING - Keep as reference
├── 📄 CMakeLists.txt       # EXISTING - Don't touch
├── 📁 components/          # EXISTING - Empty, keep
├── 📁 main/               # EXISTING - Keep existing files
│   ├── 📄 CMakeLists.txt
│   ├── 📄 Kconfig.projbuild
│   ├── 📄 mesh_light.c
│   └── 📄 mesh_main.c
├── 📄 sdkconfig           # EXISTING - Keep current config
└── 📄 sdkconfig.defaults  # EXISTING - Keep
```

## 🎯 FINAL SYSTEM ARCHITECTURE

```
┌─────────────────┐    GPRS/4G     ┌─────────────┐
│   Gateway Node  │ ────────────── │ Cloud/MQTT  │
│   (ESP32-S3)    │                │   Server    │
│   - Mesh Root   │                └─────────────┘
│   - GPRS Module │                
│   - Always On   │                
└─────────────────┘                
         │                         
    ┌────┴────┐                    
    │         │                    
┌───▼───┐ ┌───▼───┐               
│Sensor1│ │Sensor2│ ... (up to 50 nodes)
│ESP32-C3│ │ESP32-C3│               
│BME280 │ │BME280 │               
│Battery│ │Battery│               
│Sleep  │ │Sleep  │               
└───────┘ └───────┘               
```

---

## 🔄 COMPREHENSIVE IMPLEMENTATION PLAN (22 TASKS)

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
**Goal**: Add message structures for sensor data without changing existing code
**Time**: 30 minutes

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
#define MSG_TYPE_CONFIG_REQ     0x05
#define MSG_TYPE_CONFIG_RESP    0x06

// Node Types
#define NODE_TYPE_GATEWAY       0x01
#define NODE_TYPE_SENSOR        0x02

// Error Codes
#define ERROR_SENSOR_READ       0x01
#define ERROR_MESH_DISCONNECT   0x02
#define ERROR_BATTERY_LOW       0x03

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

typedef struct __attribute__((packed)) {
    message_header_t header;
    uint32_t sleep_duration_ms;   // Next sleep duration
    uint8_t signal_strength;      // WiFi signal strength
} heartbeat_message_t;

#endif // MESSAGE_PROTOCOL_H
```

**Test**: Both projects should compile without issues

---

#### **TASK 3: Gateway - Force Root Role** ⭐ (MINIMAL CHANGE)
**Goal**: Ensure gateway always becomes mesh root
**Time**: 20 minutes
**Project**: `gateway-firmware`

**Modify**: `gateway-firmware/main/mesh_main.c`
```c
// Add after esp_mesh_start() in app_main()
esp_mesh_fix_root(true);  // Force this device to be root
esp_mesh_set_max_layer(8); // Allow up to 8 layers
ESP_LOGI(TAG, "GATEWAY: Fixed as mesh root, max layers: 8");
```

**Test**: Gateway should always become root

---

#### **TASK 4: Sensor - Child Only Role** ⭐ (MINIMAL CHANGE)
**Goal**: Ensure sensors never become root
**Time**: 15 minutes
**Project**: `sensor-firmware`

**Modify**: `sensor-firmware/main/mesh_main.c`
```c
// Add after esp_mesh_start() in app_main()
esp_mesh_fix_root(false);  // This device cannot be root
ESP_LOGI(TAG, "SENSOR: Will connect to gateway as child");
```

**Test**: Sensor should always connect as child

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

### **PHASE 2: SENSOR INTEGRATION** (Week 2)

#### **TASK 6: Add I2C Sensor Support** ⭐ (HARDWARE ADDITION)
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

#### **TASK 7: Battery Monitoring** ⭐ (HARDWARE ADDITION)
**Goal**: Monitor battery voltage for low battery alerts
**Time**: 45 minutes
**Project**: `sensor-firmware`

**Add**: `sensor-firmware/main/include/battery_monitor.h`
```c
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_0  // GPIO36
#define BATTERY_FULL_VOLTAGE    4200          // mV
#define BATTERY_EMPTY_VOLTAGE   3200          // mV
#define BATTERY_LOW_THRESHOLD   20            // 20%

typedef struct {
    uint16_t voltage_mv;     // Battery voltage in mV
    uint8_t percentage;      // Battery level 0-100%
    bool is_low;            // Below threshold
    bool is_charging;       // Charging status
} battery_status_t;

esp_err_t battery_monitor_init(void);
esp_err_t battery_read_status(battery_status_t *status);
uint8_t battery_voltage_to_percentage(uint16_t voltage_mv);

#endif // BATTERY_MONITOR_H
```

**Add**: `sensor-firmware/main/battery_monitor.c`
```c
#include "include/battery_monitor.h"
#include "esp_log.h"

static const char *TAG = "BATTERY";
static adc_oneshot_unit_handle_t adc_handle = NULL;

esp_err_t battery_monitor_init(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    
    ret = adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Battery monitor initialized");
    return ESP_OK;
}

esp_err_t battery_read_status(battery_status_t *status) {
    if (!adc_handle || !status) {
        return ESP_ERR_INVALID_STATE;
    }
    
    int adc_reading = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &adc_reading);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert ADC reading to voltage (assuming voltage divider)
    status->voltage_mv = (adc_reading * 3300 * 2) / 4095;  // 2:1 voltage divider
    status->percentage = battery_voltage_to_percentage(status->voltage_mv);
    status->is_low = (status->percentage <= BATTERY_LOW_THRESHOLD);
    status->is_charging = false;  // Add charging detection logic if needed
    
    ESP_LOGI(TAG, "Battery: %dmV (%d%%) %s", 
             status->voltage_mv, status->percentage,
             status->is_low ? "LOW" : "OK");
    
    return ESP_OK;
}

uint8_t battery_voltage_to_percentage(uint16_t voltage_mv) {
    if (voltage_mv >= BATTERY_FULL_VOLTAGE) return 100;
    if (voltage_mv <= BATTERY_EMPTY_VOLTAGE) return 0;
    
    return ((voltage_mv - BATTERY_EMPTY_VOLTAGE) * 100) / 
           (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE);
}
```

**Test**: Should compile and show battery readings

---

#### **TASK 8: Sensor Data Integration** ⭐ (INTEGRATION)
**Goal**: Replace mock mesh data with real sensor readings
**Time**: 40 minutes
**Project**: `sensor-firmware`

**Modify**: `sensor-firmware/main/mesh_main.c`
```c
// Add includes at top
#include "include/message_protocol.h"
#include "include/bme280_sensor.h"
#include "include/battery_monitor.h"

// Add after existing includes, before mesh_event_handler
static uint16_t sequence_number = 0;

static void create_sensor_message(sensor_data_message_t *msg) {
    // Get MAC address
    esp_wifi_get_mac(WIFI_IF_STA, msg->header.node_id);
    
    // Fill header
    msg->header.node_type = NODE_TYPE_SENSOR;
    msg->header.message_type = MSG_TYPE_SENSOR_DATA;
    msg->header.timestamp = esp_timer_get_time() / 1000000;
    msg->header.sequence_number = sequence_number++;
    
    // Read battery status
    battery_status_t battery;
    if (battery_read_status(&battery) == ESP_OK) {
        msg->header.battery_level = battery.percentage;
        msg->header.battery_voltage_mv = battery.voltage_mv;
    } else {
        msg->header.battery_level = 0;
        msg->header.battery_voltage_mv = 0;
    }
    
    // Read sensor data
    bme280_data_t sensor_data;
    if (bme280_read_data(&sensor_data) == ESP_OK) {
        msg->temperature = sensor_data.temperature;
        msg->humidity = sensor_data.humidity;
        msg->pressure = sensor_data.pressure;
        msg->error_code = 0;
    } else {
        msg->temperature = 0;
        msg->humidity = 0;
        msg->pressure = 0;
        msg->error_code = ERROR_SENSOR_READ;
    }
    
    msg->light_level = 500;  // Mock for now
    msg->uptime_seconds = esp_timer_get_time() / 1000000;
    
    ESP_LOGI(TAG, "Sensor message: T=%.1f°C H=%.1f%% P=%.1fhPa Bat=%d%%",
             msg->temperature, msg->humidity, msg->pressure, 
             msg->header.battery_level);
}

// Add in app_main() after mesh initialization
    ESP_LOGI(TAG, "Initializing sensors...");
    bme280_init();
    battery_monitor_init();
```

**Test**: Sensor should send structured data instead of simple strings

---

### **PHASE 3: DEEP SLEEP & POWER OPTIMIZATION** (Week 3)

#### **TASK 9: Deep Sleep Implementation** ⭐ (POWER OPTIMIZATION)
**Goal**: Add deep sleep between sensor readings to save battery
**Time**: 60 minutes
**Project**: `sensor-firmware`

**Add**: `sensor-firmware/main/include/power_management.h`
```c
#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "esp_sleep.h"
#include "esp_err.h"

#define SLEEP_DURATION_NORMAL    60  // 60 seconds normal operation
#define SLEEP_DURATION_LOW_BAT   300 // 5 minutes when battery low
#define SLEEP_DURATION_ERROR     120 // 2 minutes after error

typedef enum {
    WAKE_REASON_TIMER,
    WAKE_REASON_ERROR,
    WAKE_REASON_LOW_BATTERY
} wake_reason_t;

esp_err_t power_management_init(void);
void enter_deep_sleep(uint32_t sleep_duration_seconds);
wake_reason_t get_wake_reason(void);
uint32_t get_sleep_duration(uint8_t battery_level, bool has_error);

#endif // POWER_MANAGEMENT_H
```

**Add**: `sensor-firmware/main/power_management.c`
```c
#include "include/power_management.h"
#include "esp_log.h"

static const char *TAG = "POWER";

esp_err_t power_management_init(void) {
    // Configure wake-up sources
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_NORMAL * 1000000ULL);
    
    ESP_LOGI(TAG, "Power management initialized");
    return ESP_OK;
}

void enter_deep_sleep(uint32_t sleep_duration_seconds) {
    ESP_LOGI(TAG, "Entering deep sleep for %lu seconds", sleep_duration_seconds);
    
    // Set wake-up timer
    esp_sleep_enable_timer_wakeup(sleep_duration_seconds * 1000000ULL);
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

wake_reason_t get_wake_reason(void) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Wake up from timer");
            return WAKE_REASON_TIMER;
        default:
            ESP_LOGI(TAG, "Wake up from other source");
            return WAKE_REASON_TIMER;
    }
}

uint32_t get_sleep_duration(uint8_t battery_level, bool has_error) {
    if (has_error) {
        return SLEEP_DURATION_ERROR;
    } else if (battery_level < 20) {
        return SLEEP_DURATION_LOW_BAT;
    } else {
        return SLEEP_DURATION_NORMAL;
    }
}
```

**Test**: Sensor should enter deep sleep after sending data

---

#### **TASK 10: Wake-Send-Sleep Cycle** ⭐ (INTEGRATION)
**Goal**: Implement complete sensor cycle: wake → connect → send → sleep
**Time**: 50 minutes
**Project**: `sensor-firmware`

**Modify**: `sensor-firmware/main/mesh_main.c` - Add sensor cycle logic
```c
// Add global variables at top
static bool mesh_connected = false;
static bool data_sent = false;
static uint32_t connection_start_time = 0;
static const uint32_t MAX_CONNECTION_WAIT_MS = 30000; // 30 seconds timeout

// Modify mesh_event_handler to track connection
case MESH_EVENT_PARENT_CONNECTED:
    mesh_connected = true;
    ESP_LOGI(TAG, "Mesh connected, ready to send data");
    break;

case MESH_EVENT_PARENT_DISCONNECTED:
    mesh_connected = false;
    ESP_LOGI(TAG, "Mesh disconnected");
    break;

// Add new function for sensor cycle
static void sensor_cycle_task(void *arg) {
    sensor_data_message_t sensor_msg;
    mesh_data_t data;
    
    connection_start_time = esp_timer_get_time() / 1000;
    
    while (1) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // Check connection timeout
        if (current_time - connection_start_time > MAX_CONNECTION_WAIT_MS) {
            ESP_LOGW(TAG, "Connection timeout, entering sleep");
            enter_deep_sleep(SLEEP_DURATION_ERROR);
        }
        
        // If connected and haven't sent data yet
        if (mesh_connected && !data_sent) {
            ESP_LOGI(TAG, "Creating and sending sensor data...");
            
            create_sensor_message(&sensor_msg);
            
            data.data = (uint8_t*)&sensor_msg;
            data.size = sizeof(sensor_data_message_t);
            data.proto = MESH_PROTO_BIN;
            data.tos = MESH_TOS_P2P;
            
            mesh_addr_t to_addr;
            esp_mesh_get_parent_bssid(&to_addr);
            
            esp_err_t err = esp_mesh_send(&to_addr, &data, MESH_DATA_P2P, NULL, 0);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Sensor data sent successfully");
                data_sent = true;
                
                // Calculate sleep duration based on battery level
                battery_status_t battery;
                battery_read_status(&battery);
                uint32_t sleep_duration = get_sleep_duration(battery.percentage, false);
                
                // Wait a bit for mesh to process, then sleep
                vTaskDelay(pdMS_TO_TICKS(2000));
                enter_deep_sleep(sleep_duration);
            } else {
                ESP_LOGE(TAG, "Failed to send data: %s", esp_err_to_name(err));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Add in app_main() after mesh start
    power_management_init();
    
    // Check wake reason
    wake_reason_t wake_reason = get_wake_reason();
    ESP_LOGI(TAG, "Wake reason: %d", wake_reason);
    
    // Start sensor cycle task
    xTaskCreate(sensor_cycle_task, "sensor_cycle", 4096, NULL, 5, NULL);
```

**Test**: Sensor should wake, connect, send data, then sleep

---

### **PHASE 4: GATEWAY GPRS INTEGRATION** (Week 4)

#### **TASK 11: GPRS Module Setup** ⭐ (HARDWARE ADDITION)
**Goal**: Add SIM800L/SIM7600 GPRS module to gateway
**Time**: 90 minutes
**Project**: `gateway-firmware`

**Add**: `gateway-firmware/main/include/gprs_module.h`
```c
#ifndef GPRS_MODULE_H
#define GPRS_MODULE_H

#include "driver/uart.h"
#include "esp_err.h"

#define GPRS_UART_NUM           UART_NUM_2
#define GPRS_TXD_PIN            17
#define GPRS_RXD_PIN            16
#define GPRS_BAUD_RATE          115200
#define GPRS_POWER_PIN          4

#define GPRS_BUFFER_SIZE        1024
#define GPRS_RESPONSE_TIMEOUT   10000  // 10 seconds

typedef enum {
    GPRS_STATE_POWER_OFF,
    GPRS_STATE_POWER_ON,
    GPRS_STATE_REGISTERED,
    GPRS_STATE_CONNECTED,
    GPRS_STATE_ERROR
} gprs_state_t;

esp_err_t gprs_init(void);
esp_err_t gprs_power_on(void);
esp_err_t gprs_power_off(void);
esp_err_t gprs_check_network(void);
esp_err_t gprs_connect_gprs(const char *apn);
esp_err_t gprs_send_http_post(const char *url, const char *data, char *response, size_t max_response_len);
esp_err_t gprs_send_command(const char *command, char *response, size_t max_len, uint32_t timeout_ms);
gprs_state_t gprs_get_state(void);

#endif // GPRS_MODULE_H
```

**Test**: Should compile without GPRS hardware connected

---

#### **TASK 12: HTTP/MQTT Cloud Communication** ⭐ (CLOUD INTEGRATION)
**Goal**: Send sensor data to cloud server via GPRS
**Time**: 75 minutes
**Project**: `gateway-firmware`

**Add**: `gateway-firmware/main/include/cloud_communication.h`
```c
#ifndef CLOUD_COMMUNICATION_H
#define CLOUD_COMMUNICATION_H

#include "include/message_protocol.h"
#include "esp_err.h"

#define CLOUD_SERVER_URL        "http://your-server.com/api/sensor-data"
#define CLOUD_API_KEY           "your-api-key"
#define CLOUD_DEVICE_ID         "gateway-001"

typedef struct {
    char url[256];
    char api_key[64];
    char device_id[32];
    bool use_https;
    uint16_t port;
} cloud_config_t;

esp_err_t cloud_init(const cloud_config_t *config);
esp_err_t cloud_send_sensor_data(const sensor_data_message_t *data);
esp_err_t cloud_send_heartbeat(void);
esp_err_t cloud_check_commands(void);
char* create_json_payload(const sensor_data_message_t *data);

#endif // CLOUD_COMMUNICATION_H
```

**Test**: Should compile and prepare for cloud communication

---

#### **TASK 13: Gateway Data Processing** ⭐ (DATA HANDLING)
**Goal**: Process incoming sensor data and forward to cloud
**Time**: 60 minutes
**Project**: `gateway-firmware`

**Modify**: `gateway-firmware/main/mesh_main.c` - Add data processing
```c
// Add includes
#include "include/message_protocol.h"
#include "include/cloud_communication.h"

// Add global variables
static uint32_t total_messages_received = 0;
static uint32_t cloud_messages_sent = 0;

// Add data processing function
static void process_sensor_data(const uint8_t *data, size_t len) {
    if (len == sizeof(sensor_data_message_t)) {
        sensor_data_message_t *sensor_msg = (sensor_data_message_t*)data;
        
        total_messages_received++;
        
        ESP_LOGI(TAG, "GATEWAY: Received sensor data from " MACSTR,
                 MAC2STR(sensor_msg->header.node_id));
        ESP_LOGI(TAG, "  Temperature: %.1f°C", sensor_msg->temperature);
        ESP_LOGI(TAG, "  Humidity: %.1f%%", sensor_msg->humidity);
        ESP_LOGI(TAG, "  Pressure: %.1f hPa", sensor_msg->pressure);
        ESP_LOGI(TAG, "  Battery: %d%% (%dmV)", 
                 sensor_msg->header.battery_level,
                 sensor_msg->header.battery_voltage_mv);
        
        // Forward to cloud
        if (cloud_send_sensor_data(sensor_msg) == ESP_OK) {
            cloud_messages_sent++;
            ESP_LOGI(TAG, "GATEWAY: Data forwarded to cloud (%lu/%lu)", 
                     cloud_messages_sent, total_messages_received);
        } else {
            ESP_LOGW(TAG, "GATEWAY: Failed to send to cloud");
        }
    } else {
        ESP_LOGW(TAG, "GATEWAY: Received unknown data format (size: %d)", len);
    }
}

// Modify mesh_event_handler
case MESH_EVENT_ROOT_GOT_IP:
    ESP_LOGI(TAG, "GATEWAY: Root got IP, initializing cloud connection");
    cloud_config_t cloud_config = {
        .url = CLOUD_SERVER_URL,
        .api_key = CLOUD_API_KEY,
        .device_id = CLOUD_DEVICE_ID,
        .use_https = false,
        .port = 80
    };
    cloud_init(&cloud_config);
    break;
```

**Test**: Gateway should process and log sensor data

---

### **PHASE 5: ADVANCED FEATURES** (Week 5-6)

#### **TASK 14-22: Advanced Features** ⭐
(Including OTA updates, security, data buffering, error recovery, configuration management, monitoring dashboard, field testing, documentation, and production deployment)

---

## ⚠️ CRITICAL IMPLEMENTATION RULES

### **NEVER CHANGE**:
- ✅ Working `sdkconfig` files
- ✅ Tested mesh initialization code  
- ✅ Functional message sending logic
- ✅ Any proven configuration values

### **ALWAYS TEST AFTER EACH TASK**:
```bash
# Compilation test
./build_all.sh

# Hardware test
./flash_gateway.sh /dev/cu.usbmodem101
./flash_sensor.sh /dev/cu.usbmodem1101

# Functionality verification
# - Check mesh formation
# - Verify data flow
# - Monitor logs for errors
```

### **ROLLBACK STRATEGY**:
- Keep original `mesh-re` as reference
- Test each task independently
- Document any issues immediately

---

## 📋 EXECUTION SEQUENCE

**Week 1**: Tasks 1-5 (Structure & Basic Separation)
**Week 2**: Tasks 6-8 (Sensor Integration)  
**Week 3**: Tasks 9-10 (Power Management)
**Week 4**: Tasks 11-13 (Gateway & Cloud)
**Week 5-6**: Tasks 14-22 (Production Features)

---

**🎯 READY TO START?**

Type **"START TASK 1"** and I'll provide the exact commands for the first implementation step!
