# 🏗️ Sensor Firmware Modularization Tasks

**Goal**: Extract existing code from `sensor-firmware/main/mesh_main.c` into clean, focused modules.

**Approach**: 
- ✅ **Extract existing code** into appropriate modules
- ✅ **Refactor main file** to use extracted modules
- ❌ **Do NOT create empty stub files** (save for future implementation tasks)
- ✅ **Update tasks.md** with future module implementation needs

---

## 📋 **SENSOR MODULARIZATION TASKS**

### **TASK S1: Extract Core Mesh Network Module**
**File**: Create `main/mesh_network.c` + `main/include/mesh_network.h`
**Purpose**: Handle mesh networking as sensor child node

**Code to Extract From `mesh_main.c`**:
- `mesh_event_handler()` function (lines 406-590)
- `ip_event_handler()` function (lines 590-600)
- Mesh initialization functions: `init_wifi_and_netif()` (lines 75-90), `init_mesh_base()` (lines 92-105), `configure_and_start_mesh()` (lines 107-145), `init_full_mesh_system()` (lines 147-180)
- Self-organizing child setup logic
- P2P communication setup: `esp_mesh_comm_p2p_start()` function (lines 389-405)
- Connection management and status tracking variables: `is_mesh_connected`, `mesh_layer`, `mesh_parent_addr`, `netif_sta`
- Mesh constants and macros: `MESH_ID`, `RX_SIZE`, `TX_SIZE`

**Interface Design**:
```c
// mesh_network.h
#include "esp_mesh.h"
#include "esp_event.h"
#include "esp_netif.h"

esp_err_t mesh_network_init_sensor(void);
esp_err_t mesh_network_start_p2p_communication(void);
bool mesh_network_is_connected_to_parent(void);
int mesh_network_get_current_layer(void);
esp_err_t mesh_network_init_full_system(bool is_fast_init);
void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
```

**Files Modified**:
- Create: `main/mesh_network.c`, `main/include/mesh_network.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)
- Update: `main/CMakeLists.txt` (change SRCS to include new files):
  ```cmake
  idf_component_register(SRCS "mesh_main.c" "mesh_network.c" "message_handler.c" "sensor_coordinator.c" "sensor_sleep.c" "sensor_data.c"
                      INCLUDE_DIRS "." "include")
  ```

---

### **TASK S2: Extract Message Protocol Handler**
**File**: Create `main/message_handler.c` + `main/include/message_handler.h`
**Purpose**: Handle message transmission and protocol operations

**Code to Extract From `mesh_main.c`**:
- `esp_mesh_sensor_mac_tx_main()` function (lines 285-336)
- `esp_mesh_p2p_tx_main()` function (lines 337-356)
- `esp_mesh_p2p_rx_main()` function (lines 357-387)
- `sensor_mac_msg_t` structure definition (lines 60-67) and message type constants
- MAC transmission logic and timing
- RX/TX buffer management: `rx_buf[RX_SIZE]` variable (line 44)
- `is_running` variable (line 45)

**Interface Design**:
```c
// message_handler.h
#include "esp_mesh.h"
#include <stdint.h>

// Message structure (extract from mesh_main.c)
typedef struct {
    uint8_t msg_type;           
    uint8_t mac_addr[6];        
    uint8_t layer;              
    uint32_t timestamp;         
} sensor_mac_msg_t;

#define MSG_TYPE_SENSOR_MAC 0x01

esp_err_t message_handler_init_sensor(void);
void message_handler_start_mac_tx_task(void);
void message_handler_start_p2p_tasks(void);
esp_err_t message_handler_send_mac_data(void);
void message_handler_stop(void);
```

**Files Modified**:
- Create: `main/message_handler.c`, `main/include/message_handler.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)

---

### **TASK S3: Extract Sensor Coordination Manager**
**File**: Create `main/sensor_coordinator.c` + `main/include/sensor_coordinator.h`
**Purpose**: Coordinate sensor operation cycles

**Code to Extract From `mesh_main.c`**:
- `coordinated_sensor_cycle()` function (lines 230-284)
- `check_wake_timeout()` function (lines 182-190)
- Coordination state management: `coordinated_state_t` enum (lines 27-35), `current_state` variable (line 51)
- Coordination variables: `time_synchronized`, `data_collection_done`, `wake_start_time` (lines 52-54)
- Coordination constants: `DATA_COLLECTION_WINDOW_SEC`, `MESH_CONNECTION_TIMEOUT_SEC`, `SENSOR_READ_TIMEOUT_SEC`, `DATA_SEND_TIMEOUT_SEC`, `EMERGENCY_SLEEP_SEC` (lines 16-20)

**Interface Design**:
```c
// sensor_coordinator.h
#include "esp_timer.h"
#include "esp_err.h"

typedef enum {
    COORD_STATE_WAKE_UP,
    COORD_STATE_WAIT_TIME_SYNC,
    COORD_STATE_CONNECT_MESH,
    COORD_STATE_COLLECT_DATA,
    COORD_STATE_SEND_DATA,
    COORD_STATE_PREPARE_SLEEP,
    COORD_STATE_DEEP_SLEEP
} coordinated_state_t;

// Constants
#define DATA_COLLECTION_WINDOW_SEC    60
#define MESH_CONNECTION_TIMEOUT_SEC   10
#define SENSOR_READ_TIMEOUT_SEC       5
#define DATA_SEND_TIMEOUT_SEC         10
#define EMERGENCY_SLEEP_SEC           300

esp_err_t sensor_coordinator_init(void);
esp_err_t sensor_coordinator_start_cycle(void);
esp_err_t sensor_coordinator_check_timeouts(void);
coordinated_state_t sensor_coordinator_get_current_state(void);
```

**Files Modified**:
- Create: `main/sensor_coordinator.c`, `main/include/sensor_coordinator.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)

---

### **TASK S4: Extract Basic Sleep Management**
**File**: Create `main/sensor_sleep.c` + `main/include/sensor_sleep.h`
**Purpose**: Handle basic sensor sleep operations

**Code to Extract From `mesh_main.c`**:
- `enter_coordinated_deep_sleep()` function (lines 219-228)
- Sleep duration constants and timing logic
- Wake-up cause handling (already using `esp_sleep_get_wakeup_cause()`)
- Basic sleep preparation and deep sleep entry

**Interface Design**:
```c
// sensor_sleep.h
#include "esp_sleep.h"
#include "esp_err.h"

esp_err_t sensor_sleep_init(void);
esp_sleep_wakeup_cause_t sensor_sleep_get_wakeup_cause(void);
esp_err_t sensor_sleep_enter_deep_sleep(uint32_t duration_sec);
void sensor_sleep_enter_coordinated_deep_sleep(uint32_t sleep_duration_sec);
```

**Files Modified**:
- Create: `main/sensor_sleep.c`, `main/include/sensor_sleep.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)

---

### **TASK S5: Extract Sensor Data Management**
**File**: Create `main/sensor_data.c` + `main/include/sensor_data.h`
**Purpose**: Handle sensor data reading and transmission

**Code to Extract From `mesh_main.c`**:
- `read_and_send_sensor_data_quick()` function (lines 204-217)
- `wait_for_time_sync_signal()` function (lines 192-202)
- Data collection and transmission timing logic
- Sensor data handling (basic framework)

**Interface Design**:
```c
// sensor_data.h
#include "esp_err.h"
#include <stdint.h>

esp_err_t sensor_data_init(void);
esp_err_t sensor_data_read_and_send_quick(void);
esp_err_t sensor_data_wait_for_time_sync(uint32_t timeout_ms);
```

**Files Modified**:
- Create: `main/sensor_data.c`, `main/include/sensor_data.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)

---

### **TASK S6: Refactor Sensor Main File**
**File**: Update `sensor-firmware/main/mesh_main.c`
**Purpose**: Simplify main file to use extracted modules

**Refactoring Steps**:
1. Remove all code that was extracted to modules
2. Add include statements for new modules
3. Replace existing initialization with module calls
4. Simplify `app_main()` to basic initialization + module startup

**Target Sensor Main Structure**:
```c
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "mesh_network.h"
#include "message_handler.h"
#include "sensor_coordinator.h"
#include "sensor_sleep.h"
#include "sensor_data.h"

static const char *MESH_TAG = "mesh_main";

void app_main(void) {
    ESP_LOGI(MESH_TAG, "=== 🌱 Agricultural Sensor Starting (Modular) ===");
    
    // Check wake-up reason (existing code)
    esp_sleep_wakeup_cause_t wakeup_cause = sensor_sleep_get_wakeup_cause();
    ESP_LOGI(MESH_TAG, "Wake-up cause: %d", wakeup_cause);
    
    // Initialize NVS (existing code)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize extracted modules
    ESP_ERROR_CHECK(mesh_network_init_sensor());
    ESP_ERROR_CHECK(message_handler_init_sensor());
    ESP_ERROR_CHECK(sensor_coordinator_init());
    ESP_ERROR_CHECK(sensor_sleep_init());
    ESP_ERROR_CHECK(sensor_data_init());
    
    ESP_LOGI(MESH_TAG, "=== ✅ Sensor Modules Initialized ===");
    
    // Start sensor coordination cycle (existing logic)
    ESP_LOGI(MESH_TAG, "=== 🔋 Sensor Starting Coordination Cycle ===");
    sensor_coordinator_start_cycle();
    
    // Should never reach here if coordination cycle works correctly
    ESP_LOGW(MESH_TAG, "Coordination cycle ended unexpectedly!");
}
```

**Files Modified**:
- Update: `main/mesh_main.c` (major refactoring)

---

## 🎯 **IMPLEMENTATION ORDER**

**All tasks can be completed in 1 day:**

1. **TASK S1**: Extract Core Mesh Network Module (40 minutes)
2. **TASK S2**: Extract Message Protocol Handler (30 minutes)  
3. **TASK S3**: Extract Sensor Coordination Manager (30 minutes)
4. **TASK S4**: Extract Basic Sleep Management (20 minutes)
5. **TASK S5**: Extract Sensor Data Management (20 minutes)
6. **TASK S6**: Refactor Sensor Main File (30 minutes)

**Total Time**: ~3 hours for complete sensor modularization

---

## 📁 **BEFORE vs AFTER STRUCTURE**

**Current**:
```
sensor-firmware/main/
├── mesh_main.c                  # 637 lines - ALL CODE HERE
├── CMakeLists.txt               # Only includes mesh_main.c
└── include/
    └── message_protocol.h       # Shared protocol
```

**After Modularization**:
```
sensor-firmware/main/
├── mesh_main.c                  # ~50 lines - initialization only
├── mesh_network.c               # ~250 lines - extracted mesh code
├── message_handler.c            # ~120 lines - extracted message code
├── sensor_coordinator.c         # ~150 lines - extracted coordination code
├── sensor_sleep.c               # ~50 lines - extracted sleep code
├── sensor_data.c                # ~80 lines - extracted data code
├── CMakeLists.txt               # Updated to include all source files
└── include/
    ├── mesh_network.h           # Mesh networking interface
    ├── message_handler.h        # Message handling interface
    ├── sensor_coordinator.h     # Coordination interface
    ├── sensor_sleep.h           # Sleep management interface
    ├── sensor_data.h            # Data management interface
    └── message_protocol.h       # Shared protocol (unchanged)
```

---

## ⚠️ **IMPORTANT NOTES**

### **Dependencies to Handle**:
- **ESP-IDF includes**: Each module needs proper ESP-IDF headers (`esp_timer.h`, `esp_sleep.h`, etc.)
- **Shared variables**: Variables like `is_mesh_connected`, `mesh_layer`, coordination state need to be accessed via functions
- **Task handles**: Ensure proper task creation and cleanup in message handler
- **Event handlers**: Register event handlers from module init functions
- **Timer dependencies**: `esp_timer.h` for coordination timing

### **Testing After Each Task**:
```bash
cd sensor-firmware
idf.py build    # Should compile successfully
# Test that functionality remains identical
```

### **Common Pitfalls to Avoid**:
- ❌ Don't forget to update CMakeLists.txt with all new source files
- ❌ Don't break the coordination cycle flow
- ❌ Don't duplicate variable definitions across modules
- ❌ Don't forget to include necessary ESP-IDF headers in new files
- ❌ Don't break the wake-up cause detection in main

### **Special Considerations for Sensor**:
- **Coordination cycle**: The main difference from gateway - sensor runs coordination cycle instead of simple loop
- **Sleep management**: More complex than gateway due to coordinated sleep functionality
- **State management**: Coordination state needs to be properly managed across modules

---

## ✅ **SUCCESS CRITERIA**

**Sensor Modularization Complete When**:
- ✅ Sensor main file reduced to ~50 lines
- ✅ All existing code properly extracted into focused modules
- ✅ Project compiles and runs with identical functionality
- ✅ Clean module interfaces defined
- ✅ No behavior changes - only code organization
- ✅ CMakeLists.txt properly updated
- ✅ All dependencies correctly handled
- ✅ Coordination cycle continues to work properly

**Benefits Achieved**:
- 🧹 **Clean Code Organization**: Existing code properly separated
- 🔧 **Maintainable Architecture**: Each module has single responsibility
- 🧪 **Same Functionality**: No behavior changes during modularization
- 📈 **Ready for Growth**: Clean foundation for future features

---

**🚀 Ready to Start Sensor Modularization!**

This focused approach extracts existing working code into clean modules without creating unnecessary empty files. Future features will be planned in updated `tasks.md`. 