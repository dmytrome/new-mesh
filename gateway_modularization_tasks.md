# 🏗️ Gateway Firmware Modularization Tasks

**Goal**: Extract existing code from `gateway-firmware/main/mesh_main.c` into clean, focused modules.

**Approach**: 
- ✅ **Extract existing code** into appropriate modules
- ✅ **Refactor main file** to use extracted modules
- ❌ **Do NOT create empty stub files** (save for future implementation tasks)
- ✅ **Update tasks.md** with future module implementation needs

---

## 📋 **GATEWAY MODULARIZATION TASKS**

### **TASK G1: Extract Core Mesh Network Module**
**File**: Create `main/mesh_network.c` + `main/include/mesh_network.h`
**Purpose**: Handle mesh networking as gateway root

**Code to Extract From `mesh_main.c`**:
- `mesh_event_handler()` function (lines 130-200+)
- `ip_event_handler()` function (lines 325+)
- Mesh initialization functions: `init_wifi_and_netif()`, `init_mesh_base()`, `configure_and_start_mesh()`
- P2P communication setup: `esp_mesh_comm_p2p_start()` function (lines 110-125)
- Network status tracking variables: `is_mesh_connected`, `mesh_layer`, `netif_sta`
- Mesh constants and macros: `MESH_ID`, `RX_SIZE`, `TX_SIZE`

**Interface Design**:
```c
// mesh_network.h
#include "esp_mesh.h"
#include "esp_event.h"
#include "esp_netif.h"

esp_err_t mesh_network_init_gateway(void);
esp_err_t mesh_network_start_p2p_communication(void);
bool mesh_network_is_root_established(void);
int mesh_network_get_connected_nodes_count(void);
void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
```

**Files Modified**:
- Create: `main/mesh_network.c`, `main/include/mesh_network.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)
- Update: `main/CMakeLists.txt` (change SRCS to include new files):
  ```cmake
  idf_component_register(SRCS "mesh_main.c" "mesh_network.c" "message_handler.c" "gateway_status.c"
                      INCLUDE_DIRS "." "include")
  ```

---

### **TASK G2: Extract Message Protocol Handler**
**File**: Create `main/message_handler.c` + `main/include/message_handler.h`
**Purpose**: Handle incoming sensor data and message processing

**Code to Extract From `mesh_main.c`**:
- `esp_mesh_p2p_rx_main()` function (lines 75-110)
- MAC message processing logic (sensor MAC message handling)
- `sensor_mac_msg_t` structure definition (lines 17-23)
- Message type constants: `MSG_TYPE_SENSOR_MAC` (line 25)
- RX buffer: `rx_buf[RX_SIZE]` variable (line 33)

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

esp_err_t message_handler_init_gateway(void);
void message_handler_start_rx_task(void);
esp_err_t message_handler_process_sensor_mac(const sensor_mac_msg_t *mac_msg, const mesh_addr_t *from);
```

**Files Modified**:
- Create: `main/message_handler.c`, `main/include/message_handler.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)

---

### **TASK G3: Extract Gateway Status Monitor**
**File**: Create `main/gateway_status.c` + `main/include/gateway_status.h`
**Purpose**: Monitor gateway status and network topology

**Code to Extract From `mesh_main.c`**:
- `esp_mesh_p2p_tx_main()` function (lines 42-74)
- Routing table monitoring logic
- Network status logging and reporting
- Connected sensor counting logic
- `is_running` variable (line 34)

**Interface Design**:
```c
// gateway_status.h
#include "esp_mesh.h"

esp_err_t gateway_status_init(void);
void gateway_status_start_monitoring_task(void);
int gateway_status_get_connected_sensor_count(void);
void gateway_status_stop(void);
```

**Files Modified**:
- Create: `main/gateway_status.c`, `main/include/gateway_status.h`
- Update: `main/mesh_main.c` (remove extracted code, add includes)

---

### **TASK G4: Refactor Gateway Main File**
**File**: Update `gateway-firmware/main/mesh_main.c`
**Purpose**: Simplify main file to use extracted modules

**Refactoring Steps**:
1. Remove all code that was extracted to modules
2. Add include statements for new modules
3. Replace existing initialization with module calls
4. Simplify `app_main()` to basic initialization + module startup

**Target Gateway Main Structure**:
```c
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "mesh_network.h"
#include "message_handler.h"
#include "gateway_status.h"

static const char *MESH_TAG = "mesh_main";

void app_main(void) {
    ESP_LOGI(MESH_TAG, "=== 🏗️ IoT Gateway Starting (Modular) ===");
    
    // Initialize NVS (existing code)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize extracted modules
    ESP_ERROR_CHECK(mesh_network_init_gateway());
    ESP_ERROR_CHECK(message_handler_init_gateway());
    ESP_ERROR_CHECK(gateway_status_init());
    
    ESP_LOGI(MESH_TAG, "=== ✅ Gateway Modules Initialized ===");
    
    // Start module functionality
    ESP_ERROR_CHECK(mesh_network_start_p2p_communication());
    message_handler_start_rx_task();
    gateway_status_start_monitoring_task();
    
    ESP_LOGI(MESH_TAG, "=== 🔋 Gateway Running ===");
    
    // Simple main loop (no need for complex logic here)
    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
```

**Files Modified**:
- Update: `main/mesh_main.c` (major refactoring)

---

## 🎯 **IMPLEMENTATION ORDER**

**All tasks can be completed in 1 day:**

1. **TASK G1**: Extract Core Mesh Network Module (30 minutes)
2. **TASK G2**: Extract Message Protocol Handler (20 minutes)  
3. **TASK G3**: Extract Gateway Status Monitor (20 minutes)
4. **TASK G4**: Refactor Gateway Main File (30 minutes)

**Total Time**: ~2 hours for complete gateway modularization

---

## 📁 **BEFORE vs AFTER STRUCTURE**

**Current**:
```
gateway-firmware/main/
├── mesh_main.c                  # 401 lines - ALL CODE HERE
├── CMakeLists.txt               # Only includes mesh_main.c
└── include/
    └── message_protocol.h       # Shared protocol
```

**After Modularization**:
```
gateway-firmware/main/
├── mesh_main.c                  # ~40 lines - initialization only
├── mesh_network.c               # ~200 lines - extracted mesh code
├── message_handler.c            # ~80 lines - extracted message code  
├── gateway_status.c             # ~60 lines - extracted status code
├── CMakeLists.txt               # Updated to include all source files
└── include/
    ├── mesh_network.h           # Mesh networking interface
    ├── message_handler.h        # Message handling interface
    ├── gateway_status.h         # Status monitoring interface
    └── message_protocol.h       # Shared protocol (unchanged)
```

---

## ⚠️ **IMPORTANT NOTES**

### **Dependencies to Handle**:
- **ESP-IDF includes**: Each module needs proper ESP-IDF headers
- **Shared variables**: Variables like `is_mesh_connected`, `mesh_layer` need to be accessed via functions
- **Task handles**: Ensure proper task creation and cleanup
- **Event handlers**: Register event handlers from module init functions

### **Testing After Each Task**:
```bash
cd gateway-firmware
idf.py build    # Should compile successfully
# Test that functionality remains identical
```

### **Common Pitfalls to Avoid**:
- ❌ Don't forget to update CMakeLists.txt with new source files
- ❌ Don't break event handler registration flow
- ❌ Don't duplicate variable definitions across modules
- ❌ Don't forget to include necessary ESP-IDF headers in new files

---

## ✅ **SUCCESS CRITERIA**

**Gateway Modularization Complete When**:
- ✅ Gateway main file reduced to ~40 lines
- ✅ All existing code properly extracted into focused modules
- ✅ Project compiles and runs with identical functionality
- ✅ Clean module interfaces defined
- ✅ No behavior changes - only code organization
- ✅ CMakeLists.txt properly updated
- ✅ All dependencies correctly handled

**Benefits Achieved**:
- 🧹 **Clean Code Organization**: Existing code properly separated
- 🔧 **Maintainable Architecture**: Each module has single responsibility
- 🧪 **Same Functionality**: No behavior changes during modularization
- 📈 **Ready for Growth**: Clean foundation for future features

---

**🚀 Ready to Start Gateway Modularization!**

This focused approach extracts existing working code into clean modules without creating unnecessary empty files. Future features will be planned in updated `tasks.md`. 