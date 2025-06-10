# 🏗️ Gateway Firmware Modularization Tasks

**Goal**: Extract existing code from `gateway-firmware/main/mesh_main.c` into clean, focused modules.

**Approach**: 
- ✅ **Extract existing code** into appropriate modules (ONE MODULE PER TASK)
- ✅ **Refactor main file** to use extracted modules
- ❌ **Do NOT create empty stub files** (save for future implementation tasks)
- ✅ **Follow sensor approach**: Simple, focused tasks

---

## 📋 **GATEWAY MODULARIZATION TASKS**

### **TASK G1: Extract Core Mesh Network Module**
**File**: Create `main/mesh_network.c` + `main/include/mesh_network.h`
**Purpose**: Handle basic mesh networking setup and events (similar to sensor TASK S1)

**Code to Extract From `mesh_main.c` (lines 28-155)**:
- Mesh constants: `MESH_ID`, `RX_SIZE`, `TX_SIZE` (lines 17-23)
- Network state variables: `is_mesh_connected`, `mesh_layer`, `netif_sta` (lines 31-33)
- `mesh_event_handler()` function (lines 130-155)
- `ip_event_handler()` function (lines 325-330)
- Mesh initialization in `app_main()` (lines 340-410)

**Interface Design**:
```c
// mesh_network.h
#include "esp_mesh.h"
#include "esp_event.h"

esp_err_t init_full_mesh_system(void);
bool is_mesh_connected_status(void);
int get_mesh_layer(void);
void mesh_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
```

**Files Modified**:
- Create: `main/mesh_network.c`, `main/include/mesh_network.h`
- Update: `main/mesh_main.c` (remove extracted code, add `#include "mesh_network.h"`)
- Update: `main/CMakeLists.txt` (add mesh_network.c to SRCS)

---

### **TASK G2: Extract Message Handler Module**
**File**: Create `main/message_handler.c` + `main/include/message_handler.h`
**Purpose**: Handle P2P communication and message processing (similar to sensor TASK S2)

**Code to Extract From `mesh_main.c`**:
- `esp_mesh_p2p_tx_main()` function (lines 42-74)
- `esp_mesh_p2p_rx_main()` function (lines 75-128)
- `esp_mesh_comm_p2p_start()` function (lines 110-125)
- Communication variables: `rx_buf`, `is_running` (lines 29-30)

**Interface Design**:
```c
// message_handler.h
#include "esp_mesh.h"

esp_err_t esp_mesh_comm_p2p_start(void);
void esp_mesh_p2p_tx_main(void *arg);
void esp_mesh_p2p_rx_main(void *arg);
```

**Files Modified**:
- Create: `main/message_handler.c`, `main/include/message_handler.h`
- Update: `main/mesh_main.c` (remove extracted code, add `#include "message_handler.h"`)
- Update: `main/CMakeLists.txt` (add message_handler.c to SRCS)

---

### **TASK G3: Refactor Gateway Main File**
**File**: Update `gateway-firmware/main/mesh_main.c`
**Purpose**: Simplify main file to use extracted modules (similar to final sensor cleanup)

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
#include "message_protocol.h"

static const char *MESH_TAG = "mesh_main";

void app_main(void) {
    ESP_LOGI(MESH_TAG, "=== 🏗️ IoT Gateway Starting ===");
    
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Initialize mesh network system
    ESP_ERROR_CHECK(init_full_mesh_system());
    
    ESP_LOGI(MESH_TAG, "=== ✅ Gateway Ready ===");
    
    // Keep main running
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

**Files Modified**:
- Update: `main/mesh_main.c` (major simplification)

---

## 🎯 **IMPLEMENTATION ORDER**

**Simple 3-task approach (like sensors):**

1. **TASK G1**: Extract Core Mesh Network Module (30 minutes)
2. **TASK G2**: Extract Message Handler Module (30 minutes)  
3. **TASK G3**: Refactor Gateway Main File (15 minutes)

**Total Time**: ~1.25 hours (much simpler than original plan)

---

## 📁 **BEFORE vs AFTER STRUCTURE**

**Current**:
```
gateway-firmware/main/
├── mesh_main.c                  # 433 lines - ALL CODE HERE
├── CMakeLists.txt               # Only includes mesh_main.c
└── include/
    └── message_protocol.h       # Shared protocol
```

**After Modularization**:
```
gateway-firmware/main/
├── mesh_main.c                  # ~30 lines - initialization only
├── mesh_network.c               # ~200 lines - extracted mesh code
├── message_handler.c            # ~180 lines - extracted message code  
├── CMakeLists.txt               # Updated to include all source files
└── include/
    ├── mesh_network.h           # Mesh networking interface
    ├── message_handler.h        # Message handling interface
    └── message_protocol.h       # Shared protocol (unchanged)
```

---

## ⚠️ **IMPORTANT NOTES**

### **Follow Sensor Success Pattern**:
- ✅ **One module per task** (like sensors)
- ✅ **Extract working code** (don't create empty files)
- ✅ **Test after each task** (ensure no functionality changes)
- ✅ **Simple interfaces** (like sensors)

### **Testing After Each Task**:
```bash
cd gateway-firmware
idf.py build    # Should compile successfully
```

### **Common Pitfalls to Avoid**:
- ❌ Don't make tasks too complex (learned from sensor experience)
- ❌ Don't forget to update CMakeLists.txt with new source files
- ❌ Don't break event handler registration flow
- ❌ Don't forget necessary includes

---

## ✅ **SUCCESS CRITERIA**

**Gateway Modularization Complete When**:
- ✅ Gateway main file reduced to ~30 lines (like sensors)
- ✅ All existing code properly extracted into 2 focused modules
- ✅ Project compiles and runs with identical functionality
- ✅ Clean module interfaces defined (like sensors)
- ✅ No behavior changes - only code organization

**Benefits Achieved**:
- 🧹 **Clean Code Organization**: Same as sensors
- 🔧 **Maintainable Architecture**: Each module has single responsibility
- 🧪 **Same Functionality**: No behavior changes during modularization
- 📈 **Consistent with Sensors**: Same successful approach

---

**🚀 Ready to Start Gateway Modularization!**

This simplified approach follows the successful sensor pattern: simple tasks, focused modules, one step at a time. 