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

## 🔄 COMPREHENSIVE IMPLEMENTATION PLAN

### **PHASE 0: CODE MODULARIZATION** (Day 1 - Foundation) 🏗️

**Goal**: Extract existing code into clean, maintainable modules before adding new features.

#### **GATEWAY MODULARIZATION** (~2 hours)
**Reference**: See `gateway_modularization_tasks.md` for detailed instructions

- **TASK M-G1**: Extract Core Mesh Network Module (30 min)
  - Create `main/mesh_network.c` + `main/include/mesh_network.h`
  - Extract ~200 lines of existing mesh code
  
- **TASK M-G2**: Extract Message Protocol Handler (20 min)
  - Create `main/message_handler.c` + `main/include/message_handler.h`
  - Extract ~80 lines of existing RX code
  
- **TASK M-G3**: Extract Gateway Status Monitor (20 min)
  - Create `main/gateway_status.c` + `main/include/gateway_status.h`
  - Extract ~60 lines of existing status code
  
- **TASK M-G4**: Refactor Gateway Main File (30 min)
  - Reduce `main/mesh_main.c` to ~40 lines
  - Clean initialization and module startup only

#### **SENSOR MODULARIZATION** (~3 hours)
**Reference**: See `sensor_modularization_tasks.md` for detailed instructions

- **TASK M-S1**: Extract Core Mesh Network Module (40 min)
  - Create `main/mesh_network.c` + `main/include/mesh_network.h`
  - Extract ~250 lines of existing mesh code
  
- **TASK M-S2**: Extract Message Protocol Handler (30 min)
  - Create `main/message_handler.c` + `main/include/message_handler.h`
  - Extract ~120 lines of existing MAC TX code
  
- **TASK M-S3**: Extract Sensor Coordination Manager (30 min)
  - Create `main/sensor_coordinator.c` + `main/include/sensor_coordinator.h`
  - Extract ~150 lines of existing coordination code
  
- **TASK M-S4**: Extract Basic Sleep Management (20 min)
  - Create `main/sensor_sleep.c` + `main/include/sensor_sleep.h`
  - Extract ~50 lines of existing sleep code
  
- **TASK M-S5**: Extract Sensor Data Management (20 min)
  - Create `main/sensor_data.c` + `main/include/sensor_data.h`
  - Extract ~80 lines of existing data code
  
- **TASK M-S6**: Refactor Sensor Main File (30 min)
  - Reduce `main/mesh_main.c` to ~50 lines
  - Clean initialization and module startup only

**Benefits After Modularization**:
- 🧹 **Clean codebase** - Each module has single responsibility
- 🔧 **Maintainable** - Easy to add new features to specific modules
- 🧪 **Same functionality** - No behavior changes, just organization
- 📈 **Ready for growth** - Clean foundation for new features

---

### **PHASE 1: PROJECT STRUCTURE & BASIC SEPARATION** (Week 1)

**Note**: Phase 1 is already complete. Gateway and sensor projects exist and work.

#### **TASK 1: ✅ Create Project Structure** (COMPLETED)
**Status**: ✅ **DONE** - Projects created and working

#### **TASK 2: ✅ Create Common Protocol Headers** (COMPLETED)  
**Status**: ✅ **DONE** - `message_protocol.h` exists in both projects

#### **TASK 3: ✅ Gateway - Force Root Role** (COMPLETED)
**Status**: ✅ **DONE** - Gateway working as self-organizing root

#### **TASK 4: ✅ Sensor - Child Node Role** (COMPLETED)
**Status**: ✅ **DONE** - Sensors working as self-organizing children

---

### **PHASE 2: TIME SYNCHRONIZATION & ULTRA-LOW POWER** (Week 2)

#### **TASK 5: Build & Flash Scripts** ⭐ (NEW TASK)
**Goal**: Create convenience scripts for building and flashing
**Time**: 30 minutes
**Files**: `build_all.sh`, `flash_gateway.sh`, `flash_sensor.sh`

**Create convenience scripts**:
```bash
# build_all.sh
#!/bin/bash
echo "Building gateway firmware..."
cd gateway-firmware && idf.py build
echo "Building sensor firmware..."
cd ../sensor-firmware && idf.py build

# flash_gateway.sh  
#!/bin/bash
cd gateway-firmware && idf.py flash monitor

# flash_sensor.sh
#!/bin/bash  
cd sensor-firmware && idf.py flash monitor
```

#### **TASK 6: Gateway GPRS Power Management Module** 
**Goal**: Implement GPRS on-demand power control for ultra-low power
**Time**: 2 hours
**File**: `gateway-firmware/main/gprs_power.c`

**Implementation**: Add to existing `gateway-firmware` project
- GPIO power control for GPRS module
- On-demand power cycling (only when needed)
- Energy usage tracking
- Connection establishment and teardown

#### **TASK 7: Gateway Time Sync Broadcaster Module**
**Goal**: Implement time synchronization via GPRS/NTP  
**Time**: 3 hours
**File**: `gateway-firmware/main/time_sync.c`

**Implementation**:
- NTP time retrieval via GPRS
- Time sync message broadcasting to all sensors
- RTC time management during sleep
- Coordinated wake-up scheduling

#### **TASK 8: Sensor Time Sync Client Module**
**Goal**: Implement time sync reception and coordination
**Time**: 2 hours  
**File**: `sensor-firmware/main/time_sync.c`

**Implementation**:
- Time sync message reception from gateway
- Local RTC time adjustment
- Coordinated wake-up timing
- Emergency fallback timing

#### **TASK 9: Coordinated Deep Sleep Module (Both)**
**Goal**: Implement synchronized deep sleep across all nodes
**Time**: 4 hours
**Files**: `gateway-firmware/main/gateway_sleep.c`, `sensor-firmware/main/sensor_sleep.c`

**Implementation**:
- Coordinated sleep preparation (ALL nodes sleep together)
- Synchronized wake-up timing
- Emergency sleep handling
- RTC-based wake-up scheduling

---

### **PHASE 3: SENSOR INTEGRATION & DATA COLLECTION** (Week 3)

#### **TASK 10: Agricultural Sensor Reader Module**
**Goal**: Implement BME280 and agricultural sensor interfaces
**Time**: 3 hours
**File**: `sensor-firmware/main/sensor_reader.c`

**Implementation**:
- BME280 driver integration (temperature, humidity, pressure)
- I2C sensor interfaces for soil sensors
- Agricultural sensor data collection (15 fields matching JSON)
- Sensor calibration and validation

#### **TASK 11: Battery Monitoring Module (Both)**
**Goal**: Implement battery monitoring and adaptive power management
**Time**: 2 hours
**Files**: `gateway-firmware/main/battery_manager.c`, `sensor-firmware/main/battery_manager.c`

**Implementation**:
- ADC battery voltage monitoring  
- Power level calculation and reporting
- Low battery warnings and adaptive operation
- Battery-aware sleep duration adjustment

#### **TASK 12: Cloud Communication Module (Gateway)**
**Goal**: Implement cloud data upload with agricultural JSON formatting
**Time**: 4 hours
**File**: `gateway-firmware/main/cloud_comm.c`

**Implementation**:
- HTTP/MQTT client for cloud communication
- Agricultural sensor JSON formatting (exact match to user's format)
- Batch data uploading for efficiency
- Connection retry logic and error handling

---

### **PHASE 4: ADVANCED FEATURES & OPTIMIZATION** (Week 4)

#### **TASK 13: Error Handler & Diagnostics Module (Both)**
**Goal**: Implement centralized error handling and system health monitoring
**Time**: 2 hours
**Files**: `gateway-firmware/main/error_handler.c`, `sensor-firmware/main/error_handler.c`

#### **TASK 14: Configuration Manager Module (Both)**  
**Goal**: Implement NVS-based configuration management
**Time**: 2 hours
**Files**: `gateway-firmware/main/config_manager.c`, `sensor-firmware/main/config_manager.c`

#### **TASK 15: Sensor Data Coordinator Module**
**Goal**: Implement intelligent sensor data collection and transmission
**Time**: 3 hours
**File**: `sensor-firmware/main/data_coordinator.c`

#### **TASK 16: Gateway Data Aggregator Module**
**Goal**: Implement sensor data aggregation and batch processing
**Time**: 3 hours  
**File**: `gateway-firmware/main/data_aggregator.c`

---

### **PHASE 5: ULTRA-LOW POWER OPTIMIZATION** (Week 5)

#### **TASK 17-21**: Power optimization, testing, and validation
#### **TASK 22-25**: Production deployment, OTA updates, monitoring

---

## 🎯 **NEXT IMMEDIATE ACTIONS**

1. **Complete Phase 0**: Execute modularization tasks
   - Start with `gateway_modularization_tasks.md` (2 hours)
   - Then `sensor_modularization_tasks.md` (3 hours)
   
2. **Start Phase 2**: Begin with TASK 5 (build scripts)

3. **Future Implementation**: Each subsequent task will implement one specific module with clear scope

---

## ✅ **SUCCESS CRITERIA**

**Modularization Complete When**:
- ✅ All existing code properly organized into focused modules
- ✅ Main files reduced to simple initialization (~40-50 lines)
- ✅ Project compiles and runs with identical functionality  
- ✅ Clean foundation ready for new feature implementation

**System Complete When**:
- ✅ Ultra-low power operation (< 50µA sleep current)
- ✅ Coordinated mesh wake/sleep cycles
- ✅ Agricultural sensor data collection (15 fields)
- ✅ GPRS on-demand communication
- ✅ 6+ month battery life achieved