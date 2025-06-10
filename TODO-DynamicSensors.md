# TODO: Dynamic Sensor Discovery Implementation

## Current State (Working but Limited)
- ✅ **Hardcoded for 2 sensors** - works perfectly for testing
- ✅ **Fixed array of 100 sensors max** - sufficient for most deployments  
- ✅ **Manual configuration** - requires recompilation to change sensor count

## Future Enhancement Tasks

### Phase 1: Configuration-Based Setup (Easy)
- [ ] **Add configuration file support** (`sensors.config`)
- [ ] **Runtime sensor count setting** (no recompilation needed)
- [ ] **Environment variable support** (`SENSOR_COUNT=60`)
- [ ] **Web interface for configuration** (optional)

### Phase 2: Auto-Discovery Protocol (Medium)
- [ ] **Sensor announcement system**
  - Sensors announce themselves on mesh join
  - Gateway builds active sensor list dynamically
  - Handle sensor MAC address registration
  
- [ ] **Sensor heartbeat/keepalive**
  - Detect when sensors disconnect/fail
  - Remove inactive sensors from expected list
  - Auto-adjust expected_sensor_count
  
- [ ] **Hot-plug support**
  - Add new sensors during operation
  - Remove sensors gracefully
  - Update collection expectations in real-time

### Phase 3: Large Scale Optimizations (Advanced)
- [ ] **Dynamic memory allocation**
  - Replace fixed `reported_sensors[100][6]` array
  - Use linked list or dynamic array for unlimited sensors
  - Memory-efficient storage for 1000+ sensors
  
- [ ] **Adaptive collection windows**
  - Auto-calculate collection window based on sensor count
  - 60 seconds for ≤20 sensors
  - 120 seconds for 21-60 sensors  
  - 180 seconds for 60+ sensors
  
- [ ] **Network optimization**
  - Staggered data transmission (prevent mesh congestion)
  - Priority-based collection (critical sensors first)
  - Compression for large datasets

### Phase 4: Enterprise Features (Future)
- [ ] **Sensor grouping/zones** (e.g., Field A, Field B)
- [ ] **Hierarchical collection** (zone coordinators)
- [ ] **Fault tolerance** (backup gateways)
- [ ] **Load balancing** (multiple collection gateways)

## Implementation Priority
1. **Immediate**: Configuration file support (Phase 1)
2. **Short-term**: Auto-discovery protocol (Phase 2) 
3. **Long-term**: Large scale optimizations (Phase 3)
4. **Future**: Enterprise features (Phase 4)

## Code Changes Required

### Files to Modify:
- `gateway-firmware/main/include/time_sync_manager.h` - increase limits, add config structures
- `gateway-firmware/main/time_sync_manager.c` - implement discovery logic  
- `sensor-firmware/main/mesh_main.c` - add sensor announcement
- `gateway-firmware/main/mesh_main.c` - add configuration loading

### Key Functions to Implement:
```c
// Phase 1
esp_err_t load_sensor_config_from_file(const char* config_path);
esp_err_t set_expected_sensor_count(uint16_t count);

// Phase 2  
esp_err_t handle_sensor_announcement(const uint8_t *sensor_mac);
esp_err_t remove_inactive_sensor(const uint8_t *sensor_mac);
esp_err_t auto_detect_sensor_count(void);

// Phase 3
esp_err_t init_dynamic_sensor_list(void);
esp_err_t calculate_adaptive_collection_window(uint16_t sensor_count);
```

## Testing Strategy
- **Start with 2 sensors** (current working state)
- **Test with 5 sensors** (verify scaling)
- **Test with 20 sensors** (stress test collection window)
- **Test with 60+ sensors** (full scale deployment)

## Backward Compatibility
- All changes should maintain current API
- Fallback to hardcoded mode if config fails
- Graceful degradation for legacy deployments

---
*This TODO list ensures the system can scale from 2 sensors to 1000+ sensors while maintaining reliability and performance.* 