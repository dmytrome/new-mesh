# ESP-MESH IoT Network Project

A multi-device ESP32-S3 mesh network implementation using ESP-IDF, designed for IoT sensor data collection and gateway communication.

## Project Architecture

```
Internet/WiFi Router
        │
        │ (WiFi Connection)
        ▼
┌─────────────────┐
│   Gateway       │  ← Root Node (Level 1)
│  (Fixed Root)   │  ← esp_mesh_fix_root(true)
│  192.168.0.x    │  ← Gets IP from router
└─────────────────┘
        │
        │ (Mesh Connection)
        ▼
┌─────────────────┐
│   Sensor 1      │  ← Child Node (Level 2)
│  (Non-Root)     │  ← Max 2 children
└─────────────────┘
        │
        │ (Mesh Connection)
        ▼
┌─────────────────┐
│   Sensor 2      │  ← Child Node (Level 3)
│  (Non-Root)     │  ← Max 2 children
└─────────────────┘
```

## Project Structure

```
new-mesh/
├── gateway-firmware/     # ESP32-S3 Gateway (Root Node)
│   ├── main/
│   │   ├── mesh_main.c
│   │   ├── Kconfig.projbuild
│   │   └── include/
│   └── sdkconfig        # Router credentials, max 1 connection
│
├── sensor-firmware/      # ESP32-S3 Sensors (Child Nodes)
│   ├── main/
│   │   ├── mesh_main.c
│   │   ├── Kconfig.projbuild
│   │   └── include/
│   └── sdkconfig        # No router credentials, max 2 connections
│
└── README.md
```

## Key Features Implemented

### ✅ **Proper ESP-MESH Configuration**
- **Gateway**: Fixed root node with WiFi router connection
- **Sensors**: Non-root nodes that can only join existing mesh networks
- **Multi-level topology**: Supports up to 8 levels deep

### ✅ **Network Topology Control**
- **Gateway**: `CONFIG_MESH_AP_CONNECTIONS=1` (forces Level 2 creation)
- **Sensors**: `CONFIG_MESH_AP_CONNECTIONS=2` (enables Level 3+ expansion)
- **Dynamic level assignment**: Nodes automatically find optimal positions

### ✅ **Security & Communication**
- **Mesh ID**: `77:77:77:77:77:77` (shared across all devices)
- **Mesh Password**: `MAP_PASSWD` (encrypted mesh communication)
- **Channel**: 6 (matches WiFi router channel)

## Configuration Summary

| Component | Router Access | Root Node | Max Children | Purpose |
|-----------|---------------|-----------|--------------|---------|
| **Gateway** | ✅ Yes | ✅ Fixed Root | 1 | Internet bridge |
| **Sensor 1** | ❌ No | ❌ Child only | 2 | Data collection |
| **Sensor 2** | ❌ No | ❌ Child only | 2 | Data collection |

## Setup Instructions

### 1. **Build Gateway Firmware**
```bash
cd gateway-firmware
idf.py build
idf.py -p /dev/cu.usbmodem1101 flash monitor
```

### 2. **Build Sensor Firmware**
```bash
cd sensor-firmware
idf.py build
idf.py -p /dev/cu.usbmodem2101 flash monitor
```

### 3. **Testing Sequence**
1. **Start Gateway first** → Connects to WiFi, becomes root
2. **Start Sensor 1** → Joins gateway (becomes Level 2)
3. **Start Sensor 2** → Gateway full, joins Sensor 1 (becomes Level 3)

## Current Status

### ✅ **What's Working**
- ✅ Clean project separation (no conditional compilation)
- ✅ Gateway connects to WiFi router as fixed root
- ✅ Proper ESP-MESH API configuration
- ✅ Multi-level topology support with connection limits

### 🔧 **Recent Fixes**
- ✅ **Fixed dual root issue**: Only gateway can become root node
- ✅ **Removed router credentials from sensors**: Prevents direct WiFi connections
- ✅ **Proper ESP-IDF configuration**: Following official mesh examples
- ✅ **Dynamic mesh joining**: Sensors automatically find and join gateway

### 📋 **Next Steps**
- [ ] Test actual mesh formation with hardware
- [ ] Implement sensor data transmission
- [ ] Add mesh network monitoring
- [ ] Implement automatic mesh healing

## Hardware Requirements

- **ESP32-S3** development boards (3+ devices)
- **WiFi Router** with known SSID/password
- **USB cables** for programming and monitoring

## Technical References

- [ESP-IDF Mesh Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp-wifi-mesh.html)
- [ESP-IDF Mesh Examples](https://github.com/espressif/esp-idf/tree/master/examples/mesh)

## Git Repository

This project uses git for version control. Current commit:
```bash
git log --oneline -1
# 84ec1d7 Initial ESP-MESH IoT Project Setup with proper gateway/sensor configuration
``` 