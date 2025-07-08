# ESP32-S3 Mesh Network Ultra-Low Power System - Expected Behavior

## System Overview

This document describes the expected behavior of the ESP32-S3 mesh network system designed for ultra-low power agricultural sensor monitoring. The system consists of:

- **1 Gateway Node**: Root node that connects to WiFi and MQTT broker
- **3 Sensor Nodes**: Agricultural sensors that collect and transmit data
- **Coordinated Sleep/Wake Cycles**: All nodes synchronize for power optimization

## System Architecture

### Network Topology
- **Tree Topology**: `CONFIG_MESH_TOPO_TREE=y`
- **Gateway as Root**: Acts as the mesh root node
- **Direct Connections**: All sensors connect directly to gateway (Layer 2)
- **No Chaining**: Sensors do not connect through other sensors
- **Connection Limit**: Gateway supports up to 3 direct sensor connections

### Power Management Strategy
- **Coordinated Sleep**: All nodes sleep and wake simultaneously
- **7-minute Cycles**: Nodes wake every 7 minutes for data collection
- **Deep Sleep**: Nodes enter deep sleep between cycles
- **Time Synchronization**: Gateway broadcasts unified wake time to all sensors

## Expected Operational Flow

### 1. System Startup Phase

#### Gateway Startup:
1. Initialize mesh network as root node
2. Connect to WiFi network
3. **Do NOT connect to MQTT immediately**
4. Wait for sensor connections
5. Open mesh network for sensor connections (up to 3 direct connections)

#### Sensor Startup:
1. Apply random startup delay (1-6 seconds based on MAC address)
2. Initialize mesh network as leaf node
3. Scan for and connect to gateway (root node)
4. Establish Layer 2 connection directly to gateway
5. Send initial sensor data to gateway

### 2. Data Collection and Transmission Phase

#### Sensor Behavior:
1. Collect agricultural sensor data (temperature, humidity, soil moisture, etc.)
2. Send data message to gateway via mesh network
3. **Check for connected children before proceeding**
4. If children are connected, wait for them to disconnect
5. Wait for time synchronization message from gateway
6. Synchronize local clock with received time
7. Calculate coordinated sleep duration
8. Enter deep sleep mode

#### Gateway Behavior:
1. Receive sensor data from all connected sensors
2. Store/buffer received sensor data
3. Calculate unified wake time (current time + 7 minutes)
4. Broadcast time synchronization message to all connected sensors
5. Wait for all sensors to disconnect from mesh
6. **Only then connect to MQTT broker**
7. Publish all collected sensor data to MQTT topics
8. Disconnect from MQTT broker
9. Calculate own sleep duration to match sensor wake time
10. Enter deep sleep mode

### 3. Sleep Coordination Phase

#### Time Synchronization:
- Gateway calculates next wake time: `current_time + 420 seconds` (7 minutes)
- Time sync message contains: `wake_time_unix_timestamp`
- Sensors receive time sync and set their system clock
- All nodes calculate sleep duration: `wake_time - current_time`
- Sleep duration should be approximately 420 seconds (7 minutes)

#### Sleep Entry:
- Sensors enter deep sleep first (after receiving time sync)
- Gateway enters deep sleep last (after MQTT publishing)
- All nodes wake simultaneously at the calculated wake time

### 4. Wake-up and Cycle Restart

#### Coordinated Wake-up:
1. All nodes wake up simultaneously at the pre-calculated time
2. Nodes reinitialize mesh network connections
3. Sensors reconnect to gateway
4. New data collection cycle begins

## Key Behavioral Expectations

### Network Connection Patterns

#### Correct Topology:
```
Gateway (Root, Layer 1)
├── Sensor 1 (Layer 2) - Direct connection
├── Sensor 2 (Layer 2) - Direct connection
└── Sensor 3 (Layer 2) - Direct connection
```

#### Incorrect Topology (Avoided):
```
Gateway (Root, Layer 1)
└── Sensor 1 (Layer 2)
    └── Sensor 2 (Layer 3) - Chained connection (BAD)
```

### Timing Expectations

#### Normal Operation Timeline:
- **0-10 seconds**: System startup and mesh formation
- **10-30 seconds**: Data collection and transmission
- **30-60 seconds**: Time synchronization and sleep preparation
- **60+ seconds**: Deep sleep phase (≈7 minutes)

#### Sleep Duration Validation:
- Sleep duration should be approximately 420 seconds (7 minutes)
- **Invalid**: Sleep durations > 1 hour indicate time sync issues
- **Critical**: Sleep durations > 1 year indicate severe clock problems

### MQTT Behavior

#### Gateway MQTT Operations:
1. **Connect**: Only when ready to publish sensor data
2. **Publish**: All collected sensor data in batch
3. **Disconnect**: Immediately after publishing
4. **No Persistent Connection**: MQTT connection is temporary

#### Expected MQTT Topics:
- `sensors/temperature`
- `sensors/humidity`
- `sensors/soil_moisture`
- `sensors/battery_level`
- `sensors/status`

### Error Handling and Recovery

#### Connection Failures:
- Sensors retry connection to gateway if initial attempt fails
- Random delays prevent simultaneous connection attempts
- Fallback to next available parent if gateway unavailable

#### Time Sync Failures:
- Sensors use default sleep duration if time sync not received
- Gateway continues operation even if some sensors don't respond
- System self-corrects on next wake cycle

#### Power Management Safeguards:
- Sensors check for children before sleeping
- Gateway waits for all sensors to disconnect
- Coordinated wake-up ensures no node is left behind

## Troubleshooting Indicators

### Normal Operation Logs:
```
[Gateway] WiFi connected, waiting for sensors
[Sensor1] Connected to gateway at layer 2
[Sensor2] Connected to gateway at layer 2  
[Sensor3] Connected to gateway at layer 2
[Gateway] Received data from 3 sensors
[Gateway] Broadcasting time sync: wake_time=1234567890
[Sensor1] Time synchronized, sleeping for 420 seconds
[Gateway] All sensors disconnected, connecting to MQTT
[Gateway] Published sensor data, entering deep sleep
```

### Problem Indicators:
- Sensors connecting at Layer 3+ (chaining issue)
- Sleep durations > 1 hour (time sync problem)
- MQTT connection before sensor data ready
- Sensors sleeping while children connected
- Authentication failures during mesh connection

## Performance Metrics

### Power Consumption:
- **Active Phase**: 30-60 seconds per cycle
- **Sleep Phase**: 6-7 minutes per cycle
- **Duty Cycle**: <15% active time
- **Expected Battery Life**: Months to years depending on battery capacity

### Network Reliability:
- **Connection Success Rate**: >95% for direct gateway connections
- **Data Transmission Success**: >99% for established connections
- **Time Synchronization Accuracy**: ±1 second across all nodes
- **Coordinated Wake-up Precision**: ±5 seconds across all nodes

## Configuration Parameters

### Key Settings:
- `MESH_AP_CONNECTIONS`: 3 (allows direct connection of all sensors)
- `MESH_TOPO_TREE`: Enabled (tree topology)
- `SLEEP_DURATION`: 420 seconds (7 minutes)
- `STARTUP_DELAY_RANGE`: 1-6 seconds (collision avoidance)
- `TIME_SYNC_TIMEOUT`: 30 seconds (wait for time sync)

This expected behavior ensures ultra-low power operation while maintaining reliable data collection and transmission in the agricultural sensor mesh network. 