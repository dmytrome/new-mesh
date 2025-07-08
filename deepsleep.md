You're absolutely right! Let me analyze the current implementation and understand what needs to be changed to achieve this coordinated sleep/wake cycle.

Let me first examine the current implementation to understand how we're handling time sync and sleep coordination:
Read file: gateway-firmware/main/message_handler.c
Read file: sensor-firmware/main/message_handler.c

Read file: gateway-firmware/main/message_handler.c
Read file: sensor-firmware/main/message_handler.c

Perfect! I understand the issue now. Let me analyze the current implementation and implement the coordinated sleep/wake cycle you described.

## Current Issues:

1. **Gateway calculates wake time dynamically**: Each time sync message creates a different wake-up time based on current time
2. **No unified coordination**: Sensors get different wake-up times based on when they receive time sync
3. **Gateway doesn't sleep**: Gateway stays awake instead of coordinating its own sleep cycle
4. **Inconsistent wake times**: Sensors wake up at different times instead of all together

## Your Desired Behavior:

1. **Gateway determines unified wake-up time**: `gateway_start_time + deep_sleep_period`
2. **All sensors get same wake-up time**: Everyone wakes up together
3. **Individual sleep calculation**: Each sensor calculates `unified_wake_time - current_time`
4. **Gateway also sleeps**: After publishing, gateway goes to deep sleep until unified wake-up time

Let me implement these changes:

Read file: gateway-firmware/main/include/message_handler.h


Ran tool

Perfect! I found where the gateway connects to WiFi and syncs time. In the `ip_event_handler` function in `gateway-firmware/main/mesh_network.c`, the gateway gets an IP address, initializes SNTP, and starts MQTT. This is the perfect place to initialize the time coordination.

Perfect! Now let me create a summary of all the changes made to implement the coordinated sleep/wake cycle:
Ran tool

## Summary of Changes

I've successfully implemented the coordinated sleep/wake cycle you described. Here's what was changed:

### 1. **Gateway Unified Time Coordination**
- Added global variables to track `gateway_start_time` and `unified_wake_time`
- Added `initialize_time_coordination()` function that calculates a unified wake-up time when gateway first connects to WiFi
- Modified `ip_event_handler()` to call time coordination initialization after SNTP setup
- All devices now target the same wake-up time: `gateway_start_time + sleep_period`

### 2. **Modified Time Sync Protocol**
- Gateway now sends unified wake-up time to all sensors instead of dynamic calculated times
- Each sensor calculates individual sleep duration: `unified_wake_time - current_time`
- Time sync messages include the unified wake-up time for coordination

### 3. **Gateway Deep Sleep Implementation**
- Gateway now enters deep sleep after publishing sensor data
- Gateway calculates its own sleep duration based on the unified wake-up time
- Added `gateway_enter_deep_sleep()` function with proper logging

### 4. **Sensor Sleep Coordination**
- Sensors now receive unified wake-up time from gateway
- Each sensor calculates individual sleep duration based on when they receive the time sync
- All sensors wake up at the same time regardless of when they went to sleep

### 5. **Behavioral Changes**
- **Before**: Each sensor got different wake-up times, gateway stayed awake
- **After**: All devices wake up at the same coordinated time, gateway also sleeps

## Expected Behavior

1. **Gateway startup**: Connect to WiFi → Sync time → Calculate `unified_wake_time = now + sleep_period`
2. **Sensor connections**: Gateway broadcasts unified wake-up time to all sensors
3. **Individual sleep calculation**: Each device calculates `sleep_duration = unified_wake_time - current_time`
4. **Coordinated sleep**: All devices go to deep sleep for their calculated duration
5. **Synchronized wake-up**: All devices wake up at the same `unified_wake_time`
6. **Next cycle**: Process repeats with a new unified wake-up time

This ensures that:
- **Same wake-up time**: All devices wake up together regardless of when they went to sleep
- **Individual sleep durations**: Each device sleeps for different durations but wakes up at the same time
- **Gateway coordination**: Gateway also participates in the sleep/wake cycle
- **Energy efficiency**: All devices sleep simultaneously, maximizing power savings

The example you gave is now implemented:
- Node 1 disconnects at 13:04, sleeps for (15:01 - 13:04) = 1h 57m
- Node 2 disconnects at 13:07, sleeps for (15:01 - 13:07) = 1h 54m
- Gateway publishes at 13:07, sleeps for (15:01 - 13:07) = 1h 54m
- **All devices wake up at 15:01 together**