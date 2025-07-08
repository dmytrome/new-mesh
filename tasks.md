=== Flashing Gateway Firmware ===
Target port: /dev/cu.usbmodem101
Sourcing ESP-IDF environment...
Checking "python3" ...
Python 3.13.2
"python3" has been detected
Activating ESP-IDF 5.4
Setting IDF_PATH to '/Users/dmi-3o/esp/esp-idf'.
* Checking python version ... 3.13.2
* Checking python dependencies ... OK
* Deactivating the current ESP-IDF environment (if any) ... OK
* Establishing a new ESP-IDF environment ... OK
* Identifying shell ... bash
* Detecting outdated tools in system ... OK - no outdated tools found
* Shell completion ... Autocompletion code generated
WARNING: Failed to load shell autocompletion for bash version: 3.2.57(1)-release!

Done! You can now compile ESP-IDF projects.
Go to the project directory and run:

  idf.py build
Flashing and monitoring gateway firmware...
Executing action: flash
Running ninja in directory /Users/dmi-3o/dev/projects/new-mesh/gateway-firmware/build
Executing "ninja flash"...
[1/5] cd /Users/...uild/mesh-re.bin
mesh-re.bin binary size 0xfc0d0 bytes. Smallest app partition is 0x100000 bytes. 0x3f30 bytes (2%) free.
Warning: The smallest app partition is nearly full (2% free space left)!
[1/1] cd /Users/...r/bootloader.bin
Bootloader binary size 0x5220 bytes. 0x2de0 bytes (36%) free.
[4/5] cd /Users/...erial_tool.cmake
esptool.py --chip esp32s3 -p /dev/cu.usbmodem101 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x0 bootloader/bootloader.bin 0x10000 mesh-re.bin 0x8000 partition_table/partition-table.bin
esptool.py v4.8.1
Serial port /dev/cu.usbmodem101
Connecting...
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 8MB (AP_3v3)
Crystal is 40MHz
MAC: a0:85:e3:e8:d2:bc
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 460800
Changed.
Configuring flash size...
Flash will be erased from 0x00000000 to 0x00005fff...
Flash will be erased from 0x00010000 to 0x0010cfff...
Flash will be erased from 0x00008000 to 0x00008fff...
SHA digest in image updated
Compressed 21024 bytes to 13383...
Writing at 0x00000000... (100 %)
Wrote 21024 bytes (13383 compressed) at 0x00000000 in 0.5 seconds (effective 367.3 kbit/s)...
Hash of data verified.
Compressed 1032400 bytes to 677480...
Writing at 0x00109cd5... (100 %)
Wrote 1032400 bytes (677480 compressed) at 0x00010000 in 9.4 seconds (effective 878.1 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 103...
Writing at 0x00008000... (100 %)
Wrote 3072 bytes (103 compressed) at 0x00008000 in 0.1 seconds (effective 361.7 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Executing action: monitor
Running idf_monitor in directory /Users/dmi-3o/dev/projects/new-mesh/gateway-firmware
Executing "/Users/dmi-3o/.espressif/python_env/idf5.4_py3.13_env/bin/python /Users/dmi-3o/esp/esp-idf/tools/idf_monitor.py -p /dev/cu.usbmodem101 -b 115200 --toolchain-prefix xtensa-esp32s3-elf- --target esp32s3 --revision 0 /Users/dmi-3o/dev/projects/new-mesh/gateway-firmware/build/mesh-re.elf -m '/Users/dmi-3o/.espressif/python_env/idf5.4_py3.13_env/bin/python' '/Users/dmi-3o/esp/esp-idf/tools/idf.py' '-p' '/dev/cu.usbmodem101'"...
--- esp-idf-monitor 1.6.2 on /dev/cu.usbmodem101 115200
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x403cd792
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2810,len:0x15a0
load:0x403c8700,len:0x4
load:0x403c8704,len:0xd20
load:0x403cb700,len:0x2f00
entry 0x403c8928
I (26) boot: ESP-IDF v5.4.1 2nd stage bootloader
I (27) boot: compile time Jul  8 2025 21:34:21
I (27) boot: Multicore bootloader
I (27) boot: chip revision: v0.2
I (30) boot: efuse block revision: v1.3
I (33) boot.esp32s3: Boot SPI Speed : 80MHz
I (37) boot.esp32s3: SPI Mode       : DIO
I (41) boot.esp32s3: SPI Flash Size : 2MB
I (45) boot: Enabling RNG early entropy source...
I (49) boot: Partition Table:
I (52) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (65) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (71) boot:  2 factory          factory app      00 00 00010000 00100000
I (78) boot: End of partition table
I (81) esp_image: segment 0: paddr=00010020 vaddr=3c0c0020 size=29190h (168336) map
I (118) esp_image: segment 1: paddr=000391b8 vaddr=3fc99400 size=049e0h ( 18912) load
I (122) esp_image: segment 2: paddr=0003dba0 vaddr=40374000 size=02478h (  9336) load
I (125) esp_image: segment 3: paddr=00040020 vaddr=42000020 size=b9014h (757780) map
I (263) esp_image: segment 4: paddr=000f903c vaddr=40376478 size=12f40h ( 77632) load
I (280) esp_image: segment 5: paddr=0010bf84 vaddr=600fe000 size=00100h (   256) load
I (281) esp_image: segment 6: paddr=0010c08c vaddr=600fe100 size=00020h (    32) load
I (292) boot: Loaded app from partition at offset 0x10000
I (293) boot: Disabling RNG early entropy source...
I (305) cpu_start: Multicore app
I (314) cpu_start: Pro cpu start user code
I (314) cpu_start: cpu freq: 160000000 Hz
I (315) app_init: Application information:
I (315) app_init: Project name:     mesh-re
I (318) app_init: App version:      523abb1-dirty
I (323) app_init: Compile time:     Jul  8 2025 21:34:14
I (328) app_init: ELF file SHA256:  2d6e94705...
I (332) app_init: ESP-IDF:          v5.4.1
I (336) efuse_init: Min chip rev:     v0.0
I (340) efuse_init: Max chip rev:     v0.99 
I (344) efuse_init: Chip rev:         v0.2
I (348) heap_init: Initializing. RAM available for dynamic allocation:
I (354) heap_init: At 3FCA4200 len 00045510 (277 KiB): RAM
I (359) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (364) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (369) heap_init: At 600FE120 len 00001EC8 (7 KiB): RTCRAM
I (376) spi_flash: detected chip: boya
I (378) spi_flash: flash io: dio
W (381) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (394) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (400) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (407) main_task: Started on CPU0
I (417) main_task: Calling app_main()
I (447) mesh_network: Initializing full mesh system...
I (447) pp: pp rom version: e7ae62f
I (447) net80211: net80211 rom version: e7ae62f
I (457) wifi:wifi driver task: 3fcae5d0, prio:23, stack:6656, core=0
I (467) wifi:wifi firmware version: 79fa3f41ba
I (467) wifi:wifi certification version: v7.0
I (467) wifi:config NVS flash: enabled
I (467) wifi:config nano formatting: disabled
I (467) wifi:Init data frame dynamic rx buffer num: 32
I (477) wifi:Init static rx mgmt buffer num: 5
I (477) wifi:Init management short buffer num: 32
I (487) wifi:Init dynamic tx buffer num: 32
I (487) wifi:Init static tx FG buffer num: 2
I (497) wifi:Init static rx buffer size: 1600
I (497) wifi:Init static rx buffer num: 10
I (497) wifi:Init dynamic rx buffer num: 32
I (507) wifi_init: rx ba win: 6
I (507) wifi_init: accept mbox: 6
I (507) wifi_init: tcpip mbox: 32
I (517) wifi_init: udp mbox: 6
I (517) wifi_init: tcp mbox: 6
I (517) wifi_init: tcp tx win: 5760
I (527) wifi_init: tcp rx win: 5760
I (527) wifi_init: tcp mss: 1440
I (527) wifi_init: WiFi IRAM OP enabled
I (537) wifi_init: WiFi RX IRAM OP enabled
I (537) phy_init: phy_version 700,8582a7fd,Feb 10 2025,20:13:11
I (577) wifi:mode : sta (a0:85:e3:e8:d2:bc) + softAP (a0:85:e3:e8:d2:bd)
I (577) wifi:enable tsf
I (587) wifi:Total power save buffer number: 16
I (587) wifi:Init max length of beacon: 752/752
I (587) wifi:Init max length of beacon: 752/752
I (607) mesh: <nvs>read layer:0
I (607) mesh: <nvs>read assoc:0
I (607) mesh: [IO]enable self-organizing, search parent<adaptive>
I (607) mesh_network: GATEWAY: Configured as self-organizing root
I (607) wifi:Set ps type: 0, coexist: 0

I (627) wifi:Total power save buffer number: 16
I (1047) wifi:mode : sta (a0:85:e3:e8:d2:bc)
I (1057) mesh: <MESH_NWK_LOOK_FOR_NETWORK>need_scan:0x3, need_scan_router:0x0, look_for_nwk_count:1
I (1057) mesh_network: <MESH_EVENT_MESH_STARTED>ID:77:77:77:77:77:77
I (1057) mesh_network: GATEWAY: Setting as root and creating network immediately
I (1067) mesh_network: GATEWAY: Root node started and connecting to router
I (1067) wifi:mode : sta (a0:85:e3:e8:d2:bc) + softAP (a0:85:e3:e8:d2:bd)
W (1077) wifi:<MESH AP>adjust channel:1, secondary channel offset:1(40U)
I (1087) wifi:Total power save buffer number: 16
I (1087) wifi:Init max length of beacon: 752/752
I (1097) wifi:Init max length of beacon: 752/752
I (1097) mesh_network: mesh starts successfully, heap:219888, root not fixed<0>(tree), ps:0
I (1107) mesh: [IO]disable self-organizing<reconnect>
I (1107) mesh: [CONFIG]connect to router:VM4248447, 00:00:00:00:00:00
I (1117) mqtt_handler: 🔧 Initializing MQTT handler for AWS IoT Core
I (1117) message_handler: GATEWAY: P2P TX and RX enabled
I (1127) mesh_network: GATEWAY: P2P communication started as root
I (1137) mesh_network: ⏳ Waiting 2 seconds for mesh AP to be fully ready...
I (1117) mesh: <MESH_NWK_PARENT_DISCONNECTED>already reconnecting, ignore it, stop_reconnection:0
I (1127) wifi:I (1117) message_handler: 🔄 Connected nodes changed: -1 -> 1
new:<1,1>, old:<1,1>, ap:<1,1>, sta:<1,0>, prof:1, snd_ch_cfg:0x0
I (1157) message_handler: 📊 Publish check: 0 sensors with data, 1 nodes in routing table
I (1157) wifi:state: init -> auth (0xb0)
I (1137) mqtt_handler: ✅ MQTT handler initialized
I (1177) wifi:state: auth -> assoc (0x0)
I (1187) mesh_main: === ✅ Gateway Ready ===
I (1187) mesh_main: ☁️ MQTT client will start after WiFi connection
I (1187) mesh_main: 🕐 SNTP will initialize after WiFi connection
I (1207) wifi:state: assoc -> run (0x10)
I (1237) wifi:connected with VM4248447, aid = 5, channel 1, BW20, bssid = ac:f8:cc:ef:6d:6e
I (1237) wifi:security: WPA2-PSK, phy: bgn, rssi: -39
I (1237) wifi:pm start, type: 0

I (1237) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
I (1247) wifi:set rx beacon pti, rx_bcn_pti: 0, bcn_timeout: 25000, mt_pti: 0, mt_time: 10000
I (1257) wifi:dp: 2, bi: 102400, li: 4, scale listen interval from 307200 us to 409600 us
I (1267) wifi:AP's beacon interval = 102400 us, DTIM period = 2
I (3137) mesh: <WIFI_EVENT_SCAN_DONE>status:fail, num:2, id:128
I (3137) mesh: [scan]new scanning time:600ms, beacon interval:300ms
I (3147) mesh: <flush>upstream packets, connections(max):1, waiting:0, upQ:0
I (3147) mesh_network: <MESH_EVENT_PARENT_CONNECTED>layer:0-->1, ID:77:77:77:77:77:77, duty:0
I (3147) mesh: <flush>root
I (3147) mesh_network: <MESH_EVENT_TODS_REACHABLE>state:0
I (3147) mesh: [TXQ]<max:128>up(0, be:0), down(0, be:0), mgmt:0, xon(req:0, rsp:0), bcast:0, wnd(0, parent:00:00:00:00:00:00)
I (3157) mesh_network: <MESH_EVENT_ROOT_ADDRESS>root address:a0:85:e3:e8:d2:bd
I (3167) mesh: [RXQ]<max:128 = cfg:128 + extra:0>self:0, <max:128 = cfg:128 + extra:0>tods:0
I (4177) esp_netif_handlers: sta ip: 192.168.0.208, mask: 255.255.255.0, gw: 192.168.0.1
I (4177) mesh_network: <IP_EVENT_STA_GOT_IP>IP:192.168.0.208
I (4177) mesh_network: 🕐 Initializing SNTP for time sync
I (4177) mesh_network: 🕐 Waiting for system time to be set... (1/10)
I (4617) wifi:<ba-add>idx:0 (ifx:0, ac:f8:cc:ef:6d:6e), tid:0, ssn:1, winSize:64
I (4647) wifi:<ba-add>idx:1 (ifx:0, ac:f8:cc:ef:6d:6e), tid:1, ssn:1, winSize:64
I (6187) mesh_network: ✅ System time is set: Tue Jul  8 20:40:59 2025

I (6187) mesh_network: 🕐 Time sync initialized, waiting for sensors to send data before starting MQTT
I (6187) mesh_network: ☁️ MQTT will start when all sensors disconnect and data is ready to publish
I (18587) wifi:new:<1,1>, old:<1,1>, ap:<1,1>, sta:<1,0>, prof:1, snd_ch_cfg:0x0
I (18587) wifi:station: 94:a9:90:1a:20:14 join, AID=1, bgn, 40U
I (18597) mesh_network: <MESH_EVENT_PS_CHILD_DUTY>cidx:0, 94:a9:90:1a:20:14, duty:0
W (18707) mesh_network: <MESH_EVENT_ROUTING_TABLE_ADD>add 1, new:2, layer:1
I (18707) mesh_network: <MESH_EVENT_CHILD_CONNECTED>aid:1, 94:a9:90:1a:20:14
W (22387) mesh_network: <MESH_EVENT_ROUTING_TABLE_ADD>add 1, new:3, layer:1
I (28777) message_handler: 📥 RECEIVED agricultural data: Air 22.8°C, Soil 20.8°C, 67.4% RH, 464 lux, pH 6.3 (Layer 2, Seq 0)
I (28777) message_handler: 🌱 AGRICULTURAL DATA [94:a9:90:1a:20:14 L2]: 22.8°C, 67.4% RH, 464 lux, pH 6.3
I (28787) message_handler: 🌱 SOIL DATA [94:a9:90:1a:20:14 L2]: Temp 20.8°C, Hum 69.0%, EC 846 µS/cm, NPK(38,19,198)
I (28797) message_handler: 🔋 STATUS [94:a9:90:1a:20:14 L2]: Battery 3.63V (3633mV), Seq 0, Quality 95%
I (28807) message_handler: 💾 Stored data for sensor 94:a9:90:1a:20:14 (total sensors: 1)
I (28817) message_handler: 🕐 Sending time sync to ALL connected sensors after receiving data
I (28827) message_handler: 🕐 Broadcasting time sync to 3 connected nodes
I (28827) message_handler: 🕐 Unified wake time initialized (will be used for ALL sensors):
I (28837) message_handler:    Current time: 1752007282
I (28847) message_handler:    Sleep period: 2 minutes
I (28847) message_handler:    Unified wake time: 1752007402
I (28857) message_handler: 🕐 Sent unified time sync to a0:85:e3:e8:d2:bc (now: 1752007282, unified wake: 1752007402, sleep: 120 sec)
I (28967) message_handler: 🕐 Sent unified time sync to 94:a9:90:1a:20:14 (now: 1752007282, unified wake: 1752007402, sleep: 120 sec)
I (29067) message_handler: 🕐 Sent unified time sync to fc:01:2c:ca:a0:ac (now: 1752007282, unified wake: 1752007402, sleep: 120 sec)
I (30337) message_handler: 📥 RECEIVED agricultural data: Air 22.7°C, Soil 20.6°C, 66.7% RH, 457 lux, pH 6.2 (Layer 3, Seq 0)
I (30337) message_handler: 🌱 AGRICULTURAL DATA [fc:01:2c:ca:a0:ac L3]: 22.7°C, 66.7% RH, 457 lux, pH 6.2
I (30347) message_handler: 🌱 SOIL DATA [fc:01:2c:ca:a0:ac L3]: Temp 20.6°C, Hum 68.3%, EC 839 µS/cm, NPK(31,13,191)
I (30357) message_handler: 🔋 STATUS [fc:01:2c:ca:a0:ac L3]: Battery 3.63V (3634mV), Seq 0, Quality 95%
I (30367) message_handler: 💾 Stored data for sensor fc:01:2c:ca:a0:ac (total sensors: 2)
I (30377) message_handler: 🕐 Sending time sync to ALL connected sensors after receiving data
I (30387) message_handler: 🕐 Broadcasting time sync to 3 connected nodes
I (30387) message_handler: 🕐 Sent unified time sync to a0:85:e3:e8:d2:bc (now: 1752007283, unified wake: 1752007402, sleep: 119 sec)
I (30507) message_handler: 🕐 Sent unified time sync to 94:a9:90:1a:20:14 (now: 1752007284, unified wake: 1752007402, sleep: 118 sec)
I (30607) message_handler: 🕐 Sent unified time sync to fc:01:2c:ca:a0:ac (now: 1752007284, unified wake: 1752007402, sleep: 118 sec)
I (31167) message_handler: 🔄 Connected nodes changed: 1 -> 3
I (31167) message_handler: 📊 Publish check: 2 sensors with data, 3 nodes in routing table
I (35407) wifi:[ADDBA]RX DELBA, reason:37, delete tid:1, initiator:1(originator)
I (35407) wifi:<ba-del>idx:1, tid:1
I (39057) wifi:[ADDBA]RX DELBA, reason:37, delete tid:0, initiator:1(originator)
I (39057) wifi:<ba-del>idx:0, tid:0
W (42247) mesh_network: <MESH_EVENT_ROUTING_TABLE_REMOVE>remove 1, new:2, layer:1
I (46167) message_handler: 🔄 Connected nodes changed: 3 -> 2
I (46167) message_handler: 📊 Publish check: 2 sensors with data, 2 nodes in routing table
W (54567) wifi:inactive timer: now=340b19b last_rx_time=2959dae diff=2bcb, aid[1]94:a9:90:1a:20:14 leave
I (54587) wifi:station: 94:a9:90:1a:20:14 leave, AID = 1, reason = 4, bss_flags is 756835, bss:0x3fcc5ec0
I (54587) wifi:new:<1,0>, old:<1,1>, ap:<1,1>, sta:<1,0>, prof:1, snd_ch_cfg:0x0
I (54597) mesh_network: <MESH_EVENT_CHILD_DISCONNECTED>aid:1, 94:a9:90:1a:20:14
I (54597) mesh: [scan]new scanning time:300ms, beacon interval:100ms
W (54597) mesh_network: <MESH_EVENT_ROUTING_TABLE_REMOVE>remove 1, new:1, layer:1
I (56167) message_handler: 🔄 Connected nodes changed: 2 -> 1
I (56167) message_handler: 📊 Publish check: 2 sensors with data, 1 nodes in routing table
I (56167) message_handler: 📤 Publishing trigger: 2 sensors with data, 1 nodes in routing table (all sensors disconnected)
I (56177) mqtt_handler: 🚀 Starting MQTT client
I (56177) mqtt_handler: 🚀 MQTT client started
I (56177) mqtt_handler: ℹ️ MQTT Other event id:7
I (56187) message_handler: ⏳ Waiting for MQTT connection before publishing...
I (56247) wifi:<ba-add>idx:0 (ifx:0, ac:f8:cc:ef:6d:6e), tid:0, ssn:4, winSize:64
I (57487) mqtt_handler: ✅ MQTT Connected to AWS IoT Core
I (58197) message_handler: 📤 Publishing combined payload with 2 sensors (all sensors disconnected)
I (58197) message_handler: 📄 JSON: {"timestamp":"1752007311","nodes":[{"sensor_id":"94:a9:90:1a:20:14","data":{"lux":464,"temp_air":22.8,"hum_air":67.4,"temp_ground":20.8,"soil_temp":20.8,"soil_hum":69.0,"soil_ec":0.85,"soil_ph":6.3,"soil_n":38.0,"soil_p":19.0,"soil_k":198.0,"soil_salinity":0.00,"soil_tds_npk":0.00,"bat_lvl":3.6,"bat_vol":3633}},{"sensor_id":"fc:01:2c:ca:a0:ac","data":{"lux":457,"temp_air":22.7,"hum_air":66.7,"temp_ground":20.6,"soil_temp":20.6,"soil_hum":68.3,"soil_ec":0.84,"soil_ph":6.2,"soil_n":31.0,"soil_p":13.0,"soil_k":191.0,"soil_salinity":0.00,"soil_tds_npk":0.00,"bat_lvl":3.6,"bat_vol":3634}}]}
I (58247) mqtt_handler: 📤 Publishing sensor data to AWS IoT Core:
I (58257) mqtt_handler:   Topic: gateway/1/metrics
I (58257) mqtt_handler:   Payload: {"timestamp":"1752007311","nodes":[{"sensor_id":"94:a9:90:1a:20:14","data":{"lux":464,"temp_air":22.8,"hum_air":67.4,"temp_ground":20.8,"soil_temp":20.8,"soil_hum":69.0,"soil_ec":0.85,"soil_ph":6.3,"soil_n":38.0,"soil_p":19.0,"soil_k":198.0,"soil_salinity":0.00,"soil_tds_npk":0.00,"bat_lvl":3.6,"bat_vol":3633}},{"sensor_id":"fc:01:2c:ca:a0:ac","data":{"lux":457,"temp_air":22.7,"hum_air":66.7,"temp_ground":20.6,"soil_temp":20.6,"soil_hum":68.3,"soil_ec":0.84,"soil_ph":6.2,"soil_n":31.0,"soil_p":13.0,"soil_k":191.0,"soil_salinity":0.00,"soil_tds_npk":0.00,"bat_lvl":3.6,"bat_vol":3634}}]}
I (58317) mqtt_handler: ✅ MQTT message published with ID: 29811
I (58317) message_handler: ☁️ Combined data published to AWS IoT Core successfully
I (58327) mqtt_handler: 🛑 Stopping MQTT client
I (58367) mqtt_handler: ✅ MQTT client stopped
I (58367) message_handler: 🛑 MQTT client stopped after publishing
I (58367) message_handler: 🌙 All sensors sleeping, data published. Gateway preparing for deep sleep...
I (63377) message_handler: 🌙 Gateway entering deep sleep:
I (63377) message_handler:    Current time: 1752007316
I (63377) message_handler:    Wake time: 1752007402
I (63377) message_handler:    Sleep duration: 86 seconds
I (64377) message_handler: 💤 Gateway deep sleep started
--- Error: read failed: [Errno 6] Device not configured
--- Waiting for the device to reconnect...........................................





=== Flashing Sensor Firmware ===
Target port: /dev/cu.usbmodem1101
Sourcing ESP-IDF environment...
Checking "python3" ...
Python 3.13.2
"python3" has been detected
Activating ESP-IDF 5.4
Setting IDF_PATH to '/Users/dmi-3o/esp/esp-idf'.
* Checking python version ... 3.13.2
* Checking python dependencies ... OK
* Deactivating the current ESP-IDF environment (if any) ... OK
* Establishing a new ESP-IDF environment ... OK
* Identifying shell ... bash
* Detecting outdated tools in system ... OK - no outdated tools found
* Shell completion ... Autocompletion code generated
WARNING: Failed to load shell autocompletion for bash version: 3.2.57(1)-release!

Done! You can now compile ESP-IDF projects.
Go to the project directory and run:

  idf.py build
Flashing and monitoring sensor firmware...
Executing action: flash
Running ninja in directory /Users/dmi-3o/dev/projects/new-mesh/sensor-firmware/build
Executing "ninja flash"...
[1/5] cd /Users/...uild/mesh-re.bin
mesh-re.bin binary size 0xdc2b0 bytes. Smallest app partition is 0x100000 bytes. 0x23d50 bytes (14%) free.
[1/1] cd /Users/...r/bootloader.bin
Bootloader binary size 0x5220 bytes. 0x2de0 bytes (36%) free.
[4/5] cd /Users/...erial_tool.cmake
esptool.py --chip esp32s3 -p /dev/cu.usbmodem1101 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x0 bootloader/bootloader.bin 0x10000 mesh-re.bin 0x8000 partition_table/partition-table.bin
esptool.py v4.8.1
Serial port /dev/cu.usbmodem1101
Connecting...
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 8MB (AP_3v3)
Crystal is 40MHz
MAC: 94:a9:90:1a:20:14
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 460800
Changed.
Configuring flash size...
Flash will be erased from 0x00000000 to 0x00005fff...
Flash will be erased from 0x00010000 to 0x000ecfff...
Flash will be erased from 0x00008000 to 0x00008fff...
SHA digest in image updated
Compressed 21024 bytes to 13385...
Writing at 0x00000000... (100 %)
Wrote 21024 bytes (13385 compressed) at 0x00000000 in 0.5 seconds (effective 372.0 kbit/s)...
Hash of data verified.
Compressed 901808 bytes to 590661...
Writing at 0x000ebdc9... (100 %)
Wrote 901808 bytes (590661 compressed) at 0x00010000 in 8.5 seconds (effective 852.1 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 103...
Writing at 0x00008000... (100 %)
Wrote 3072 bytes (103 compressed) at 0x00008000 in 0.1 seconds (effective 351.5 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Executing action: monitor
Running idf_monitor in directory /Users/dmi-3o/dev/projects/new-mesh/sensor-firmware
Executing "/Users/dmi-3o/.espressif/python_env/idf5.4_py3.13_env/bin/python /Users/dmi-3o/esp/esp-idf/tools/idf_monitor.py -p /dev/cu.usbmodem1101 -b 115200 --toolchain-prefix xtensa-esp32s3-elf- --target esp32s3 --revision 0 /Users/dmi-3o/dev/projects/new-mesh/sensor-firmware/build/mesh-re.elf -m '/Users/dmi-3o/.espressif/python_env/idf5.4_py3.13_env/bin/python' '/Users/dmi-3o/esp/esp-idf/tools/idf.py' '-p' '/dev/cu.usbmodem1101'"...
--- esp-idf-monitor 1.6.2 on /dev/cu.usbmodem1101 115200
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H
I (270) boot: Loaded app from partition at offset 0x10000
I (270) boot: Disabling RNG early entropy source...
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x40048c29
--- 0x40048c29: uart_tx_flush in ROM

SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2810,len:0x15a0
load:0x403c8700,len:0x4
load:0x403c8704,len:0xd20
load:0x403cb700,len:0x2f00
entry 0x403c8928
I (26) boot: ESP-IDF v5.4.1 2nd stage bootloader
I (27) boot: compile time Jul  8 2025 21:40:27
I (27) boot: Multicore bootloader
I (27) boot: chip revision: v0.2
I (30) boot: efuse block revision: v1.3
I (33) boot.esp32s3: Boot SPI Speed : 80MHz
I (37) boot.esp32s3: SPI Mode       : DIO
I (41) boot.esp32s3: SPI Flash Size : 2MB
I (45) boot: Enabling RNG early entropy source...
I (49) boot: Partition Table:
I (52) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (65) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (71) boot:  2 factory          factory app      00 00 00010000 00100000
I (78) boot: End of partition table
I (81) esp_image: segment 0: paddr=00010020 vaddr=3c0a0020 size=22c70h (142448) map
I (113) esp_image: segment 1: paddr=00032c98 vaddr=3fc99400 size=04960h ( 18784) load
I (118) esp_image: segment 2: paddr=00037600 vaddr=40374000 size=08a18h ( 35352) load
I (126) esp_image: segment 3: paddr=00040020 vaddr=42000020 size=9f858h (653400) map
I (241) esp_image: segment 4: paddr=000df880 vaddr=4037ca18 size=0c8dch ( 51420) load
I (253) esp_image: segment 5: paddr=000ec164 vaddr=600fe000 size=00100h (   256) load
I (253) esp_image: segment 6: paddr=000ec26c vaddr=600fe100 size=00020h (    32) load
I (265) boot: Loaded app from partition at offset 0x10000
I (265) boot: Disabling RNG early entropy source...
I (278) cpu_start: Multicore app
I (287) cpu_start: Pro cpu start user code
I (287) cpu_start: cpu freq: 160000000 Hz
I (287) app_init: Application information:
I (287) app_init: Project name:     mesh-re
I (291) app_init: App version:      523abb1-dirty
I (295) app_init: Compile time:     Jul  8 2025 21:40:19
I (300) app_init: ELF file SHA256:  b160f9482...
I (305) app_init: ESP-IDF:          v5.4.1
I (309) efuse_init: Min chip rev:     v0.0
I (312) efuse_init: Max chip rev:     v0.99 
I (316) efuse_init: Chip rev:         v0.2
I (320) heap_init: Initializing. RAM available for dynamic allocation:
I (327) heap_init: At 3FCA2FD0 len 00046740 (281 KiB): RAM
I (332) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (337) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (342) heap_init: At 600FE120 len 00001EC8 (7 KiB): RTCRAM
I (348) spi_flash: detected chip: boya
I (351) spi_flash: flash io: dio
W (354) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (366) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (372) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (379) main_task: Started on CPU0
I (389) main_task: Calling app_main()
I (419) mesh_main: 🔄 Adding startup delay: 4212 ms to avoid connection conflicts
I (4629) mesh_main: 🔄 Normal boot - initializing standard mesh operation
I (4629) mesh_network: Initializing full mesh system...
I (4629) mesh_network: Initializing WiFi and network interfaces...
I (4629) pp: pp rom version: e7ae62f
I (4639) net80211: net80211 rom version: e7ae62f
I (4649) wifi:wifi driver task: 3fcad348, prio:23, stack:6656, core=0
I (4659) wifi:wifi firmware version: 79fa3f41ba
I (4659) wifi:wifi certification version: v7.0
I (4659) wifi:config NVS flash: enabled
I (4659) wifi:config nano formatting: disabled
I (4659) wifi:Init data frame dynamic rx buffer num: 32
I (4669) wifi:Init static rx mgmt buffer num: 5
I (4669) wifi:Init management short buffer num: 32
I (4679) wifi:Init dynamic tx buffer num: 32
I (4679) wifi:Init static tx FG buffer num: 2
I (4689) wifi:Init static rx buffer size: 1600
I (4689) wifi:Init static rx buffer num: 10
I (4689) wifi:Init dynamic rx buffer num: 32
I (4699) wifi_init: rx ba win: 6
I (4699) wifi_init: accept mbox: 6
I (4699) wifi_init: tcpip mbox: 32
I (4709) wifi_init: udp mbox: 6
I (4709) wifi_init: tcp mbox: 6
I (4709) wifi_init: tcp tx win: 5760
I (4719) wifi_init: tcp rx win: 5760
I (4719) wifi_init: tcp mss: 1440
I (4719) wifi_init: WiFi IRAM OP enabled
I (4729) wifi_init: WiFi RX IRAM OP enabled
I (4729) phy_init: phy_version 700,8582a7fd,Feb 10 2025,20:13:11
I (4779) wifi:mode : sta (94:a9:90:1a:20:14) + softAP (94:a9:90:1a:20:15)
I (4779) wifi:enable tsf
I (4789) wifi:Total power save buffer number: 16
I (4789) wifi:Init max length of beacon: 752/752
I (4789) wifi:Init max length of beacon: 752/752
I (4799) mesh_network: Initializing mesh base configuration...
I (4809) mesh: <nvs>read layer:0
I (4809) mesh: <nvs>read assoc:0
I (4809) mesh: [IO]enable self-organizing<adaptive>
I (4809) wifi:Set ps type: 0, coexist: 0

I (4819) mesh_network: Configuring and starting mesh...
I (4829) wifi:Total power save buffer number: 16
I (5259) wifi:mode : sta (94:a9:90:1a:20:14)
I (5269) mesh: <MESH_NWK_LOOK_FOR_NETWORK>need_scan:0x3, need_scan_router:0x0, look_for_nwk_count:1
I (5269) mesh_network: <MESH_EVENT_MESH_STARTED>ID:77:77:77:77:77:77
I (5269) mesh_main: mesh starts successfully, heap:224872, root not fixed<0>(tree), ps:0
I (5279) mesh_main: 💡 Hint: To enable ultra-low power mode, trigger a timer wake-up
I (5289) mesh_main: 🔄 Running in standard continuous mode for now
I (5289) main_task: Returned from app_main()
I (9469) mesh: find root:ESPM_E8D2BC, root_cap:1(max:300), new channel:1, old channel:0
I (9469) mesh: [FIND][ch:0]AP:13, otherID:0, MAP:1, idle:0, candidate:0, root:1[00:00:00:00:00:00]
I (9479) mesh: [FIND:1]find a network, channel:1, cfg<channel:0, router:dummy, 00:00:00:00:00:00>

I (9489) wifi:mode : sta (94:a9:90:1a:20:14) + softAP (94:a9:90:1a:20:15)
W (9489) wifi:<MESH AP>adjust channel:1, secondary channel offset:1(40U)
I (9499) wifi:Total power save buffer number: 16
I (9499) wifi:Init max length of beacon: 752/752
I (9509) wifi:Init max length of beacon: 752/752
I (9489) mesh_network: <MESH_EVENT_FIND_NETWORK>new channel:1, router BSSID:00:00:00:00:00:00
W (9519) wifi:<MESH AP>adjust channel:1, secondary channel offset:1(40U)
I (9529) wifi:Total power save buffer number: 16
I (9829) mesh: [SCAN][ch:1]AP:5, other(ID:0, RD:0), MAP:1, idle:0, candidate:1, root:1, topMAP:0[c:0,i:0][00:00:00:00:00:00]<>
I (9839) mesh: 7393[selection]try rssi_threshold:-78, backoff times:0, max:5<-78,-82,-85>
I (9839) mesh: [DONE]connect to parent:ESPM_E8D2BC, channel:1, rssi:-45, a0:85:e3:e8:d2:bd[layer:1, assoc:0], my_vote_num:0/voter_num:0, rc[00:00:00:00:00:00/-120/0]
I (9859) mesh: set router bssid:ac:f8:cc:ef:6d:6e
I (9959) wifi:new:<1,1>, old:<1,1>, ap:<1,1>, sta:<1,1>, prof:1, snd_ch_cfg:0x0
I (9959) wifi:state: init -> auth (0xb0)
I (9969) wifi:state: auth -> assoc (0x0)
I (9999) wifi:state: assoc -> run (0x10)
I (9999) mesh: <MESH_NWK_MIE_CHANGE><><><><ROOT ADDR><><><>
I (10009) mesh: <MESH_NWK_ROOT_ADDR>from assoc, layer:2, root_addr:a0:85:e3:e8:d2:bd, root_cap:1
I (10009) mesh: <MESH_NWK_ROOT_ADDR>idle, layer:2, root_addr:a0:85:e3:e8:d2:bd, conflict_roots.num:0<>
I (10019) mesh_network: <MESH_EVENT_ROOT_ADDRESS>root address:a0:85:e3:e8:d2:bd
I (10099) wifi:connected with ESPM_E8D2BC, aid = 1, channel 1, 40U, bssid = a0:85:e3:e8:d2:bd
I (10099) wifi:security: WPA2-PSK, phy: bgn, rssi: -45
I (10099) wifi:pm start, type: 0

I (10099) wifi:set rx beacon pti, rx_bcn_pti: 0, bcn_timeout: 25000, mt_pti: 0, mt_time: 10000
I (10109) mesh: [scan]new scanning time:600ms, beacon interval:300ms
I (10129) mesh: 2013<arm>parent monitor, my layer:2(cap:8)(node), interval:7386ms, retries:1<normal connected>
I (10129) mesh_network: <MESH_EVENT_PARENT_CONNECTED>layer:0-->2, parent:a0:85:e3:e8:d2:bd<layer2>, ID:77:77:77:77:77:77, duty:0
I (10139) message_handler: 🌱 SENSOR: P2P RX enabled + Agricultural data TX enabled
I (10139) message_handler: 🌱 Agricultural sensor TX task started - MAC: a0:85:e3:e8:d2:bd
I (10139) mesh_network: <MESH_EVENT_TODS_REACHABLE>state:0
I (10149) message_handler: ⏳ Waiting 10 seconds before sending data to allow mesh connections...
I (10259) wifi:AP's beacon interval = 307200 us, DTIM period = 1
I (11609) wifi:new:<1,1>, old:<1,1>, ap:<1,1>, sta:<1,1>, prof:1, snd_ch_cfg:0x0
I (11609) wifi:station: fc:01:2c:ca:a0:ac join, AID=1, bgn, 40U
I (11609) mesh_network: <MESH_EVENT_PS_CHILD_DUTY>cidx:0, fc:01:2c:ca:a0:ac, duty:0
I (11649) mesh_network: <MESH_EVENT_CHILD_CONNECTED>aid:1, fc:01:2c:ca:a0:ac
I (11659) wifi:<ba-add>idx:0 (ifx:0, a0:85:e3:e8:d2:bd), tid:5, ssn:0, winSize:64
I (11659) message_handler: 🕐 Broadcast time sync to new sensors (current time: 26)
I (17689) mesh: 5172<active>parent layer:1(node), channel:1, rssi:-45, assoc:0(cnx rssi threshold:-120)my_assoc:1
I (19189) mesh: 5971<scan>parent layer:1, rssi:-45, assoc:1(cnx rssi threshold:-120)
I (19189) mesh: [SCAN][ch:1]AP:2, other(ID:0, RD:0), MAP:2, idle:0, candidate:0, root:1, topMAP:0[c:2,i:2][ac:f8:cc:ef:6d:6e]<weak>
I (19199) mesh: [FAIL][1]root:0, fail:1, normal:0, <pre>backoff:0[weak]

I (19209) mesh: 2013<arm>parent monitor, my layer:2(cap:8)(node), interval:7136ms, retries:2<>
I (20169) message_handler: 📤 Sending agricultural sensor data to gateway...
I (20169) message_handler: 🌱 Sent agricultural data: Air 22.8°C, Soil 20.8°C, 67.4% RH, 464 lux, pH 6.3 (Layer 2, Seq 0)
I (20169) message_handler: ⏳ Waiting 10 seconds after sending data before next cycle...
I (20359) message_handler: 📨 RX #1: from a0:85:e3:e8:d2:bc, size:28, expected time_sync size:28
I (20359) message_handler: 🕐 Received message with time sync size - checking message type...
I (20369) message_handler:    Message type: 0x05 (expected: 0x05)
I (20369) message_handler: 🕐 Received unified time sync from gateway:
I (20379) message_handler:    Gateway current time: 1752007282
I (20379) message_handler:    Unified wake time: 1752007402
I (20389) message_handler:    My current time: 35
I (20389) message_handler: 🕐 Updated system time to gateway time: 1752007282
I (20399) message_handler: 🕐 Synchronized current time: 1752007282
I (20409) message_handler: 🔍 Sleep check: routing_table_size=2, calculated_children=1, layer=2, is_leaf=NO
W (20419) message_handler: 🌳 Cannot sleep yet - still have mesh children (layer 2, children: 1)
I (20419) message_handler: ⏳ Waiting for children to disconnect before sleeping...
I (21899) message_handler: 📨 RX #2: from a0:85:e3:e8:d2:bc, size:28, expected time_sync size:28
I (21899) message_handler: 🕐 Received message with time sync size - checking message type...
I (21899) message_handler:    Message type: 0x05 (expected: 0x05)
I (21909) message_handler: 🕐 Received unified time sync from gateway:
I (21909) message_handler:    Gateway current time: 1752007284
I (21919) message_handler:    Unified wake time: 1752007402
I (21919) message_handler:    My current time: 1752007283
I (21929) message_handler: 🕐 Updated system time to gateway time: 1752007284
I (21939) message_handler: 🕐 Synchronized current time: 1752007284
I (21939) message_handler: 🔍 Sleep check: routing_table_size=2, calculated_children=1, layer=2, is_leaf=NO
W (21949) message_handler: 🌳 Cannot sleep yet - still have mesh children (layer 2, children: 1)
I (21959) message_handler: ⏳ Waiting for children to disconnect before sleeping...
I (26529) mesh: 5172<active>parent layer:1(node), channel:1, rssi:-45, assoc:1(cnx rssi threshold:-120)my_assoc:1
I (28029) mesh: 5971<scan>parent layer:1, rssi:-46, assoc:1(cnx rssi threshold:-120)
I (28029) mesh: [SCAN][ch:1]AP:1, other(ID:0, RD:0), MAP:1, idle:0, candidate:0, root:1, topMAP:0[c:2,i:2][ac:f8:cc:ef:6d:6e]<weak>
I (28039) mesh: [FAIL][1]root:0, fail:1, normal:0, <pre>backoff:0[weak]

I (28039) mesh: 2013<arm>parent monitor, my layer:2(cap:8)(node), interval:11350ms, retries:3<>
I (30179) message_handler: 🌿 Checking for mesh children (max wait: 300s, check interval: 5s)...
I (30179) message_handler: 🔍 Mesh analysis: routing_table_size=2, calculated_children=1, parent_layer=2
I (30179) message_handler: 🌳 Still have mesh children (layer 2, children: 1). Waiting before sleep... (1/60 checks)
W (33599) wifi:inactive timer: now=200b182 last_rx_time=14fa1c7 diff=2d53, aid[1]fc:01:2c:ca:a0:ac leave
I (33619) wifi:station: fc:01:2c:ca:a0:ac leave, AID = 1, reason = 4, bss_flags is 756835, bss:0x3fca5fd4
I (33619) wifi:new:<1,1>, old:<1,1>, ap:<1,1>, sta:<1,1>, prof:1, snd_ch_cfg:0x0
I (33619) mesh: [TX-RMV]child:fc:01:2c:ca:a0:ac, children_cnt:1, rmv_children_cnt:1
I (33629) mesh: [scan]new scanning time:300ms, beacon interval:100ms
I (33629) mesh_network: unknown id:4
I (33969) message_handler: 🌿 Already have time sync and now confirmed as leaf node - proceeding to sleep
I (33969) message_handler: 🔍 Sleep check: routing_table_size=1, calculated_children=0, layer=2, is_leaf=YES
I (35189) message_handler: 🔍 Mesh analysis: routing_table_size=1, calculated_children=0, parent_layer=2
I (35189) message_handler: 🌿 Confirmed leaf node (layer 2, children: 0). Ready for deep sleep after time sync.
I (35189) message_handler: 🌿 Leaf node confirmed - waiting for time sync from gateway...
I (35969) message_handler: ⏰ Final coordinated sleep calculation (calculated right before sleep):
I (35969) message_handler:    Current time: 1752007298
I (35969) message_handler:    Unified wake time: 1752007402
I (35969) message_handler:    My layer: 2 (no layer delay applied)
I (35979) message_handler:    Exact sleep duration: 104 seconds
I (35979) message_handler:    Expected wake: 1752007402
I (35989) message_handler: 💤 Entering coordinated deep sleep for 104 seconds until unified wake time
--- Error: read failed: [Errno 6] Device not configured
--- Waiting for the device to reconnect...........................................................................................................



=== Flashing Sensor Firmware ===
Target port: /dev/cu.usbmodem2101
Sourcing ESP-IDF environment...
Checking "python3" ...
Python 3.13.2
"python3" has been detected
Activating ESP-IDF 5.4
Setting IDF_PATH to '/Users/dmi-3o/esp/esp-idf'.
* Checking python version ... 3.13.2
* Checking python dependencies ... OK
* Deactivating the current ESP-IDF environment (if any) ... OK
* Establishing a new ESP-IDF environment ... OK
* Identifying shell ... bash
* Detecting outdated tools in system ... OK - no outdated tools found
* Shell completion ... Autocompletion code generated
WARNING: Failed to load shell autocompletion for bash version: 3.2.57(1)-release!

Done! You can now compile ESP-IDF projects.
Go to the project directory and run:

  idf.py build
Flashing and monitoring sensor firmware...
Executing action: flash
Running ninja in directory /Users/dmi-3o/dev/projects/new-mesh/sensor-firmware/build
Executing "ninja flash"...
[1/5] cd /Users/...uild/mesh-re.bin
mesh-re.bin binary size 0xdc2b0 bytes. Smallest app partition is 0x100000 bytes. 0x23d50 bytes (14%) free.
[1/1] cd /Users/...r/bootloader.bin
Bootloader binary size 0x5220 bytes. 0x2de0 bytes (36%) free.
[4/5] cd /Users/...erial_tool.cmake
esptool.py --chip esp32s3 -p /dev/cu.usbmodem2101 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x0 bootloader/bootloader.bin 0x10000 mesh-re.bin 0x8000 partition_table/partition-table.bin
esptool.py v4.8.1
Serial port /dev/cu.usbmodem2101
Connecting...
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 8MB (AP_3v3)
Crystal is 40MHz
MAC: fc:01:2c:ca:a0:ac
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 460800
Changed.
Configuring flash size...
Flash will be erased from 0x00000000 to 0x00005fff...
Flash will be erased from 0x00010000 to 0x000ecfff...
Flash will be erased from 0x00008000 to 0x00008fff...
SHA digest in image updated
Compressed 21024 bytes to 13385...
Writing at 0x00000000... (100 %)
Wrote 21024 bytes (13385 compressed) at 0x00000000 in 0.4 seconds (effective 392.0 kbit/s)...
Hash of data verified.
Compressed 901808 bytes to 590661...
Writing at 0x000ebdc9... (100 %)
Wrote 901808 bytes (590661 compressed) at 0x00010000 in 7.7 seconds (effective 939.4 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 103...
Writing at 0x00008000... (100 %)
Wrote 3072 bytes (103 compressed) at 0x00008000 in 0.1 seconds (effective 361.9 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
Executing action: monitor
Running idf_monitor in directory /Users/dmi-3o/dev/projects/new-mesh/sensor-firmware
Executing "/Users/dmi-3o/.espressif/python_env/idf5.4_py3.13_env/bin/python /Users/dmi-3o/esp/esp-idf/tools/idf_monitor.py -p /dev/cu.usbmodem2101 -b 115200 --toolchain-prefix xtensa-esp32s3-elf- --target esp32s3 --revision 0 /Users/dmi-3o/dev/projects/new-mesh/sensor-firmware/build/mesh-re.elf -m '/Users/dmi-3o/.espressif/python_env/idf5.4_py3.13_env/bin/python' '/Users/dmi-3o/esp/esp-idf/tools/idf.py' '-p' '/dev/cu.usbmodem2101'"...
--- esp-idf-monitor 1.6.2 on /dev/cu.usbmodem2101 115200
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x8 (SPI_FAST_FLASH_BOOT)
Saved PC:0x403cd9ae
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2810,len:0x15a0
load:0x403c8700,len:0x4
load:0x403c8704,len:0xd20
load:0x403cb700,len:0x2f00
entry 0x403c8928
I (26) boot: ESP-IDF v5.4.1 2nd stage bootloader
I (27) boot: compile time Jul  8 2025 21:40:27
I (27) boot: Multicore bootloader
I (27) boot: chip revision: v0.2
I (30) boot: efuse block revision: v1.3
I (33) boot.esp32s3: Boot SPI Speed : 80MHz
I (37) boot.esp32s3: SPI Mode       : DIO
I (41) boot.esp32s3: SPI Flash Size : 2MB
I (45) boot: Enabling RNG early entropy source...
I (49) boot: Partition Table:
I (52) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (65) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (71) boot:  2 factory          factory app      00 00 00010000 00100000
I (78) boot: End of partition table
I (81) esp_image: segment 0: paddr=00010020 vaddr=3c0a0020 size=22c70h (142448) map
I (113) esp_image: segment 1: paddr=00032c98 vaddr=3fc99400 size=04960h ( 18784) load
I (118) esp_image: segment 2: paddr=00037600 vaddr=40374000 size=08a18h ( 35352) load
I (126) esp_image: segment 3: paddr=00040020 vaddr=42000020 size=9f858h (653400) map
I (241) esp_image: segment 4: paddr=000df880 vaddr=4037ca18 size=0c8dch ( 51420) load
I (253) esp_image: segment 5: paddr=000ec164 vaddr=600fe000 size=00100h (   256) load
I (253) esp_image: segment 6: paddr=000ec26c vaddr=600fe100 size=00020h (    32) load
I (265) boot: Loaded app from partition at offset 0x10000
I (265) boot: Disabling RNG early entropy source...
I (278) cpu_start: Multicore app
I (287) cpu_start: Pro cpu start user code
I (287) cpu_start: cpu freq: 160000000 Hz
I (287) app_init: Application information:
I (287) app_init: Project name:     mesh-re
I (291) app_init: App version:      523abb1-dirty
I (295) app_init: Compile time:     Jul  8 2025 21:40:19
I (300) app_init: ELF file SHA256:  b160f9482...
I (305) app_init: ESP-IDF:          v5.4.1
I (309) efuse_init: Min chip rev:     v0.0
I (312) efuse_init: Max chip rev:     v0.99 
I (316) efuse_init: Chip rev:         v0.2
I (320) heap_init: Initializing. RAM available for dynamic allocation:
I (327) heap_init: At 3FCA2FD0 len 00046740 (281 KiB): RAM
I (332) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (337) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (342) heap_init: At 600FE120 len 00001EC8 (7 KiB): RTCRAM
I (348) spi_flash: detected chip: boya
I (351) spi_flash: flash io: dio
W (354) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (366) sleep_gpio: Configure to isolate all GPIO pins in sleep state
I (372) sleep_gpio: Enable automatic switching of GPIO sleep configuration
I (379) main_task: Started on CPU0
I (389) main_task: Calling app_main()
I (409) mesh_main: 🔄 Adding startup delay: 2724 ms to avoid connection conflicts
I (3129) mesh_main: 🔄 Normal boot - initializing standard mesh operation
I (3129) mesh_network: Initializing full mesh system...
I (3129) mesh_network: Initializing WiFi and network interfaces...
I (3129) pp: pp rom version: e7ae62f
I (3139) net80211: net80211 rom version: e7ae62f
I (3149) wifi:wifi driver task: 3fcad348, prio:23, stack:6656, core=0
I (3159) wifi:wifi firmware version: 79fa3f41ba
I (3159) wifi:wifi certification version: v7.0
I (3159) wifi:config NVS flash: enabled
I (3159) wifi:config nano formatting: disabled
I (3159) wifi:Init data frame dynamic rx buffer num: 32
I (3169) wifi:Init static rx mgmt buffer num: 5
I (3169) wifi:Init management short buffer num: 32
I (3179) wifi:Init dynamic tx buffer num: 32
I (3179) wifi:Init static tx FG buffer num: 2
I (3189) wifi:Init static rx buffer size: 1600
I (3189) wifi:Init static rx buffer num: 10
I (3189) wifi:Init dynamic rx buffer num: 32
I (3199) wifi_init: rx ba win: 6
I (3199) wifi_init: accept mbox: 6
I (3199) wifi_init: tcpip mbox: 32
I (3209) wifi_init: udp mbox: 6
I (3209) wifi_init: tcp mbox: 6
I (3209) wifi_init: tcp tx win: 5760
I (3219) wifi_init: tcp rx win: 5760
I (3219) wifi_init: tcp mss: 1440
I (3219) wifi_init: WiFi IRAM OP enabled
I (3229) wifi_init: WiFi RX IRAM OP enabled
I (3229) phy_init: phy_version 700,8582a7fd,Feb 10 2025,20:13:11
I (3269) phy_init: Saving new calibration data due to checksum failure or outdated calibration data, mode(0)
I (3349) wifi:mode : sta (fc:01:2c:ca:a0:ac)
I (3349) wifi:enable tsf
I (3349) mesh_network: Initializing mesh base configuration...
I (3349) wifi:mode : sta (fc:01:2c:ca:a0:ac) + softAP (fc:01:2c:ca:a0:ad)
I (3359) wifi:Total power save buffer number: 16
I (3359) wifi:Init max length of beacon: 752/752
I (3359) wifi:Init max length of beacon: 752/752
I (3379) mesh: <nvs>read layer:0, err:0x1102
I (3379) mesh: <nvs>read assoc:0, err:0x1102
I (3379) mesh: [IO]enable self-organizing<adaptive>
I (3379) wifi:Set ps type: 0, coexist: 0

I (3379) mesh_network: Configuring and starting mesh...
I (3399) wifi:Total power save buffer number: 16
I (3819) wifi:mode : sta (fc:01:2c:ca:a0:ac)
I (3829) mesh: <MESH_NWK_LOOK_FOR_NETWORK>need_scan:0x3, need_scan_router:0x0, look_for_nwk_count:1
I (3829) mesh_network: <MESH_EVENT_MESH_STARTED>ID:77:77:77:77:77:77
I (3829) mesh_main: mesh starts successfully, heap:224708, root not fixed<0>(tree), ps:0
I (3839) mesh_main: 💡 Hint: To enable ultra-low power mode, trigger a timer wake-up
I (3849) mesh_main: 🔄 Running in standard continuous mode for now
I (3849) main_task: Returned from app_main()
I (8029) mesh: find root:ESPM_E8D2BC, root_cap:1(max:300), new channel:1, old channel:0
I (8029) mesh: [FIND][ch:0]AP:16, otherID:0, MAP:1, idle:0, candidate:0, root:1[00:00:00:00:00:00]
I (8039) mesh: [FIND:1]find a network, channel:1, cfg<channel:0, router:dummy, 00:00:00:00:00:00>

I (8049) wifi:mode : sta (fc:01:2c:ca:a0:ac) + softAP (fc:01:2c:ca:a0:ad)
W (8049) wifi:<MESH AP>adjust channel:1, secondary channel offset:1(40U)
I (8059) wifi:Total power save buffer number: 16
I (8059) wifi:Init max length of beacon: 752/752
I (8069) wifi:Init max length of beacon: 752/752
I (8049) mesh_network: <MESH_EVENT_FIND_NETWORK>new channel:1, router BSSID:00:00:00:00:00:00
W (8079) wifi:<MESH AP>adjust channel:1, secondary channel offset:1(40U)
I (8089) wifi:Total power save buffer number: 16
I (8389) mesh: [SCAN][ch:1]AP:8, other(ID:0, RD:0), MAP:2, idle:0, candidate:1, root:1, topMAP:1[c:0,i:0][00:00:00:00:00:00]<>
I (8399) mesh: 7393[selection]try rssi_threshold:-78, backoff times:0, max:5<-78,-82,-85>
I (8399) mesh: [DONE]connect to parent:ESPM_1A2014, channel:1, rssi:-44, 94:a9:90:1a:20:15[layer:2, assoc:0], my_vote_num:0/voter_num:0, rc[00:00:00:00:00:00/-120/0]
I (8419) mesh: set router bssid:ac:f8:cc:ef:6d:6e
I (8629) wifi:new:<1,1>, old:<1,1>, ap:<1,1>, sta:<1,1>, prof:1, snd_ch_cfg:0x0
I (8629) wifi:state: init -> auth (0xb0)
I (8639) wifi:state: auth -> assoc (0x0)
I (8649) wifi:state: assoc -> run (0x10)
I (8659) mesh: <MESH_NWK_MIE_CHANGE><><><><ROOT ADDR><><><>
I (8659) mesh: <MESH_NWK_ROOT_ADDR>from assoc, layer:3, root_addr:a0:85:e3:e8:d2:bd, root_cap:1
I (8659) mesh: <MESH_NWK_ROOT_ADDR>idle, layer:3, root_addr:a0:85:e3:e8:d2:bd, conflict_roots.num:0<>
I (8669) mesh_network: <MESH_EVENT_ROOT_ADDRESS>root address:a0:85:e3:e8:d2:bd
I (8679) wifi:connected with ESPM_1A2014, aid = 1, channel 1, 40U, bssid = 94:a9:90:1a:20:15
I (8689) wifi:security: WPA2-PSK, phy: bgn, rssi: -45
I (8689) wifi:pm start, type: 0

I (8689) wifi:set rx beacon pti, rx_bcn_pti: 0, bcn_timeout: 25000, mt_pti: 0, mt_time: 10000
I (8699) mesh: [scan]new scanning time:600ms, beacon interval:300ms
I (8719) mesh: 2013<arm>parent monitor, my layer:3(cap:8)(node), interval:6310ms, retries:1<normal connected>
I (8719) mesh_network: <MESH_EVENT_PARENT_CONNECTED>layer:0-->3, parent:94:a9:90:1a:20:15, ID:77:77:77:77:77:77, duty:0
I (8729) message_handler: 🌱 SENSOR: P2P RX enabled + Agricultural data TX enabled
I (8729) message_handler: 🌱 Agricultural sensor TX task started - MAC: 94:a9:90:1a:20:15
I (8739) mesh_network: <MESH_EVENT_TODS_REACHABLE>state:0
I (8739) message_handler: ⏳ Waiting 10 seconds before sending data to allow mesh connections...
I (8939) wifi:AP's beacon interval = 307200 us, DTIM period = 1
I (15179) mesh: 5172<active>parent layer:2(node), channel:1, rssi:-44, assoc:0(cnx rssi threshold:-120)my_assoc:0
I (16679) mesh: 5971<scan>parent layer:2, rssi:-45, assoc:1(cnx rssi threshold:-120)
I (16679) mesh: [SCAN][ch:1]AP:2, other(ID:0, RD:0), MAP:2, idle:0, candidate:0, root:1, topMAP:1[c:3,i:3][ac:f8:cc:ef:6d:6e]<weak>
I (16689) mesh: [FAIL][1]root:0, fail:1, normal:0, <pre>backoff:0[weak]

I (16699) mesh: 2013<arm>parent monitor, my layer:3(cap:8)(node), interval:4967ms, retries:2<>
I (17489) message_handler: 📨 RX #1: from a0:85:e3:e8:d2:bc, size:28, expected time_sync size:28
I (17489) message_handler: 🕐 Received message with time sync size - checking message type...
I (17499) message_handler:    Message type: 0x05 (expected: 0x05)
I (17499) message_handler: 🕐 Received unified time sync from gateway:
I (17509) message_handler:    Gateway current time: 1752007282
I (17519) message_handler:    Unified wake time: 1752007402
I (17519) message_handler:    My current time: 1752007283
I (17529) message_handler: 🕐 Updated system time to gateway time: 1752007282
I (17529) message_handler: 🕐 Synchronized current time: 1752007282
I (17539) message_handler: 🔍 Sleep check: routing_table_size=1, calculated_children=0, layer=3, is_leaf=YES
I (17549) message_handler: 🌿 Confirmed leaf node (layer 3, children: 0). Ready for deep sleep.
I (17559) message_handler: ⏰ Coordinated sleep calculation:
I (17559) message_handler:    Unified wake time: 1752007402
I (17569) message_handler:    My layer: 3 (all layers wake at same time)
I (17569) message_handler:    My sleep duration: 120 seconds
I (17579) message_handler:    Expected wake: 1752007402
I (18759) message_handler: 📤 Sending agricultural sensor data to gateway...
I (18759) message_handler: 🌱 Sent agricultural data: Air 22.7°C, Soil 20.6°C, 66.7% RH, 457 lux, pH 6.2 (Layer 3, Seq 0)
I (18759) message_handler: ⏳ Waiting 10 seconds after sending data before next cycle...
I (19589) message_handler: ⏰ Final coordinated sleep calculation (calculated right before sleep):
I (19589) message_handler:    Current time: 1752007284
I (19589) message_handler:    Unified wake time: 1752007402
I (19589) message_handler:    My layer: 3 (no layer delay applied)
I (19599) message_handler:    Exact sleep duration: 118 seconds
I (19599) message_handler:    Expected wake: 1752007402
I (19609) message_handler: 💤 Entering coordinated deep sleep for 118 seconds until unified wake time
--- Error: read failed: [Errno 6] Device not configured
--- Waiting for the device to reconnect...........................................................................................................................................