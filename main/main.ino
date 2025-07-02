/*
 *  ESP32-S3  →  AWS IoT Core (MQTT over WiFi)
 *  --------------------------------------------------------
 *  – WiFi provides internet connectivity
 *  – mbedTLS (WiFiClientSecure) gives TLS
 *  – One telemetry topic (TOPIC_METRICS) published every 30 s
 */

// Comment out TinyGSM-related defines as we're not using GSM anymore
// #define TINY_GSM_MODEM_SIM800       // or SIM7600 if you use 4G
// #define TINY_GSM_RX_BUFFER 1024     // long enough for TLS records

#include <Arduino.h>
// #include <TinyGsmClient.h>      // Not needed for WiFi
#include <WiFi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>

#include "config.h"
#include "certs.h"

// Sensor libraries
#include "BH1750_lib.hpp"
#include "AM2302_lib.hpp"
#include "DS18B20_lib.hpp"
#include "NPK_lib.hpp"
#include "CO2_lib.hpp"

/* --------- Globals ----------------------------------------------------- */
// HardwareSerial   SerialAT(1);        // UART1 for modem - not needed for WiFi
// TinyGsm          modem(SerialAT);    // Not needed for WiFi
WiFiClientSecure tlsClient;
PubSubClient     mqtt(tlsClient);

// Battery monitoring
#if CONFIG_IDF_TARGET_ESP32S3
  #define BATTERY_SENSE_PIN 1  // ADC1_CH0 on ESP32-S3
#else 
  #define BATTERY_SENSE_PIN 36 // VP pin on ESP32
#endif

/* --------- Forward declarations --------------------------------------- */
void initSerial();
void initNetworkCore();
// void initModem();             // Not needed for WiFi
// void connectGPRS();           // Not needed for WiFi
void connectWiFi();           // New function for WiFi connection
void initTLS();
void connectMQTT();
void publishMetrics();
void mqttCallback(char*, byte*, unsigned);
float getBatteryVoltage();    // Function to read battery voltage

/* ============================  setup()  =============================== */
void setup()
{
  initSerial();               // USB-CDC Serial0 for debug
  initNetworkCore();          // create event-loop & netif, keep queue valid

  // Replace GSM/GPRS initialization with WiFi
  // initModem();                // reset SIM800
  // connectGPRS();              // PPP link up
  connectWiFi();              // Connect to WiFi
  initTLS();                  // load cert chain

  mqtt.setServer(AWS_ENDPOINT, AWS_PORT);
  mqtt.setCallback(mqttCallback);
  
  // Initialize sensors
  bh1750_init();
  am2302_init();
  ds18b20_init();
  NPK_init();
  co2_init();
}

/* ============================  loop()  ================================ */
void loop()
{
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // Check if WiFi is still connected, reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi connection lost, reconnecting..."));
    connectWiFi();
  }

  static uint32_t lastPub = 0;
  if (millis() - lastPub > 30'000) {
    lastPub = millis();
    publishMetrics();
  }
}

/* ====================================================================== */
/*   Helpers                                                              */
/* ====================================================================== */
void initSerial()
{
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) ;   // wait for USB-CDC
  Serial.println(F("\n=== ESP32-S3 AWS WiFi ==="));

  // SerialAT initialization not needed for WiFi
  // SerialAT.begin(SERIAL_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  // delay(500);
}

void initNetworkCore()
{
  esp_netif_init();
  esp_event_loop_create_default();   // ensures the LWIP queues exist
}

// GSM/GPRS functions no longer needed
/*
void initModem()
{
  Serial.print(F("Restarting modem… "));
  modem.restart();
  Serial.println(F("done"));

  if (*GSM_PIN && modem.getSimStatus() != 3) {
    Serial.print(F("Unlocking SIM… "));
    modem.simUnlock(GSM_PIN);
    Serial.println(F("ok"));
  }
}

void connectGPRS()
{
  Serial.printf("Connecting to APN %s … ", APN);
  if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
    Serial.println(F("FAIL – rebooting"));
    ESP.restart();
  }

  // Wait until PPP really allocates an IP
  while (!modem.isGprsConnected()) delay(250);
  Serial.println(F("ok"));
}
*/

// New function to connect to WiFi
void connectWiFi()
{
  Serial.printf("Connecting to WiFi SSID %s ... ", WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Wait for connection
  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("\nFAIL - rebooting"));
    ESP.restart();
  }
  
  Serial.println(F("\nConnected!"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());


  Serial.println(WiFi.localIP());

  Serial.print(F("Battery voltage: "));
  Serial.println(getBatteryVoltage());
}

void initTLS()
{
  tlsClient.setCACert(AWS_CA_CERT);
  tlsClient.setCertificate(DEVICE_CERT);
  tlsClient.setPrivateKey(DEVICE_KEY);
}

void connectMQTT()
{
  Serial.print(F("MQTT... "));
  while (!mqtt.connected()) {
    if (mqtt.connect(THING_NAME)) {
      Serial.println(F("connected"));
      // mqtt.subscribe(TOPIC_METRICS);   // optional downlink
    } else {
      Serial.printf("fail rc=%d – retry 5 s\n", mqtt.state());
      delay(5000);
    }
  }
}

float getBatteryVoltage() {
  analogReadResolution(12);
  
  // Read the battery voltage
  int rawValue = 0;
  for(int i = 0; i < 10; i++) {
    rawValue += analogRead(BATTERY_SENSE_PIN);
    delay(5);
  }
  rawValue /= 10;

  Serial.print("Raw ADC value: ");
  Serial.println(rawValue);
  
  float voltage = (rawValue * 3.3) / 4095.0;
  
  float batteryVoltage = voltage;
  
  if (batteryVoltage < 0.1) {
    batteryVoltage = 3.7;
  }
  
  return batteryVoltage;
}

void publishMetrics()
{
  // Get current Unix timestamp
  time_t now;
  time(&now);
  
  // Get sensor data
  float lux = bh1750_get_lx();
  
  float air_temp = 0, air_hum = 0;
  am2302_get_data(&air_temp, &air_hum);
  
  float ground_temp = ds18b20_get_temperature();
  
  npk_data_t npk_data;
  NPK_get(&npk_data);
  
  // Battery voltage
  float batteryVoltage = getBatteryVoltage();
  int batteryLevel = (int)((batteryVoltage - 3.0) * 100.0 / 1.2);  // 3.0-4.2V
  if (batteryLevel > 100) batteryLevel = 100;
  if (batteryLevel < 0) batteryLevel = 0;
  So the task to create mesh in the way when we have one gateway only that connects to the wifi router. Also we have leaf nodes that can be different number, on different layer but need to connect to each other. THey can be replaced with other nodes or removed at all and nothing should be brocken. All leaf nodes should send data from their sensors to the gateway. Gateway collect all data from all sensors and send it to the mqtt . All leaf nodes will have the same frimware and be sure we do not have anything hardcoded or fixed like layer or number
  char json[512];
  snprintf(json, sizeof(json), 
    "{"
    "\"timestamp\": \"%ld\","
    "\"nodes\": ["
      "{"
        "\"sensor_id\": \"1:0\","
        "\"data\": {"
          "\"lux\": %.2f,"
          "\"temp_air\": %.2f,"
          "\"hum_air\": %.2f,"
          "\"temp_ground\": %.2f,"
          "\"soil_temp\": %.2f,"
          "\"soil_hum\": %.2f,"
          "\"soil_ec\": %.2f,"
          "\"soil_ph\": %.2f,"
          "\"soil_n\": %.2f,"
          "\"soil_p\": %.2f,"
          "\"soil_k\": %.2f,"
          "\"soil_salinity\": %.2f,"
          "\"soil_tds_npk\": %.2f,"
          "\"bat_lvl\": %.2f,"
          "\"bat_vol\": %.0f"
        "}"
      "}"
    "]"
    "}",
    now,
    lux,
    air_temp,
    air_hum,
    ground_temp,
    npk_data.temperature,
    npk_data.humidity,
    npk_data.conductivity,
    npk_data.pH,
    npk_data.Nitrogen,
    npk_data.Phosphorus,
    npk_data.Potassium,
    npk_data.Salinity,
    npk_data.TDS,
    (float)batteryLevel,
    batteryVoltage * 1000  // mV
  );

  mqtt.publish(TOPIC_METRICS, json, /*retain=*/false);
  Serial.printf("[MQTT] %s\n", json);
}

void mqttCallback(char* topic, byte* payload, unsigned len)
{
  Serial.printf("\n[RX] %s ➜ ", topic);
  while (len--) Serial.write(*payload++);
  Serial.println();
}
