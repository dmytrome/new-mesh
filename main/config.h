#pragma once
#include <stdint.h>  // for uint16_t

/* ──────────────────────────────────────────────
 *  Hardware & carrier
 * ──────────────────────────────────────────── */
#define MODEM_TX   38      // change if your board labels 27
#define MODEM_RX   37      // change if your board labels 26
#define SERIAL_BAUD 115200

// Disable GSM/GPRS connection (commented out)
// static const char GSM_PIN[]  = "";           // SIM PIN
// static const char APN[]      = "internet";   // carrier APN
// static const char GPRS_USER[] = "";
// static const char GPRS_PASS[] = "";

/* ──────────────────────────────────────────────
 *  WiFi Configuration
 * ──────────────────────────────────────────── */
static const char* WIFI_SSID = "VM4248447";      // Replace with your WiFi SSID
static const char* WIFI_PASS = "Rt5dvpsk6mpd";  // Replace with your WiFi password

/* ──────────────────────────────────────────────
 *  AWS IoT Core
 * ──────────────────────────────────────────── */
static const char* AWS_ENDPOINT = "a1w68ba53cj3wf-ats.iot.eu-central-1.amazonaws.com";
static const uint16_t AWS_PORT  = 8883;
static const char* THING_NAME   = "ag_thing";     // must equal AWS Thing name

/* ──────────────────────────────────────────────
 *  MQTT (single topic)
 * ──────────────────────────────────────────── */
static const char* TOPIC_METRICS = "gateway/1/metrics";
