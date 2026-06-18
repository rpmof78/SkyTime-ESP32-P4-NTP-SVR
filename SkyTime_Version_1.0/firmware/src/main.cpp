/*
 * SkyTime Stratum-1 NTP Server
 * Version 1.0
 *
 * Copyright (C) 2026 RPMof78
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/*
  SkyTime GPS/PPS Stratum-1 NTP Server

  Board:   Waveshare ESP32-P4-ETH
  Display: ST7789 240x240 SPI
  GPS:     ATGM336H Serial2 (GPIO21 RX / GPIO22 TX)
  PPS:     GPIO46, rising edge
  Button:  GPIO47, active LOW

  Current Features:
  - GPS/PPS disciplined timing core
  - Stratum-1 NTP server
  - Ethernet web interface
  - SD-hosted web UI
  - Dashboard page
  - System Status page
  - Read-only Log Viewer
  - Identity Configuration
  - Network Configuration
  - Web GUI reboot control
  - Licensing page
  - Event and NTP logging
  - Local LCD status screens

  Release: Version 1.0
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <TinyGPS++.h>
#include <time.h>
#include "esp_timer.h"
#include <WiFi.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <FS.h>
#include <SD_MMC.h>
#include <WebServer.h>

// ============================================================
// GPIO Pin Definitions - Waveshare ESP32-P4-ETH
// ============================================================

#define SPI_SCK_PIN              20
#define SPI_MOSI_PIN             23
#define SPI_CS_PIN               26
#define SPI_DC_PIN               27
#define SPI_RST_PIN              32
#define BL_PIN                   33

#define I2C_SDA_PIN              6
#define I2C_SCL_PIN              7

#define GPS_RX_PIN               21
#define GPS_TX_PIN               22
#define PPS_IN_PIN               46
#define BUTTON_PIN               47

// ============================================================
// Display Configuration
// ============================================================

#define DISPLAY_WIDTH            240
#define DISPLAY_HEIGHT           240

// ============================================================
// GPS / Timing Configuration
// ============================================================

#define GPS_BAUD_RATE            9600

// ATGM336H NMEA output is UTC. Do not subtract GPS leap seconds
// from parsed NMEA time. Leap seconds apply to raw GPS time, not
// normal NMEA UTC sentences.
#define NTP_UNIX_OFFSET          2208988800UL

#define PPS_INTERVAL_MIN_US      950000ULL
#define PPS_INTERVAL_MAX_US      1050000ULL
#define PPS_TIMEOUT_US           2500000ULL

// ISR-level glitch filter.
// A valid PPS is 1 Hz, so any rising edge arriving too soon after the
// previous accepted edge is almost certainly noise, ringing, or a double edge.
#define PPS_ISR_REJECT_US        800000ULL

#define GPS_STALE_MS             3000UL
#define GPS_HOLDOVER_MAX_MS      300000UL
#define GPS_LOCK_MIN_SATS        4

// PPS/GPS alignment guard.
// The GPS UTC second must be reasonably fresh when latching it to PPS.
#define PPS_GPS_ALIGN_MAX_AGE_MS 1500UL

// ============================================================
// Button Configuration
// ============================================================

#define BUTTON_DEBOUNCE_MS       40UL
#define LONG_PRESS_TIME_MS       5000UL
#define ALT_SCREEN_TIME_MS       30000UL

// ============================================================
// Scheduler Configuration
// ============================================================

#define DISPLAY_FAST_MS          100UL     // PPS microsecond field
#define DISPLAY_NORMAL_MS        250UL     // primary screen
#define DISPLAY_SLOW_MS          1000UL    // date / diagnostic slow fields
#define DEBUG_INTERVAL_MS        5000UL

// ============================================================
// FreeRTOS Task Configuration
// ============================================================

#define TASK_TIMING_STACK        8192
#define TASK_DISPLAY_STACK       8192
#define TASK_NETWORK_STACK       12288
#define TASK_TIMING_PRIORITY     3
#define TASK_NETWORK_PRIORITY    2
#define TASK_DISPLAY_PRIORITY    1
#define TASK_TIMING_CORE         0
#define TASK_NETWORK_CORE        0
#define TASK_DISPLAY_CORE        0

// ============================================================
// RGB565 Color Definitions
// ============================================================

#define COLOR_BLACK              0x0000
#define COLOR_WHITE              0xFFFF
#define COLOR_GREEN              0x07E0
#define COLOR_RED                0xF800
#define COLOR_YELLOW             0xFFE0
#define COLOR_BLUE               0x001F
#define COLOR_CYAN               0x07FF
#define COLOR_DARKGRAY           0x4208
#define COLOR_LIGHTGRAY          0x8410
#define COLOR_ORANGE             0xFD20
#define COLOR_MAGENTA            0xF81F

// ============================================================
// Arduino GFX Display Setup
// ============================================================

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  SPI_DC_PIN,
  SPI_CS_PIN,
  SPI_SCK_PIN,
  SPI_MOSI_PIN,
  -1
);

Arduino_GFX *gfx = new Arduino_ST7789(
  bus,
  SPI_RST_PIN,
  1,
  true,
  DISPLAY_WIDTH,
  DISPLAY_HEIGHT,
  0,
  0
);

// ============================================================
// Global Objects
// ============================================================

TinyGPSPlus gps;
portMUX_TYPE pps_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
// Data Structures
// ============================================================

enum DisplayMode {
  PRIMARY_SCREEN = 0,
  NETWORK_SCREEN = 1,
  DIAGNOSTIC_SCREEN = 2
};

enum ScreenEvent {
  SCREEN_EVENT_NONE = 0,
  SCREEN_EVENT_SHORT_PRESS = 1,
  SCREEN_EVENT_TIMEOUT = 2,
  SCREEN_EVENT_FORCE_MAIN = 3
};

enum GPSLockState {
  GPS_SEARCHING = 0,
  GPS_LOCKED = 1,
  GPS_STALE = 2,
  GPS_HOLDOVER = 3
};

enum PPSLockState {
  PPS_WAITING = 0,
  PPS_LOCKED = 1,
  PPS_BAD_INTERVAL = 2,
  PPS_TIMEOUT = 3,
  PPS_ALIGN_WAIT = 4,
  PPS_ALIGN_BAD = 5
};

enum ButtonPhysicalState {
  BUTTON_IDLE = 0,
  BUTTON_DEBOUNCE_PRESS = 1,
  BUTTON_HELD = 2,
  BUTTON_DEBOUNCE_RELEASE = 3
};

struct PPSCapture {
  volatile uint64_t pps_time_us;
  volatile uint32_t pps_count;
  volatile uint32_t rejected_edges;
  volatile bool pps_triggered;
};

struct PPSData {
  uint64_t current_pps_time_us;
  uint64_t previous_pps_time_us;
  uint64_t last_interval_us;

  uint64_t min_interval_us;
  uint64_t max_interval_us;
  uint64_t avg_interval_us;
  uint64_t jitter_us;
  uint64_t jitter_accum_us;
  uint32_t jitter_samples;

  uint32_t pps_count;
  uint32_t last_pps_epoch;
  uint32_t valid_count;
  uint32_t bad_count;
  uint32_t align_good_count;
  uint32_t align_bad_count;

  PPSLockState state;
  bool edge_seen;
  bool gps_aligned;
};

struct TimingData {
  uint32_t current_epoch;
  uint64_t current_time_us;
  uint32_t microseconds_since_pps;
  uint32_t ntp_fraction;
  int8_t ntp_precision;
  uint32_t root_delay;
  uint32_t root_dispersion;
  bool disciplined;
  bool holdover;
  uint32_t holdover_start_ms;
};

struct GPSData {
  GPSLockState state;
  bool locked;
  bool time_valid;
  bool date_valid;
  bool location_valid;
  uint8_t satellites;
  double latitude;
  double longitude;
  double altitude;
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  uint32_t last_update_ms;
  uint32_t last_time_update_ms;
  uint32_t last_location_update_ms;
  uint32_t last_locked_ms;
};

struct ButtonState {
  ButtonPhysicalState state;
  bool irq_pending;
  uint32_t transition_ms;
  uint32_t press_start_ms;
  bool long_press_reported;
};

struct ScreenManager {
  DisplayMode current;
  DisplayMode previous;
  uint32_t entered_ms;
  bool timed_screen;
  bool redraw_required;
};

struct DisplayCache {
  DisplayMode active_mode;
  DisplayMode drawn_mode;
  bool force_full_redraw;

  char time_text[40];
  char date_text[40];
  char pps_us_text[40];
  char pps_interval_text[40];
  char epoch_text[40];
  char fraction_text[40];
  char location_lat_text[40];
  char location_lon_text[40];
  char altitude_text[40];
  char countdown_text[40];
  char diagnostic_1[40];
  char diagnostic_2[40];
  char diagnostic_3[40];
  char diagnostic_4[40];
  char diagnostic_5[40];
  char diagnostic_6[40];
};

// ============================================================
// Global Variables
// ============================================================

PPSCapture pps_capture = {0, 0, 0, false};
PPSData pps_data = {
  0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  PPS_WAITING, false, false
};

TimingData timing_data = {
  0, 0, 0, 0, -20, 1000, 5000, false, false, 0
};

GPSData gps_data = {
  GPS_SEARCHING,
  false,
  false,
  false,
  false,
  0,
  0.0,
  0.0,
  0.0,
  0, 0, 0,
  0, 0, 0,
  0, 0, 0, 0
};

ButtonState button_state = {
  BUTTON_IDLE,
  false,
  0,
  0,
  false
};

DisplayCache display_cache;

ScreenManager screen_manager = {
  PRIMARY_SCREEN,
  PRIMARY_SCREEN,
  0,
  false,
  true
};

TaskHandle_t timing_task_handle = nullptr;
TaskHandle_t network_task_handle = nullptr;
TaskHandle_t display_task_handle = nullptr;

portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t last_display_fast_ms = 0;
uint32_t last_display_normal_ms = 0;
uint32_t last_display_slow_ms = 0;
uint32_t last_debug_time_ms = 0;

char ip_address[16] = "192.168.0.123";

// Version 1.0
// These values are displayed locally now and will later be editable through
// the Micro SD hosted web configuration page.
struct NetworkConfig {
  char ip[16];
  char gateway[16];
  char subnet[16];
  char dns[16];
  char hostname[32];
  bool dhcp_enabled;
  bool ethernet_enabled;
  bool ntp_enabled;
  bool web_config_from_sd;
};

NetworkConfig network_config = {
  "192.168.0.123",
  "192.168.0.1",
  "255.255.255.0",
  "192.168.0.1",
  "skytime-p4",
  false,
  false,
  false,
  true
};


// ============================================================
// Ethernet Configuration
// ============================================================

// IMPORTANT:
// PHY settings are isolated here so they can be changed easily if a
// Waveshare board revision or Arduino core requires different values.
//
// The default release build uses ETH.begin() with board-package defaults.
// If explicit PHY parameters are required, set SKYTIME_USE_EXPLICIT_ETH_PHY to 1
// and set the values below.

#define SKYTIME_USE_EXPLICIT_ETH_PHY 0

#if SKYTIME_USE_EXPLICIT_ETH_PHY
  #define SKYTIME_ETH_PHY_ADDR     1
  #define SKYTIME_ETH_PHY_POWER   -1
  #define SKYTIME_ETH_MDC_PIN     31
  #define SKYTIME_ETH_MDIO_PIN    52
  #define SKYTIME_ETH_PHY_TYPE    ETH_PHY_IP101
  #define SKYTIME_ETH_CLK_MODE    ETH_CLOCK_GPIO0_IN
#endif

IPAddress eth_static_ip(192, 168, 0, 123);
IPAddress eth_gateway(192, 168, 0, 1);
IPAddress eth_subnet(255, 255, 255, 0);
IPAddress eth_dns(192, 168, 0, 1);

enum EthernetState {
  ETH_STATE_DISABLED = 0,
  ETH_STATE_STARTING = 1,
  ETH_STATE_LINK_DOWN = 2,
  ETH_STATE_LINK_UP = 3,
  ETH_STATE_GOT_IP = 4,
  ETH_STATE_ERROR = 5
};

struct EthernetRuntime {
  EthernetState state;
  bool started;
  bool link_up;
  bool got_ip;
  bool static_ip_applied;
  uint32_t last_event_ms;
  uint32_t link_up_count;
  uint32_t link_down_count;
  uint32_t got_ip_count;
  char ip[16];
  char gateway[16];
  char subnet[16];
  char mac[24];
};

EthernetRuntime ethernet_runtime = {
  ETH_STATE_DISABLED,
  false,
  false,
  false,
  false,
  0,
  0,
  0,
  0,
  "0.0.0.0",
  "0.0.0.0",
  "0.0.0.0",
  "--:--:--:--:--:--"
};

bool ethernet_init_retry_done = false;


// ============================================================
// NTP UDP Listener
// ============================================================

#define NTP_PORT                 123
#define NTP_PACKET_SIZE          48

WiFiUDP ntp_udp;
WiFiUDP udp4123;

enum NtpServiceState {
  NTP_STATE_DISABLED = 0,
  NTP_STATE_WAIT_ETH = 1,
  NTP_STATE_LISTENING = 2,
  NTP_STATE_ERROR = 3
};

struct NtpRuntime {
  NtpServiceState state;
  bool udp_started;
  bool replies_enabled;
  uint32_t packets_rx;
  uint32_t packets_tx;
  uint32_t packets_ignored;
  uint32_t packets_short;
  uint32_t packets_bad_mode;
  uint32_t packets_not_ready;
  uint32_t last_packet_ms;
  IPAddress last_remote_ip;
  uint16_t last_remote_port;
  uint16_t last_packet_size;
};

NtpRuntime ntp_runtime = {
  NTP_STATE_DISABLED,
  false,
  true,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  IPAddress(0, 0, 0, 0),
  0,
  0
};


struct Udp123Debug {
  uint32_t total_rx;
  uint32_t ntp_rx;
  uint32_t short_rx;
  uint32_t ignored_rx;
  uint16_t last_size;
  uint16_t last_port;
  IPAddress last_ip;
};

Udp123Debug udp123_debug = {
  0,
  0,
  0,
  0,
  0,
  0,
  IPAddress(0, 0, 0, 0)
};


struct Udp4123Debug {
  uint32_t total_rx;
  uint16_t last_size;
  uint16_t last_port;
  IPAddress last_ip;
};

Udp4123Debug udp4123_debug = {
  0,
  0,
  0,
  IPAddress(0, 0, 0, 0)
};




// ============================================================
// MicroSD / TF Card
// ============================================================

#define SD_MOUNT_POINT           "/sdcard"
#define SD_TEST_FILE             "/skytime_sd_test.txt"

// Waveshare ESP32-P4-ETH onboard TF card slot uses SDIO 3.0.
// Start in 4-bit SD_MMC mode. If a specific board/core revision has
// trouble, set SD_FORCE_1BIT_MODE to 1 for diagnostic fallback.
#define SD_FORCE_1BIT_MODE       0

enum SdCardState {
  SD_STATE_DISABLED = 0,
  SD_STATE_STARTING = 1,
  SD_STATE_MOUNTED = 2,
  SD_STATE_NO_CARD = 3,
  SD_STATE_TEST_FAILED = 4,
  SD_STATE_ERROR = 5
};

struct SdRuntime {
  SdCardState state;
  bool mounted;
  bool test_passed;
  uint64_t card_size_bytes;
  uint64_t total_bytes;
  uint64_t used_bytes;
  uint32_t mount_attempts;
  uint32_t test_writes;
  uint32_t test_reads;
  uint32_t errors;
  char card_type[16];
  char last_error[48];
};

SdRuntime sd_runtime = {
  SD_STATE_DISABLED,
  false,
  false,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  "NONE",
  ""
};


// ============================================================
// Configuration Files
// ============================================================

#define CONFIG_NETWORK_FILE      "/config/network.json"
#define CONFIG_SYSTEM_FILE       "/config/system.json"

enum ConfigState {
  CONFIG_STATE_DEFAULTS = 0,
  CONFIG_STATE_LOADED = 1,
  CONFIG_STATE_PARTIAL = 2,
  CONFIG_STATE_ERROR = 3
};

struct SystemConfig {
  char device_name[32];
  char node_id[32];
  char role[24];
  char site_name[40];
  uint32_t screen_timeout_seconds;
  uint32_t holdover_minutes;
  bool debug_serial;
  bool web_enabled;
};

SystemConfig system_config = {
  "SkyTime",
  "SkyTime",
  "Standalone",
  "Enter Location in Configuration",
  30,
  60,
  true,
  false
};

struct ConfigRuntime {
  ConfigState state;
  bool network_loaded;
  bool system_loaded;
  bool defaults_created;
  uint32_t load_attempts;
  uint32_t errors;
  char last_error[64];
};

ConfigRuntime config_runtime = {
  CONFIG_STATE_DEFAULTS,
  false,
  false,
  false,
  0,
  0,
  "defaults"
};


// ============================================================
// Web Server
// ============================================================

#define WEB_SERVER_PORT          80
#define WEB_INDEX_FILE           "/web/index.html"
#define WEB_ENABLE_RAW_8080_DEBUG 0

WebServer web_server(WEB_SERVER_PORT);
#if WEB_ENABLE_RAW_8080_DEBUG
WiFiServer raw_http_debug_server(8080);
#endif

enum WebState {
  WEB_STATE_DISABLED = 0,
  WEB_STATE_WAIT_ETH = 1,
  WEB_STATE_RUNNING = 2,
  WEB_STATE_ERROR = 3
};

struct WebRuntime {
  WebState state;
  bool started;
  bool sd_index_available;
  uint32_t requests_total;
  uint32_t requests_status;
  uint32_t requests_api;
  uint32_t requests_static;
  uint32_t requests_not_found;
  uint32_t raw8080_requests;
  uint32_t handle_ticks;
  uint32_t start_attempts;
  uint32_t errors;
  char last_uri[64];
  char last_error[64];
};

WebRuntime web_runtime = {
  WEB_STATE_DISABLED,
  false,
  false,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  "",
  ""
};


// ============================================================
// Operational Logging
// ============================================================

#define LOG_DIR_PATH       "/logs"
#define NTP_LOG_FILE       "/logs/ntp.log"
#define EVENT_LOG_FILE     "/logs/events.log"
#define LOG_MAX_BYTES      (1024UL * 1024UL)
#define LOG_VIEWER_MAX_LINES 100
#define LOG_VIEWER_MAX_BYTES 12000

struct LogRuntime {
  bool enabled;
  bool log_dir_ready;
  uint32_t ntp_entries;
  uint32_t ntp_errors;
  uint32_t event_entries;
  uint32_t event_errors;
  char last_error[64];
};

LogRuntime log_runtime = {
  true,
  false,
  0,
  0,
  0,
  0,
  "OK"
};


// ============================================================
// Operational Statistics
// ============================================================

struct OperationalStats {
  uint32_t gps_lock_start_ms;
  bool gps_lock_timer_active;
  uint32_t last_ntp_query_epoch;
};

OperationalStats operational_stats = {
  0,
  false,
  0
};

// ============================================================
// Function Prototypes
// ============================================================

void init_systems();
void display_startup();

void process_gps();
void update_gps_health();
void update_pps_state();
void update_pps_quality_stats(uint64_t interval_us);
bool validate_pps_gps_alignment();
void calculate_ntp_timestamp();

void handle_button();
void on_short_press();
void on_long_press();

void screen_manager_init();
void screen_manager_handle_event(ScreenEvent event);
DisplayMode screen_manager_get_current();
uint32_t screen_manager_get_elapsed_ms();
bool screen_manager_take_redraw_required();

void timing_task(void *parameter);
void network_task(void *parameter);
void display_task(void *parameter);

void scheduler_update_display();
void update_display();
void draw_screen_static(DisplayMode mode);
void update_primary_fast_fields();
void update_primary_normal_fields();
void update_network_screen();
void update_diagnostic_screen();

void draw_field_if_changed(
  int16_t x,
  int16_t y,
  int16_t w,
  int16_t h,
  uint8_t text_size,
  uint16_t color,
  const char *new_text,
  char *cache_text
);

void clear_cache();
void init_ethernet();
void WiFiEvent(WiFiEvent_t event);
void update_ethernet_runtime();
const char *ethernet_state_text(EthernetState state);
void init_ntp_listener();
void update_ntp_listener();
const char *ntp_state_text(NtpServiceState state);
void write_ntp_u32(uint8_t *packet, int offset, uint32_t value);
uint32_t us_to_ntp_short(uint32_t microseconds);
void snapshot_ntp_time(uint32_t *seconds, uint32_t *fraction);
bool send_ntp_reply(const uint8_t *request_packet, uint8_t client_version, uint8_t client_mode);



void init_sd_card();
void update_sd_runtime();
bool sd_self_test();
const char *sd_state_text(SdCardState state);
const char *sd_card_type_text(uint8_t card_type);

void init_config();
bool load_config_files();
bool load_network_config();
bool load_system_config();
bool create_default_config_files();
const char *config_state_text(ConfigState state);
bool read_text_file(const char *path, char *buffer, size_t buffer_size);
bool write_text_file(const char *path, const char *content);
bool json_get_string(const char *json, const char *key, char *out, size_t out_size);
bool json_get_bool(const char *json, const char *key, bool *out);
bool json_get_uint32(const char *json, const char *key, uint32_t *out);
bool ip_from_string(const char *text, IPAddress *ip);
void apply_network_config_to_runtime();

void init_web_server();
void update_web_server();
const char *web_state_text(WebState state);

// ============================================================
// Read-Only Log Viewer
// ============================================================

void handle_web_logs_page() {
  web_runtime.requests_total++;
  web_runtime.requests_static++;

  if (!sd_runtime.mounted) {
    web_server.send(503, "text/plain", "SD card not mounted");
    return;
  }

  const char *path = "/web/logs.html";

  if (!SD_MMC.exists(path)) {
    web_server.send(404, "text/plain", "logs.html not found");
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file) {
    web_server.send(500, "text/plain", "Unable to open logs.html");
    return;
  }

  web_server.streamFile(file, "text/html");
  file.close();
}

void handle_api_log_file(const char *path, const char *log_name) {
  web_runtime.requests_total++;
  web_runtime.requests_api++;

  if (!sd_runtime.mounted) {
    web_server.send(503, "application/json", "{\"ok\":false,\"error\":\"SD not mounted\"}");
    return;
  }

  if (!SD_MMC.exists(path)) {
    char missing[192];
    snprintf(
      missing,
      sizeof(missing),
      "{\"ok\":true,\"name\":\"%s\",\"path\":\"%s\",\"lines\":[],\"count\":0,\"message\":\"Log file not found\"}",
      log_name,
      path
    );
    web_server.send(200, "application/json", missing);
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file) {
    web_server.send(500, "application/json", "{\"ok\":false,\"error\":\"Unable to open log\"}");
    return;
  }

  size_t size = file.size();

  if (size > LOG_VIEWER_MAX_BYTES) {
    file.seek(size - LOG_VIEWER_MAX_BYTES);

    while (file.available()) {
      char c = file.read();
      if (c == '\n') {
        break;
      }
    }
  }

  String lines[LOG_VIEWER_MAX_LINES];
  uint16_t line_count = 0;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) {
      continue;
    }

    if (line_count < LOG_VIEWER_MAX_LINES) {
      lines[line_count++] = line;
    } else {
      for (uint16_t i = 1; i < LOG_VIEWER_MAX_LINES; i++) {
        lines[i - 1] = lines[i];
      }
      lines[LOG_VIEWER_MAX_LINES - 1] = line;
    }
  }

  file.close();

  String json;
  json.reserve(LOG_VIEWER_MAX_BYTES + 256);
  json += "{\"ok\":true,\"name\":\"";
  json += log_name;
  json += "\",\"path\":\"";
  json += path;
  json += "\",\"count\":";
  json += String(line_count);
  json += ",\"lines\":[";

  for (int i = (int)line_count - 1; i >= 0; i--) {
    if (i != (int)line_count - 1) {
      json += ",";
    }

    json += "\"";

    for (uint16_t j = 0; j < lines[i].length(); j++) {
      char c = lines[i][j];

      if (c == '"' || c == '\\') {
        json += "\\";
        json += c;
      } else if (c != '\r' && c != '\n') {
        json += c;
      }
    }

    json += "\"";
  }

  json += "]}";

  web_server.send(200, "application/json", json);
}

void handle_api_log_events() {
  handle_api_log_file(EVENT_LOG_FILE, "events");
}

void handle_api_log_ntp() {
  handle_api_log_file(NTP_LOG_FILE, "ntp");
}



// Forward declarations needed by early web/config handlers
bool send_sd_file(const char *path, const char *content_type);
void log_event_line(const char *event, const char *detail);

// ============================================================
// Identity Configuration API
// ============================================================

bool valid_identity_text(const char *text, size_t max_len) {
  if (!text) {
    return false;
  }

  size_t len = strlen(text);

  if (len == 0 || len >= max_len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    char c = text[i];

    bool ok =
      (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') ||
      c == ' ' ||
      c == '-' ||
      c == '_' ||
      c == '.';

    if (!ok) {
      return false;
    }
  }

  return true;
}

void json_escape_string(const char *src, char *dst, size_t dst_size) {
  if (!dst || dst_size == 0) {
    return;
  }

  dst[0] = '\0';

  if (!src) {
    return;
  }

  size_t out = 0;

  for (size_t i = 0; src[i] && out < dst_size - 1; i++) {
    char c = src[i];

    if (c == '"' || c == '\\') {
      if (out + 2 >= dst_size) {
        break;
      }

      dst[out++] = '\\';
      dst[out++] = c;
    } else if (c >= 32 && c <= 126) {
      dst[out++] = c;
    }
  }

  dst[out] = '\0';
}

bool save_system_config_file() {
  if (!sd_runtime.mounted) {
    return false;
  }

  char content[768];

  snprintf(
    content,
    sizeof(content),
    "{\n"
    "  \"device_name\": \"%s\",\n"
    "  \"node_id\": \"%s\",\n"
    "  \"role\": \"%s\",\n"
    "  \"site_name\": \"%s\",\n"
    "  \"screen_timeout\": %lu,\n"
    "  \"holdover_minutes\": %lu,\n"
    "  \"debug_serial\": %s,\n"
    "  \"web_enabled\": %s\n"
    "}\n",
    system_config.device_name,
    system_config.node_id,
    system_config.role,
    system_config.site_name,
    (unsigned long)system_config.screen_timeout_seconds,
    (unsigned long)system_config.holdover_minutes,
    system_config.debug_serial ? "true" : "false",
    system_config.web_enabled ? "true" : "false"
  );

  if (SD_MMC.exists(CONFIG_SYSTEM_FILE)) {
    SD_MMC.remove(CONFIG_SYSTEM_FILE);
  }

  return write_text_file(CONFIG_SYSTEM_FILE, content);
}

void handle_web_config_page() {
  web_runtime.requests_total++;
  web_runtime.requests_static++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  if (!send_sd_file("/web/config.html", "text/html")) {
    web_server.send(404, "text/plain", "config.html not found");
  }
}

void handle_api_config_identity_get() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  char node[80];
  char role[80];
  char site[96];

  json_escape_string(system_config.node_id, node, sizeof(node));
  json_escape_string(system_config.role, role, sizeof(role));
  json_escape_string(system_config.site_name, site, sizeof(site));

  char json[384];

  snprintf(
    json,
    sizeof(json),
    "{\"ok\":true,\"node_id\":\"%s\",\"role\":\"%s\",\"site_name\":\"%s\"}",
    node,
    role,
    site
  );

  web_server.send(200, "application/json", json);
}

void handle_api_config_identity_post() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  String body = web_server.arg("plain");

  if (body.length() == 0) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Missing request body\"}");
    return;
  }

  char json[512];
  snprintf(json, sizeof(json), "%s", body.c_str());

  char node_id[32];
  char role[24];
  char site_name[40];

  if (!json_get_string(json, "node_id", node_id, sizeof(node_id)) ||
      !json_get_string(json, "role", role, sizeof(role)) ||
      !json_get_string(json, "site_name", site_name, sizeof(site_name))) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Missing identity fields\"}");
    return;
  }

  if (!valid_identity_text(node_id, sizeof(system_config.node_id)) ||
      !valid_identity_text(role, sizeof(system_config.role)) ||
      !valid_identity_text(site_name, sizeof(system_config.site_name))) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Invalid characters or field length\"}");
    return;
  }

  snprintf(system_config.node_id, sizeof(system_config.node_id), "%s", node_id);
  snprintf(system_config.role, sizeof(system_config.role), "%s", role);
  snprintf(system_config.site_name, sizeof(system_config.site_name), "%s", site_name);

  if (!save_system_config_file()) {
    web_server.send(500, "application/json", "{\"ok\":false,\"error\":\"Unable to save system.json\"}");
    return;
  }

  char detail[128];
  snprintf(
    detail,
    sizeof(detail),
    "System=%s Role=%s Site=%s",
    system_config.node_id,
    system_config.role,
    system_config.site_name
  );

  log_event_line("CONFIG", detail);

  web_server.send(200, "application/json", "{\"ok\":true,\"message\":\"Identity configuration saved\"}");
}



// ============================================================
// Network Configuration API
// ============================================================

bool valid_hostname_text(const char *text, size_t max_len) {
  if (!text) {
    return false;
  }

  size_t len = strlen(text);

  if (len == 0 || len >= max_len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    char c = text[i];

    bool ok =
      (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') ||
      (c >= '0' && c <= '9') ||
      c == '-' ||
      c == '_';

    if (!ok) {
      return false;
    }
  }

  return true;
}

bool valid_ip_config_text(const char *text) {
  IPAddress ip;
  return ip_from_string(text, &ip);
}

bool save_network_config_file() {
  if (!sd_runtime.mounted) {
    return false;
  }

  char content[640];

  snprintf(
    content,
    sizeof(content),
    "{\n"
    "  \"hostname\": \"%s\",\n"
    "  \"dhcp\": %s,\n"
    "  \"ip\": \"%s\",\n"
    "  \"subnet\": \"%s\",\n"
    "  \"gateway\": \"%s\",\n"
    "  \"dns\": \"%s\",\n"
    "  \"ntp_enabled\": %s\n"
    "}\n",
    network_config.hostname,
    network_config.dhcp_enabled ? "true" : "false",
    network_config.ip,
    network_config.subnet,
    network_config.gateway,
    network_config.dns,
    network_config.ntp_enabled ? "true" : "false"
  );

  if (SD_MMC.exists(CONFIG_NETWORK_FILE)) {
    SD_MMC.remove(CONFIG_NETWORK_FILE);
  }

  return write_text_file(CONFIG_NETWORK_FILE, content);
}

void handle_api_config_network_get() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  char hostname[80];
  char ip[32];
  char subnet[32];
  char gateway[32];
  char dns[32];

  json_escape_string(network_config.hostname, hostname, sizeof(hostname));
  json_escape_string(network_config.ip, ip, sizeof(ip));
  json_escape_string(network_config.subnet, subnet, sizeof(subnet));
  json_escape_string(network_config.gateway, gateway, sizeof(gateway));
  json_escape_string(network_config.dns, dns, sizeof(dns));

  char json[512];

  snprintf(
    json,
    sizeof(json),
    "{\"ok\":true,\"hostname\":\"%s\",\"dhcp\":%s,\"ip\":\"%s\",\"subnet\":\"%s\",\"gateway\":\"%s\",\"dns\":\"%s\",\"ntp_enabled\":%s,\"reboot_required\":true}",
    hostname,
    network_config.dhcp_enabled ? "true" : "false",
    ip,
    subnet,
    gateway,
    dns,
    network_config.ntp_enabled ? "true" : "false"
  );

  web_server.send(200, "application/json", json);
}

void handle_api_config_network_post() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  String body = web_server.arg("plain");

  if (body.length() == 0) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Missing request body\"}");
    return;
  }

  char json[640];
  snprintf(json, sizeof(json), "%s", body.c_str());

  char hostname[32];
  char ip[16];
  char subnet[16];
  char gateway[16];
  char dns[16];
  bool dhcp = false;
  bool ntp_enabled = true;

  if (!json_get_string(json, "hostname", hostname, sizeof(hostname)) ||
      !json_get_bool(json, "dhcp", &dhcp) ||
      !json_get_string(json, "ip", ip, sizeof(ip)) ||
      !json_get_string(json, "subnet", subnet, sizeof(subnet)) ||
      !json_get_string(json, "gateway", gateway, sizeof(gateway)) ||
      !json_get_string(json, "dns", dns, sizeof(dns))) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Missing network fields\"}");
    return;
  }

  json_get_bool(json, "ntp_enabled", &ntp_enabled);

  if (!valid_hostname_text(hostname, sizeof(network_config.hostname)) ||
      !valid_ip_config_text(ip) ||
      !valid_ip_config_text(subnet) ||
      !valid_ip_config_text(gateway) ||
      !valid_ip_config_text(dns)) {
    web_server.send(400, "application/json", "{\"ok\":false,\"error\":\"Invalid hostname or IP address\"}");
    return;
  }

  snprintf(network_config.hostname, sizeof(network_config.hostname), "%s", hostname);
  network_config.dhcp_enabled = dhcp;
  snprintf(network_config.ip, sizeof(network_config.ip), "%s", ip);
  snprintf(network_config.subnet, sizeof(network_config.subnet), "%s", subnet);
  snprintf(network_config.gateway, sizeof(network_config.gateway), "%s", gateway);
  snprintf(network_config.dns, sizeof(network_config.dns), "%s", dns);
  network_config.ntp_enabled = ntp_enabled;

  if (!save_network_config_file()) {
    web_server.send(500, "application/json", "{\"ok\":false,\"error\":\"Unable to save network.json\"}");
    return;
  }

  char detail[160];
  snprintf(
    detail,
    sizeof(detail),
    "Network Saved hostname=%s dhcp=%s ip=%s gateway=%s reboot_required=YES",
    network_config.hostname,
    network_config.dhcp_enabled ? "true" : "false",
    network_config.ip,
    network_config.gateway
  );

  log_event_line("CONFIG", detail);

  web_server.send(200, "application/json", "{\"ok\":true,\"message\":\"Network configuration saved. Reboot required to apply network changes.\",\"reboot_required\":true}");
}



// ============================================================
// System Reboot API - Version 1.0
// ============================================================

void handle_api_system_reboot() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  log_event_line("CONFIG", "System reboot requested from Web UI");

  web_server.send(
    200,
    "application/json",
    "{\"ok\":true,\"message\":\"System reboot requested. SkyTime will reboot shortly.\"}"
  );

  delay(750);
  ESP.restart();
}


void handle_web_root();
void handle_web_status_page();
void handle_web_api_status();
void handle_web_config_page();
void handle_api_config_identity_get();
void handle_api_config_identity_post();
void handle_api_config_network_get();
void handle_api_config_network_post();
void handle_api_system_reboot();
bool save_network_config_file();
bool valid_hostname_text(const char *text, size_t max_len);
bool valid_ip_config_text(const char *text);

bool save_system_config_file();
bool valid_identity_text(const char *text, size_t max_len);
void json_escape_string(const char *src, char *dst, size_t dst_size);

void handle_web_logs_page();
void handle_api_log_events();
void handle_api_log_ntp();
void handle_api_log_file(const char *path, const char *log_name);

void handle_web_ping();
void update_raw_http_debug_server();

void handle_web_ping() {
  web_runtime.requests_total++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());
  web_server.send(200, "text/plain", "pong");
}

#if WEB_ENABLE_RAW_8080_DEBUG
void update_raw_http_debug_server() {
  WiFiClient client = raw_http_debug_server.available();

  if (!client) {
    return;
  }

  web_runtime.raw8080_requests++;

  uint32_t start_ms = millis();

  while (client.connected() && millis() - start_ms < 250UL) {
    while (client.available()) {
      client.read();
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("SkyTime raw TCP debug OK");
    client.print("IP: ");
    client.println(ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip);
    client.print("NTP RX: ");
    client.println((unsigned long)ntp_runtime.packets_rx);
    client.print("NTP TX: ");
    client.println((unsigned long)ntp_runtime.packets_tx);
    client.print("Web state: ");
    client.println(web_state_text(web_runtime.state));
    break;
  }

  delay(1);
  client.stop();

  Serial.printf(
    "[WEB8080] RX #%lu raw TCP debug request\n",
    (unsigned long)web_runtime.raw8080_requests
  );
}

#else
void update_raw_http_debug_server() {
  // Raw TCP debug server disabled for normal Version 1.0
}
#endif


void handle_web_not_found();
bool send_sd_file(const char *path, const char *content_type);
const char *content_type_from_path(const char *path);
void build_status_json(char *buffer, size_t buffer_size);
void build_status_html(char *buffer, size_t buffer_size);

void init_logging();
bool ensure_log_directory();
void format_utc_timestamp(char *buffer, size_t buffer_size);
bool append_log_line(const char *path, const char *line);
void rotate_log_if_needed(const char *path);

void format_log_utc(char *out, size_t out_size) {
  if (!out || out_size == 0) {
    return;
  }

  if (timing_data.current_epoch == 0) {
    snprintf(out, out_size, "0000-00-00 00:00:00 UTC");
    return;
  }

  uint32_t unix_epoch = timing_data.current_epoch - 2208988800UL;
  time_t raw = (time_t)unix_epoch;
  struct tm tm_utc;

  gmtime_r(&raw, &tm_utc);

  snprintf(
    out,
    out_size,
    "%04d-%02d-%02d %02d:%02d:%02d UTC",
    tm_utc.tm_year + 1900,
    tm_utc.tm_mon + 1,
    tm_utc.tm_mday,
    tm_utc.tm_hour,
    tm_utc.tm_min,
    tm_utc.tm_sec
  );
}


void log_event_line(const char *event, const char *detail);
void log_ntp_request(uint8_t version, uint8_t mode, bool tx_ok);

void update_operational_stats();
uint32_t gps_lock_seconds();

void debug_output();

static inline bool isLeapYear(int year);
static inline uint32_t makeTimeUTC(
  int year,
  int month,
  int day,
  int hour,
  int minute,
  int second
);

static inline uint64_t atomic_read_pps_time();
static inline uint32_t atomic_read_pps_count();
static inline uint32_t atomic_read_pps_rejected();
static inline bool atomic_take_pps_triggered();

// ============================================================
// Utility Functions
// ============================================================

static inline bool isLeapYear(int year) {
  return ((year % 4 == 0 && year % 100 != 0) ||
          (year % 400 == 0));
}

static inline uint32_t makeTimeUTC(
  int year,
  int month,
  int day,
  int hour,
  int minute,
  int second
) {
  static const int monthDays[] = {
    31, 28, 31, 30, 31, 30,
    31, 31, 30, 31, 30, 31
  };

  if (year < 1970) year = 1970;
  if (month < 1) month = 1;
  if (month > 12) month = 12;
  if (day < 1) day = 1;
  if (day > 31) day = 31;
  if (hour < 0) hour = 0;
  if (hour > 23) hour = 23;
  if (minute < 0) minute = 0;
  if (minute > 59) minute = 59;
  if (second < 0) second = 0;
  if (second > 59) second = 59;

  uint32_t days = 0;

  for (int y = 1970; y < year; y++) {
    days += isLeapYear(y) ? 366 : 365;
  }

  for (int m = 1; m < month; m++) {
    days += monthDays[m - 1];
    if (m == 2 && isLeapYear(year)) {
      days++;
    }
  }

  days += day - 1;

  uint32_t unix_epoch =
    (((days * 24UL + hour) * 60UL + minute) * 60UL + second);

  return unix_epoch + NTP_UNIX_OFFSET;
}

static inline uint64_t atomic_read_pps_time() {
  uint64_t value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.pps_time_us;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

static inline uint32_t atomic_read_pps_count() {
  uint32_t value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.pps_count;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

static inline uint32_t atomic_read_pps_rejected() {
  uint32_t value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.rejected_edges;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

static inline bool atomic_take_pps_triggered() {
  bool value;
  portENTER_CRITICAL(&pps_mux);
  value = pps_capture.pps_triggered;
  pps_capture.pps_triggered = false;
  portEXIT_CRITICAL(&pps_mux);
  return value;
}

// ============================================================
// Interrupt Handlers
// ============================================================

void IRAM_ATTR pps_interrupt_handler() {
  uint64_t now_us = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&pps_mux);

  uint64_t last_us = pps_capture.pps_time_us;

  if (last_us != 0 && (now_us - last_us) < PPS_ISR_REJECT_US) {
    pps_capture.rejected_edges = pps_capture.rejected_edges + 1;
    portEXIT_CRITICAL_ISR(&pps_mux);
    return;
  }

  pps_capture.pps_time_us = now_us;
  pps_capture.pps_count = pps_capture.pps_count + 1;
  pps_capture.pps_triggered = true;

  portEXIT_CRITICAL_ISR(&pps_mux);
}

void IRAM_ATTR button_interrupt_handler() {
  button_state.irq_pending = true;
}

// ============================================================
// System Initialization
// ============================================================

void init_systems() {
  Serial.print("[I2C] Initializing... ");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  delay(50);
  Serial.println("OK");

  Serial.print("[GFX] Initializing display... ");
  if (!gfx->begin()) {
    Serial.println("FAILED");
    while (1) {
      delay(100);
    }
  }
  gfx->fillScreen(COLOR_BLACK);
  Serial.println("OK");

  Serial.print("[GPS] Initializing Serial2... ");
  Serial2.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(50);
  Serial.println("OK");

  Serial.print("[PPS] Initializing GPIO46... ");
  pinMode(PPS_IN_PIN, INPUT_PULLDOWN);
  attachInterrupt(
    digitalPinToInterrupt(PPS_IN_PIN),
    pps_interrupt_handler,
    RISING
  );
  Serial.println("OK");

  Serial.print("[BUTTON] Initializing GPIO47... ");
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(BUTTON_PIN),
    button_interrupt_handler,
    FALLING
  );
  Serial.println("OK");

  Serial.print("[GPIO] Setting backlight... ");
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);
  Serial.println("OK");

  clear_cache();

  Serial.println();
  Serial.println("========================================");
  Serial.println(" SkyTime GPS/PPS Timing Base");
  Serial.println(" Version 1.0");
  Serial.println(" Identity Configuration");
  Serial.println("========================================");
  Serial.println();
}

void display_startup() {
  gfx->fillScreen(COLOR_BLACK);

  gfx->setTextSize(3);
  gfx->setTextColor(COLOR_CYAN, COLOR_BLACK);
  gfx->setCursor(30, 45);
  gfx->println("SkyTime");

  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_WHITE, COLOR_BLACK);
  gfx->setCursor(38, 92);
  gfx->println("Version 1.0");

  gfx->setTextSize(1);
  gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
  gfx->setCursor(18, 135);
  gfx->println("GPS/PPS NTP Server");

  gfx->setCursor(18, 155);
  gfx->println("Config Enabled");

  gfx->setCursor(18, 190);
  gfx->println("Short: screen   Long: reboot");

  delay(1500);

  display_cache.force_full_redraw = true;
}

// ============================================================
// GPS Processing
// ============================================================

void process_gps() {
  bool sentence_updated = false;

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (gps.encode(c)) {
      sentence_updated = true;
    }
  }

  if (!sentence_updated) {
    update_gps_health();
    return;
  }

  uint32_t now_ms = millis();
  gps_data.last_update_ms = now_ms;

  gps_data.time_valid = gps.time.isValid();
  gps_data.date_valid = gps.date.isValid();
  gps_data.location_valid = gps.location.isValid();

  if (gps.satellites.isValid()) {
    gps_data.satellites = gps.satellites.value();
  }

  if (gps.location.isValid()) {
    gps_data.latitude = gps.location.lat();
    gps_data.longitude = gps.location.lng();
    gps_data.last_location_update_ms = now_ms;
  }

  if (gps.altitude.isValid()) {
    gps_data.altitude = gps.altitude.meters();
  }

  if (gps.date.isValid()) {
    gps_data.year = gps.date.year();
    gps_data.month = gps.date.month();
    gps_data.day = gps.date.day();
  }

  if (gps.time.isValid()) {
    gps_data.hour = gps.time.hour();
    gps_data.minute = gps.time.minute();
    gps_data.second = gps.time.second();
    gps_data.last_time_update_ms = now_ms;
  }

  update_gps_health();
}

void update_gps_health() {
  uint32_t now_ms = millis();

  bool time_fresh =
    gps_data.last_time_update_ms > 0 &&
    (now_ms - gps_data.last_time_update_ms) <= GPS_STALE_MS;

  bool fix_good =
    gps_data.time_valid &&
    gps_data.date_valid &&
    gps_data.satellites >= GPS_LOCK_MIN_SATS &&
    time_fresh;

  if (fix_good) {
    gps_data.locked = true;
    gps_data.state = GPS_LOCKED;
    gps_data.last_locked_ms = now_ms;

    timing_data.holdover = false;
    timing_data.holdover_start_ms = 0;
    return;
  }

  bool pps_usable =
    pps_data.edge_seen &&
    pps_data.current_pps_time_us > 0 &&
    pps_data.state != PPS_TIMEOUT;

  bool holdover_allowed =
    gps_data.last_locked_ms > 0 &&
    (now_ms - gps_data.last_locked_ms) <= GPS_HOLDOVER_MAX_MS &&
    pps_usable &&
    pps_data.last_pps_epoch > 0;

  if (holdover_allowed) {
    gps_data.locked = false;
    gps_data.state = GPS_HOLDOVER;

    if (!timing_data.holdover) {
      timing_data.holdover = true;
      timing_data.holdover_start_ms = now_ms;
    }
  } else if (gps_data.last_update_ms > 0 &&
             (now_ms - gps_data.last_update_ms) > GPS_STALE_MS) {
    gps_data.locked = false;
    gps_data.state = GPS_STALE;
    timing_data.holdover = false;
  } else {
    gps_data.locked = false;
    gps_data.state = GPS_SEARCHING;
    timing_data.holdover = false;
  }
}

// ============================================================
// PPS / Timing Core
// ============================================================

void update_pps_quality_stats(uint64_t interval_us) {
  if (interval_us == 0) {
    return;
  }

  if (pps_data.min_interval_us == 0 || interval_us < pps_data.min_interval_us) {
    pps_data.min_interval_us = interval_us;
  }

  if (interval_us > pps_data.max_interval_us) {
    pps_data.max_interval_us = interval_us;
  }

  if (pps_data.avg_interval_us == 0) {
    pps_data.avg_interval_us = interval_us;
  } else {
    // Lightweight IIR average to avoid overflow over long runtimes.
    pps_data.avg_interval_us =
      ((pps_data.avg_interval_us * 15ULL) + interval_us) / 16ULL;
  }

  uint64_t error_us =
    (interval_us > 1000000ULL) ?
    (interval_us - 1000000ULL) :
    (1000000ULL - interval_us);

  pps_data.jitter_us = error_us;
  pps_data.jitter_accum_us += error_us;
  pps_data.jitter_samples++;
}

bool validate_pps_gps_alignment() {
  if (!gps_data.time_valid || !gps_data.date_valid) {
    pps_data.gps_aligned = false;
    pps_data.align_bad_count++;
    return false;
  }

  if (gps_data.last_time_update_ms == 0) {
    pps_data.gps_aligned = false;
    pps_data.align_bad_count++;
    return false;
  }

  uint32_t age_ms = millis() - gps_data.last_time_update_ms;

  if (age_ms > PPS_GPS_ALIGN_MAX_AGE_MS) {
    pps_data.gps_aligned = false;
    pps_data.align_bad_count++;
    return false;
  }

  pps_data.gps_aligned = true;
  pps_data.align_good_count++;
  return true;
}

void update_pps_state() {
  bool new_pps = atomic_take_pps_triggered();
  uint64_t now_us = esp_timer_get_time();

  if (new_pps) {
    pps_data.previous_pps_time_us = pps_data.current_pps_time_us;
    pps_data.current_pps_time_us = atomic_read_pps_time();
    pps_data.pps_count = atomic_read_pps_count();
    pps_data.edge_seen = true;

    if (pps_data.previous_pps_time_us > 0) {
      pps_data.last_interval_us =
        pps_data.current_pps_time_us - pps_data.previous_pps_time_us;

      update_pps_quality_stats(pps_data.last_interval_us);

      if (pps_data.last_interval_us >= PPS_INTERVAL_MIN_US &&
          pps_data.last_interval_us <= PPS_INTERVAL_MAX_US) {
        pps_data.state = PPS_LOCKED;
        pps_data.valid_count++;
      } else {
        pps_data.state = PPS_BAD_INTERVAL;
        pps_data.bad_count++;
      }
    } else {
      pps_data.state = PPS_WAITING;
    }

    if ((gps_data.locked || gps_data.state == GPS_HOLDOVER) &&
        gps_data.date_valid &&
        gps_data.time_valid) {
      if (validate_pps_gps_alignment()) {
        // NMEA UTC sentence time lags the PPS edge by one second.
        // PPS marks the start of the next UTC second, so latch +1.
        pps_data.last_pps_epoch = makeTimeUTC(
          gps_data.year,
          gps_data.month,
          gps_data.day,
          gps_data.hour,
          gps_data.minute,
          gps_data.second
        ) + 1;
      } else if (pps_data.last_pps_epoch == 0) {
        pps_data.state = PPS_ALIGN_WAIT;
      } else {
        pps_data.state = PPS_ALIGN_BAD;
      }
    }
  }

  if (pps_data.edge_seen &&
      pps_data.current_pps_time_us > 0 &&
      (now_us - pps_data.current_pps_time_us) > PPS_TIMEOUT_US) {
    pps_data.state = PPS_TIMEOUT;
    timing_data.disciplined = false;
  }
}

void calculate_ntp_timestamp() {
  if (pps_data.last_pps_epoch == 0 ||
      pps_data.current_pps_time_us == 0 ||
      pps_data.state == PPS_TIMEOUT) {
    timing_data.disciplined = false;
    return;
  }

  uint64_t now_us = esp_timer_get_time();
  uint64_t delta_us = now_us - pps_data.current_pps_time_us;

  uint32_t elapsed_seconds = delta_us / 1000000ULL;
  uint32_t fractional_us = delta_us % 1000000ULL;

  timing_data.current_epoch =
    pps_data.last_pps_epoch + elapsed_seconds;

  timing_data.current_time_us = now_us;
  timing_data.microseconds_since_pps = fractional_us;

  timing_data.ntp_fraction =
    ((uint64_t)fractional_us << 32) / 1000000ULL;

  timing_data.ntp_precision = -20;
  timing_data.root_delay = 1000;

  if (pps_data.state == PPS_LOCKED && gps_data.locked) {
    timing_data.root_dispersion = 5000;
    timing_data.disciplined = true;
    timing_data.holdover = false;
  } else if (pps_data.state == PPS_LOCKED &&
             gps_data.state == GPS_HOLDOVER &&
             pps_data.last_pps_epoch > 0) {
    uint32_t holdover_age_s =
      timing_data.holdover_start_ms > 0 ?
      ((millis() - timing_data.holdover_start_ms) / 1000UL) :
      0;

    timing_data.root_dispersion =
      5000UL + (holdover_age_s * 20UL);

    timing_data.disciplined = true;
    timing_data.holdover = true;
  } else {
    timing_data.root_dispersion = 50000;
    timing_data.disciplined = false;
  }
}


// ============================================================
// Screen Manager
// ============================================================

void screen_manager_init() {
  portENTER_CRITICAL(&state_mux);
  screen_manager.current = PRIMARY_SCREEN;
  screen_manager.previous = PRIMARY_SCREEN;
  screen_manager.entered_ms = millis();
  screen_manager.timed_screen = false;
  screen_manager.redraw_required = true;
  portEXIT_CRITICAL(&state_mux);
}

void screen_manager_handle_event(ScreenEvent event) {
  uint32_t now_ms = millis();

  portENTER_CRITICAL(&state_mux);

  if (event == SCREEN_EVENT_FORCE_MAIN) {
    screen_manager.previous = screen_manager.current;
    screen_manager.current = PRIMARY_SCREEN;
    screen_manager.entered_ms = now_ms;
    screen_manager.timed_screen = false;
    screen_manager.redraw_required = true;
    portEXIT_CRITICAL(&state_mux);
    return;
  }

  if (event == SCREEN_EVENT_TIMEOUT) {
    if (screen_manager.timed_screen) {
      screen_manager.previous = screen_manager.current;
      screen_manager.current = PRIMARY_SCREEN;
      screen_manager.entered_ms = now_ms;
      screen_manager.timed_screen = false;
      screen_manager.redraw_required = true;
    }
    portEXIT_CRITICAL(&state_mux);
    return;
  }

  if (event == SCREEN_EVENT_SHORT_PRESS) {
    screen_manager.previous = screen_manager.current;

    if (screen_manager.current == PRIMARY_SCREEN) {
      screen_manager.current = NETWORK_SCREEN;
      screen_manager.timed_screen = true;
    } else if (screen_manager.current == NETWORK_SCREEN) {
      screen_manager.current = DIAGNOSTIC_SCREEN;
      screen_manager.timed_screen = true;
    } else {
      screen_manager.current = PRIMARY_SCREEN;
      screen_manager.timed_screen = false;
    }

    screen_manager.entered_ms = now_ms;
    screen_manager.redraw_required = true;
  }

  portEXIT_CRITICAL(&state_mux);
}

DisplayMode screen_manager_get_current() {
  DisplayMode mode;
  portENTER_CRITICAL(&state_mux);
  mode = screen_manager.current;
  portEXIT_CRITICAL(&state_mux);
  return mode;
}

uint32_t screen_manager_get_elapsed_ms() {
  uint32_t entered;
  portENTER_CRITICAL(&state_mux);
  entered = screen_manager.entered_ms;
  portEXIT_CRITICAL(&state_mux);
  return millis() - entered;
}

bool screen_manager_take_redraw_required() {
  bool value;
  portENTER_CRITICAL(&state_mux);
  value = screen_manager.redraw_required;
  screen_manager.redraw_required = false;
  portEXIT_CRITICAL(&state_mux);
  return value;
}

// ============================================================
// Button Handling
// ============================================================

void handle_button() {
  uint32_t now_ms = millis();
  bool level_low = (digitalRead(BUTTON_PIN) == LOW);

  if (button_state.irq_pending &&
      button_state.state == BUTTON_IDLE) {
    button_state.irq_pending = false;
    button_state.state = BUTTON_DEBOUNCE_PRESS;
    button_state.transition_ms = now_ms;
  }

  switch (button_state.state) {
    case BUTTON_IDLE:
      break;

    case BUTTON_DEBOUNCE_PRESS:
      if ((now_ms - button_state.transition_ms) >= BUTTON_DEBOUNCE_MS) {
        if (level_low) {
          button_state.state = BUTTON_HELD;
          button_state.press_start_ms = now_ms;
          button_state.long_press_reported = false;
        } else {
          button_state.state = BUTTON_IDLE;
        }
      }
      break;

    case BUTTON_HELD:
      if (!level_low) {
        button_state.state = BUTTON_DEBOUNCE_RELEASE;
        button_state.transition_ms = now_ms;
      } else if (!button_state.long_press_reported &&
                 (now_ms - button_state.press_start_ms) >= LONG_PRESS_TIME_MS) {
        button_state.long_press_reported = true;
        on_long_press();
      }
      break;

    case BUTTON_DEBOUNCE_RELEASE:
      if ((now_ms - button_state.transition_ms) >= BUTTON_DEBOUNCE_MS) {
        if (!level_low) {
          if (!button_state.long_press_reported) {
            on_short_press();
          }
          button_state.state = BUTTON_IDLE;
        } else {
          button_state.state = BUTTON_HELD;
        }
      }
      break;
  }
}

void on_short_press() {
  screen_manager_handle_event(SCREEN_EVENT_SHORT_PRESS);
  display_cache.force_full_redraw = true;
}

void on_long_press() {
  gfx->fillScreen(COLOR_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(COLOR_RED, COLOR_BLACK);
  gfx->setCursor(50, 110);
  gfx->println("REBOOTING...");
  delay(500);
  ESP.restart();
}

// ============================================================
// Display Helpers
// ============================================================

void clear_cache() {
  memset(&display_cache, 0, sizeof(display_cache));
  display_cache.active_mode = PRIMARY_SCREEN;
  display_cache.drawn_mode = PRIMARY_SCREEN;
  display_cache.force_full_redraw = true;
}

void draw_field_if_changed(
  int16_t x,
  int16_t y,
  int16_t w,
  int16_t h,
  uint8_t text_size,
  uint16_t color,
  const char *new_text,
  char *cache_text
) {
  if (!display_cache.force_full_redraw &&
      strcmp(new_text, cache_text) == 0) {
    return;
  }

  gfx->fillRect(x, y, w, h, COLOR_BLACK);
  gfx->setTextSize(text_size);
  gfx->setTextColor(color, COLOR_BLACK);
  gfx->setCursor(x, y);
  gfx->print(new_text);

  strncpy(cache_text, new_text, 39);
  cache_text[39] = '\0';
}

void draw_screen_static(DisplayMode mode) {
  gfx->fillScreen(COLOR_BLACK);

  if (mode == PRIMARY_SCREEN) {
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);

    gfx->setCursor(8, 22);
    gfx->print("Satellites");

    gfx->setCursor(8, 54);
    gfx->print("PPS");

    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
    gfx->setCursor(5, 218);
    gfx->print("Short : Screens  | Long : Reboot");
  }

  if (mode == NETWORK_SCREEN) {
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_CYAN, COLOR_BLACK);
    gfx->setCursor(55, 20);
    gfx->print("NETWORK");

    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
    gfx->setCursor(10, 210);
    gfx->print("Short: Diag | Long: Reboot");
  }

  if (mode == DIAGNOSTIC_SCREEN) {
    gfx->setTextSize(2);
    gfx->setTextColor(COLOR_CYAN, COLOR_BLACK);
    gfx->setCursor(30, 15);
    gfx->print("DIAGNOSTICS");

    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_LIGHTGRAY, COLOR_BLACK);
    gfx->setCursor(10, 220);
    gfx->print("Short: Main | Long: Reboot");
  }

  display_cache.drawn_mode = mode;
  display_cache.force_full_redraw = true;

  memset(display_cache.time_text, 0, sizeof(display_cache.time_text));
  memset(display_cache.date_text, 0, sizeof(display_cache.date_text));
  memset(display_cache.pps_us_text, 0, sizeof(display_cache.pps_us_text));
  memset(display_cache.pps_interval_text, 0, sizeof(display_cache.pps_interval_text));
  memset(display_cache.epoch_text, 0, sizeof(display_cache.epoch_text));
  memset(display_cache.fraction_text, 0, sizeof(display_cache.fraction_text));
  memset(display_cache.location_lat_text, 0, sizeof(display_cache.location_lat_text));
  memset(display_cache.location_lon_text, 0, sizeof(display_cache.location_lon_text));
  memset(display_cache.altitude_text, 0, sizeof(display_cache.altitude_text));
  memset(display_cache.countdown_text, 0, sizeof(display_cache.countdown_text));
  memset(display_cache.diagnostic_1, 0, sizeof(display_cache.diagnostic_1));
  memset(display_cache.diagnostic_2, 0, sizeof(display_cache.diagnostic_2));
  memset(display_cache.diagnostic_3, 0, sizeof(display_cache.diagnostic_3));
  memset(display_cache.diagnostic_4, 0, sizeof(display_cache.diagnostic_4));
  memset(display_cache.diagnostic_5, 0, sizeof(display_cache.diagnostic_5));
  memset(display_cache.diagnostic_6, 0, sizeof(display_cache.diagnostic_6));
}

void update_primary_fast_fields() {
  char text[32];

  snprintf(
    text,
    sizeof(text),
    "%lu us",
    (unsigned long)timing_data.microseconds_since_pps
  );

  draw_field_if_changed(
    5,
    180,
    230,
    16,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.pps_us_text
  );
}

void update_primary_normal_fields() {
  char text[40];

  snprintf(
    text,
    sizeof(text),
    "%u",
    gps_data.satellites
  );

  draw_field_if_changed(
    145,
    22,
    36,
    22,
    2,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_1
  );

  uint16_t gps_dot_color = COLOR_RED;
  if (gps_data.state == GPS_LOCKED) {
    gps_dot_color = COLOR_GREEN;
  } else if (gps_data.state == GPS_HOLDOVER) {
    gps_dot_color = COLOR_ORANGE;
  } else if (gps_data.satellites > 0) {
    gps_dot_color = COLOR_YELLOW;
  }

  static GPSLockState last_dot_state = GPS_STALE;
  static uint8_t last_dot_sats = 255;

  if (display_cache.force_full_redraw ||
      last_dot_state != gps_data.state ||
      last_dot_sats != gps_data.satellites) {
    // Clear both the old and new dot regions in case a previous build left
    // the indicator lower on the line.
    gfx->fillRect(188, 18, 32, 44, COLOR_BLACK);

    // Dot aligned with the shifted Satellites line.
    gfx->fillCircle(202, 32, 7, gps_dot_color);
    gfx->drawCircle(202, 32, 7, COLOR_WHITE);

    last_dot_state = gps_data.state;
    last_dot_sats = gps_data.satellites;
  }

  const char *pps_text = "WAIT";
  uint16_t pps_color = COLOR_YELLOW;

  if (pps_data.state == PPS_LOCKED) {
    pps_text = "LOCKED";
    pps_color = COLOR_GREEN;
  } else if (pps_data.state == PPS_BAD_INTERVAL) {
    pps_text = "BAD";
    pps_color = COLOR_YELLOW;
  } else if (pps_data.state == PPS_TIMEOUT) {
    pps_text = "LOST";
    pps_color = COLOR_RED;
  } else if (pps_data.state == PPS_ALIGN_WAIT) {
    pps_text = "ALIGN";
    pps_color = COLOR_ORANGE;
  } else if (pps_data.state == PPS_ALIGN_BAD) {
    pps_text = "GPS?";
    pps_color = COLOR_ORANGE;
  }

  draw_field_if_changed(
    105,
    54,
    120,
    22,
    2,
    pps_color,
    pps_text,
    display_cache.diagnostic_2
  );

  snprintf(
    text,
    sizeof(text),
    "%02d:%02d:%02d",
    gps_data.hour,
    gps_data.minute,
    gps_data.second
  );

  draw_field_if_changed(
    36,
    94,
    175,
    30,
    3,
    COLOR_CYAN,
    text,
    display_cache.time_text
  );

  snprintf(
    text,
    sizeof(text),
    "%02d/%02d/%04d",
    gps_data.month,
    gps_data.day,
    gps_data.year
  );

  draw_field_if_changed(
    58,
    137,
    140,
    22,
    2,
    COLOR_WHITE,
    text,
    display_cache.date_text
  );
}

void update_network_screen() {
  char text[40];

  snprintf(text, sizeof(text), "Ethernet: %s",
           ethernet_state_text(ethernet_runtime.state));
  draw_field_if_changed(
    10,
    58,
    220,
    14,
    1,
    ethernet_runtime.got_ip ? COLOR_GREEN :
      (ethernet_runtime.link_up ? COLOR_YELLOW : COLOR_RED),
    text,
    display_cache.diagnostic_1
  );

  snprintf(text, sizeof(text), "IP: %s",
           ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip);
  draw_field_if_changed(
    10,
    78,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.location_lat_text
  );

  snprintf(text, sizeof(text), "GW: %s",
           ethernet_runtime.got_ip ? ethernet_runtime.gateway : network_config.gateway);
  draw_field_if_changed(
    10,
    98,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.location_lon_text
  );

  snprintf(text, sizeof(text), "Mask: /25");
  draw_field_if_changed(
    10,
    118,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.altitude_text
  );

  snprintf(text, sizeof(text), "System: %s",
           system_config.node_id);
  draw_field_if_changed(
    10,
    138,
    220,
    14,
    1,
    COLOR_CYAN,
    text,
    display_cache.diagnostic_2
  );

  snprintf(text, sizeof(text), "NTP RX:%lu TX:%lu",
           (unsigned long)udp123_debug.ntp_rx,
           (unsigned long)ntp_runtime.packets_tx);
  draw_field_if_changed(
    10,
    158,
    220,
    14,
    1,
    udp123_debug.ntp_rx > 0 ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.diagnostic_3
  );

  snprintf(text, sizeof(text), "Web:%s Cfg:%s",
           web_state_text(web_runtime.state),
           config_state_text(config_runtime.state));
  draw_field_if_changed(
    10,
    174,
    220,
    14,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.diagnostic_4
  );

  snprintf(text, sizeof(text), "SD: %s %s",
           sd_state_text(sd_runtime.state),
           sd_runtime.card_type);
  draw_field_if_changed(
    10,
    190,
    220,
    14,
    1,
    sd_runtime.state == SD_STATE_MOUNTED ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.diagnostic_4
  );

  uint32_t elapsed = screen_manager_get_elapsed_ms();
  uint32_t screen_timeout_ms = system_config.screen_timeout_seconds * 1000UL;
  uint32_t remaining =
    elapsed >= screen_timeout_ms ?
    0 :
    screen_timeout_ms - elapsed;

  snprintf(
    text,
    sizeof(text),
    "Auto-return: %lus",
    (unsigned long)(remaining / 1000UL)
  );

  draw_field_if_changed(
    10,
    208,
    220,
    14,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.countdown_text
  );
}

void update_diagnostic_screen() {
  char text[40];

  snprintf(
    text,
    sizeof(text),
    "GPS: %s SAT:%u",
    gps_data.state == GPS_LOCKED ? "LOCK" :
      (gps_data.state == GPS_HOLDOVER ? "HOLD" :
       (gps_data.state == GPS_STALE ? "STALE" : "SEARCH")),
    gps_data.satellites
  );
  draw_field_if_changed(
    10,
    52,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_1
  );

  snprintf(
    text,
    sizeof(text),
    "PPS count: %lu",
    (unsigned long)pps_data.pps_count
  );
  draw_field_if_changed(
    10,
    72,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_2
  );

  snprintf(
    text,
    sizeof(text),
    "PPS interval: %lu us",
    (unsigned long)pps_data.last_interval_us
  );
  draw_field_if_changed(
    10,
    92,
    220,
    14,
    1,
    pps_data.state == PPS_LOCKED ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.pps_interval_text
  );

  snprintf(
    text,
    sizeof(text),
    "NTP epoch: %lu",
    (unsigned long)timing_data.current_epoch
  );
  draw_field_if_changed(
    10,
    112,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.epoch_text
  );

  snprintf(
    text,
    sizeof(text),
    "NTP frac: 0x%08lX",
    (unsigned long)timing_data.ntp_fraction
  );
  draw_field_if_changed(
    10,
    132,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.fraction_text
  );

  snprintf(
    text,
    sizeof(text),
    "Good:%lu Bad:%lu Rj:%lu",
    (unsigned long)pps_data.valid_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long)atomic_read_pps_rejected()
  );
  draw_field_if_changed(
    10,
    152,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_3
  );

  snprintf(
    text,
    sizeof(text),
    "Jit:%lu Avg:%lu",
    (unsigned long)pps_data.jitter_us,
    (unsigned long)pps_data.avg_interval_us
  );
  draw_field_if_changed(
    10,
    172,
    220,
    14,
    1,
    COLOR_WHITE,
    text,
    display_cache.diagnostic_4
  );

  snprintf(
    text,
    sizeof(text),
    "Align G:%lu B:%lu",
    (unsigned long)pps_data.align_good_count,
    (unsigned long)pps_data.align_bad_count
  );
  draw_field_if_changed(
    10,
    188,
    220,
    14,
    1,
    pps_data.gps_aligned ? COLOR_GREEN : COLOR_YELLOW,
    text,
    display_cache.diagnostic_5
  );

  uint32_t elapsed = screen_manager_get_elapsed_ms();
  uint32_t screen_timeout_ms = system_config.screen_timeout_seconds * 1000UL;
  uint32_t remaining =
    elapsed >= screen_timeout_ms ?
    0 :
    screen_timeout_ms - elapsed;

  snprintf(
    text,
    sizeof(text),
    "Auto-return: %lus",
    (unsigned long)(remaining / 1000UL)
  );
  draw_field_if_changed(
    10,
    205,
    220,
    14,
    1,
    COLOR_LIGHTGRAY,
    text,
    display_cache.countdown_text
  );
}

void update_display() {
  DisplayMode mode = screen_manager_get_current();

  if (mode != PRIMARY_SCREEN &&
      screen_manager_get_elapsed_ms() >= (system_config.screen_timeout_seconds * 1000UL)) {
    screen_manager_handle_event(SCREEN_EVENT_TIMEOUT);
    mode = screen_manager_get_current();
    display_cache.force_full_redraw = true;
  }

  if (screen_manager_take_redraw_required()) {
    display_cache.force_full_redraw = true;
  }

  if (display_cache.force_full_redraw ||
      display_cache.drawn_mode != mode) {
    draw_screen_static(mode);
  }

  if (mode == PRIMARY_SCREEN) {
    update_primary_normal_fields();
    update_primary_fast_fields();
  } else if (mode == NETWORK_SCREEN) {
    update_network_screen();
  } else {
    update_diagnostic_screen();
  }

  display_cache.force_full_redraw = false;
}

void scheduler_update_display() {
  uint32_t now_ms = millis();

  static GPSLockState last_seen_gps_state = GPS_STALE;
  static PPSLockState last_seen_pps_state = PPS_TIMEOUT;
  static uint8_t last_seen_sats = 255;

  if (gps_data.state != last_seen_gps_state ||
      pps_data.state != last_seen_pps_state ||
      gps_data.satellites != last_seen_sats) {
    last_seen_gps_state = gps_data.state;
    last_seen_pps_state = pps_data.state;
    last_seen_sats = gps_data.satellites;
    display_cache.force_full_redraw = true;
  }

  DisplayMode current_mode = screen_manager_get_current();

  if (current_mode == PRIMARY_SCREEN) {
    if (now_ms - last_display_fast_ms >= DISPLAY_FAST_MS) {
      last_display_fast_ms = now_ms;

      if (display_cache.drawn_mode == PRIMARY_SCREEN &&
          !display_cache.force_full_redraw) {
        update_primary_fast_fields();
      }
    }

    if (now_ms - last_display_normal_ms >= DISPLAY_NORMAL_MS ||
        display_cache.force_full_redraw) {
      last_display_normal_ms = now_ms;
      update_display();
    }
  } else {
    if (now_ms - last_display_slow_ms >= DISPLAY_SLOW_MS ||
        display_cache.force_full_redraw) {
      last_display_slow_ms = now_ms;
      update_display();
    }
  }
}


// ============================================================
// Ethernet Bring-Up / Runtime
// ============================================================

const char *ethernet_state_text(EthernetState state) {
  switch (state) {
    case ETH_STATE_DISABLED:
      return "DISABLED";
    case ETH_STATE_STARTING:
      return "STARTING";
    case ETH_STATE_LINK_DOWN:
      return "LINK DOWN";
    case ETH_STATE_LINK_UP:
      return "LINK UP";
    case ETH_STATE_GOT_IP:
      return "UP";
    case ETH_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

void WiFiEvent(WiFiEvent_t event) {
  ethernet_runtime.last_event_ms = millis();

  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ethernet_runtime.started = true;
      ethernet_runtime.state = ETH_STATE_STARTING;
      ETH.setHostname(network_config.hostname);
      Serial.println("[ETH] Started");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      ethernet_runtime.link_up = true;
      ethernet_runtime.link_up_count++;
      ethernet_runtime.state = ETH_STATE_LINK_UP;
      Serial.println("[ETH] Link up");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
      ethernet_runtime.got_ip = true;
      ethernet_runtime.link_up = true;
      ethernet_runtime.got_ip_count++;
      ethernet_runtime.state = ETH_STATE_GOT_IP;

      snprintf(ethernet_runtime.ip, sizeof(ethernet_runtime.ip), "%s",
               ETH.localIP().toString().c_str());
      snprintf(ethernet_runtime.gateway, sizeof(ethernet_runtime.gateway), "%s",
               ETH.gatewayIP().toString().c_str());
      snprintf(ethernet_runtime.subnet, sizeof(ethernet_runtime.subnet), "%s",
               ETH.subnetMask().toString().c_str());
      snprintf(ethernet_runtime.mac, sizeof(ethernet_runtime.mac), "%s",
               ETH.macAddress().c_str());

      network_config.ethernet_enabled = true;

      Serial.print("[ETH] IP: ");
      Serial.println(ETH.localIP());
      Serial.print("[ETH] Gateway: ");
      Serial.println(ETH.gatewayIP());
      Serial.print("[ETH] Subnet: ");
      Serial.println(ETH.subnetMask());
      Serial.print("[ETH] MAC: ");
      Serial.println(ETH.macAddress());
      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      ethernet_runtime.link_up = false;
      ethernet_runtime.got_ip = false;
      ethernet_runtime.link_down_count++;
      ethernet_runtime.state = ETH_STATE_LINK_DOWN;
      network_config.ethernet_enabled = false;
      Serial.println("[ETH] Link down");
      break;

    case ARDUINO_EVENT_ETH_STOP:
      ethernet_runtime.started = false;
      ethernet_runtime.link_up = false;
      ethernet_runtime.got_ip = false;
      ethernet_runtime.state = ETH_STATE_DISABLED;
      network_config.ethernet_enabled = false;
      Serial.println("[ETH] Stopped");
      break;

    default:
      break;
  }
}

void init_ethernet() {
  Serial.println("[ETH] Initializing Ethernet bring-up");

  ethernet_runtime.state = ETH_STATE_STARTING;
  ethernet_runtime.started = false;
  ethernet_runtime.link_up = false;
  ethernet_runtime.got_ip = false;
  ethernet_runtime.static_ip_applied = false;
  ethernet_runtime.last_event_ms = millis();

  WiFi.onEvent(WiFiEvent);

#if SKYTIME_USE_EXPLICIT_ETH_PHY
  bool begin_ok = ETH.begin(
    SKYTIME_ETH_PHY_TYPE,
    SKYTIME_ETH_PHY_ADDR,
    SKYTIME_ETH_MDC_PIN,
    SKYTIME_ETH_MDIO_PIN,
    SKYTIME_ETH_PHY_POWER,
    SKYTIME_ETH_CLK_MODE
  );
#else
  bool begin_ok = ETH.begin();
#endif

  Serial.printf("[ETH] begin_ok=%s\n", begin_ok ? "YES" : "NO");

  if (!begin_ok) {
    ethernet_runtime.state = ETH_STATE_ERROR;
    network_config.ethernet_enabled = false;
    Serial.println("[ETH] ETH.begin() failed");
    return;
  }

  // In Arduino-ESP32, static IP is most reliable when applied after ETH.begin().
  if (!network_config.dhcp_enabled) {
    bool config_ok = ETH.config(
      eth_static_ip,
      eth_gateway,
      eth_subnet,
      eth_dns,
      eth_dns
    );

    ethernet_runtime.static_ip_applied = config_ok;

    if (config_ok) {
      Serial.println("[ETH] Static IP config applied");
      Serial.print("[ETH] Static IP: ");
      Serial.println(eth_static_ip);
    } else {
      Serial.println("[ETH] Static IP config failed");
    }
  }

  network_config.ethernet_enabled = true;
}

void update_ethernet_runtime() {
  if (!ethernet_runtime.started) {
    return;
  }

  bool link = ETH.linkUp();

  if (link != ethernet_runtime.link_up) {
    ethernet_runtime.link_up = link;
    ethernet_runtime.last_event_ms = millis();

    if (link) {
      ethernet_runtime.link_up_count++;
      if (!ethernet_runtime.got_ip) {
        ethernet_runtime.state = ETH_STATE_LINK_UP;
      }
    } else {
      ethernet_runtime.link_down_count++;
      ethernet_runtime.got_ip = false;
      ethernet_runtime.state = ETH_STATE_LINK_DOWN;
      network_config.ethernet_enabled = false;
    }
  }

  if (link) {
    IPAddress ip = ETH.localIP();

    if (ip != IPAddress(0, 0, 0, 0)) {
      ethernet_runtime.got_ip = true;
      ethernet_runtime.state = ETH_STATE_GOT_IP;
      network_config.ethernet_enabled = true;

      snprintf(ethernet_runtime.ip, sizeof(ethernet_runtime.ip), "%s",
               ip.toString().c_str());
      snprintf(ethernet_runtime.gateway, sizeof(ethernet_runtime.gateway), "%s",
               ETH.gatewayIP().toString().c_str());
      snprintf(ethernet_runtime.subnet, sizeof(ethernet_runtime.subnet), "%s",
               ETH.subnetMask().toString().c_str());
      snprintf(ethernet_runtime.mac, sizeof(ethernet_runtime.mac), "%s",
               ETH.macAddress().c_str());
    }
  }
}



// ============================================================
// NTP UDP Listener - Diagnostic Only
// ============================================================

const char *ntp_state_text(NtpServiceState state) {
  switch (state) {
    case NTP_STATE_DISABLED:
      return "DISABLED";
    case NTP_STATE_WAIT_ETH:
      return "WAIT ETH";
    case NTP_STATE_LISTENING:
      return "LISTEN";
    case NTP_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

void init_ntp_listener() {
  ntp_runtime.replies_enabled = true;

  if (!ethernet_runtime.got_ip) {
    if (ntp_runtime.state != NTP_STATE_WAIT_ETH) {
      Serial.println("[NTPD] Waiting for Ethernet IP before binding UDP");
    }
    ntp_runtime.state = NTP_STATE_WAIT_ETH;
    network_config.ntp_enabled = false;
    return;
  }

  if (ntp_runtime.udp_started) {
    return;
  }

  Serial.println("[NTPD] Starting UDP listener on port 123");

  if (udp4123.begin(4123)) {
    Serial.println("[UDP4123] Debug listener active on port 4123");
  } else {
    Serial.println("[UDP4123] Debug listener failed");
  }

  if (ntp_udp.begin(NTP_PORT)) {
    ntp_runtime.udp_started = true;
    ntp_runtime.state = NTP_STATE_LISTENING;
    network_config.ntp_enabled = true;
    Serial.println("[NTPD] UDP listener active");
    Serial.println("[NTPD] Replies enabled - Stratum-1 GPS/PPS mode");
  } else {
    ntp_runtime.udp_started = false;
    ntp_runtime.state = NTP_STATE_ERROR;
    network_config.ntp_enabled = false;
    Serial.println("[NTPD] UDP listener failed");
  }
}


void write_ntp_u32(uint8_t *packet, int offset, uint32_t value) {
  packet[offset + 0] = (uint8_t)((value >> 24) & 0xFF);
  packet[offset + 1] = (uint8_t)((value >> 16) & 0xFF);
  packet[offset + 2] = (uint8_t)((value >> 8) & 0xFF);
  packet[offset + 3] = (uint8_t)(value & 0xFF);
}

uint32_t us_to_ntp_short(uint32_t microseconds) {
  // Convert microseconds to NTP short format, 16.16 seconds.
  return (uint32_t)(((uint64_t)microseconds * 65536ULL) / 1000000ULL);
}

void snapshot_ntp_time(uint32_t *seconds, uint32_t *fraction) {
  *seconds = timing_data.current_epoch;
  *fraction = timing_data.ntp_fraction;
}

bool send_ntp_reply(
  const uint8_t *request_packet,
  uint8_t client_version,
  uint8_t client_mode
) {
  if (client_mode != 3) {
    ntp_runtime.packets_bad_mode++;

    Serial.printf(
      "[NTPD] Bad mode:%u - not replying\n",
      client_mode
    );

    return false;
  }

  if (timing_data.current_epoch == 0 ||
      !timing_data.disciplined) {
    ntp_runtime.packets_not_ready++;

    Serial.println("[NTPD] Not disciplined - not replying");

    return false;
  }

  uint8_t reply[NTP_PACKET_SIZE];
  memset(reply, 0, sizeof(reply));

  uint32_t receive_seconds = 0;
  uint32_t receive_fraction = 0;
  uint32_t transmit_seconds = 0;
  uint32_t transmit_fraction = 0;

  snapshot_ntp_time(&receive_seconds, &receive_fraction);

  uint8_t li = 0;
  uint8_t vn = client_version;

  if (vn < 3 || vn > 4) {
    vn = 4;
  }

  // LI=0, VN=client version, Mode=4 server.
  reply[0] = (uint8_t)((li << 6) | (vn << 3) | 4);

  // Stratum 1 while GPS/PPS disciplined; stratum 2 during holdover.
  reply[1] = timing_data.holdover ? 2 : 1;

  // Poll copied from client. Precision from timing engine.
  reply[2] = request_packet[2];
  reply[3] = (uint8_t)timing_data.ntp_precision;

  write_ntp_u32(reply, 4, us_to_ntp_short(timing_data.root_delay));
  write_ntp_u32(reply, 8, us_to_ntp_short(timing_data.root_dispersion));

  // Reference ID "GPS".
  reply[12] = 'G';
  reply[13] = 'P';
  reply[14] = 'S';
  reply[15] = 0;

  // Reference timestamp: last PPS epoch, fractional zero.
  uint32_t ref_epoch =
    pps_data.last_pps_epoch > 0 ?
    pps_data.last_pps_epoch :
    timing_data.current_epoch;

  write_ntp_u32(reply, 16, ref_epoch);
  write_ntp_u32(reply, 20, 0);

  // Originate timestamp: client's transmit timestamp from request.
  memcpy(&reply[24], &request_packet[40], 8);

  // Receive timestamp.
  write_ntp_u32(reply, 32, receive_seconds);
  write_ntp_u32(reply, 36, receive_fraction);

  snapshot_ntp_time(&transmit_seconds, &transmit_fraction);

  // Transmit timestamp.
  write_ntp_u32(reply, 40, transmit_seconds);
  write_ntp_u32(reply, 44, transmit_fraction);

  ntp_udp.beginPacket(
    ntp_runtime.last_remote_ip,
    ntp_runtime.last_remote_port
  );

  ntp_udp.write(reply, NTP_PACKET_SIZE);
  bool ok = ntp_udp.endPacket();

  if (ok) {
    ntp_runtime.packets_tx++;

    Serial.printf(
      "[NTPD] TX #%lu to %s:%u Epoch:%lu Frac:0x%08lX Stratum:%u\n",
      (unsigned long)ntp_runtime.packets_tx,
      ntp_runtime.last_remote_ip.toString().c_str(),
      ntp_runtime.last_remote_port,
      (unsigned long)transmit_seconds,
      (unsigned long)transmit_fraction,
      reply[1]
    );
  } else {
    ntp_runtime.packets_ignored++;
    Serial.println("[NTPD] TX failed");
  }

  operational_stats.last_ntp_query_epoch = timing_data.current_epoch;
  log_ntp_request(client_version, client_mode, ok);

  return ok;
}


void update_ntp_listener() {
  if (!ethernet_runtime.got_ip) {
    if (!ntp_runtime.udp_started) {
      ntp_runtime.state = NTP_STATE_WAIT_ETH;
      network_config.ntp_enabled = false;
    }
    return;
  }

  if (!ntp_runtime.udp_started) {
    init_ntp_listener();
    return;
  }


  int debug_size = udp4123.parsePacket();

  if (debug_size > 0) {
    udp4123_debug.total_rx++;
    udp4123_debug.last_size = (uint16_t)debug_size;
    udp4123_debug.last_port = udp4123.remotePort();
    udp4123_debug.last_ip = udp4123.remoteIP();

    Serial.printf(
      "[UDP4123] RX #%lu Size:%u From:%s:%u\n",
      (unsigned long)udp4123_debug.total_rx,
      (unsigned int)udp4123_debug.last_size,
      udp4123_debug.last_ip.toString().c_str(),
      udp4123_debug.last_port
    );

    while (udp4123.available()) {
      udp4123.read();
    }
  }

  int packet_size = ntp_udp.parsePacket();

  if (packet_size <= 0) {
    return;
  }

  udp123_debug.total_rx++;
  udp123_debug.last_size = (uint16_t)packet_size;
  udp123_debug.last_port = ntp_udp.remotePort();
  udp123_debug.last_ip = ntp_udp.remoteIP();

  ntp_runtime.last_packet_ms = millis();
  ntp_runtime.last_remote_ip = udp123_debug.last_ip;
  ntp_runtime.last_remote_port = udp123_debug.last_port;
  ntp_runtime.last_packet_size = udp123_debug.last_size;

  Serial.printf(
    "[UDP123] RX #%lu Size:%u From:%s:%u\n",
    (unsigned long)udp123_debug.total_rx,
    (unsigned int)udp123_debug.last_size,
    udp123_debug.last_ip.toString().c_str(),
    udp123_debug.last_port
  );

  if (packet_size < NTP_PACKET_SIZE) {
    udp123_debug.short_rx++;
    ntp_runtime.packets_short++;

    Serial.printf(
      "[UDP123] SHORT #%lu Size:%u\n",
      (unsigned long)udp123_debug.short_rx,
      (unsigned int)udp123_debug.last_size
    );

    while (ntp_udp.available()) {
      ntp_udp.read();
    }

    return;
  }

  uint8_t packet[NTP_PACKET_SIZE];
  int bytes_read = ntp_udp.read(packet, NTP_PACKET_SIZE);

  while (ntp_udp.available()) {
    ntp_udp.read();
  }

  if (bytes_read != NTP_PACKET_SIZE) {
    udp123_debug.ignored_rx++;
    ntp_runtime.packets_ignored++;

    Serial.printf(
      "[UDP123] READ ERROR Read:%d Expected:%u\n",
      bytes_read,
      NTP_PACKET_SIZE
    );

    return;
  }

  udp123_debug.ntp_rx++;
  ntp_runtime.packets_rx++;

  uint8_t li = (packet[0] >> 6) & 0x03;
  uint8_t version = (packet[0] >> 3) & 0x07;
  uint8_t mode = packet[0] & 0x07;

  Serial.printf(
    "[NTPD] VALID #%lu LI:%u VN:%u Mode:%u Replies:%s\n",
    (unsigned long)udp123_debug.ntp_rx,
    li,
    version,
    mode,
    ntp_runtime.replies_enabled ? "ON" : "OFF"
  );

  if (ntp_runtime.replies_enabled) {
    send_ntp_reply(packet, version, mode);
  }
}



// ============================================================
// MicroSD / TF Card Runtime
// ============================================================

const char *sd_state_text(SdCardState state) {
  switch (state) {
    case SD_STATE_DISABLED:
      return "DISABLED";
    case SD_STATE_STARTING:
      return "STARTING";
    case SD_STATE_MOUNTED:
      return "MOUNTED";
    case SD_STATE_NO_CARD:
      return "NO CARD";
    case SD_STATE_TEST_FAILED:
      return "TEST FAIL";
    case SD_STATE_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

const char *sd_card_type_text(uint8_t card_type) {
  switch (card_type) {
    case CARD_NONE:
      return "NONE";
    case CARD_MMC:
      return "MMC";
    case CARD_SD:
      return "SDSC";
    case CARD_SDHC:
      return "SDHC";
    default:
      return "UNKNOWN";
  }
}

bool sd_self_test() {
  if (!sd_runtime.mounted) {
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "not mounted");
    return false;
  }

  File file = SD_MMC.open(SD_TEST_FILE, FILE_WRITE);

  if (!file) {
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "open write failed");
    return false;
  }

  uint32_t now_ms = millis();

  file.printf("SkyTime SD self-test\n");
  file.printf("Millis: %lu\n", (unsigned long)now_ms);
  file.printf("NTP epoch: %lu\n", (unsigned long)timing_data.current_epoch);
  file.close();

  sd_runtime.test_writes++;

  file = SD_MMC.open(SD_TEST_FILE, FILE_READ);

  if (!file) {
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "open read failed");
    return false;
  }

  size_t available = file.available();
  file.close();

  if (available == 0) {
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "read empty");
    return false;
  }

  sd_runtime.test_reads++;
  sd_runtime.test_passed = true;
  snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "OK");

  return true;
}

void init_sd_card() {
  Serial.println("[SD] Initializing onboard TF card via SD_MMC");

  sd_runtime.state = SD_STATE_STARTING;
  sd_runtime.mount_attempts++;
  sd_runtime.mounted = false;
  sd_runtime.test_passed = false;

#if SD_FORCE_1BIT_MODE
  bool one_bit = true;
#else
  bool one_bit = false;
#endif

  if (!SD_MMC.begin(SD_MOUNT_POINT, one_bit, false)) {
    sd_runtime.state = SD_STATE_ERROR;
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "mount failed");

    Serial.println("[SD] Mount failed");
    Serial.println("[SD] If the card is inserted, try SD_FORCE_1BIT_MODE=1");
    return;
  }

  uint8_t card_type = SD_MMC.cardType();

  snprintf(
    sd_runtime.card_type,
    sizeof(sd_runtime.card_type),
    "%s",
    sd_card_type_text(card_type)
  );

  if (card_type == CARD_NONE) {
    sd_runtime.state = SD_STATE_NO_CARD;
    sd_runtime.errors++;
    snprintf(sd_runtime.last_error, sizeof(sd_runtime.last_error), "no card");

    Serial.println("[SD] No card detected");
    return;
  }

  sd_runtime.mounted = true;
  sd_runtime.card_size_bytes = SD_MMC.cardSize();
  sd_runtime.total_bytes = SD_MMC.totalBytes();
  sd_runtime.used_bytes = SD_MMC.usedBytes();

  Serial.printf("[SD] Mounted type:%s\n", sd_runtime.card_type);
  Serial.printf(
    "[SD] Card:%llu MB Total:%llu MB Used:%llu MB\n",
    (unsigned long long)(sd_runtime.card_size_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.total_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.used_bytes / (1024ULL * 1024ULL))
  );

  if (sd_self_test()) {
    sd_runtime.state = SD_STATE_MOUNTED;
    Serial.println("[SD] Read/write self-test PASS");
  } else {
    sd_runtime.state = SD_STATE_TEST_FAILED;
    Serial.print("[SD] Read/write self-test FAIL: ");
    Serial.println(sd_runtime.last_error);
  }
}

void update_sd_runtime() {
  if (!sd_runtime.mounted) {
    return;
  }

  static uint32_t last_sd_refresh_ms = 0;
  uint32_t now_ms = millis();

  if (now_ms - last_sd_refresh_ms < 10000UL) {
    return;
  }

  last_sd_refresh_ms = now_ms;

  sd_runtime.total_bytes = SD_MMC.totalBytes();
  sd_runtime.used_bytes = SD_MMC.usedBytes();
}



// ============================================================
// Configuration File Runtime
// ============================================================

const char *config_state_text(ConfigState state) {
  switch (state) {
    case CONFIG_STATE_DEFAULTS: return "DEFAULTS";
    case CONFIG_STATE_LOADED:   return "LOADED";
    case CONFIG_STATE_PARTIAL:  return "PARTIAL";
    case CONFIG_STATE_ERROR:    return "ERROR";
    default:                    return "UNKNOWN";
  }
}

bool read_text_file(const char *path, char *buffer, size_t buffer_size) {
  if (!sd_runtime.mounted) return false;
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) return false;

  size_t idx = 0;
  while (file.available() && idx < buffer_size - 1) {
    buffer[idx++] = (char)file.read();
  }
  buffer[idx] = '\0';
  file.close();
  return idx > 0;
}

bool write_text_file(const char *path, const char *content) {
  if (!sd_runtime.mounted) return false;
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) return false;
  file.print(content);
  file.close();
  return true;
}

bool json_get_string(const char *json, const char *key, char *out, size_t out_size) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (*p != '"') return false;
  p++;

  size_t idx = 0;
  while (*p && *p != '"' && idx < out_size - 1) {
    out[idx++] = *p++;
  }
  out[idx] = '\0';
  return idx > 0;
}

bool json_get_bool(const char *json, const char *key, bool *out) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);

  const char *p = strstr(json, pattern);
  if (!p) {
    return false;
  }

  p = strchr(p, ':');
  if (!p) {
    return false;
  }

  p++;

  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
    p++;
  }

  if (strncasecmp(p, "true", 4) == 0) {
    *out = true;
    return true;
  }

  if (strncasecmp(p, "false", 5) == 0) {
    *out = false;
    return true;
  }

  if (*p == '"') {
    p++;

    if (strncasecmp(p, "true", 4) == 0) {
      *out = true;
      return true;
    }

    if (strncasecmp(p, "false", 5) == 0) {
      *out = false;
      return true;
    }

    if (*p == '1') {
      *out = true;
      return true;
    }

    if (*p == '0') {
      *out = false;
      return true;
    }
  }

  if (*p == '1') {
    *out = true;
    return true;
  }

  if (*p == '0') {
    *out = false;
    return true;
  }

  return false;
}

bool json_get_uint32(const char *json, const char *key, uint32_t *out) {
  char pattern[48];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char *p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (*p < '0' || *p > '9') return false;
  *out = (uint32_t)strtoul(p, nullptr, 10);
  return true;
}

bool ip_from_string(const char *text, IPAddress *ip) {
  int a, b, c, d;
  if (sscanf(text, "%d.%d.%d.%d", &a, &b, &c, &d) != 4) return false;
  if (a < 0 || a > 255 || b < 0 || b > 255 ||
      c < 0 || c > 255 || d < 0 || d > 255) return false;
  *ip = IPAddress((uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d);
  return true;
}

void apply_network_config_to_runtime() {
  IPAddress parsed;
  if (ip_from_string(network_config.ip, &parsed)) eth_static_ip = parsed;
  if (ip_from_string(network_config.gateway, &parsed)) eth_gateway = parsed;
  if (ip_from_string(network_config.subnet, &parsed)) eth_subnet = parsed;
  if (ip_from_string(network_config.dns, &parsed)) eth_dns = parsed;
  snprintf(ip_address, sizeof(ip_address), "%s", network_config.ip);
}

bool load_network_config() {
  char json[1024];
  Serial.print("[CFG] Loading ");
  Serial.println(CONFIG_NETWORK_FILE);

  if (!read_text_file(CONFIG_NETWORK_FILE, json, sizeof(json))) {
    Serial.println("[CFG] network.json missing or unreadable");
    return false;
  }

  char value[64];
  bool bool_value = false;

  if (json_get_string(json, "hostname", value, sizeof(value))) {
    snprintf(network_config.hostname, sizeof(network_config.hostname), "%s", value);
  }
  if (json_get_bool(json, "dhcp", &bool_value)) {
    network_config.dhcp_enabled = bool_value;
  }
  if (json_get_string(json, "ip", value, sizeof(value))) {
    snprintf(network_config.ip, sizeof(network_config.ip), "%s", value);
  }
  if (json_get_string(json, "subnet", value, sizeof(value))) {
    snprintf(network_config.subnet, sizeof(network_config.subnet), "%s", value);
  }
  if (json_get_string(json, "gateway", value, sizeof(value))) {
    snprintf(network_config.gateway, sizeof(network_config.gateway), "%s", value);
  }
  if (json_get_string(json, "dns", value, sizeof(value))) {
    snprintf(network_config.dns, sizeof(network_config.dns), "%s", value);
  }
  if (json_get_bool(json, "ntp_enabled", &bool_value)) {
    network_config.ntp_enabled = bool_value;
  }

  apply_network_config_to_runtime();

  Serial.println("[CFG] Network config loaded");
  Serial.print("[CFG] Hostname: "); Serial.println(network_config.hostname);
  Serial.print("[CFG] IP: "); Serial.println(network_config.ip);
  Serial.print("[CFG] Subnet: "); Serial.println(network_config.subnet);
  Serial.print("[CFG] Gateway: "); Serial.println(network_config.gateway);
  Serial.print("[CFG] DNS: "); Serial.println(network_config.dns);

  return true;
}

bool load_system_config() {
  char json[1024];
  Serial.print("[CFG] Loading ");
  Serial.println(CONFIG_SYSTEM_FILE);

  if (!read_text_file(CONFIG_SYSTEM_FILE, json, sizeof(json))) {
    Serial.println("[CFG] system.json missing or unreadable");
    return false;
  }

  char value[64];
  bool bool_value = false;
  uint32_t uint_value = 0;

  if (json_get_string(json, "device_name", value, sizeof(value))) {
    snprintf(system_config.device_name, sizeof(system_config.device_name), "%s", value);
  }

  if (json_get_string(json, "node_id", value, sizeof(value))) {
    snprintf(system_config.node_id, sizeof(system_config.node_id), "%s", value);
  }

  if (json_get_string(json, "role", value, sizeof(value))) {
    snprintf(system_config.role, sizeof(system_config.role), "%s", value);
  }

  if (json_get_string(json, "site_name", value, sizeof(value))) {
    snprintf(system_config.site_name, sizeof(system_config.site_name), "%s", value);
  }
  if (json_get_uint32(json, "screen_timeout", &uint_value)) {
    if (uint_value >= 5 && uint_value <= 3600) system_config.screen_timeout_seconds = uint_value;
  }
  if (json_get_uint32(json, "holdover_minutes", &uint_value)) {
    if (uint_value >= 1 && uint_value <= 1440) system_config.holdover_minutes = uint_value;
  }
  if (json_get_bool(json, "debug_serial", &bool_value)) {
    system_config.debug_serial = bool_value;
  }
  if (json_get_bool(json, "web_enabled", &bool_value)) {
    system_config.web_enabled = bool_value;
  }

  Serial.println("[CFG] System config loaded");
  Serial.print("[CFG] Device: "); Serial.println(system_config.device_name);
  Serial.print("[CFG] Screen timeout: "); Serial.println(system_config.screen_timeout_seconds);
  Serial.print("[CFG] Holdover minutes: "); Serial.println(system_config.holdover_minutes);
  Serial.print("[CFG] Debug serial: "); Serial.println(system_config.debug_serial ? "true" : "false");
  Serial.print("[CFG] Web enabled: "); Serial.println(system_config.web_enabled ? "true" : "false");

  return true;
}

bool create_default_config_files() {
  if (!sd_runtime.mounted) return false;

  bool created_any = false;

  if (!SD_MMC.exists("/config")) {
    SD_MMC.mkdir("/config");
  }

  if (!SD_MMC.exists(CONFIG_NETWORK_FILE)) {
    const char *network_default =
      "{\n"
      "  \"hostname\": \"skytime-p4\",\n"
      "  \"dhcp\": false,\n"
      "  \"ip\": \"192.168.0.123\",\n"
      "  \"subnet\": \"255.255.255.0\",\n"
      "  \"gateway\": \"192.168.0.1\",\n"
      "  \"dns\": \"192.168.0.1\",\n"
      "  \"ntp_enabled\": true\n"
      "}\n";
    if (write_text_file(CONFIG_NETWORK_FILE, network_default)) {
      Serial.println("[CFG] Created default network.json");
      created_any = true;
    }
  }

  if (!SD_MMC.exists(CONFIG_SYSTEM_FILE)) {
    const char *system_default =
      "{\n"
      "  \"device_name\": \"SkyTime\",\n"
      "  \"node_id\": \"SkyTime\",\n"
      "  \"role\": \"Standalone\",\n"
      "  \"site_name\": \"Enter Location in Configuration\",\n"
      "  \"screen_timeout\": 30,\n"
      "  \"holdover_minutes\": 60,\n"
      "  \"debug_serial\": true,\n"
      "  \"web_enabled\": true\n"
      "}\n";
    if (write_text_file(CONFIG_SYSTEM_FILE, system_default)) {
      Serial.println("[CFG] Created default system.json");
      created_any = true;
    }
  }

  config_runtime.defaults_created = created_any;
  return true;
}

bool load_config_files() {
  config_runtime.load_attempts++;

  if (!sd_runtime.mounted) {
    config_runtime.state = CONFIG_STATE_DEFAULTS;
    config_runtime.errors++;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "SD not mounted");
    Serial.println("[CFG] SD not mounted - using defaults");
    return false;
  }

  create_default_config_files();

  config_runtime.network_loaded = load_network_config();
  config_runtime.system_loaded = load_system_config();

  if (config_runtime.network_loaded && config_runtime.system_loaded) {
    config_runtime.state = CONFIG_STATE_LOADED;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "OK");
  } else if (config_runtime.network_loaded || config_runtime.system_loaded) {
    config_runtime.state = CONFIG_STATE_PARTIAL;
    config_runtime.errors++;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "partial config");
  } else {
    config_runtime.state = CONFIG_STATE_DEFAULTS;
    config_runtime.errors++;
    snprintf(config_runtime.last_error, sizeof(config_runtime.last_error), "using defaults");
  }

  return config_runtime.network_loaded || config_runtime.system_loaded;
}

void init_config() {
  Serial.println("[CFG] Initializing configuration framework");
  load_config_files();
}



// ============================================================
// Web Server Runtime
// ============================================================

const char *web_state_text(WebState state) {
  switch (state) {
    case WEB_STATE_DISABLED: return "DISABLED";
    case WEB_STATE_WAIT_ETH:  return "WAIT ETH";
    case WEB_STATE_RUNNING:   return "RUNNING";
    case WEB_STATE_ERROR:     return "ERROR";
    default:                  return "UNKNOWN";
  }
}

const char *content_type_from_path(const char *path) {
  const char *ext = strrchr(path, '.');

  if (!ext) return "text/plain";

  if (strcasecmp(ext, ".html") == 0) return "text/html";
  if (strcasecmp(ext, ".htm") == 0)  return "text/html";
  if (strcasecmp(ext, ".css") == 0)  return "text/css";
  if (strcasecmp(ext, ".js") == 0)   return "application/javascript";
  if (strcasecmp(ext, ".json") == 0) return "application/json";
  if (strcasecmp(ext, ".txt") == 0)  return "text/plain";
  if (strcasecmp(ext, ".png") == 0)  return "image/png";
  if (strcasecmp(ext, ".jpg") == 0)  return "image/jpeg";
  if (strcasecmp(ext, ".ico") == 0)  return "image/x-icon";

  return "application/octet-stream";
}

bool send_sd_file(const char *path, const char *content_type) {
  if (!sd_runtime.mounted) {
    return false;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file || file.isDirectory()) {
    if (file) {
      file.close();
    }
    return false;
  }

  web_server.streamFile(file, content_type);
  file.close();

  web_runtime.requests_static++;
  return true;
}

void build_status_json(char *buffer, size_t buffer_size) {
  snprintf(
    buffer,
    buffer_size,
    "{"
      "\"device\":\"%s\","
      "\"identity\":{\"node_id\":\"%s\",\"role\":\"%s\",\"site_name\":\"%s\"},"
      "\"release\":\"1.0\","
      "\"gps\":{\"state\":\"%s\",\"satellites\":%u,\"lat\":%.6f,\"lon\":%.6f},"
      "\"pps\":{\"state\":%d,\"count\":%lu,\"good\":%lu,\"bad\":%lu,\"rejected\":%lu,\"jitter_us\":%llu,\"align_bad\":%lu},"
      "\"ntp\":{\"disciplined\":%s,\"holdover\":%s,\"rx\":%lu,\"tx\":%lu,\"bad_mode\":%lu,\"not_ready\":%lu,\"epoch\":%lu},"
      "\"ethernet\":{\"state\":\"%s\",\"ip\":\"%s\",\"gateway\":\"%s\",\"link\":%s},"
      "\"sd\":{\"state\":\"%s\",\"mounted\":%s,\"test\":\"%s\",\"type\":\"%s\",\"total_mb\":%llu,\"used_mb\":%llu},"
      "\"system\":{\"uptime_ms\":%lu,\"uptime_seconds\":%lu},"
      "\"config\":{\"state\":\"%s\",\"network\":%s,\"system\":%s},"
      "\"stats\":{\"uptime_seconds\":%lu,\"gps_lock_seconds\":%lu,\"web_requests\":%lu,\"api_requests\":%lu},"
      "\"ntp_stats\":{\"requests\":%lu,\"replies\":%lu,\"bad_mode\":%lu,\"not_ready\":%lu},"
      "\"client\":{\"ip\":\"%s\",\"port\":%u,\"size\":%u,\"last_query_epoch\":%lu},"
      "\"web\":{\"state\":\"%s\",\"requests\":%lu}"
    "}",
    system_config.device_name,
    system_config.node_id,
    system_config.role,
    system_config.site_name,
    gps_data.state == GPS_LOCKED ? "LOCKED" :
      (gps_data.state == GPS_HOLDOVER ? "HOLDOVER" :
       (gps_data.state == GPS_STALE ? "STALE" : "SEARCH")),
    gps_data.satellites,
    gps_data.latitude,
    gps_data.longitude,
    (int)pps_data.state,
    (unsigned long)pps_data.pps_count,
    (unsigned long)pps_data.valid_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long)atomic_read_pps_rejected(),
    (unsigned long long)pps_data.jitter_us,
    (unsigned long)pps_data.align_bad_count,
    timing_data.disciplined ? "true" : "false",
    timing_data.holdover ? "true" : "false",
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)ntp_runtime.packets_bad_mode,
    (unsigned long)ntp_runtime.packets_not_ready,
    (unsigned long)timing_data.current_epoch,
    ethernet_state_text(ethernet_runtime.state),
    ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip,
    ethernet_runtime.got_ip ? ethernet_runtime.gateway : network_config.gateway,
    ethernet_runtime.link_up ? "true" : "false",
    sd_state_text(sd_runtime.state),
    sd_runtime.mounted ? "true" : "false",
    sd_runtime.test_passed ? "PASS" : "NO",
    sd_runtime.card_type,
    (unsigned long long)(sd_runtime.total_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.used_bytes / (1024ULL * 1024ULL)),
    (unsigned long)millis(),
    (unsigned long)(millis() / 1000UL),
    config_state_text(config_runtime.state),
    config_runtime.network_loaded ? "true" : "false",
    config_runtime.system_loaded ? "true" : "false",
    (unsigned long)(millis() / 1000UL),
    (unsigned long)gps_lock_seconds(),
    (unsigned long)web_runtime.requests_total,
    (unsigned long)web_runtime.requests_api,
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)ntp_runtime.packets_bad_mode,
    (unsigned long)ntp_runtime.packets_not_ready,
    ntp_runtime.last_remote_ip.toString().c_str(),
    ntp_runtime.last_remote_port,
    ntp_runtime.last_packet_size,
    (unsigned long)operational_stats.last_ntp_query_epoch,
    web_state_text(web_runtime.state),
    (unsigned long)web_runtime.requests_total
  );
}

void build_status_html(char *buffer, size_t buffer_size) {
  snprintf(
    buffer,
    buffer_size,
    "<!doctype html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>SkyTime Status</title>"
    "<style>"
    "body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:20px;}"
    ".card{background:#1d1d1d;border:1px solid #333;border-radius:8px;padding:14px;margin:12px 0;}"
    ".ok{color:#5cff7a}.warn{color:#ffd45c}.bad{color:#ff6666}"
    "table{border-collapse:collapse;width:100%%}td{padding:4px 8px;border-bottom:1px solid #333}"
    "h1,h2{color:#67d7ff}"
    "</style></head><body>"
    "<h1>SkyTime</h1>"
    "<div class='card'><h2>Timing</h2><table>"
    "<tr><td>GPS</td><td class='%s'>%s SAT:%u</td></tr>"
    "<tr><td>PPS</td><td class='%s'>State:%d Count:%lu Bad:%lu Jitter:%llu us</td></tr>"
    "<tr><td>NTP</td><td class='%s'>Disciplined:%s RX:%lu TX:%lu</td></tr>"
    "<tr><td>Epoch</td><td>%lu</td></tr>"
    "</table></div>"
    "<div class='card'><h2>Network</h2><table>"
    "<tr><td>Ethernet</td><td class='%s'>%s</td></tr>"
    "<tr><td>IP</td><td>%s</td></tr>"
    "<tr><td>Gateway</td><td>%s</td></tr>"
    "</table></div>"
    "<div class='card'><h2>Storage / Config</h2><table>"
    "<tr><td>SD</td><td class='%s'>%s %s</td></tr>"
    "<tr><td>Config</td><td>%s</td></tr>"
    "<tr><td>Web</td><td>%s Requests:%lu</td></tr>"
    "</table></div>"
    "<p><a href='/api/status' style='color:#67d7ff'>JSON Status</a></p>"
    "</body></html>",
    gps_data.state == GPS_LOCKED ? "ok" : "warn",
    gps_data.state == GPS_LOCKED ? "LOCKED" :
      (gps_data.state == GPS_HOLDOVER ? "HOLDOVER" :
       (gps_data.state == GPS_STALE ? "STALE" : "SEARCH")),
    gps_data.satellites,
    pps_data.state == PPS_LOCKED ? "ok" : "warn",
    (int)pps_data.state,
    (unsigned long)pps_data.pps_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long long)pps_data.jitter_us,
    timing_data.disciplined ? "ok" : "bad",
    timing_data.disciplined ? "YES" : "NO",
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)timing_data.current_epoch,
    ethernet_runtime.got_ip ? "ok" : "bad",
    ethernet_state_text(ethernet_runtime.state),
    ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip,
    ethernet_runtime.got_ip ? ethernet_runtime.gateway : network_config.gateway,
    sd_runtime.state == SD_STATE_MOUNTED ? "ok" : "warn",
    sd_state_text(sd_runtime.state),
    sd_runtime.card_type,
    config_state_text(config_runtime.state),
    web_state_text(web_runtime.state),
    (unsigned long)web_runtime.requests_total
  );
}

void handle_web_root() {
  web_runtime.requests_total++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  if (web_runtime.sd_index_available &&
      send_sd_file(WEB_INDEX_FILE, "text/html")) {
    return;
  }

  handle_web_status_page();
}

void handle_web_status_page() {
  web_runtime.requests_total++;
  web_runtime.requests_status++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  static char html[4096];
  build_status_html(html, sizeof(html));
  web_server.send(200, "text/html", html);
}

void handle_web_api_status() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  static char json[4096];
  build_status_json(json, sizeof(json));
  web_server.send(200, "application/json", json);
}

void handle_web_not_found() {
  web_runtime.requests_total++;
  web_runtime.requests_not_found++;
  snprintf(web_runtime.last_uri, sizeof(web_runtime.last_uri), "%s", web_server.uri().c_str());

  String uri = web_server.uri();

  if (uri.indexOf("..") >= 0) {
    web_server.send(400, "text/plain", "Bad request");
    return;
  }

  if (uri.endsWith("/")) {
    uri += "index.html";
  }

  char path[128];

  if (uri.startsWith("/web/")) {
    snprintf(path, sizeof(path), "%s", uri.c_str());
  } else {
    snprintf(path, sizeof(path), "/web%s", uri.c_str());
  }

  if (send_sd_file(path, content_type_from_path(path))) {
    return;
  }

  web_server.send(404, "text/plain", "Not found");
}

void init_web_server() {
  if (!system_config.web_enabled) {
    web_runtime.state = WEB_STATE_DISABLED;
    Serial.print("[WEB] Disabled by system config; web_enabled=");
    Serial.println(system_config.web_enabled ? "true" : "false");
    return;
  }

  if (!ethernet_runtime.got_ip) {
    web_runtime.state = WEB_STATE_WAIT_ETH;
    return;
  }

  if (web_runtime.started) {
    return;
  }

  web_runtime.start_attempts++;

  web_runtime.sd_index_available =
    sd_runtime.mounted && SD_MMC.exists(WEB_INDEX_FILE);

  web_server.on("/", HTTP_GET, handle_web_root);
  web_server.on("/status", HTTP_GET, handle_web_status_page);
  web_server.on("/api/status", HTTP_GET, handle_web_api_status);
  web_server.on("/logs.html", HTTP_GET, handle_web_logs_page);
  web_server.on("/config.html", HTTP_GET, handle_web_config_page);
  web_server.on("/api/config/identity", HTTP_GET, handle_api_config_identity_get);
  web_server.on("/api/config/identity", HTTP_POST, handle_api_config_identity_post);
  web_server.on("/api/config/network", HTTP_GET, handle_api_config_network_get);
  web_server.on("/api/config/network", HTTP_POST, handle_api_config_network_post);
  web_server.on("/api/system/reboot", HTTP_POST, handle_api_system_reboot);
  web_server.on("/api/logs/events", HTTP_GET, handle_api_log_events);
  web_server.on("/api/logs/ntp", HTTP_GET, handle_api_log_ntp);
  web_server.on("/ping", HTTP_GET, handle_web_ping);
  web_server.onNotFound(handle_web_not_found);

  web_server.begin();
#if WEB_ENABLE_RAW_8080_DEBUG
  raw_http_debug_server.begin();
#endif

  web_runtime.started = true;
  web_runtime.state = WEB_STATE_RUNNING;

  snprintf(web_runtime.last_error, sizeof(web_runtime.last_error), "OK");

  Serial.println("[WEB] HTTP server started on port 80");
  Serial.print("[WEB] SD index: ");
  Serial.println(web_runtime.sd_index_available ? "YES" : "NO");
  Serial.print("[WEB] URL: http://");
  Serial.println(ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip);
#if WEB_ENABLE_RAW_8080_DEBUG
  Serial.print("[WEB] Raw debug URL: http://");
  Serial.print(ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip);
  Serial.println(":8080/");
#endif
}

void update_web_server() {
  if (!system_config.web_enabled) {
    web_runtime.state = WEB_STATE_DISABLED;
    return;
  }

  if (!ethernet_runtime.got_ip) {
    web_runtime.state = WEB_STATE_WAIT_ETH;
    return;
  }

  if (!web_runtime.started) {
    init_web_server();
    return;
  }

  web_runtime.handle_ticks++;

  update_raw_http_debug_server();
  web_server.handleClient();
}




// ============================================================
// Deferred Event Log Timestamp Support
// ============================================================

#define PENDING_EVENT_LOG_MAX 6

struct PendingEventLogEntry {
  bool used;
  char event[24];
  char detail[96];
};

PendingEventLogEntry pending_event_logs[PENDING_EVENT_LOG_MAX];
bool pending_event_logs_flushed = false;

bool utc_log_time_valid() {
  return (
    gps_data.year >= 2024 &&
    gps_data.month >= 1 &&
    gps_data.month <= 12 &&
    gps_data.day >= 1 &&
    gps_data.day <= 31
  );
}

void queue_event_log_line(const char *event, const char *detail) {
  for (uint8_t i = 0; i < PENDING_EVENT_LOG_MAX; i++) {
    if (!pending_event_logs[i].used) {
      pending_event_logs[i].used = true;
      snprintf(pending_event_logs[i].event, sizeof(pending_event_logs[i].event), "%s", event ? event : "");
      snprintf(pending_event_logs[i].detail, sizeof(pending_event_logs[i].detail), "%s", detail ? detail : "");
      return;
    }
  }
}

void flush_pending_event_logs();


// ============================================================
// Operational Logging Runtime
// ============================================================

void format_utc_timestamp(char *buffer, size_t buffer_size) {
  if (gps_data.year > 0 &&
      gps_data.month > 0 &&
      gps_data.day > 0) {
    snprintf(
      buffer,
      buffer_size,
      "%04u-%02u-%02u %02u:%02u:%02u UTC",
      gps_data.year,
      gps_data.month,
      gps_data.day,
      gps_data.hour,
      gps_data.minute,
      gps_data.second
    );
  } else {
    snprintf(
      buffer,
      buffer_size,
      "UTC_PENDING uptime:%lu",
      (unsigned long)(millis() / 1000UL)
    );
  }
}

bool ensure_log_directory() {
  if (!sd_runtime.mounted) {
    snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "SD not mounted");
    return false;
  }

  if (!SD_MMC.exists(LOG_DIR_PATH)) {
    if (!SD_MMC.mkdir(LOG_DIR_PATH)) {
      snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "mkdir failed");
      return false;
    }
  }

  log_runtime.log_dir_ready = true;
  snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "OK");
  return true;
}

void rotate_log_if_needed(const char *path) {
  if (!sd_runtime.mounted || !SD_MMC.exists(path)) {
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);

  if (!file) {
    return;
  }

  size_t size = file.size();
  file.close();

  if (size < LOG_MAX_BYTES) {
    return;
  }

  char old_path[64];
  snprintf(old_path, sizeof(old_path), "%s.old", path);

  if (SD_MMC.exists(old_path)) {
    SD_MMC.remove(old_path);
  }

  SD_MMC.rename(path, old_path);
}

bool append_log_line(const char *path, const char *line) {
  if (!log_runtime.enabled) {
    return false;
  }

  if (!ensure_log_directory()) {
    return false;
  }

  rotate_log_if_needed(path);

  File file = SD_MMC.open(path, FILE_APPEND);

  if (!file) {
    snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "append open failed");
    return false;
  }

  file.println(line);
  file.close();

  snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "OK");
  return true;
}

void log_event_line(const char *event, const char *detail) {
  if (!utc_log_time_valid()) {
    queue_event_log_line(event, detail);
    return;
  }
  char timestamp[32];
  char line[192];

  format_utc_timestamp(timestamp, sizeof(timestamp));

  snprintf(
    line,
    sizeof(line),
    "%s | EVENT | %s | %s",
    timestamp,
    event,
    detail ? detail : ""
  );

  if (append_log_line(EVENT_LOG_FILE, line)) {
    log_runtime.event_entries++;
  } else {
    log_runtime.event_errors++;
  }
}


void flush_pending_event_logs() {
  if (pending_event_logs_flushed) {
    return;
  }

  if (!utc_log_time_valid()) {
    return;
  }

  for (uint8_t i = 0; i < PENDING_EVENT_LOG_MAX; i++) {
    if (pending_event_logs[i].used) {
      log_event_line(pending_event_logs[i].event, pending_event_logs[i].detail);
      pending_event_logs[i].used = false;
    }
  }

  pending_event_logs_flushed = true;
}



void log_ntp_request(uint8_t version, uint8_t mode, bool tx_ok) {
  if (!log_runtime.enabled || !sd_runtime.mounted) {
    return;
  }

  char timestamp[32];
  char line[256];

  format_utc_timestamp(timestamp, sizeof(timestamp));

  snprintf(
    line,
    sizeof(line),
    "%s | NTP | client=%s:%u | vn=%u | mode=%u | tx=%s | stratum=%u | disciplined=%s | holdover=%s",
    timestamp,
    ntp_runtime.last_remote_ip.toString().c_str(),
    ntp_runtime.last_remote_port,
    version,
    mode,
    tx_ok ? "OK" : "FAIL",
    timing_data.holdover ? 2 : 1,
    timing_data.disciplined ? "YES" : "NO",
    timing_data.holdover ? "YES" : "NO"
  );

  if (append_log_line(NTP_LOG_FILE, line)) {
    log_runtime.ntp_entries++;
  } else {
    log_runtime.ntp_errors++;
  }
}

void init_logging() {
  Serial.println("[LOG] Initializing operational logging");

  if (!sd_runtime.mounted) {
    log_runtime.enabled = false;
    snprintf(log_runtime.last_error, sizeof(log_runtime.last_error), "SD not mounted");
    Serial.println("[LOG] Disabled - SD not mounted");
    return;
  }

  log_runtime.enabled = true;

  if (ensure_log_directory()) {
    Serial.println("[LOG] Log directory ready");
    log_event_line("BOOT", "SkyTime logging initialized");
  } else {
    log_runtime.enabled = false;
    Serial.print("[LOG] Disabled - ");
    Serial.println(log_runtime.last_error);
  }
}



// ============================================================
// Operational Statistics Runtime
// ============================================================

void update_operational_stats() {
  bool gps_locked_now = (gps_data.state == GPS_LOCKED);

  if (gps_locked_now && !operational_stats.gps_lock_timer_active) {
    operational_stats.gps_lock_start_ms = millis();
    operational_stats.gps_lock_timer_active = true;
  }

  if (!gps_locked_now) {
    operational_stats.gps_lock_timer_active = false;
    operational_stats.gps_lock_start_ms = 0;
  }
}

uint32_t gps_lock_seconds() {
  if (!operational_stats.gps_lock_timer_active) {
    return 0;
  }

  return (millis() - operational_stats.gps_lock_start_ms) / 1000UL;
}


// ============================================================
// Debug Output
// ============================================================

void debug_output() {
  Serial.println();
  Serial.println("=======================================");

  Serial.printf(
    "[GPS] %s | SAT:%u | Lat:%.5f | Lon:%.5f\n",
    gps_data.state == GPS_LOCKED ? "LOCKED" :
      (gps_data.state == GPS_HOLDOVER ? "HOLDOVER" :
       (gps_data.state == GPS_STALE ? "STALE" : "SEARCH")),
    gps_data.satellites,
    gps_data.latitude,
    gps_data.longitude
  );

  Serial.printf(
    "[GPS] Time: %02d:%02d:%02d | Date: %02d/%02d/%04d\n",
    gps_data.hour,
    gps_data.minute,
    gps_data.second,
    gps_data.month,
    gps_data.day,
    gps_data.year
  );

  Serial.printf(
    "[PPS] Count:%lu | State:%d | Interval:%llu us | Good:%lu | Bad:%lu | Rejected:%lu\n",
    (unsigned long)pps_data.pps_count,
    (int)pps_data.state,
    (unsigned long long)pps_data.last_interval_us,
    (unsigned long)pps_data.valid_count,
    (unsigned long)pps_data.bad_count,
    (unsigned long)atomic_read_pps_rejected()
  );

  Serial.printf(
    "[PPS] Min:%llu us | Max:%llu us | Avg:%llu us | Jitter:%llu us | Align G:%lu B:%lu\n",
    (unsigned long long)pps_data.min_interval_us,
    (unsigned long long)pps_data.max_interval_us,
    (unsigned long long)pps_data.avg_interval_us,
    (unsigned long long)pps_data.jitter_us,
    (unsigned long)pps_data.align_good_count,
    (unsigned long)pps_data.align_bad_count
  );

  if (timing_data.current_epoch > 0) {
    Serial.printf(
      "[NTP] Epoch:%lu | Fraction:0x%08lX | us:%lu | Disciplined:%s | Holdover:%s\n",
      (unsigned long)timing_data.current_epoch,
      (unsigned long)timing_data.ntp_fraction,
      (unsigned long)timing_data.microseconds_since_pps,
      timing_data.disciplined ? "YES" : "NO",
      timing_data.holdover ? "YES" : "NO"
    );

    Serial.printf(
      "[NTP] Precision:2^%d | Root Delay:%lu us | Dispersion:%lu us\n",
      timing_data.ntp_precision,
      (unsigned long)timing_data.root_delay,
      (unsigned long)timing_data.root_dispersion
    );
  } else {
    Serial.println("[NTP] Waiting for GPS/PPS discipline...");
  }

  Serial.printf(
    "[ETH] State:%s | Link:%s | IP:%s | GW:%s | Static:%s | Up:%lu Down:%lu\n",
    ethernet_state_text(ethernet_runtime.state),
    ethernet_runtime.link_up ? "UP" : "DOWN",
    ethernet_runtime.got_ip ? ethernet_runtime.ip : network_config.ip,
    ethernet_runtime.got_ip ? ethernet_runtime.gateway : network_config.gateway,
    ethernet_runtime.static_ip_applied ? "YES" : "NO",
    (unsigned long)ethernet_runtime.link_up_count,
    (unsigned long)ethernet_runtime.link_down_count
  );

  Serial.printf(
    "[NTPD] State:%s | UDP:%s | RX:%lu | TX:%lu | BadMode:%lu | NotReady:%lu | Replies:%s\n",
    ntp_state_text(ntp_runtime.state),
    ntp_runtime.udp_started ? "UP" : "DOWN",
    (unsigned long)ntp_runtime.packets_rx,
    (unsigned long)ntp_runtime.packets_tx,
    (unsigned long)ntp_runtime.packets_bad_mode,
    (unsigned long)ntp_runtime.packets_not_ready,
    ntp_runtime.replies_enabled ? "ON" : "OFF"
  );

  Serial.printf(
    "[UDP123] Total:%lu Valid:%lu Short:%lu Ignored:%lu Last:%u\n",
    (unsigned long)udp123_debug.total_rx,
    (unsigned long)udp123_debug.ntp_rx,
    (unsigned long)udp123_debug.short_rx,
    (unsigned long)udp123_debug.ignored_rx,
    (unsigned int)udp123_debug.last_size
  );

  if (udp123_debug.total_rx > 0) {
    Serial.printf(
      "[UDP123] Last From:%s:%u\n",
      udp123_debug.last_ip.toString().c_str(),
      udp123_debug.last_port
    );
  }

  Serial.printf(
    "[UDP4123] Total:%lu Last:%u\n",
    (unsigned long)udp4123_debug.total_rx,
    (unsigned int)udp4123_debug.last_size
  );

  if (udp4123_debug.total_rx > 0) {
    Serial.printf(
      "[UDP4123] Last From:%s:%u\n",
      udp4123_debug.last_ip.toString().c_str(),
      udp4123_debug.last_port
    );
  }

  Serial.printf(
    "[SD] State:%s | Mounted:%s | Test:%s | Type:%s | Total:%llu MB | Used:%llu MB | Err:%lu | Last:%s\n",
    sd_state_text(sd_runtime.state),
    sd_runtime.mounted ? "YES" : "NO",
    sd_runtime.test_passed ? "PASS" : "NO",
    sd_runtime.card_type,
    (unsigned long long)(sd_runtime.total_bytes / (1024ULL * 1024ULL)),
    (unsigned long long)(sd_runtime.used_bytes / (1024ULL * 1024ULL)),
    (unsigned long)sd_runtime.errors,
    sd_runtime.last_error
  );

  Serial.printf(
    "[CFG] State:%s | Network:%s | System:%s | DefaultsCreated:%s | Errors:%lu | Last:%s\n",
    config_state_text(config_runtime.state),
    config_runtime.network_loaded ? "YES" : "NO",
    config_runtime.system_loaded ? "YES" : "NO",
    config_runtime.defaults_created ? "YES" : "NO",
    (unsigned long)config_runtime.errors,
    config_runtime.last_error
  );

  Serial.printf(
    "[WEB] State:%s | Started:%s | Ticks:%lu | Req:%lu | API:%lu | Static:%lu | 404:%lu | Raw8080:%lu | Last:%s\n",
    web_state_text(web_runtime.state),
    web_runtime.started ? "YES" : "NO",
    (unsigned long)web_runtime.handle_ticks,
    (unsigned long)web_runtime.requests_total,
    (unsigned long)web_runtime.requests_api,
    (unsigned long)web_runtime.requests_static,
    (unsigned long)web_runtime.requests_not_found,
    (unsigned long)web_runtime.raw8080_requests,
    web_runtime.last_uri
  );

  Serial.printf(
    "[CFG] WebEnabledRuntime:%s | Device:%s | Timeout:%lu\n",
    system_config.web_enabled ? "true" : "false",
    system_config.device_name,
    (unsigned long)system_config.screen_timeout_seconds
  );

  Serial.printf(
    "[CFG] Identity:%s | Role:%s | Site Name:%s\n",
    system_config.node_id,
    system_config.role,
    system_config.site_name
  );

  Serial.printf(
    "[LOG] Enabled:%s | Dir:%s | NTP:%lu Err:%lu | Event:%lu Err:%lu | Last:%s\n",
    log_runtime.enabled ? "YES" : "NO",
    log_runtime.log_dir_ready ? "YES" : "NO",
    (unsigned long)log_runtime.ntp_entries,
    (unsigned long)log_runtime.ntp_errors,
    (unsigned long)log_runtime.event_entries,
    (unsigned long)log_runtime.event_errors,
    log_runtime.last_error
  );

  Serial.print("[DISPLAY] Mode:");
  Serial.print((int)screen_manager_get_current());
  Serial.print(" | ScreenAge:");
  Serial.print((unsigned long)screen_manager_get_elapsed_ms());
  Serial.print(" ms | RedrawPending:");
  Serial.println(display_cache.force_full_redraw ? "YES" : "NO");

  Serial.println("=======================================");
  Serial.println();
}


// ============================================================
// FreeRTOS Tasks
// ============================================================

void timing_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1);

  while (true) {
    process_gps();
    update_pps_state();
    calculate_ntp_timestamp();
    handle_button();

    uint32_t now_ms = millis();

    if (now_ms - last_debug_time_ms >= DEBUG_INTERVAL_MS) {
      last_debug_time_ms = now_ms;
      debug_output();
    }

    vTaskDelayUntil(&last_wake, period);
  }
}


void network_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(5);

  // Give Ethernet events a moment to settle after setup.
  vTaskDelay(pdMS_TO_TICKS(500));

  Serial.println("[WEB] Network task servicing web");

  while (true) {
    update_ethernet_runtime();
    update_ntp_listener();
    update_sd_runtime();
    update_web_server();

    vTaskDelayUntil(&last_wake, period);
  }
}

void display_task(void *parameter) {
  (void)parameter;

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);

  while (true) {
    scheduler_update_display();
    vTaskDelayUntil(&last_wake, period);
  }
}

// ============================================================
// Arduino Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  init_systems();
  screen_manager_init();
  display_startup();

  Serial.println("[BOOT] Starting SD card");
  init_sd_card();

  Serial.println("[BOOT] Starting configuration");
  init_config();

  Serial.println("[BOOT] Starting logging");
  init_logging();

  Serial.println("[BOOT] Starting Ethernet");
  init_ethernet();
  init_ntp_listener();

  Serial.println("[BOOT] Web server will start from network task");
  Serial.println("[BOOT] Web enabled is controlled by /config/system.json");



  Serial.println("[BOOT] System initialized successfully");
  Serial.println("[BOOT] Release: Version 1.0 Configuration Framework Polish");
  Serial.println("[TASK] Starting timing/control, network, and display tasks");

  BaseType_t timing_ok = xTaskCreatePinnedToCore(
    timing_task,
    "SkyTimeTiming",
    TASK_TIMING_STACK,
    nullptr,
    TASK_TIMING_PRIORITY,
    &timing_task_handle,
    TASK_TIMING_CORE
  );

  BaseType_t network_ok = xTaskCreatePinnedToCore(
    network_task,
    "SkyTimeNetwork",
    TASK_NETWORK_STACK,
    nullptr,
    TASK_NETWORK_PRIORITY,
    &network_task_handle,
    TASK_NETWORK_CORE
  );

  BaseType_t display_ok = xTaskCreatePinnedToCore(
    display_task,
    "SkyTimeDisplay",
    TASK_DISPLAY_STACK,
    nullptr,
    TASK_DISPLAY_PRIORITY,
    &display_task_handle,
    TASK_DISPLAY_CORE
  );

  if (timing_ok != pdPASS || network_ok != pdPASS || display_ok != pdPASS) {
    Serial.println("[TASK] Failed to create one or more tasks");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[TASK] Tasks running");
}

void loop() {
  update_operational_stats();
  flush_pending_event_logs();
  // All production work is handled by FreeRTOS tasks.
  // Keep Arduino loop alive but idle.
  delay(1000);
}
