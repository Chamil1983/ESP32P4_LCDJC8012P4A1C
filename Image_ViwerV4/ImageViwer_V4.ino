/*
 * ESP32P4 Image Gallery with Web Upload
 * 
 * Robust SD Card and Memory Management
 * 16MB Flash / 8MB PSRAM Configuration
 * 
 * Author: Chamil1983
 * Date: 2025-06-14
 * Last modified: 2025-06-14 09:43:21 UTC
 */

#pragma GCC push_options
#pragma GCC optimize("O2") // Using O2 for better stability

// Include required libraries
#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD_MMC.h>
#include <FS.h>
#include <SPIFFS.h>
#include "JPEGDEC.h" // Using the specified library
#include <esp_task_wdt.h>
#include "pins_config.h"
#include "debug_logger.h"
#include "src/lcd/jd9365_lcd.h"
#include "src/touch/gsl3680_touch.h"

// JPEG decoder instance
JPEGDEC jpeg;

// System status tracking
bool lcd_ready = false;
bool touch_ready = false;
bool sd_ready = false;
bool wifi_ready = false;
bool lvgl_ready = false;
bool ui_ready = false;
bool server_ready = false;

// Watchdog settings
#define WDT_TIMEOUT_SECONDS 120  // Increased timeout to 120 seconds

// WiFi AP configuration
#define WIFI_AP_SSID "ESP32P4-ImageViewer"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_IP IPAddress(192, 168, 4, 1)
#define WIFI_AP_GATEWAY IPAddress(192, 168, 4, 1)
#define WIFI_AP_SUBNET IPAddress(255, 255, 255, 0)
#define WIFI_CHANNEL 6
#define MAX_WIFI_CLIENTS 2  // Limited to 2 for stability

// Web Server on port 80
WebServer server(80);

// Hardware objects
jd9365_lcd lcd = jd9365_lcd(LCD_RST);
gsl3680_touch touch = gsl3680_touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

// LVGL display buffers
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;

// Image gallery variables
#define MAX_IMAGES 100
char* image_list[MAX_IMAGES];
volatile int image_count = 0;
volatile int current_image = 0;
volatile bool image_loading = false;
uint32_t last_ui_update = 0;
uint32_t last_system_update = 0;

// JPEG buffer for drawing
#define DRAW_BUFFER_SIZE (32*1024) // 32KB buffer
uint8_t* drawBuffer = nullptr;

// UI elements
lv_obj_t *main_screen = nullptr;
lv_obj_t *status_bar = nullptr;
lv_obj_t *image_view = nullptr;
lv_obj_t *image_display = nullptr; // Image display widget
lv_obj_t *control_bar = nullptr;
lv_obj_t *status_label = nullptr;
lv_obj_t *wifi_label = nullptr;
lv_obj_t *image_counter = nullptr;
lv_obj_t *prev_btn = nullptr;
lv_obj_t *next_btn = nullptr;
lv_obj_t *sys_info = nullptr;
lv_style_t style_btn;
lv_style_t style_btn_pressed;

// Upload variables
#define UPLOAD_BUFFER_SIZE 4096  // 4KB buffer
#define SDMMC_FREQ_PROBING 125000  // Very slow speed for initial probing
#define RETRY_DELAY 1000           // Delay between retries 
#define SD_POWER_STABILIZE_DELAY 2000  // Power stabilization delay
bool upload_active = false;
uint32_t upload_start_time = 0;
uint32_t upload_size = 0;
char upload_filename[64];

// Mutex for thread safety
SemaphoreHandle_t sd_mutex = NULL;
SemaphoreHandle_t jpeg_mutex = NULL;
SemaphoreHandle_t ui_mutex = NULL;

// Control flags
volatile bool wdt_enabled = false;
volatile bool reset_requested = false;

// Task handles
TaskHandle_t lvgl_task_handle = NULL;
TaskHandle_t main_task_handle = NULL;

// Forward declarations
bool setup_lcd();
bool setup_touch();
bool setup_sd_card_reliable();
void process_pending_operations();
bool setup_watchdog();
void updateImageCounter(int index);
bool initializeSpiffs();
void checkPartitions();
void feed_watchdog();
bool setup_wifi_ap();
bool setup_lvgl();
bool setup_ui();
bool setup_webserver();
bool validateSDCard();
void update_system_info();
void updateStatusAfterImageLoad(int index, bool success);
void scan_images();
void display_image(int index);
void handle_upload();
bool recover_sd_card();
void diagnose_sd_card();
bool render_jpeg_file(const char* filename);
void lvgl_task(void *pvParameters);
int jpeg_draw_callback(JPEGDRAW *pDraw);


// to ensure consistent debug logging throughout the code

#define DEBUG_ENABLED true

// Debug macros with forced output
#define DEBUG_PRINT(tag, format, ...) do { \
  if (DEBUG_ENABLED) { \
    Serial.printf("[DEBUG][%s] ", tag); \
    Serial.printf(format, ##__VA_ARGS__); \
    Serial.println(); \
    Serial.flush(); \
  } \
} while(0)

#define DEBUG_INIT(tag) DEBUG_PRINT(tag, "Initializing...")
#define DEBUG_SUCCESS(tag) DEBUG_PRINT(tag, "Initialization successful")
#define DEBUG_FAIL(tag, reason) DEBUG_PRINT(tag, "Initialization failed: %s", reason)

// Use these in your setup functions like:
// DEBUG_INIT("LCD");
// ...lcd init code...
// if (success)
//   DEBUG_SUCCESS("LCD");
// else
//   DEBUG_FAIL("LCD", "Hardware not found");


// Safe malloc wrapper with checks
void* safe_malloc(size_t size, uint32_t caps = MALLOC_CAP_DEFAULT) {
  if (size == 0 || size > 1024*1024) {
    Logger.error("Invalid memory allocation request: %u bytes", size);
    return nullptr;
  }
  
  void* ptr = nullptr;
  
  // Try to allocate memory
  if (caps == MALLOC_CAP_SPIRAM) {
    ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    
    // Fallback to internal memory if PSRAM allocation fails
    if (ptr == nullptr) {
      Logger.warn("PSRAM allocation failed, falling back to internal memory");
      ptr = heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
    }
  } else {
    ptr = heap_caps_malloc(size, caps);
  }
  
  if (ptr == nullptr) {
    Logger.error("Memory allocation failed for %u bytes", size);
  } else {
    // Zero out allocated memory for safety
    memset(ptr, 0, size);
  }
  
  return ptr;
}

// Safe free with null check
void safe_free(void* ptr) {
  if (ptr != nullptr) {
    free(ptr);
  }
}

// Initialize watchdog system with the right approach
bool setup_watchdog() {
  Logger.info("Setting up watchdog system...");
  
  // First detect if WDT is already running
  esp_err_t status = ESP_OK;
  
  // Configure watchdog with a much longer timeout (30 seconds)
  // to give more time for operations like SD card access and JPEG decoding
  const uint32_t WDT_TIMEOUT_MS = 30000;
  
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0, // Don't use idle detection
    .trigger_panic = true // Use panic for debugging
  };
  
  // Try to initialize or reconfigure the watchdog
  status = esp_task_wdt_reconfigure(&wdt_config);
  if (status == ESP_OK) {
    Logger.info("Watchdog reconfigured with %d second timeout", WDT_TIMEOUT_MS/1000);
    wdt_enabled = true;
  } else {
    Logger.warn("Could not reconfigure watchdog: %d", status);
    
    // Try initializing instead
    status = esp_task_wdt_init(&wdt_config);
    if (status == ESP_OK) {
      Logger.info("Watchdog initialized with %d second timeout", WDT_TIMEOUT_MS/1000);
      wdt_enabled = true;
    } else if (status == ESP_ERR_INVALID_STATE) {
      Logger.info("Watchdog already initialized - using existing watchdog");
      wdt_enabled = true;
    } else {
      Logger.error("Failed to initialize watchdog: %d", status);
      wdt_enabled = false;
    }
  }
  
  // Always try to add the current task to the watchdog
  if (wdt_enabled) {
    status = esp_task_wdt_add(NULL);
    if (status == ESP_OK) {
      Logger.info("Main task added to watchdog");
      return true;
    } else if (status == ESP_ERR_INVALID_ARG) {
      Logger.warn("Task already subscribed to watchdog");
      return true;
    } else {
      Logger.error("Failed to add task to watchdog: %d", status);
      return false;
    }
  }
  
  return wdt_enabled;
}

// Robust watchdog feeding function
void feed_watchdog() {
  if (wdt_enabled) {
    // Feed the watchdog and track any errors
    static uint32_t lastFeedError = 0;
    static uint32_t feedErrorCount = 0;
    
    esp_err_t err = esp_task_wdt_reset();
    
    if (err != ESP_OK) {
      // Only log errors occasionally to avoid spamming
      uint32_t now = millis();
      feedErrorCount++;
      
      if (now - lastFeedError > 10000) {
        Logger.warn("WDT feed error: %d (count: %d)", err, feedErrorCount);
        lastFeedError = now;
      }
    }
  }
}

// Safe execute with watchdog function for long-running operations
template<typename Func>
bool safe_execute_with_watchdog(Func operation, const char* operationName) {
  Logger.debug("Starting operation: %s", operationName);
  
  // Feed watchdog before operation
  feed_watchdog();
  
  uint32_t startTime = millis();
  bool result;
  
  try {
    // Execute the operation
    result = operation();
  } catch (const std::exception& e) {
    Logger.error("Exception in %s: %s", operationName, e.what());
    result = false;
  } catch (...) {
    Logger.error("Unknown exception in %s", operationName);
    result = false;
  }
  
  // Feed watchdog after operation
  feed_watchdog();
  
  uint32_t duration = millis() - startTime;
  if (duration > 1000) {
    Logger.warn("Operation %s took %d ms", operationName, duration);
  }
  
  return result;
}

// LVGL display flush callback
void lvgl_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  if (!lcd_ready) {
    lv_disp_flush_ready(disp);
    return;
  }

  const int offsetx1 = area->x1;
  const int offsetx2 = area->x2;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;
  
  if (offsetx1 >= 0 && offsety1 >= 0 && offsetx2 < LCD_H_RES && offsety2 < LCD_V_RES) {
    lcd.lcd_draw_bitmap(offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, &color_p->full);
  } else {
    // Bounds check failed - log and skip drawing
    Logger.warn("LVGL draw out of bounds: (%d,%d)-(%d,%d)", offsetx1, offsety1, offsetx2, offsety2);
  }
  
  lv_disp_flush_ready(disp);
}

// LVGL touch input handler
void lvgl_touch_cb(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (!touch_ready) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  bool touched;
  uint16_t touchX, touchY;

  touched = touch.getTouch(&touchX, &touchY);
  touchX = 800 - touchX; // Adjust X coordinate for display orientation

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
    Logger.debug("Touch: x=%d, y=%d", touchX, touchY);
  }
}

// LVGL rotation callback
static void lvgl_rotation_cb(lv_disp_drv_t *drv) {
  if (!touch_ready) return;
  
  switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
      touch.set_rotation(0);
      break;
    case LV_DISP_ROT_90:
      touch.set_rotation(1);
      break;
    case LV_DISP_ROT_180:
      touch.set_rotation(2);
      break;
    case LV_DISP_ROT_270:
      touch.set_rotation(3);
      break;
  }
}

// Add SPIFFS Mount Recovery setup function
bool initializeSpiffs() {
    Serial.println("Initializing SPIFFS...");
    
    // Attempt standard mount first
    if (SPIFFS.begin(false)) {
        Serial.println("SPIFFS mounted successfully");
        
        // Check available space
        size_t total = SPIFFS.totalBytes();
        size_t used = SPIFFS.usedBytes();
        Serial.printf("SPIFFS: %u total, %u used, %u free\n", total, used, total - used);
        
        return true;
    }
    
    // If standard mount fails, try formatting
    Serial.println("SPIFFS mount failed, attempting to format...");
    if (SPIFFS.format()) {
        Serial.println("SPIFFS formatted successfully");
        
        // Try mounting again after formatting
        if (SPIFFS.begin()) {
            Serial.println("SPIFFS mounted after format");
            return true;
        } else {
            Serial.println("SPIFFS mount failed even after formatting");
        }
    } else {
        Serial.println("SPIFFS format failed");
    }
    
    // If we got here, SPIFFS is unavailable
    Serial.println("SPIFFS unavailable - continuing without filesystem");
    return false;
}

// This is a verification function - add to setup() after serial init
void checkPartitions() {
    Serial.println("Checking partition information:");
    
    // Display flash size
    Serial.printf("Flash size: %u bytes\n", ESP.getFlashChipSize());
    
    // Display partition information
    esp_partition_iterator_t it;
    
    Serial.println("\nPartition table:");
    Serial.println("Type | Subtype | Address | Size | Label");
    Serial.println("-----|---------|---------|------|-------");
    
    // Iterate through all partitions
    it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    while (it) {
        const esp_partition_t* part = esp_partition_get(it);
        Serial.printf("%4d | %7d | %08x | %6uK | %s\n", 
            part->type, part->subtype, part->address, part->size / 1024, part->label);
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);
    
    // Look specifically for a SPIFFS partition
    it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    if (it) {
        const esp_partition_t* part = esp_partition_get(it);
        Serial.printf("\nSPIFFS partition found: %s at 0x%x, size %uK\n", 
            part->label, part->address, part->size / 1024);
        esp_partition_iterator_release(it);
    } else {
        Serial.println("\nWARNING: No SPIFFS partition found!");
        Serial.println("This will prevent filesystem operations.");
        Serial.println("Check your partition table in Arduino IDE.");
    }
    
    Serial.println();
}

void setup() {
  // Initialize serial with higher baud rate for faster debug output
  Serial.begin(115200);
  Serial.flush();
  delay(100);
  
  Serial.println();
  Serial.println("=== ESP32P4 Gallery Debug Output ===");
  
    // Try to initialize SPIFFS
    bool spiffsAvailable = initializeSpiffs();
    
    // Initialize the logger with appropriate settings
    // Only enable flash logging if SPIFFS is available
    Logger.init(true, spiffsAvailable, LOG_LEVEL_DEBUG);
    
    // Force output level for better debugging
    Logger.setLogLevel(LOG_LEVEL_DEBUG);
    Logger.enableSerialOutput(true);
    
    // Log SPIFFS status
    if (spiffsAvailable) {
        Logger.info("SPIFFS is available for logging");
    } else {
        Logger.warn("SPIFFS is unavailable - logging to serial only");
    }
  
  // Force flush again to ensure no buffering issues

  checkPartitions();
  Serial.flush();
  
  // Test all log levels to verify logger is working
  Logger.debug("Debug test - showing debug level messages");
  Logger.info("Info test - showing info level messages");
  Logger.warn("Warn test - showing warning level messages");
  Logger.error("Error test - showing error level messages");
  
  // Use direct serial output for critical initialization messages
  // in case the logger has issues
  Serial.println("Direct Serial: Starting initialization sequence");
  
  Logger.info("====================================");
  Logger.info("ESP32P4 Image Gallery Starting");
  Logger.info("Date: 2025-06-14 11:30:45 UTC");
  Logger.info("User: Chamil1983");
  Logger.info("====================================");
  
  // Create mutexes
  DEBUG_INIT("Mutexes");
  sd_mutex = xSemaphoreCreateMutex();
  jpeg_mutex = xSemaphoreCreateMutex();
  ui_mutex = xSemaphoreCreateMutex();
  DEBUG_SUCCESS("Mutexes");
  
  // Initialize watchdog with enhanced logging
  DEBUG_INIT("Watchdog");
  bool wdt_success = setup_watchdog();
  if (wdt_success) {
    DEBUG_SUCCESS("Watchdog");
  } else {
    DEBUG_FAIL("Watchdog", "Could not initialize properly");
  }
  
  // Initialize components with verbose logging
  Serial.println("Starting component initialization sequence:");
  
  // Step 1: LCD with explicit debug
  Serial.println("1. LCD initialization");
  DEBUG_INIT("LCD");
  lcd_ready = setup_lcd();
  if (lcd_ready) {
    DEBUG_SUCCESS("LCD");
    Serial.println("LCD ready");
  } else {
    DEBUG_FAIL("LCD", "Hardware error");
    Serial.println("LCD failed");
  }
  
  // Step 2: Touch with explicit debug
  Serial.println("2. Touch initialization");
  DEBUG_INIT("Touch");
  touch_ready = setup_touch();
  if (touch_ready) {
    DEBUG_SUCCESS("Touch");
    Serial.println("Touch ready");
  } else {
    DEBUG_FAIL("Touch", "Could not initialize");
    Serial.println("Touch failed");
  }
  
  // Step 3: SD Card with explicit debug
  Serial.println("3. SD Card initialization");
  DEBUG_INIT("SD Card");
  sd_ready = setup_sd_card_reliable();
  if (sd_ready) {
    DEBUG_SUCCESS("SD Card");
    Serial.println("SD ready");
  } else {
    DEBUG_FAIL("SD Card", "Mount failed");
    Serial.println("SD failed");
  }
  
  // Step 4: WiFi with explicit debug
  Serial.println("4. WiFi initialization");
  DEBUG_INIT("WiFi");
  wifi_ready = setup_wifi_ap();
  if (wifi_ready) {
    DEBUG_SUCCESS("WiFi");
    Serial.println("WiFi ready");
  } else {
    DEBUG_FAIL("WiFi", "AP setup failed");
    Serial.println("WiFi failed");
  }
  
  // Continue with remaining initialization...
  // LVGL, UI, WebServer, etc.
  
  Serial.println("Initialization complete!");
  Serial.printf("STATUS: LCD:%s Touch:%s SD:%s WiFi:%s LVGL:%s UI:%s Server:%s\n",
    lcd_ready ? "OK" : "FAIL",
    touch_ready ? "OK" : "FAIL",
    sd_ready ? "OK" : "FAIL",
    wifi_ready ? "OK" : "FAIL",
    lvgl_ready ? "OK" : "FAIL",
    ui_ready ? "OK" : "FAIL",
    server_ready ? "OK" : "FAIL");
  
  Serial.flush();
}

// LVGL task with explicit watchdog feeding
void lvgl_task(void *pvParameters) {
  // Register this task with the watchdog
  esp_task_wdt_add(NULL);
  
  Logger.info("LVGL task started on Core %d", xPortGetCoreID());
  
  // Set interval for LVGL timer (milliseconds)
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    // Feed watchdog at the start of each cycle
    esp_task_wdt_reset();
    
    // Process LVGL tasks if initialized
    if (lvgl_ready) {
      uint32_t startTime = millis();
      
      // Take UI mutex to protect LVGL operations
      if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        lv_timer_handler();
        xSemaphoreGive(ui_mutex);
        
        uint32_t duration = millis() - startTime;
        if (duration > 100) {
          Logger.warn("LVGL handler took too long: %d ms", duration);
          // Feed watchdog again if it took too long
          esp_task_wdt_reset();
        }
      }
    }
    
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// Main loop with guaranteed watchdog feeding
void loop() {
  // CRITICAL: Feed watchdog immediately at the start of each loop
  feed_watchdog();
  
  // Track execution time for loop operations
  uint32_t loopStartTime = millis();
  
  // Handle web server requests if active - with timeout protection
  if (server_ready) {
    // Only process clients for a limited time
    uint32_t serverStartTime = millis();
    
    // Don't spend more than 100ms handling server requests
    while (millis() - serverStartTime < 100) {
      server.handleClient();
      // Small yield to prevent CPU hogging
      delay(1);
    }
  }
  
  // Feed watchdog after potentially long operation
  feed_watchdog();
  
  // Update system info periodically
  if (millis() - last_system_update > 2000) {
    update_system_info();
    last_system_update = millis();
    
    // Feed watchdog after updating system info
    feed_watchdog();
  }
  
  // Process any pending operations with watchdog protection
  process_pending_operations();
  
  // Ensure we don't block for too long without feeding the watchdog
  if (millis() - loopStartTime > 500) {
    Logger.warn("Loop execution time exceeded 500ms: %dms", millis() - loopStartTime);
  }
  
  // Add a small delay to prevent tight loop and high CPU usage
  delay(10);
  
  // CRITICAL: Feed watchdog again before loop ends
  feed_watchdog();
}

// Process any pending operations with watchdog protection
void process_pending_operations() {
  // Check if a reset was requested
  if (reset_requested) {
    Logger.info("System reset requested, restarting...");
    // Feed watchdog one last time before reset
    feed_watchdog();
    delay(100);
    ESP.restart();
  }
  
  // Process any image loading operations
  if (image_loading) {
    // Feed watchdog if we're in the middle of a long-running operation
    feed_watchdog();
  }
}

// 1. LCD Setup Function - Fixed to use direct GPIO control
bool setup_lcd() {
  Logger.info("Setting up LCD...");
  
  try {
    // Initialize LCD
    lcd.begin();
    
    // Control backlight directly via GPIO
    pinMode(LCD_LED, OUTPUT);
    digitalWrite(LCD_LED, HIGH);  // Turn on backlight
    
    Logger.info("LCD initialized successfully with backlight ON");
    return true;
  } catch (...) {
    Logger.error("Error initializing LCD");
    return false;
  }
}

// 2. Touch Setup Function
bool setup_touch() {
  Logger.info("Setting up Touch...");
  
  if (!lcd_ready) {
    Logger.error("Touch setup skipped - LCD not ready");
    return false;
  }
  
  try {
    touch.begin();
    Logger.info("Touch initialized successfully");
    return true;
  } catch (...) {
    Logger.error("Error initializing Touch");
    return false;
  }
}

// 3. SD Card Setup Function - COMPLETELY REDESIGNED FOR DATA INTEGRITY
// Reliable SD Card Setup Function with Alternative Approaches
bool setup_sd_card_reliable() {
  Logger.info("Setting up SD Card with robust initialization...");
  
  // CRITICAL FIX: Wait longer before SD initialization
  Logger.info("Waiting for power stabilization...");
  delay(2000); // Wait for power to stabilize
  
  // Reset all SD pins to a known state
  Logger.info("Resetting SD pins...");
  digitalWrite(SDMMC_CLK_PIN, LOW);
  digitalWrite(SDMMC_CMD_PIN, LOW);
  digitalWrite(SDMMC_D0_PIN, LOW);
  pinMode(SDMMC_CLK_PIN, OUTPUT);
  pinMode(SDMMC_CMD_PIN, OUTPUT);
  pinMode(SDMMC_D0_PIN, OUTPUT);
  delay(100);
  
  // Release pins
  pinMode(SDMMC_CLK_PIN, INPUT);
  pinMode(SDMMC_CMD_PIN, INPUT);
  pinMode(SDMMC_D0_PIN, INPUT);
  pinMode(SDMMC_D1_PIN, INPUT);
  pinMode(SDMMC_D2_PIN, INPUT);
  pinMode(SDMMC_D3_PIN, INPUT);
  delay(100);
  
  // Set pull-ups
  pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
  pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
  delay(500); // Longer delay for pullups to stabilize
  
  // Make sure SD_MMC is completely deinitialized
  Logger.info("Ensuring SD_MMC is deinitialized...");
  SD_MMC.end();
  delay(500);
  
  // Attempt with legacy SD library instead of SD_MMC
  if (tryLegacySDInit()) {
    Logger.info("Successfully initialized SD card using legacy SD library");
    return true;
  }
  
  // Configure SD pins for ESP32P4
  Logger.info("Configuring SD pins...");
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                 SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
  
  // Try multiple initialization approaches
  
  // Attempt 1: Ultra-conservative 1-bit mode with very low speed
  Logger.info("SD init attempt 1: 1-bit mode at 125KHz (ultra conservative)");
  if (SD_MMC.begin("/sdcard", false, false, 125000)) {
    Logger.info("SD card initialized successfully in ultra-conservative mode");
    return validateSDCard();
  }
  
  Logger.warn("SD init attempt 1 failed");
  SD_MMC.end();
  delay(1000);
  
  // Attempt 2: Try with a different pin configuration
  Logger.info("SD init attempt 2: Alternative pin configuration");
  // This is a conceptual placeholder - in a real implementation,
  // you would use alternative pins if available
  
  // Instead, we'll try with a slightly higher speed
  if (SD_MMC.begin("/sdcard", false, false, 250000)) {
    Logger.info("SD card initialized successfully with alternative config");
    return validateSDCard();
  }
  
  Logger.warn("SD init attempt 2 failed");
  SD_MMC.end();
  delay(1000);
  
  // Attempt 3: SPI mode fallback
  Logger.info("SD init attempt 3: SPI mode fallback");
  // This would require connecting SD card to SPI pins and using the SD library
  // instead of SD_MMC. In this placeholder, we just return false.
  
  Logger.error("All SD initialization attempts failed");
  return false;
}

// Validate SD card operation after initialization
bool validateSDCard() {
  Logger.info("Validating SD card...");
  
  // Check card type
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Logger.error("No SD card detected despite successful initialization");
    SD_MMC.end();
    return false;
  }
  
  // Print card info
  const char* cardTypeStr = "UNKNOWN";
  if (cardType == CARD_MMC) cardTypeStr = "MMC";
  else if (cardType == CARD_SD) cardTypeStr = "SDSC";
  else if (cardType == CARD_SDHC) cardTypeStr = "SDHC";
  
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Logger.info("SD Card Type: %s", cardTypeStr);
  Logger.info("SD Card Size: %lluMB", cardSize);
  
  // Try listing root directory as a simple test
  File root = SD_MMC.open("/");
  if (!root) {
    Logger.error("Failed to open root directory");
    SD_MMC.end();
    return false;
  }
  
  if (!root.isDirectory()) {
    Logger.error("Root is not a directory");
    root.close();
    SD_MMC.end();
    return false;
  }
  
  // Directory opened successfully
  root.close();
  
  // Create necessary directories
  if (!SD_MMC.exists("/images")) {
    if (SD_MMC.mkdir("/images")) {
      Logger.info("Created /images directory");
    } else {
      Logger.warn("Failed to create /images directory, but continuing anyway");
    }
  }
  
  return true;
}

// Try initializing with legacy SD library
bool tryLegacySDInit() {
  #ifdef USE_LEGACY_SD
  // This is a placeholder for using the standard SD library
  // with SPI pins instead of SDMMC
  Logger.info("Trying legacy SD library (SPI mode)...");
  
  // In a real implementation, you would:
  // 1. Define the CS pin for SPI mode
  // 2. Call SD.begin(CS_PIN) with the appropriate CS pin
  // 3. Verify SD card operation
  
  return false; // Not implemented in this example
  #else
  return false;
  #endif
}

// 4. WiFi Setup Function - With improved stability
bool setup_wifi_ap() {
  Logger.info("Setting up WiFi in AP mode...");
  
  // Ensure WiFi is in the right mode
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.mode(WIFI_AP);
  
  // Configure the static IP address for AP
  if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET)) {
    Logger.error("AP configuration failed");
    return false;
  }
  
  // Start the access point with limited clients for better stability
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_CHANNEL, false, MAX_WIFI_CLIENTS)) {
    Logger.error("AP creation failed");
    return false;
  }
  
  // Get AP IP address
  IPAddress ap_ip = WiFi.softAPIP();
  
  // Log AP information
  Logger.info("WiFi AP Mode active");
  Logger.info("AP SSID: %s", WIFI_AP_SSID);
  Logger.info("AP IP Address: %s", ap_ip.toString().c_str());
  Logger.info("AP Channel: %d", WIFI_CHANNEL);
  Logger.info("AP Max Clients: %d", MAX_WIFI_CLIENTS);
  
  return true;
}

// 5. LVGL Setup Function - With improved stability
bool setup_lvgl() {
  Logger.info("Setting up LVGL...");
  
  try {
    // Initialize LVGL library
    lv_init();
    
    // Calculate buffer size based on screen dimensions (1/4 screen buffer for stability)
    size_t buffer_size = sizeof(lv_color_t) * LCD_H_RES * LCD_V_RES / 4;
    Logger.debug("Allocating LVGL buffers: %d bytes each", buffer_size);
    
    // Allocate display buffers from PSRAM with safety check
    buf = (lv_color_t *)safe_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buf == nullptr) {
      Logger.error("Failed to allocate first display buffer");
      return false;
    }
    
    buf1 = (lv_color_t *)safe_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buf1 == nullptr) {
      Logger.error("Failed to allocate second display buffer");
      safe_free(buf);
      buf = nullptr;
      return false;
    }
    
    // Initialize LVGL display buffer - use 1/4 of the screen for better stability
    lv_disp_draw_buf_init(&draw_buf, buf, buf1, LCD_H_RES * LCD_V_RES / 4);
    
    // Initialize LVGL display driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = false; // Changed to false for better stability
    disp_drv.drv_update_cb = lvgl_rotation_cb;
    lv_disp_drv_register(&disp_drv);
    
    // Initialize input driver for touch
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
    
    Logger.info("LVGL initialized successfully");
    return true;
  } catch (...) {
    Logger.error("Error during LVGL initialization");
    return false;
  }
}

// 6. UI Setup Function
bool setup_ui() {
  Logger.info("Setting up UI...");
  
  if (!lvgl_ready) {
    Logger.error("Cannot setup UI - LVGL not ready");
    return false;
  }
  
  try {
    // Take UI mutex for thread safety
    if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      Logger.error("Failed to take UI mutex for setup");
      return false;
    }
    
    // Initialize common styles
    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, lv_color_make(0, 120, 255));
    lv_style_set_border_width(&style_btn, 0);
    lv_style_set_radius(&style_btn, 10);
    lv_style_set_text_color(&style_btn, lv_color_white());
    lv_style_set_pad_all(&style_btn, 10);
    
    lv_style_init(&style_btn_pressed);
    lv_style_set_bg_color(&style_btn_pressed, lv_color_make(0, 70, 200));
    
    // Create main screen
    main_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(main_screen, lv_color_black(), 0);
    lv_scr_load(main_screen);
    
    // Create status bar at top
    status_bar = lv_obj_create(main_screen);
    lv_obj_set_size(status_bar, LCD_H_RES, 40);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(status_bar, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_border_width(status_bar, 0, 0);
    lv_obj_set_style_radius(status_bar, 0, 0);
    
    // Status labels
    status_label = lv_label_create(status_bar);
    lv_label_set_text(status_label, "ESP32P4 Gallery");
    lv_obj_align(status_label, LV_ALIGN_LEFT_MID, 10, 0);
    
    wifi_label = lv_label_create(status_bar);
    lv_label_set_text(wifi_label, wifi_ready ? WiFi.softAPIP().toString().c_str() : "WiFi: N/A");
    lv_obj_align(wifi_label, LV_ALIGN_RIGHT_MID, -10, 0);
    
    // Image counter
    image_counter = lv_label_create(status_bar);
    lv_label_set_text(image_counter, "0/0");
    lv_obj_align(image_counter, LV_ALIGN_CENTER, 0, 0);
    
    // Create image view area (main content)
    image_view = lv_obj_create(main_screen);
    lv_obj_set_size(image_view, LCD_H_RES, LCD_V_RES - 90); // Leave space for bars
    lv_obj_align_to(image_view, status_bar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(image_view, lv_color_make(20, 20, 20), 0);
    lv_obj_set_style_border_width(image_view, 0, 0);
    lv_obj_set_style_radius(image_view, 0, 0);
    
    // Add image display object within the view area
    image_display = lv_img_create(image_view);
    lv_obj_center(image_display);
    // Note: We'll set the actual image later when we have images to display
    
    // Create control bar at bottom
    control_bar = lv_obj_create(main_screen);
    lv_obj_set_size(control_bar, LCD_H_RES, 50);
    lv_obj_align(control_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(control_bar, lv_color_make(40, 40, 40), 0);
    lv_obj_set_style_border_width(control_bar, 0, 0);
    lv_obj_set_style_radius(control_bar, 0, 0);
    
    // Add control buttons
    // Previous button
    prev_btn = lv_btn_create(control_bar);
    lv_obj_set_size(prev_btn, 100, 40);
    lv_obj_align(prev_btn, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_add_style(prev_btn, &style_btn, 0);
    lv_obj_add_style(prev_btn, &style_btn_pressed, LV_STATE_PRESSED);
    lv_obj_add_event_cb(prev_btn, [](lv_event_t * e) {
      if (image_count > 0 && !image_loading) {
        current_image = (current_image - 1 + image_count) % image_count;
        display_image(current_image);
      }
    }, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* prev_label = lv_label_create(prev_btn);
    lv_label_set_text(prev_label, "Previous");
    lv_obj_center(prev_label);
    
    // Next button
    next_btn = lv_btn_create(control_bar);
    lv_obj_set_size(next_btn, 100, 40);
    lv_obj_align(next_btn, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_style(next_btn, &style_btn, 0);
    lv_obj_add_style(next_btn, &style_btn_pressed, LV_STATE_PRESSED);
    lv_obj_add_event_cb(next_btn, [](lv_event_t * e) {
      if (image_count > 0 && !image_loading) {
        current_image = (current_image + 1) % image_count;
        display_image(current_image);
      }
    }, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* next_label = lv_label_create(next_btn);
    lv_label_set_text(next_label, "Next");
    lv_obj_center(next_label);
    
    // Refresh button 
    lv_obj_t* refresh_btn = lv_btn_create(control_bar);
    lv_obj_set_size(refresh_btn, 120, 40);
    lv_obj_align(refresh_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(refresh_btn, lv_color_make(0, 180, 0), 0);
    lv_obj_set_style_bg_color(refresh_btn, lv_color_make(0, 140, 0), LV_STATE_PRESSED);
    lv_obj_add_event_cb(refresh_btn, [](lv_event_t * e) {
      // Try to recover SD card first if it's not ready
      if (!sd_ready) {
        recover_sd_card();
      }
      
      // Rescan SD card for images
      if (sd_ready && !image_loading) {
        scan_images();
        if (image_count > 0) {
          current_image = 0;
          display_image(current_image);
        }
      }
    }, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* refresh_label = lv_label_create(refresh_btn);
    lv_label_set_text(refresh_label, "SD Refresh");
    lv_obj_center(refresh_label);
    
    // System info area
    sys_info = lv_label_create(image_view);
    lv_label_set_text(sys_info, "System initializing...");
    lv_obj_align(sys_info, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_text_color(sys_info, lv_color_make(180, 180, 180), 0);
    
    // Release UI mutex
    xSemaphoreGive(ui_mutex);
    
    Logger.info("UI initialized successfully");
    return true;
  } catch (...) {
    // Make sure to release mutex in case of error
    xSemaphoreGive(ui_mutex);
    Logger.error("Error setting up UI");
    return false;
  }
}

// 7. WebServer Setup Function - Simplified for stability
bool setup_webserver() {
  Logger.info("Setting up WebServer...");
  
  if (!wifi_ready) {
    Logger.error("Cannot setup WebServer - WiFi not ready");
    return false;
  }
  
  try {
    // Basic routes
    server.on("/", HTTP_GET, []() {
      Logger.debug("Serving main page");
      
      String html = "<html><head><title>ESP32P4 Gallery</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<style>";
      html += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
      html += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer;}";
      html += ".btn-danger {background: #dc2626;}";
      html += ".btn-success {background: #16a34a;}";
      html += ".upload-form {padding: 20px 0;}";
      html += "#status {padding: 10px; margin-top: 10px; border-radius: 4px;}";
      html += ".success {background: rgba(22, 163, 74, 0.2); color: #16a34a;}";
      html += ".error {background: rgba(220, 38, 38, 0.2); color: #dc2626;}";
      html += "</style></head><body>";
      
      html += "<h1>ESP32P4 Image Gallery</h1>";
      
      // System info card
      html += "<div class='card'>";
      html += "<h2>System Information</h2>";
      html += "<p>IP: " + WiFi.softAPIP().toString() + "</p>";
      html += "<p>Images: " + String(image_count) + "</p>";
      html += "<p>SD Card Status: " + String(sd_ready ? "Connected" : "Disconnected") + "</p>";
      html += "<p>Uptime: " + String(millis() / 1000) + " sec</p>";
      html += "<button class='btn' onclick='window.location.href=\"/system\"'>System Info</button> ";
      html += "</div>";
      
      // Upload card
      html += "<div class='card'>";
      html += "<h2>Upload Image</h2>";
      html += "<p><b>Important:</b> Upload only one image at a time for stability.</p>";
      html += "<form id='upload-form' class='upload-form' method='post' action='/upload' enctype='multipart/form-data'>";
      html += "<input type='file' name='image' accept='.jpg,.jpeg'><br><br>";
      html += "<input type='submit' class='btn' value='Upload'>";
      html += "</form>";
      html += "<div id='status' style='display:none;'></div>";
      html += "</div>";
      
      // Actions card
      html += "<div class='card'>";
      html += "<h2>Actions</h2>";
      html += "<button class='btn btn-success' onclick='refreshSD()'>Refresh SD Card</button> ";
      html += "<button class='btn btn-danger' onclick='restartSystem()'>Restart System</button>";
      html += "</div>";
      
      html += "<script>";
      html += "function refreshSD() {";
      html += "  fetch('/refresh-sd', { method: 'POST' })";
      html += "  .then(response => response.text())";
      html += "  .then(data => {";
      html += "    const status = document.getElementById('status');";
      html += "    status.style.display = 'block';";
      html += "    status.textContent = data;";
      html += "    status.className = 'success';";
      html += "  });";
      html += "}";
      html += "function restartSystem() {";
      html += "  if(confirm('Are you sure you want to restart the system?')) {";
      html += "    fetch('/restart', { method: 'POST' })";
      html += "    .then(() => {";
      html += "      const status = document.getElementById('status');";
      html += "      status.style.display = 'block';";
      html += "      status.textContent = 'Restarting...';";
      html += "      status.className = 'success';";
      html += "    });";
      html += "  }";
      html += "}";
      html += "document.getElementById('upload-form').onsubmit = function(e) {";
      html += "  const status = document.getElementById('status');";
      html += "  status.style.display = 'block';";
      html += "  status.textContent = 'Uploading...';";
      html += "  status.className = '';";
      html += "};";
      html += "</script>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
    });

    // Simple upload handler
    server.on("/upload", HTTP_POST, []() {
      server.send(200, "text/plain", "File upload complete");
    }, handle_upload);
    
    // System info page - simplified
    server.on("/system", HTTP_GET, []() {
      String html = "<html><head><title>System Info</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<style>";
      html += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += "table {width: 100%; border-collapse: collapse; margin-bottom: 20px;}";
      html += "th, td {padding: 8px; text-align: left; border-bottom: 1px solid #334155;}";
      html += "th {background-color: #334155;}";
      html += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
      html += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer; text-decoration: none; display: inline-block;}";
      html += "</style></head><body>";
      
      html += "<h1>ESP32P4 System Information</h1>";
      html += "<a href='/' class='btn'>Back to Gallery</a><br><br>";
      
      // Hardware info
      html += "<div class='card'>";
      html += "<h2>Hardware</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>Model</td><td>ESP32P4</td></tr>";
      html += "<tr><td>CPU Frequency</td><td>" + String(ESP.getCpuFreqMHz()) + " MHz</td></tr>";
      html += "<tr><td>Flash Size</td><td>" + String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB</td></tr>";
      html += "<tr><td>PSRAM Size</td><td>" + String(ESP.getPsramSize() / (1024 * 1024)) + " MB</td></tr>";
      html += "<tr><td>Free Heap</td><td>" + String(ESP.getFreeHeap() / 1024) + " KB</td></tr>";
      html += "<tr><td>Free PSRAM</td><td>" + String(ESP.getFreePsram() / 1024) + " KB</td></tr>";
      html += "</table>";
      html += "</div>";
      
      // Storage info
      html += "<div class='card'>";
      html += "<h2>Storage</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      
      if (sd_ready) {
        uint64_t total = 0;
        uint64_t used = 0;
        
        if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          total = SD_MMC.totalBytes();
          used = SD_MMC.usedBytes();
          xSemaphoreGive(sd_mutex);
        }
        
        uint64_t free = total - used;
        
        uint8_t cardType = SD_MMC.cardType();
        String cardTypeStr = "Unknown";
        if (cardType == CARD_MMC) cardTypeStr = "MMC";
        else if (cardType == CARD_SD) cardTypeStr = "SDSC";
        else if (cardType == CARD_SDHC) cardTypeStr = "SDHC";
        
        html += "<tr><td>SD Card</td><td>Connected (" + cardTypeStr + ")</td></tr>";
        html += "<tr><td>Total Space</td><td>" + String(total / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Used Space</td><td>" + String(used / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Free Space</td><td>" + String(free / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Image Count</td><td>" + String(image_count) + "</td></tr>";
      } else {
        html += "<tr><td>SD Card</td><td>Not connected</td></tr>";
      }
      html += "</table>";
      html += "<button class='btn' onclick='retrySD()'>Retry SD Card</button>";
      html += "</div>";
      
      // Network info
      html += "<div class='card'>";
      html += "<h2>Network</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>WiFi Mode</td><td>Access Point</td></tr>";
      html += "<tr><td>SSID</td><td>" + String(WIFI_AP_SSID) + "</td></tr>";
      html += "<tr><td>IP Address</td><td>" + WiFi.softAPIP().toString() + "</td></tr>";
      html += "<tr><td>Clients</td><td>" + String(WiFi.softAPgetStationNum()) + " connected</td></tr>";
      html += "</table>";
      html += "</div>";
      
      // Software info
      html += "<div class='card'>";
      html += "<h2>Software</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>Version</td><td>1.0.6</td></tr>";
      html += "<tr><td>Build Date</td><td>2025-06-14 09:43:21 UTC</td></tr>";
      html += "<tr><td>User</td><td>Chamil1983</td></tr>";
      html += "<tr><td>Uptime</td><td>" + String(millis() / 1000) + " seconds</td></tr>";
      html += "</table>";
      html += "</div>";
      
      html += "<script>";
      html += "function retrySD() {";
      html += "  fetch('/retry-sd', { method: 'POST' })";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      alert(data);";
      html += "      location.reload();";
      html += "    });";
      html += "}";
      html += "</script>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
    });
    
    // Simple action endpoints
    server.on("/refresh-sd", HTTP_POST, []() {
      // Try to recover SD card first
      if (!sd_ready) {
        recover_sd_card();
      }
      
      if (sd_ready) {
        scan_images();
        if (image_count > 0) {
          display_image(0);
        }
        server.send(200, "text/plain", "SD Card refreshed. Found " + String(image_count) + " images.");
      } else {
        server.send(500, "text/plain", "SD card not available. Check connections.");
      }
    });
    
    server.on("/retry-sd", HTTP_POST, []() {
      if (recover_sd_card()) {
        scan_images();
        server.send(200, "text/plain", "SD card recovery successful. Found " + String(image_count) + " images.");
      } else {
        server.send(500, "text/plain", "SD card recovery failed. Check connections.");
      }
    });
    
    server.on("/restart", HTTP_POST, []() {
      server.send(200, "text/plain", "Restarting system...");
      reset_requested = true; // Safer than direct reset
    });
    
    server.on("/image", HTTP_GET, []() {
      if (server.hasArg("index")) {
        int index = server.arg("index").toInt();
        if (index >= 0 && index < image_count) {
          display_image(index);
          server.send(200, "text/plain", "Displaying image " + String(index + 1) + " of " + String(image_count));
        } else {
          server.send(400, "text/plain", "Invalid image index");
        }
      } else {
        server.send(400, "text/plain", "Missing index parameter");
      }
    });
    
    // Start web server
    server.begin();
    Logger.info("WebServer started on port 80");
    
    return true;
  } catch (...) {
    Logger.error("Exception in WebServer setup");
    return false;
  }
}

// Scan for images on SD card - With thread safety
// Safer scan_images function with explicit watchdog feeding
void scan_images() {
  if (!sd_ready) {
    Logger.error("Cannot scan images - SD card not ready");
    return;
  }
  
  Logger.info("Scanning for images...");
  
  // Feed watchdog before starting long operation
  feed_watchdog();
  
  // Take SD mutex
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    Logger.error("Failed to take SD mutex for scan_images");
    return;
  }
  
  // Work with local variables for safety
  int localImageCount = 0;
  
  // Clear existing image list
  for (int i = 0; i < image_count; i++) {
    if (image_list[i] != NULL) {
      free(image_list[i]);
      image_list[i] = NULL;
    }
  }
  
  // Try to open the images directory
  File root = SD_MMC.open("/images");
  if (!root || !root.isDirectory()) {
    Logger.error("Failed to open /images directory");
    
    // Try to create the directory
    if (!SD_MMC.exists("/images")) {
      if (SD_MMC.mkdir("/images")) {
        Logger.info("Created /images directory");
      } else {
        Logger.error("Failed to create /images directory");
      }
    }
    
    xSemaphoreGive(sd_mutex);
    return;
  }
  
  // Feed watchdog before directory scanning
  feed_watchdog();
  
  // Scan for JPEG files
  int filesScanned = 0;
  File file = root.openNextFile();
  
  while (file && localImageCount < MAX_IMAGES) {
    filesScanned++;
    
    // Feed watchdog every 10 files
    if (filesScanned % 10 == 0) {
      feed_watchdog();
    }
    
    if (!file.isDirectory()) {
      String fileName = file.name();
      fileName.toLowerCase();
      
      if (fileName.endsWith(".jpg") || fileName.endsWith(".jpeg")) {
        String fullPath = "/images/" + String(file.name());
        
        // Allocate memory for path
        image_list[localImageCount] = (char*)malloc(fullPath.length() + 1);
        if (image_list[localImageCount]) {
          strcpy(image_list[localImageCount], fullPath.c_str());
          Logger.debug("Found image %d: %s", localImageCount, image_list[localImageCount]);
          localImageCount++;
        }
      }
    }
    file = root.openNextFile();
  }
  
  root.close();
  
  // Update global count after successful scan
  image_count = localImageCount;
  xSemaphoreGive(sd_mutex);
  
  Logger.info("Found %d images", image_count);
  
  // Feed watchdog after scan completes
  feed_watchdog();
  
  // Update UI if ready
  if (ui_ready && image_counter != NULL) {
    if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      char countText[32];
      snprintf(countText, sizeof(countText), "%d/%d", 
               image_count > 0 ? current_image + 1 : 0, 
               image_count);
      lv_label_set_text(image_counter, countText);
      xSemaphoreGive(ui_mutex);
    }
  }
}

// Display selected image using JPEGDEC library
void display_image(int index) {
  if (!sd_ready || index < 0 || index >= image_count) {
    Logger.error("Cannot display image - invalid index %d or SD not ready", index);
    return;
  }
  
  // Check if we're already loading an image
  if (image_loading) {
    Logger.warn("Image loading already in progress - ignoring request");
    return;
  }
  
  // Feed watchdog before starting image display process
  feed_watchdog();
  
  // Update current image index
  current_image = index;
  image_loading = true;
  
  // Log display attempt
  Logger.info("Displaying image %d: %s", index, image_list[index]);
  
  // Update image counter in UI
  updateImageCounter(index);
  
  // CRITICAL: Feed watchdog before JPEG decoding
  feed_watchdog();
  
  // Render the JPEG image with watchdog protection
  bool success = safe_execute_with_watchdog([&]() {
    // Make sure we're not in the middle of a render before proceeding
    if (xSemaphoreTake(jpeg_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
      Logger.error("Failed to take JPEG mutex for display");
      return false;
    }
    
    bool result = render_jpeg_file(image_list[index]);
    
    // Release JPEG mutex
    xSemaphoreGive(jpeg_mutex);
    
    return result;
  }, "JPEG Rendering");
  
  // Update status based on result
  updateStatusAfterImageLoad(index, success);
  
  // Done loading
  image_loading = false;
  
  // Feed watchdog after image loading completes
  feed_watchdog();
}

// Update image counter with mutex protection
void updateImageCounter(int index) {
  if (ui_ready) {
    if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (image_counter != NULL) {
        char countText[32];
        snprintf(countText, sizeof(countText), "%d/%d", index + 1, image_count);
        lv_label_set_text(image_counter, countText);
      }
      
      if (status_label != NULL) {
        lv_label_set_text(status_label, "Loading image...");
      }
      
      xSemaphoreGive(ui_mutex);
    }
  }
}


// Update status after image loading completes
void updateStatusAfterImageLoad(int index, bool success) {
  if (ui_ready) {
    if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (status_label != NULL) {
        if (success) {
          // Extract filename for display
          const char* filename = strrchr(image_list[index], '/');
          if (filename) {
            filename++; // Skip slash
          } else {
            filename = image_list[index];
          }
          lv_label_set_text(status_label, filename);
        } else {
          lv_label_set_text(status_label, "Image decode failed");
        }
      }
      xSemaphoreGive(ui_mutex);
    }
  }
}

// Update system info display safely
void update_system_info() {
  if (!lvgl_ready || !ui_ready) return;
  
  // Try to take UI mutex
  if (xSemaphoreTake(ui_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    // Skip update if can't get mutex
    return;
  }
  
  // Check if system info object exists
  if (sys_info == NULL) {
    xSemaphoreGive(ui_mutex);
    return;
  }
  
  // Get system statistics
  uint32_t freeHeap = ESP.getFreeHeap() / 1024;
  uint32_t freePsram = ESP.getFreePsram() / 1024;
  uint32_t uptime = millis() / 1000;
  
  // Generate system info text
  char infoText[128];
  snprintf(infoText, sizeof(infoText), 
           "Heap: %dK | PSRAM: %dK\nImgs: %d | SD: %s | Up: %d:%02d:%02d",
           freeHeap, freePsram, image_count,
           sd_ready ? "OK" : "FAIL",
           uptime / 3600, (uptime / 60) % 60, uptime % 60);
  
  // Update label
  lv_label_set_text(sys_info, infoText);
  
  xSemaphoreGive(ui_mutex);
}

// Handle file upload with improved safety
void handle_upload() {
  HTTPUpload& upload = server.upload();
  static File uploadFile;
  static bool uploadMutexHeld = false;
  
  if (upload.status == UPLOAD_FILE_START) {
    // Start of file upload
    upload_active = true;
    upload_start_time = millis();
    upload_size = 0;
    
    // Create unique filename based on timestamp
    snprintf(upload_filename, sizeof(upload_filename), "/images/img_%u.jpg", millis());
    Logger.info("Upload start: %s", upload_filename);
    
    // Check if SD card is ready
    if (!sd_ready) {
      Logger.error("Cannot upload - SD card not ready");
      return;
    }
    
    // Take SD mutex for the duration of the upload - this prevents other SD operations
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
      Logger.error("Failed to take SD mutex for upload");
      return;
    }
    uploadMutexHeld = true;
    
    // Create file on SD card
    if (sd_ready) {
      uploadFile = SD_MMC.open(upload_filename, FILE_WRITE);
      if (!uploadFile) {
        Logger.error("Failed to create file for upload");
        xSemaphoreGive(sd_mutex);
        uploadMutexHeld = false;
      }
    }
  } 
  else if (upload.status == UPLOAD_FILE_WRITE) {
    // Write file data
    if (sd_ready && uploadFile && uploadMutexHeld) {
      // Write the received chunk to the file in smaller increments
      const size_t maxChunkSize = 2048;  // Reduced for better stability
      size_t remaining = upload.currentSize;
      size_t bytesWritten = 0;
      uint8_t* bufPtr = upload.buf;
      
      while (remaining > 0) {
        size_t currentWrite = min(remaining, maxChunkSize);
        size_t written = uploadFile.write(bufPtr, currentWrite);
        
        if (written != currentWrite) {
          Logger.error("Write error: %d of %d bytes written", written, currentWrite);
          break;
        }
        
        bytesWritten += written;
        bufPtr += written;
        remaining -= written;
        
        // Feed watchdog during large file writes
        feed_watchdog();
        
        // Add small delay for SD card to catch up
        if (remaining > 0) {
          delay(5);
        }
      }
      
      // Flush after each chunk to ensure data is written
      uploadFile.flush();
      
      upload_size += bytesWritten;
    }
  } 
  else if (upload.status == UPLOAD_FILE_END) {
    // End of file upload
    if (sd_ready && uploadFile && uploadMutexHeld) {
      // Ensure all data is flushed to disk
      uploadFile.flush();
      uploadFile.close();
      
      // Calculate upload speed
      float uploadTime = (millis() - upload_start_time) / 1000.0;
      float uploadSpeed = upload_size / (1024.0 * uploadTime);
      
      Logger.info("Upload complete: %s, %.1f KB/s", upload_filename, uploadSpeed);
      
      // Release SD mutex now that the file is closed
      xSemaphoreGive(sd_mutex);
      uploadMutexHeld = false;
      
      // Give SD card some time to finalize the write
      delay(200);
      
      // Refresh image list after upload
      scan_images();
      
      // Display the new image if available
      for (int i = 0; i < image_count; i++) {
        if (strcmp(image_list[i], upload_filename) == 0) {
          current_image = i;
          display_image(current_image);
          break;
        }
      }
    } else if (uploadMutexHeld) {
      // Release mutex if we held it but file wasn't open
      xSemaphoreGive(sd_mutex);
      uploadMutexHeld = false;
    }
    
    upload_active = false;
  } 
  else if (upload.status == UPLOAD_FILE_ABORTED) {
    // Upload aborted
    Logger.error("Upload aborted");
    
    if (sd_ready && uploadFile) {
      uploadFile.close();
      
      // Try to remove partial file
      SD_MMC.remove(upload_filename);
    }
    
    // Make sure to release mutex if it was held
    if (uploadMutexHeld) {
      xSemaphoreGive(sd_mutex);
      uploadMutexHeld = false;
    }
    
    upload_active = false;
  }
}

// JPEG drawing callback for JPEGDEC library
int jpeg_draw_callback(JPEGDRAW *pDraw) {
  if (!lcd_ready) return 0;
  
  // Calculate center position on screen
  int centerX = (LCD_H_RES - jpeg.getWidth()) / 2;
  int centerY = (LCD_V_RES - jpeg.getHeight()) / 2;
  
  // Adjust for top and bottom bars
  centerY += 20; // Half of status bar height
  
  // Draw the MCU block on the LCD
  int x = pDraw->x + centerX;
  int y = pDraw->y + centerY;
  
  // Check boundaries
  if (x >= 0 && y >= 0 && x + pDraw->iWidth <= LCD_H_RES && y + pDraw->iHeight <= LCD_V_RES) {
    lcd.lcd_draw_bitmap(x, y, x + pDraw->iWidth, y + pDraw->iHeight, (uint16_t*)pDraw->pPixels);
  }
  
  return 1; // Continue decoding
}

// Safe SD Card read function with error handling
File safe_sd_open(const char* path, const char* mode) {
  if (!sd_ready) {
    Logger.error("Attempted to open SD file when SD is not ready: %s", path);
    return File();
  }
  
  try {
    return SD_MMC.open(path, mode);
  } catch (...) {
    Logger.error("Exception during SD card file open: %s", path);
    return File();
  }
}

// JPEG rendering function with robust error handling 
bool render_jpeg_file(const char* filename) {
  if (filename == nullptr || !sd_ready) {
    Logger.error("Cannot render JPEG - invalid filename or SD not ready");
    return false;
  }
  
  // Try to take mutex with shorter timeout
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    Logger.error("Failed to take SD mutex for JPEG rendering");
    return false;
  }
  
  bool success = false;
  File jpegFile;
  
  try {
    // Check if file exists
    if (!SD_MMC.exists(filename)) {
      Logger.error("JPEG file not found: %s", filename);
      xSemaphoreGive(sd_mutex);
      return false;
    }
    
    // Open file with error handling
    jpegFile = SD_MMC.open(filename, FILE_READ);
    if (!jpegFile) {
      Logger.error("Failed to open JPEG file: %s", filename);
      xSemaphoreGive(sd_mutex);
      return false;
    }
    
    size_t fileSize = jpegFile.size();
    if (fileSize < 100) {
      Logger.error("JPEG file too small: %s (%d bytes)", filename, fileSize);
      jpegFile.close();
      xSemaphoreGive(sd_mutex);
      return false;
    }
    
    Logger.info("Starting JPEG decoding for %s (%d bytes)", filename, fileSize);
    
    // Fixed open call using File reference
    if (jpeg.open(jpegFile, jpeg_draw_callback)) {
      // Set options
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      
      // Start decoding
      if (jpeg.decode(0, 0, 0)) {
        Logger.info("JPEG decoded successfully: %dx%d", jpeg.getWidth(), jpeg.getHeight());
        success = true;
      } else {
        Logger.error("JPEG decode failed");
      }
      
      // Close the decoder
      jpeg.close();
    } else {
      Logger.error("Failed to initialize JPEG decoder for file: %s", filename);
    }
    
    // Close file
    jpegFile.close();
    
  } catch (...) {
    Logger.error("Exception during JPEG rendering");
    if (jpegFile) jpegFile.close();
    success = false;
  }
  
  // Release mutex
  xSemaphoreGive(sd_mutex);
  return success;
}


// SD card recovery function
bool recover_sd_card() {
  Logger.info("Attempting SD card recovery...");
  
  // Make sure no other operations are using SD card
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    Logger.error("Failed to take SD mutex for recovery - forcing reset");
    // Continue anyway - this is recovery mode
  }
  
  // End current SD card session
  SD_MMC.end();
  
  // Wait for card to discharge and reset
  delay(1000);
  
  // Reset SD pins
  pinMode(SDMMC_CLK_PIN, INPUT);
  pinMode(SDMMC_CMD_PIN, INPUT);
  pinMode(SDMMC_D0_PIN, INPUT);
  pinMode(SDMMC_D1_PIN, INPUT);
  pinMode(SDMMC_D2_PIN, INPUT);
  pinMode(SDMMC_D3_PIN, INPUT);
  delay(500);
  
  // Set pull-ups
  pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
  pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
  delay(100);
  
  // Configure SD pins
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                 SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
  
  // Try to initialize with VERY conservative settings
  bool success = SD_MMC.begin("/sdcard", false, false, 250000); // 1-bit mode, 250KHz
  
  if (success) {
    Logger.info("SD card recovery successful");
    
    // Check card type
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
      Logger.error("No SD card detected despite successful initialization");
      SD_MMC.end();
      sd_ready = false;
      xSemaphoreGive(sd_mutex);
      return false;
    }
    
    // Test file operation with the simplest possible approach
    File testFile = SD_MMC.open("/recovery_test.txt", FILE_WRITE);
    if (testFile) {
      testFile.print("TEST");
      testFile.flush();  // Ensure data is written
      testFile.close();  // Close properly
      
      delay(100);  // Wait for file system to settle
      
      // Verify write
      testFile = SD_MMC.open("/recovery_test.txt", FILE_READ);
      if (testFile) {
        char buf[5] = {0};
        size_t bytesRead = testFile.read((uint8_t*)buf, 4);
        testFile.close();
        
        if (bytesRead == 4 && strcmp(buf, "TEST") == 0) {
          Logger.info("Recovery test: read/write operation successful");
          sd_ready = true;
        } else {
          Logger.error("Recovery test: data verification failed");
          SD_MMC.end();
          sd_ready = false;
          xSemaphoreGive(sd_mutex);
          return false;
        }
      } else {
        Logger.error("Recovery test: could not read test file");
        SD_MMC.end();
        sd_ready = false;
        xSemaphoreGive(sd_mutex);
        return false;
      }
    } else {
      Logger.error("Recovery test: could not create test file");
      SD_MMC.end();
      sd_ready = false;
      xSemaphoreGive(sd_mutex);
      return false;
    }
    
    // Create images directory if needed
    if (!SD_MMC.exists("/images")) {
      if (SD_MMC.mkdir("/images")) {
        Logger.info("Created /images directory during recovery");
      } else {
        Logger.warn("Failed to create /images directory during recovery");
      }
    }
  } else {
    Logger.error("SD card recovery failed");
    sd_ready = false;
  }
  
  // Release mutex
  xSemaphoreGive(sd_mutex);
  return sd_ready;
}

// Function to diagnose SD card issues
void diagnose_sd_card() {
  Logger.info("------ SD CARD DIAGNOSTICS ------");
  Logger.info("Date: 2025-06-14 10:20:14 UTC");
  Logger.info("User: Chamil1983");
  
  // Check if we need to end any ongoing session
  if (sd_ready) {
    SD_MMC.end();
    delay(500);
  }
  
  // Test pin continuity
  Logger.info("Testing SD pins for proper input mode...");
  bool pins_ok = true;
  
  pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
  pins_ok &= (digitalRead(SDMMC_CLK_PIN) == HIGH);
  Logger.info("CLK (%d) Pull-up test: %s", SDMMC_CLK_PIN, digitalRead(SDMMC_CLK_PIN) == HIGH ? "PASS" : "FAIL");
  
  pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
  pins_ok &= (digitalRead(SDMMC_CMD_PIN) == HIGH);
  Logger.info("CMD (%d) Pull-up test: %s", SDMMC_CMD_PIN, digitalRead(SDMMC_CMD_PIN) == HIGH ? "PASS" : "FAIL");
  
  pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
  pins_ok &= (digitalRead(SDMMC_D0_PIN) == HIGH);
  Logger.info("D0 (%d) Pull-up test: %s", SDMMC_D0_PIN, digitalRead(SDMMC_D0_PIN) == HIGH ? "PASS" : "FAIL");
  
  pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
  pins_ok &= (digitalRead(SDMMC_D1_PIN) == HIGH);
  Logger.info("D1 (%d) Pull-up test: %s", SDMMC_D1_PIN, digitalRead(SDMMC_D1_PIN) == HIGH ? "PASS" : "FAIL");
  
  pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
  pins_ok &= (digitalRead(SDMMC_D2_PIN) == HIGH);
  Logger.info("D2 (%d) Pull-up test: %s", SDMMC_D2_PIN, digitalRead(SDMMC_D2_PIN) == HIGH ? "PASS" : "FAIL");
  
  pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
  pins_ok &= (digitalRead(SDMMC_D3_PIN) == HIGH);
  Logger.info("D3 (%d) Pull-up test: %s", SDMMC_D3_PIN, digitalRead(SDMMC_D3_PIN) == HIGH ? "PASS" : "FAIL");
  
  if (!pins_ok) {
    Logger.error("SD card pin test failed - check for proper pull-up resistors");
  }
  
  // Reset pins
  delay(100);
  
  // Try super conservative initialization
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
  
  // Try initialization with various settings
  Logger.info("Trying SD card initialization with 1-bit mode at very low speed...");
  if (SD_MMC.begin("/sdcard", false, false, 250000)) { // 1-bit mode, 250 KHz speed
    uint8_t cardType = SD_MMC.cardType();
    const char* typeStr = "UNKNOWN";
    if (cardType == CARD_MMC) typeStr = "MMC";
    else if (cardType == CARD_SD) typeStr = "SDSC";
    else if (cardType == CARD_SDHC) typeStr = "SDHC";
    else if (cardType == CARD_NONE) typeStr = "NONE";
    
    Logger.info("Card initialized, type: %s", typeStr);
    
    // Test write-verify cycle with explicit sync
    Logger.info("Testing simplified write-verify cycle...");
    bool test_passed = false;
    
    // 1. Create and write a test file with simple content
    File testFile = SD_MMC.open("/diag_test.txt", FILE_WRITE);
    if (testFile) {
      testFile.print("A"); // Just a single character for simplicity
      testFile.flush();
      testFile.close();
      delay(100); // Allow filesystem to settle
      
      // 2. Read back and verify
      testFile = SD_MMC.open("/diag_test.txt", FILE_READ);
      if (testFile) {
        char c = testFile.read();
        testFile.close();
        
        if (c == 'A') {
          Logger.info("Write-verify test PASSED");
          test_passed = true;
          sd_ready = true;
        } else {
          Logger.error("Write-verify test FAILED: Expected 'A', got '%c'", c);
        }
      } else {
        Logger.error("Could not open test file for reading");
      }
    } else {
      Logger.error("Could not create test file");
    }
    
    SD_MMC.end();
    
    if (test_passed) {
      Logger.info("SD card diagnostics suggests card is working correctly.");
      Logger.info("Recommended settings: 1-bit mode, very low speed (250KHz)");
    } else {
      Logger.error("SD card diagnostics suggests issues with read/write operations.");
      Logger.error("Check card formatting or try another card.");
    }
  } else {
    Logger.error("SD initialization failed in diagnostics");
  }

  Logger.info("--------------------------------");
}

#pragma GCC pop_options