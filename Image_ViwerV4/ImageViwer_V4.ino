/*
 * ESP32P4 Image Gallery with Web Upload
 * 
 * Robust SD Card and Memory Management
 * 16MB Flash / 8MB PSRAM Configuration
 * 
 * Author: Chamil1983
 * Date: 2025-06-14
 * Last modified: 2025-06-14 13:28:00 UTC
 */

#pragma GCC push_options
#pragma GCC optimize("O2")  // Using O2 for better stability

// Include required libraries
#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD_MMC.h>
#include <FS.h>
#include <SPIFFS.h>
#include "JPEGDEC.h"  // Using the specified library
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
char *image_list[MAX_IMAGES];
volatile int image_count = 0;
volatile int current_image = 0;
uint32_t last_ui_update = 0;
uint32_t last_system_update = 0;

// JPEG buffer for drawing
#define DRAW_BUFFER_SIZE (32 * 1024)  // 32KB buffer
uint8_t *drawBuffer = nullptr;

// Global flag for image rotation (used by JPEG decoder and draw callback)
bool image_needs_rotation = false;

// Global flag to track image loading state
bool image_loading = false;


// Slideshow variables
bool slideshow_active = false;                                   // Flag to track if slideshow is running
uint32_t slideshow_last_change = 0;                              // Timestamp of last image change
uint32_t slideshow_interval = 5000;                              // Default interval: 5 seconds
hw_timer_t *slideshow_timer = NULL;                              // Hardware timer for slideshow
portMUX_TYPE slideshow_timerMux = portMUX_INITIALIZER_UNLOCKED;  // Timer mutex
TaskHandle_t slideshow_task_handle = NULL;                       // Task handle for slideshow



// UI elements
lv_obj_t *main_screen = nullptr;
lv_obj_t *status_bar = nullptr;
lv_obj_t *image_view = nullptr;
lv_obj_t *image_display = nullptr;  // Image display widget
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
#define UPLOAD_BUFFER_SIZE 4096        // 4KB buffer
#define SDMMC_FREQ_PROBING 125000      // Very slow speed for initial probing
#define RETRY_DELAY 1000               // Delay between retries
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
bool emergency_sd_recovery();
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
bool recover_sd_card_for_upload();
void display_image(int index);
void handle_upload();
bool recover_sd_card();
void diagnose_sd_card();
bool render_jpeg_file(const char *filename);
void lvgl_task(void *pvParameters);
int jpeg_draw_callback(JPEGDRAW *pDraw);
void clearLCDScreen();
void slideshowTask(void *parameter);
void startSlideshow(uint32_t interval_ms = 5000);
void stopSlideshow();
void toggleSlideshow(uint32_t interval_ms = 5000);
void handle_touch_event(uint16_t x, uint16_t y);
void check_buttons();
uint32_t max_u32(uint32_t a, uint32_t b);


// to ensure consistent debug logging throughout the code

#define DEBUG_ENABLED true

// Debug macros with forced output
#define DEBUG_PRINT(tag, format, ...) \
  do { \
    if (DEBUG_ENABLED) { \
      Serial.printf("[DEBUG][%s] ", tag); \
      Serial.printf(format, ##__VA_ARGS__); \
      Serial.println(); \
      Serial.flush(); \
    } \
  } while (0)

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

// Slideshow task function
void slideshowTask(void *parameter) {
  Logger.info("Slideshow task started");

  while (true) {
    // Check if slideshow is active and enough time has passed
    if (slideshow_active && (millis() - slideshow_last_change >= slideshow_interval)) {
      // Update timestamp
      slideshow_last_change = millis();

      // Move to next image
      int next_image = (current_image + 1) % image_count;

      // Log slideshow progress
      Logger.info("Slideshow: Changing to image %d of %d", next_image + 1, image_count);

      // Display next image
      display_image(next_image);

      // Feed watchdog to avoid reset during long operations
      feed_watchdog();
    }

    // Small delay to prevent CPU hogging
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// Function definitions (without default arguments)
void startSlideshow(uint32_t interval_ms) {
  if (image_count <= 1) {
    Logger.warn("Cannot start slideshow: Not enough images available");
    return;
  }
  
  // Update interval
  slideshow_interval = (interval_ms < 1000) ? 1000 : interval_ms;  // Minimum 1 secon
  
  // Reset timer
  slideshow_last_change = millis();
  
  // Set flag
  slideshow_active = true;
  
  Logger.info("Slideshow started with %d ms interval", slideshow_interval);
  
  // Create task if not already created
  if (slideshow_task_handle == NULL) {
    xTaskCreatePinnedToCore(
      slideshowTask,          // Function to implement the task
      "slideshow_task",       // Name of the task
      4096,                   // Stack size in words
      NULL,                   // Task input parameter
      1,                      // Priority of the task
      &slideshow_task_handle, // Task handle
      0);                     // Core where the task should run (0 = PRO_CPU)
  }
}

void stopSlideshow() {
  slideshow_active = false;
  Logger.info("Slideshow stopped");
}

void toggleSlideshow(uint32_t interval_ms) {
  if (slideshow_active) {
    stopSlideshow();
  } else {
    startSlideshow(interval_ms);
  }
}

// Assuming you have a touchscreen event handler, add this to handle slideshow toggle
void handle_touch_event(uint16_t x, uint16_t y) {
  static uint32_t last_touch_time = 0;
  uint32_t now = millis();
  
  // Simple debounce - ignore rapid touches
  if (now - last_touch_time < 500) {
    return;
  }
  
  last_touch_time = now;
  
  // Define touch regions
  // Bottom right corner for slideshow toggle
  if (x > (LCD_H_RES - 100) && y > (LCD_V_RES - 100)) {
    toggleSlideshow();
  } 
  // Top right corner for next image
  else if (x > (LCD_H_RES - 100) && y < 100) {
    int next_image = (current_image + 1) % image_count;
    display_image(next_image);
  }
  // Top left corner for previous image
  else if (x < 100 && y < 100) {
    int prev_image = (current_image - 1 + image_count) % image_count;
    display_image(prev_image);
  }
}

// Add this to your loop() function or wherever you check button states
void check_buttons() {
  // Example: button on GPIO pin 0
  static bool last_button_state = HIGH;
  bool button_state = digitalRead(0);
  
  // Check for button press (falling edge)
  if (button_state == LOW && last_button_state == HIGH) {
    toggleSlideshow();
  }
  
  last_button_state = button_state;
}



// Safe malloc wrapper with checks
void *safe_malloc(size_t size, uint32_t caps = MALLOC_CAP_DEFAULT) {
  if (size == 0 || size > 1024 * 1024) {
    Logger.error("Invalid memory allocation request: %u bytes", size);
    return nullptr;
  }

  void *ptr = nullptr;

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

// Helper function to get the maximum of two uint32_t values
uint32_t max_u32(uint32_t a, uint32_t b) {
  return (a > b) ? a : b;
}

// Safe free with null check
void safe_free(void *ptr) {
  if (ptr != nullptr) {
    free(ptr);
  }
}

// Initialize watchdog system with the right approach
// Update your setup_watchdog() function to better handle crashes
bool setup_watchdog() {
  Logger.info("Setting up watchdog system with crash recovery...");

  // First detect if WDT is already running
  esp_err_t status = ESP_OK;

  // Configure watchdog with a much longer timeout (30 seconds)
  // to give more time for operations like SD card access and JPEG decoding
  const uint32_t WDT_TIMEOUT_MS = 30000;

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = 0,   // Don't use idle detection
    .trigger_panic = true  // Use panic for debugging - this helps with crash detection
  };

  // Try to initialize or reconfigure the watchdog
  status = esp_task_wdt_reconfigure(&wdt_config);
  if (status == ESP_OK) {
    Logger.info("Watchdog reconfigured with %d second timeout and panic enabled", WDT_TIMEOUT_MS / 1000);
    wdt_enabled = true;
  } else {
    Logger.warn("Could not reconfigure watchdog: %d", status);

    // Try initializing instead
    status = esp_task_wdt_init(&wdt_config);
    if (status == ESP_OK) {
      Logger.info("Watchdog initialized with %d second timeout and panic enabled", WDT_TIMEOUT_MS / 1000);
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
      Logger.info("Main task added to watchdog with crash detection");
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
bool safe_execute_with_watchdog(Func operation, const char *operationName) {
  Logger.debug("Starting operation: %s", operationName);

  // Feed watchdog before operation
  feed_watchdog();

  uint32_t startTime = millis();
  bool result;

  try {
    // Execute the operation
    result = operation();
  } catch (const std::exception &e) {
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
  touchX = 800 - touchX;  // Adjust X coordinate for display orientation

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
    const esp_partition_t *part = esp_partition_get(it);
    Serial.printf("%4d | %7d | %08x | %6uK | %s\n",
                  part->type, part->subtype, part->address, part->size / 1024, part->label);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);

  // Look specifically for a SPIFFS partition
  it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
  if (it) {
    const esp_partition_t *part = esp_partition_get(it);
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

// Improved JPEG rendering with full resolution and proper centering
bool render_jpeg_file(const char *filename) {
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

    // Set decoder options for RGB565 format (16-bit per pixel)
    jpeg.setPixelType(RGB565_BIG_ENDIAN);

    // Open JPEG file for decoding
    if (jpeg.open(jpegFile, jpeg_draw_callback)) {
      // Get image details
      int imgWidth = jpeg.getWidth();
      int imgHeight = jpeg.getHeight();

      // Record orientation for the draw callback
      // For LCD dimensions of 800x1280 (portrait), landscape images (width > height) need rotation
      image_needs_rotation = (imgWidth > imgHeight) && (LCD_V_RES > LCD_H_RES);

      Logger.info("Decoding image at full resolution%s", image_needs_rotation ? " (with rotation)" : "");

      // Use scale 0 for full resolution
      if (jpeg.decode(0, 0, 0)) {  // 0 = no scaling (full resolution)
        if (image_needs_rotation) {
          Logger.info("JPEG decoded successfully (rotated): %dx%d -> %dx%d",
                      imgWidth, imgHeight, imgHeight, imgWidth);
        } else {
          Logger.info("JPEG decoded successfully: %dx%d", imgWidth, imgHeight);
        }
        success = true;
      } else {
        // If full resolution fails due to memory constraints, try half resolution
        Logger.warn("Full resolution decode failed, trying half resolution");
        if (jpeg.open(jpegFile, jpeg_draw_callback) && jpeg.decode(0, 0, JPEG_SCALE_HALF)) {
          Logger.info("JPEG decoded at half resolution");
          success = true;
        } else {
          Logger.error("JPEG decode failed at both resolutions");
        }
      }

      // Reset rotation flag
      image_needs_rotation = false;

      // Close the decoder
      jpeg.close();
    } else {
      Logger.error("Failed to initialize JPEG decoder for file: %s", filename);
    }

    // Close file
    jpegFile.close();

  } catch (...) {
    Logger.error("Unknown exception during JPEG rendering");
    if (jpegFile) jpegFile.close();
    success = false;
  }

  // Release mutex
  xSemaphoreGive(sd_mutex);
  return success;
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
  Logger.info("Date: 2025-06-14 14:15:04 UTC");
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

  // This will help recover from crashes
#ifdef CONFIG_ESP_TASK_WDT_PANIC
  // If task watchdog panic is enabled, we can use it for crash recovery
  Logger.info("Task watchdog panic is enabled for crash detection");
#else
  Logger.info("Task watchdog will timeout but not panic on crashes");
#endif

// Register a pre-sleep callback that will run before deep sleep or crash
#ifdef CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE
  // Not using esp_register_shutdown_handler as it might not be available in all versions
  // Instead, ensure WDT is properly configured to catch hangs
  Logger.info("System configured to handle crashes via watchdog");
#endif

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

  // Step 5: LVGL initialization
  Serial.println("5. LVGL initialization");
  DEBUG_INIT("LVGL");
  lvgl_ready = setup_lvgl();
  if (lvgl_ready) {
    DEBUG_SUCCESS("LVGL");
    Serial.println("LVGL ready");
  } else {
    DEBUG_FAIL("LVGL", "Could not initialize");
    Serial.println("LVGL failed");
  }

  // Step 6: UI initialization
  Serial.println("6. UI initialization");
  DEBUG_INIT("UI");
  if (lvgl_ready) {
    ui_ready = setup_ui();
    if (ui_ready) {
      DEBUG_SUCCESS("UI");
      Serial.println("UI ready");
    } else {
      DEBUG_FAIL("UI", "Could not initialize");
      Serial.println("UI failed");
    }
  } else {
    DEBUG_FAIL("UI", "LVGL not ready");
    Serial.println("UI skipped - LVGL not ready");
  }

  // Step 7: Web Server initialization
  Serial.println("7. Web Server initialization");
  DEBUG_INIT("WebServer");
  if (wifi_ready) {
    server_ready = setup_webserver();
    if (server_ready) {
      DEBUG_SUCCESS("WebServer");
      Serial.println("WebServer ready");
    } else {
      DEBUG_FAIL("WebServer", "Could not initialize");
      Serial.println("WebServer failed");
    }
  } else {
    DEBUG_FAIL("WebServer", "WiFi not ready");
    Serial.println("WebServer skipped - WiFi not ready");
  }

  // Step 8: Start LVGL task
  if (lvgl_ready) {
    Serial.println("8. Starting LVGL task");
    // Create LVGL handler task
    xTaskCreatePinnedToCore(
      lvgl_task,         /* Task function */
      "lvgl_task",       /* Name */
      8192,              /* Stack size (bytes) */
      NULL,              /* Parameter */
      5,                 /* Priority */
      &lvgl_task_handle, /* Task handle */
      1                  /* Core where the task should run */
    );

    if (lvgl_task_handle != NULL) {
      Serial.println("LVGL task started successfully");
    } else {
      Serial.println("Failed to start LVGL task");
    }
  }

  // Step 9: Scan for images if SD card is ready
  if (sd_ready) {
    Serial.println("9. Scanning for images");
    scan_images();
    Serial.printf("Found %d images\n", image_count);

    // If we have any images, display the first one
    if (image_count > 0) {
      Serial.println("10. Loading first image");
      display_image(0);
    }
  }

  Serial.println("Initialization complete!");
  Serial.printf("STATUS: LCD:%s Touch:%s SD:%s WiFi:%s LVGL:%s UI:%s Server:%s\n",
                lcd_ready ? "OK" : "FAIL",
                touch_ready ? "OK" : "FAIL",
                sd_ready ? "OK" : "FAIL",
                wifi_ready ? "OK" : "FAIL",
                lvgl_ready ? "OK" : "FAIL",
                ui_ready ? "OK" : "FAIL",
                server_ready ? "OK" : "FAIL");

  Serial.println("=========== After Setup Start ============");
  // Print memory information
  Serial.println("INTERNAL Memory Info:");
  Serial.println("------------------------------------------");
  Serial.printf("  Total Size        :   %6d B (%5.1f KB)\n", ESP.getHeapSize(), ESP.getHeapSize() / 1024.0);
  Serial.printf("  Free Bytes        :   %6d B (%5.1f KB)\n", ESP.getFreeHeap(), ESP.getFreeHeap() / 1024.0);
  Serial.printf("  Allocated Bytes   :   %6d B (%5.1f KB)\n", ESP.getHeapSize() - ESP.getFreeHeap(), (ESP.getHeapSize() - ESP.getFreeHeap()) / 1024.0);
  Serial.printf("  Minimum Free Bytes:   %6d B (%5.1f KB)\n", ESP.getMinFreeHeap(), ESP.getMinFreeHeap() / 1024.0);
  Serial.printf("  Largest Free Block:   %6d B (%5.1f KB)\n", ESP.getMaxAllocHeap(), ESP.getMaxAllocHeap() / 1024.0);
  Serial.println("------------------------------------------");

  // Print PSRAM information
  Serial.println("SPIRAM Memory Info:");
  Serial.println("------------------------------------------");
  Serial.printf("  Total Size        : %8d B (%5.1f KB)\n", ESP.getPsramSize(), ESP.getPsramSize() / 1024.0);
  Serial.printf("  Free Bytes        : %8d B (%5.1f KB)\n", ESP.getFreePsram(), ESP.getFreePsram() / 1024.0);
  Serial.printf("  Allocated Bytes   : %8d B (%5.1f KB)\n", ESP.getPsramSize() - ESP.getFreePsram(), (ESP.getPsramSize() - ESP.getFreePsram()) / 1024.0);
  Serial.printf("  Minimum Free Bytes: %8d B (%5.1f KB)\n", ESP.getMinFreePsram(), ESP.getMinFreePsram() / 1024.0);
  Serial.printf("  Largest Free Block: %8d B (%5.1f KB)\n", ESP.getMaxAllocPsram(), ESP.getMaxAllocPsram() / 1024.0);
  Serial.println("------------------------------------------");

  // Print GPIO information
  Serial.println("GPIO Info:");
  Serial.println("------------------------------------------");
  Serial.println("  GPIO : BUS_TYPE[bus/unit][chan]");
  Serial.println("  --------------------------------------  ");
  // Add the pins that are in use
  Serial.println("    23 : GPIO");
  Serial.println("    24 : USB_DM");
  Serial.println("    25 : USB_DP");
  Serial.println("    37 : UART_TX[0]");
  Serial.println("    38 : UART_RX[0]");
  Serial.println("    39 : SDMMC_D0");
  Serial.println("    40 : SDMMC_D1");
  Serial.println("    41 : SDMMC_D2");
  Serial.println("    42 : SDMMC_D3");
  Serial.println("    43 : SDMMC_CLK");
  Serial.println("    44 : SDMMC_CMD");
  Serial.println("    45 : SDMMC_POWER");

  Serial.println("============ After Setup End =============");

  Serial.flush();

  startSlideshow(10000); // Start with 10-second interval
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

  check_buttons();

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
  delay(2000);  // Wait for power to stabilize

  // Reset all SD pins to a known state
  Logger.info("Resetting SD pins...");

  // First set pins as input to avoid any conflicts
  pinMode(SDMMC_CLK_PIN, INPUT);
  pinMode(SDMMC_CMD_PIN, INPUT);
  pinMode(SDMMC_D0_PIN, INPUT);
  pinMode(SDMMC_D1_PIN, INPUT);
  pinMode(SDMMC_D2_PIN, INPUT);
  pinMode(SDMMC_D3_PIN, INPUT);
  delay(100);

  // Then set them as outputs to drive LOW
  pinMode(SDMMC_CLK_PIN, OUTPUT);
  pinMode(SDMMC_CMD_PIN, OUTPUT);
  pinMode(SDMMC_D0_PIN, OUTPUT);
  pinMode(SDMMC_D1_PIN, OUTPUT);
  pinMode(SDMMC_D2_PIN, OUTPUT);
  pinMode(SDMMC_D3_PIN, OUTPUT);

  // Drive all pins LOW to discharge
  digitalWrite(SDMMC_CLK_PIN, LOW);
  digitalWrite(SDMMC_CMD_PIN, LOW);
  digitalWrite(SDMMC_D0_PIN, LOW);
  digitalWrite(SDMMC_D1_PIN, LOW);
  digitalWrite(SDMMC_D2_PIN, LOW);
  digitalWrite(SDMMC_D3_PIN, LOW);
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
  delay(500);  // Longer delay for pullups to stabilize

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
  const char *cardTypeStr = "UNKNOWN";
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

  return false;  // Not implemented in this example
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
    disp_drv.full_refresh = false;  // Changed to false for better stability
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
    lv_obj_set_size(image_view, LCD_H_RES, LCD_V_RES - 90);  // Leave space for bars
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
    lv_obj_add_event_cb(
      prev_btn, [](lv_event_t *e) {
        if (image_count > 0 && !image_loading) {
          current_image = (current_image - 1 + image_count) % image_count;
          display_image(current_image);
        }
      },
      LV_EVENT_CLICKED, NULL);

    lv_obj_t *prev_label = lv_label_create(prev_btn);
    lv_label_set_text(prev_label, "Previous");
    lv_obj_center(prev_label);

    // Next button
    next_btn = lv_btn_create(control_bar);
    lv_obj_set_size(next_btn, 100, 40);
    lv_obj_align(next_btn, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_style(next_btn, &style_btn, 0);
    lv_obj_add_style(next_btn, &style_btn_pressed, LV_STATE_PRESSED);
    lv_obj_add_event_cb(
      next_btn, [](lv_event_t *e) {
        if (image_count > 0 && !image_loading) {
          current_image = (current_image + 1) % image_count;
          display_image(current_image);
        }
      },
      LV_EVENT_CLICKED, NULL);

    lv_obj_t *next_label = lv_label_create(next_btn);
    lv_label_set_text(next_label, "Next");
    lv_obj_center(next_label);

    // Refresh button
    lv_obj_t *refresh_btn = lv_btn_create(control_bar);
    lv_obj_set_size(refresh_btn, 120, 40);
    lv_obj_align(refresh_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(refresh_btn, lv_color_make(0, 180, 0), 0);
    lv_obj_set_style_bg_color(refresh_btn, lv_color_make(0, 140, 0), LV_STATE_PRESSED);
    lv_obj_add_event_cb(
      refresh_btn, [](lv_event_t *e) {
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
      },
      LV_EVENT_CLICKED, NULL);

    lv_obj_t *refresh_label = lv_label_create(refresh_btn);
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

// 7. WebServer Setup Function - Enhanced for stability and improved upload handling


// WebServer Setup Function with Slideshow Features
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
      html += ".btn-warning {background: #f59e0b;}";
      html += ".upload-form {padding: 20px 0;}";
      html += "#status {padding: 10px; margin-top: 10px; border-radius: 4px;}";
      html += ".success {background: rgba(22, 163, 74, 0.2); color: #16a34a;}";
      html += ".error {background: rgba(220, 38, 38, 0.2); color: #dc2626;}";
      html += ".info {background: rgba(59, 130, 246, 0.2); color: #3b82f6;}";
      html += ".warning {background: rgba(245, 158, 11, 0.2); color: #f59e0b;}";
      html += "#progress-container {width: 100%; background-color: #1e293b; height: 20px; border-radius: 10px; margin: 10px 0; display: none;}";
      html += "#progress-bar {width: 0%; background-color: #16a34a; height: 100%; border-radius: 10px; transition: width 0.3s;}";
      html += ".file-input-wrapper {position: relative; overflow: hidden; display: inline-block;}";
      html += ".file-input-wrapper input[type=file] {font-size: 100px; position: absolute; left: 0; top: 0; opacity: 0; cursor: pointer;}";
      html += ".file-input-wrapper .btn {display: inline-block; cursor: pointer;}";
      html += "input[type=number] {background: #475569; color: white; border: 1px solid #64748b; padding: 5px; border-radius: 4px;}";
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
      html += "<button class='btn' onclick='window.location.href=\"/fix-sd-upload\"'>Upload Troubleshooter</button> ";
      html += "<button class='btn' onclick='window.location.href=\"/system-status\"'>System Monitor</button>";
      html += "</div>";
      
      // Upload card with enhanced progress display and size limit warning
      html += "<div class='card'>";
      html += "<h2>Upload Image</h2>";
      html += "<p><b>Important:</b> Upload only JPEG images smaller than 1MB for best results</p>";
      html += "<form id='upload-form' class='upload-form' method='post' action='/upload' enctype='multipart/form-data'>";
      html += "<div class='file-input-wrapper'>";
      html += "<button class='btn'>Choose File</button>";
      html += "<input type='file' name='image' accept='.jpg,.jpeg' id='file-input'>";
      html += "</div>";
      html += "<div id='file-info' style='margin: 10px 0;'><span id='file-selected'>No file selected</span></div>";
      html += "<input type='submit' id='upload-button' class='btn btn-success' value='Upload'>";
      html += "</form>";
      html += "<div id='status' class='info' style='display:none;'></div>";
      html += "<div id='progress-container'><div id='progress-bar'></div></div>";
      html += "</div>";
      
      // Image navigation card
      html += "<div class='card'>";
      html += "<h2>Image Control</h2>";
      html += "<button class='btn' onclick='prevImage()'>Previous Image</button> ";
      html += "<button class='btn' onclick='nextImage()'>Next Image</button> ";
      html += "<span style='margin-left: 10px;'>Current image: <span id='current-image'>" + String(current_image + 1) + "</span>/" + String(image_count) + "</span>";
      html += "</div>";
      
      // Slideshow control card
      html += "<div class='card'>";
      html += "<h2>Slideshow</h2>";
      html += "<button class='btn btn-success' onclick='startSlideshow()'>Start Slideshow</button> ";
      html += "<button class='btn btn-danger' onclick='stopSlideshow()'>Stop Slideshow</button><br><br>";
      html += "<label>Interval (seconds): </label>";
      html += "<input type='number' id='slideshow-interval' value='5' min='1' max='60' style='width: 60px; margin-right: 10px;'>";
      html += "<button class='btn' onclick='setInterval()'>Set Interval</button>";
      html += "<div id='slideshow-status' style='margin-top: 10px; font-style: italic;'></div>";
      html += "</div>";
      
      // Actions card
      html += "<div class='card'>";
      html += "<h2>Actions</h2>";
      html += "<button class='btn btn-success' onclick='refreshSD()'>Refresh SD Card</button> ";
      html += "<button class='btn btn-danger' onclick='restartSystem()'>Restart System</button>";
      html += "</div>";
      
      // Embed JavaScript for better UI interaction
      html += "<script>";
      // File selection handler with size validation
      html += "document.getElementById('file-input').addEventListener('change', function() {";
      html += "  const fileInfo = document.getElementById('file-info');";
      html += "  const fileSelected = document.getElementById('file-selected');";
      html += "  const uploadButton = document.getElementById('upload-button');";
      html += "  ";
      html += "  if (this.files.length) {";
      html += "    const file = this.files[0];";
      html += "    fileSelected.textContent = file.name + ' (' + formatBytes(file.size) + ')';";
      html += "    ";
      html += "    // Check if file is too large (over 1MB)";
      html += "    if (file.size > 1024 * 1024) {";
      html += "      fileInfo.innerHTML = fileSelected.outerHTML + '<div style=\"color:#dc2626;margin-top:5px;\">Warning: File is large and may fail. Try a smaller image.</div>';";
      html += "    } else if (file.size > 512 * 1024) {";
      html += "      fileInfo.innerHTML = fileSelected.outerHTML + '<div style=\"color:#f59e0b;margin-top:5px;\">Note: File is medium sized. Upload should work but may be slow.</div>';";
      html += "    } else {";
      html += "      fileInfo.innerHTML = fileSelected.outerHTML + '<div style=\"color:#16a34a;margin-top:5px;\">File size is good for reliable upload.</div>';";
      html += "    }";
      html += "  } else {";
      html += "    fileSelected.textContent = 'No file selected';";
      html += "    fileInfo.innerHTML = fileSelected.outerHTML;";
      html += "  }";
      html += "});";

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
      
      // Image navigation functions
      html += "function prevImage() {";
      html += "  fetch('/image?action=prev')";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      // Update current image display";
      html += "      const match = data.match(/Displaying image (\\d+) of (\\d+)/);";
      html += "      if (match && match.length >= 3) {";
      html += "        document.getElementById('current-image').textContent = match[1];";
      html += "      }";
      html += "    });";
      html += "}";
      
      html += "function nextImage() {";
      html += "  fetch('/image?action=next')";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      // Update current image display";
      html += "      const match = data.match(/Displaying image (\\d+) of (\\d+)/);";
      html += "      if (match && match.length >= 3) {";
      html += "        document.getElementById('current-image').textContent = match[1];";
      html += "      }";
      html += "    });";
      html += "}";
      
      // Slideshow control functions
      html += "function updateSlideshowStatus() {";
      html += "  fetch('/slideshow/status')";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      document.getElementById('slideshow-status').textContent = data;";
      html += "    });";
      html += "}";

      html += "function startSlideshow() {";
      html += "  const interval = document.getElementById('slideshow-interval').value * 1000;";
      html += "  fetch('/slideshow/start?interval=' + interval)";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      updateSlideshowStatus();";
      html += "    });";
      html += "}";

      html += "function stopSlideshow() {";
      html += "  fetch('/slideshow/stop')";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      updateSlideshowStatus();";
      html += "    });";
      html += "}";

      html += "function setInterval() {";
      html += "  const interval = document.getElementById('slideshow-interval').value * 1000;";
      html += "  fetch('/slideshow/set-interval?ms=' + interval)";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      updateSlideshowStatus();";
      html += "    });";
      html += "}";
      
      // Updated Progress Bar Code
      html += "document.addEventListener('DOMContentLoaded', function() {";
      html += "  const form = document.getElementById('upload-form');";
      html += "  const status = document.getElementById('status');";
      html += "  const progressBar = document.getElementById('progress-bar');";
      html += "  const progressContainer = document.getElementById('progress-container');";
      html += "  const uploadButton = document.getElementById('upload-button');";
      html += "  const fileInput = document.getElementById('file-input');";
      html += "  updateSlideshowStatus();";
      html += "  // Set interval to check status periodically";
      html += "  setInterval(updateSlideshowStatus, 5000);";

      html += "  if (form) {";
      html += "    form.addEventListener('submit', function(e) {";
      html += "      e.preventDefault();";
      html += "      ";
      html += "      if (!fileInput || !fileInput.files.length) {";
      html += "        alert('Please select a file to upload');";
      html += "        return;";
      html += "      }";
      html += "      ";
      html += "      const file = fileInput.files[0];";
      html += "      if (file.size > 2 * 1024 * 1024) {";
      html += "        if (!confirm('File is very large (' + formatBytes(file.size) + '). Upload may fail. Continue anyway?')) {";
      html += "          return;";
      html += "        }";
      html += "      }";
      html += "      ";
      html += "      // Show progress UI";
      html += "      status.textContent = 'Preparing upload...';";
      html += "      status.style.display = 'block';";
      html += "      status.className = 'info';";
      html += "      progressContainer.style.display = 'block';";
      html += "      progressBar.style.width = '0%';";
      html += "      uploadButton.disabled = true;";
      html += "      fileInput.disabled = true;";
      html += "      ";
      html += "      // Create FormData and append file";
      html += "      const formData = new FormData();";
      html += "      formData.append('image', file);";
      html += "      ";
      html += "      // Create and configure XHR";
      html += "      const xhr = new XMLHttpRequest();";
      html += "      ";
      html += "      // Set up upload progress handler";
      html += "      xhr.upload.addEventListener('progress', function(e) {";
      html += "        if (e.lengthComputable) {";
      html += "          const percent = Math.round((e.loaded / e.total) * 100);";
      html += "          progressBar.style.width = percent + '%';";
      html += "          status.textContent = `Uploading: ${percent}% (${formatBytes(e.loaded)} of ${formatBytes(e.total)})`;";
      html += "          console.log(`Progress: ${percent}%`);"; // Add console logging for debug
      html += "        }";
      html += "      });";
      html += "      ";
      html += "      // Set up load handler";
      html += "      xhr.addEventListener('load', function() {";
      html += "        console.log('Upload complete, status:', xhr.status);"; // Debug
      html += "        if (xhr.status === 200) {";
      html += "          progressBar.style.width = '100%';";
      html += "          status.textContent = 'Upload complete! Processing...';";
      html += "          status.className = 'success';";
      html += "          ";
      html += "          // Add processing animation";
      html += "          let dots = '';";
      html += "          const processingInterval = setInterval(function() {";
      html += "            dots = dots.length >= 3 ? '' : dots + '.';";
      html += "            status.textContent = 'Upload complete! Processing' + dots;";
      html += "          }, 500);";
      html += "          ";
      html += "          // Refresh page after delay";
      html += "          setTimeout(function() {";
      html += "            clearInterval(processingInterval);";
      html += "            window.location.reload();";
      html += "          }, 3000);";
      html += "        } else {";
      html += "          status.textContent = 'Upload failed with status: ' + xhr.status;";
      html += "          status.className = 'error';";
      html += "          uploadButton.disabled = false;";
      html += "          fileInput.disabled = false;";
      html += "        }";
      html += "      });";
      html += "      ";
      html += "      // Set up error handler";
      html += "      xhr.addEventListener('error', function(e) {";
      html += "        console.error('Upload error:', e);"; // Debug
      html += "        status.textContent = 'Upload failed due to network error';";
      html += "        status.className = 'error';";
      html += "        uploadButton.disabled = false;";
      html += "        fileInput.disabled = false;";
      html += "      });";
      html += "      ";
      html += "      // Set up abort handler";
      html += "      xhr.addEventListener('abort', function() {";
      html += "        status.textContent = 'Upload aborted';";
      html += "        status.className = 'error';";
      html += "        uploadButton.disabled = false;";
      html += "        fileInput.disabled = false;";
      html += "      });";
      html += "      ";
      html += "      // Open connection and send form data";
      html += "      xhr.open('POST', '/upload');";
      html += "      xhr.send(formData);";
      html += "    });";
      html += "  }";
      html += "});";

      html += "function formatBytes(bytes) {";
      html += "  if (bytes === 0) return '0 Bytes';";
      html += "  const k = 1024;";
      html += "  const sizes = ['Bytes', 'KB', 'MB', 'GB'];";
      html += "  const i = Math.floor(Math.log(bytes) / Math.log(k));";
      html += "  return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];";
      html += "}";
      html += "</script>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
    });

    // Simple upload handler
    server.on("/upload", HTTP_POST, []() {
      server.send(200, "text/plain", "File upload complete");
    }, handle_upload);
    
    // Enhanced image route with next/prev actions
    server.on("/image", HTTP_GET, []() {
      if (server.hasArg("index")) {
        int index = server.arg("index").toInt();
        if (index >= 0 && index < image_count) {
          display_image(index);
          server.send(200, "text/plain", "Displaying image " + String(index + 1) + " of " + String(image_count));
        } else {
          server.send(400, "text/plain", "Invalid image index");
        }
      } else if (server.hasArg("action")) {
        String action = server.arg("action");
        
        if (action == "next") {
          int next_image = (current_image + 1) % image_count;
          display_image(next_image);
          server.send(200, "text/plain", "Displaying image " + String(next_image + 1) + " of " + String(image_count));
        } else if (action == "prev") {
          int prev_image = (current_image - 1 + image_count) % image_count;
          display_image(prev_image);
          server.send(200, "text/plain", "Displaying image " + String(prev_image + 1) + " of " + String(image_count));
        } else {
          server.send(400, "text/plain", "Invalid action parameter");
        }
      } else {
        server.send(400, "text/plain", "Missing index or action parameter");
      }
    });
    
    // Slideshow control routes
    server.on("/slideshow/start", HTTP_GET, []() {
      uint32_t interval = 5000; // Default 5 seconds
      
      // Check if interval parameter is provided
      if (server.hasArg("interval")) {
        uint32_t value = server.arg("interval").toInt();
        interval = (value < 1000) ? 1000 : value;
      }
      
      startSlideshow(interval);
      server.send(200, "text/plain", "Slideshow started with " + String(interval) + "ms interval");
    });

    server.on("/slideshow/stop", HTTP_GET, []() {
      stopSlideshow();
      server.send(200, "text/plain", "Slideshow stopped");
    });

    server.on("/slideshow/toggle", HTTP_GET, []() {
      uint32_t interval = 5000; // Default 5 seconds
      
      // Check if interval parameter is provided
      if (server.hasArg("interval")) {
        uint32_t value = server.arg("interval").toInt();
        interval = (value < 1000) ? 1000 : value;
      }
      
      toggleSlideshow(interval);
      server.send(200, "text/plain", slideshow_active ? 
                  "Slideshow started with " + String(slideshow_interval) + "ms interval" : 
                  "Slideshow stopped");
    });

    server.on("/slideshow/status", HTTP_GET, []() {
      String status = "Slideshow is ";
      status += slideshow_active ? "active" : "inactive";
      status += " (Interval: " + String(slideshow_interval / 1000) + " seconds)";
      server.send(200, "text/plain", status);
    });

    server.on("/slideshow/set-interval", HTTP_GET, []() {
      if (server.hasArg("ms")) {
        uint32_t value = server.arg("ms").toInt();
        slideshow_interval = (value < 1000) ? 1000 : value;
        server.send(200, "text/plain", "Slideshow interval set to " + String(slideshow_interval) + "ms");
      } else {
        server.send(400, "text/plain", "Missing 'ms' parameter");
      }
    });
    
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
      html += "<tr><td>Version</td><td>1.0.7</td></tr>";
      html += "<tr><td>Build Date</td><td>2025-06-15 04:13:12 UTC</td></tr>";
      html += "<tr><td>User</td><td>Chamil1983</td></tr>";
      html += "<tr><td>Uptime</td><td>" + String(millis() / 1000) + " seconds</td></tr>";
      html += "</table>";
      html += "</div>";
      
      // Slideshow status
      html += "<div class='card'>";
      html += "<h2>Slideshow Status</h2>";
      html += "<table>";
      html += "<tr><th>Property</th><th>Value</th></tr>";
      html += "<tr><td>Status</td><td>" + String(slideshow_active ? "Active" : "Inactive") + "</td></tr>";
      html += "<tr><td>Current Interval</td><td>" + String(slideshow_interval / 1000) + " seconds</td></tr>";
      html += "<tr><td>Current Image</td><td>" + String(current_image + 1) + " of " + String(image_count) + "</td></tr>";
      html += "</table>";
      html += "<button class='btn' onclick=\"window.location.href='/slideshow/" + String(slideshow_active ? "stop" : "start") + "'\">"; 
      html += slideshow_active ? "Stop Slideshow" : "Start Slideshow";
      html += "</button>";
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
    
    // SD Upload helper page
    server.on("/fix-sd-upload", HTTP_GET, []() {
      String html = "<html><head><title>SD Card Upload Fix</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<style>";
      html += "body {font-family: Arial, sans-serif; background: #1e293b; color: #f8fafc; padding: 20px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += ".card {background: #334155; padding: 15px; border-radius: 8px; margin-bottom: 20px;}";
      html += ".btn {background: #3b82f6; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer; margin: 5px;}";
      html += ".btn-success {background: #16a34a;}";
      html += ".status {padding: 10px; margin: 10px 0; border-radius: 4px;}";
      html += ".success {background: rgba(22, 163, 74, 0.2); color: #16a34a;}";
      html += ".error {background: rgba(220, 38, 38, 0.2); color: #dc2626;}";
      html += "</style></head><body>";
      
      html += "<h1>SD Card Upload Troubleshooter</h1>";
      
      html += "<div class='card'>";
      html += "<h2>Current SD Card Status</h2>";
      html += "<p>SD Card: " + String(sd_ready ? "Connected" : "Disconnected") + "</p>";
      html += "<p>Image Count: " + String(image_count) + "</p>";
      html += "</div>";
      
      html += "<div class='card'>";
      html += "<h2>SD Card Recovery</h2>";
      html += "<p>If you're having trouble uploading images, try these recovery options:</p>";
      html += "<button class='btn btn-success' onclick='recoverSD()'>Recover SD Card (Gentle)</button> ";
      html += "<button class='btn' onclick='resetSD()'>Reset SD Card (Full)</button>";
      html += "<p id='status' class='status' style='display:none;'></p>";
      html += "</div>";
      
      html += "<div class='card'>";
      html += "<h2>Upload Recommendations</h2>";
      html += "<ul>";
      html += "<li>Use smaller images (less than 1MB) for more reliable uploads</li>";
      html += "<li>Wait 10-15 seconds between uploads</li>";
      html += "<li>If upload fails, try the recovery options above</li>";
      html += "<li>Make sure the SD card is properly seated</li>";
      html += "</ul>";
      html += "<a href='/' class='btn'>Return to Gallery</a>";
      html += "</div>";
      
      html += "<script>";
      html += "function recoverSD() {";
      html += "  document.getElementById('status').textContent = 'Recovering SD card...';";
      html += "  document.getElementById('status').className = 'status';";
      html += "  document.getElementById('status').style.display = 'block';";
      html += "  fetch('/recover-sd-upload', { method: 'POST' })";
      html += "    .then(response => response.text())";
      html += "    .then(data => {";
      html += "      document.getElementById('status').textContent = data;";
      html += "      document.getElementById('status').className = data.includes('failed') ? 'status error' : 'status success';";
      html += "    });";
      html += "}";
      html += "function resetSD() {";
      html += "  if(confirm('This will reset the SD card interface completely. Continue?')) {";
      html += "    document.getElementById('status').textContent = 'Resetting SD card...';";
      html += "    document.getElementById('status').className = 'status';";
      html += "    document.getElementById('status').style.display = 'block';";
      html += "    fetch('/reset-sd-upload', { method: 'POST' })";
      html += "      .then(response => response.text())";
      html += "      .then(data => {";
      html += "        document.getElementById('status').textContent = data;";
      html += "        document.getElementById('status').className = data.includes('failed') ? 'status error' : 'status success';";
      html += "      });";
      html += "  }";
      html += "}";
      html += "</script>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
    });

    // Recovery endpoint for uploads
    server.on("/recover-sd-upload", HTTP_POST, []() {
      if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        bool success = recover_sd_card_for_upload();
        xSemaphoreGive(sd_mutex);
        
        if (success) {
          scan_images();
          server.send(200, "text/plain", "SD card recovered successfully. Found " + 
                      String(image_count) + " images. Upload should work now.");
        } else {
          server.send(200, "text/plain", "SD card recovery failed. Try the full reset option.");
        }
      } else {
        server.send(200, "text/plain", "Could not get exclusive access to SD card. Try again.");
      }
    });

    // Full reset endpoint for uploads
    server.on("/reset-sd-upload", HTTP_POST, []() {
      if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        // Full reset process
        SD_MMC.end();
        delay(2000);
        
        // Reset pins
        pinMode(SDMMC_CLK_PIN, OUTPUT);
        pinMode(SDMMC_CMD_PIN, OUTPUT);
        pinMode(SDMMC_D0_PIN, OUTPUT);
        pinMode(SDMMC_D1_PIN, OUTPUT);
        pinMode(SDMMC_D2_PIN, OUTPUT);
        pinMode(SDMMC_D3_PIN, OUTPUT);
        
        digitalWrite(SDMMC_CLK_PIN, LOW);
        digitalWrite(SDMMC_CMD_PIN, LOW);
        digitalWrite(SDMMC_D0_PIN, LOW);
        digitalWrite(SDMMC_D1_PIN, LOW);
        digitalWrite(SDMMC_D2_PIN, LOW);
        digitalWrite(SDMMC_D3_PIN, LOW);
        delay(1000);
        
        // Reset to initial state
        pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
        pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
        pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
        delay(1000);
        
        // Reconfigure pins
        SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN, 
                      SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);
        
        // Super safe initialization at lowest speed
        bool success = SD_MMC.begin("/sdcard", false, false, 125000);
        xSemaphoreGive(sd_mutex);
        
        if (success) {
          sd_ready = true;
          scan_images();
          server.send(200, "text/plain", "SD card reset successful. Found " + 
                      String(image_count) + " images. Upload should work now.");
        } else {
          sd_ready = false;
          server.send(200, "text/plain", "SD card reset failed. Please check physical connections or try restarting the device.");
        }
      } else {
        server.send(200, "text/plain", "Could not get exclusive access to SD card. Try again or restart the device.");
      }
    });
    
    // System status endpoint to monitor resources
    server.on("/system-status", HTTP_GET, []() {
      String html = "<html><head><title>ESP32P4 System Status</title>";
      html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
      html += "<meta http-equiv=\"refresh\" content=\"5\">";
      html += "<style>";
      html += "body {font-family: monospace; background: #1e293b; color: #f8fafc; padding: 10px;}";
      html += "h1, h2 {color: #3b82f6;}";
      html += "pre {background: #334155; padding: 10px; border-radius: 5px; white-space: pre-wrap;}";
      html += ".good {color: #16a34a;}";
      html += ".warn {color: #f59e0b;}";
      html += ".error {color: #dc2626;}";
      html += "table {width: 100%; border-collapse: collapse;}";
      html += "th, td {text-align: left; padding: 8px; border-bottom: 1px solid #475569;}";
      html += "th {background-color: #334155;}";
      html += ".meter {height: 20px; background: #1e293b; border-radius: 5px; position: relative; margin: 5px 0;}";
      html += ".meter-fill {height: 100%; border-radius: 5px; transition: width 0.3s;}";
      html += ".meter-text {position: absolute; top: 0; left: 0; width: 100%; text-align: center; line-height: 20px;}";
      html += ".good-bg {background-color: #16a34a;}";
      html += ".warn-bg {background-color: #f59e0b;}";
      html += ".error-bg {background-color: #dc2626;}";
      html += "</style></head><body>";
      
      html += "<h1>ESP32P4 System Status Monitor</h1>";
      html += "<p>Auto refresh every 5 seconds</p>";
      
      // Memory section
      html += "<h2>Memory</h2>";
      
      // Heap memory
      uint32_t heapSize = ESP.getHeapSize();
      uint32_t freeHeap = ESP.getFreeHeap();
      uint32_t usedHeap = heapSize - freeHeap;
      uint32_t minFreeHeap = ESP.getMinFreeHeap();
      uint32_t maxHeapBlock = ESP.getMaxAllocHeap();
      
      int heapPercent = (int)(usedHeap * 100.0f / heapSize);
      String heapClass = heapPercent > 80 ? "error-bg" : (heapPercent > 60 ? "warn-bg" : "good-bg");
      
      html += "<h3>Heap</h3>";
      html += "<table><tr><th>Metric</th><th>Value</th></tr>";
      html += "<tr><td>Total Size</td><td>" + String(heapSize / 1024) + " KB</td></tr>";
      html += "<tr><td>Free</td><td>" + String(freeHeap / 1024) + " KB</td></tr>";
      html += "<tr><td>Used</td><td>" + String(usedHeap / 1024) + " KB</td></tr>";
      html += "<tr><td>Min Free Ever</td><td>" + String(minFreeHeap / 1024) + " KB</td></tr>";
      html += "<tr><td>Largest Free Block</td><td>" + String(maxHeapBlock / 1024) + " KB</td></tr>";
      html += "</table>";
      
      html += "<div class='meter'>";
      html += "<div class='meter-fill " + heapClass + "' style='width: " + String(heapPercent) + "%;'></div>";
      html += "<div class='meter-text'>" + String(heapPercent) + "% used</div>";
      html += "</div>";
      
      // PSRAM memory
      uint32_t psramSize = ESP.getPsramSize();
      uint32_t freePsram = ESP.getFreePsram();
      uint32_t usedPsram = psramSize - freePsram;
      uint32_t minFreePsram = ESP.getMinFreePsram();
      uint32_t maxPsramBlock = ESP.getMaxAllocPsram();
      
      int psramPercent = psramSize > 0 ? (int)(usedPsram * 100.0f / psramSize) : 0;
      String psramClass = psramPercent > 80 ? "error-bg" : (psramPercent > 60 ? "warn-bg" : "good-bg");
      
      html += "<h3>PSRAM</h3>";
      html += "<table><tr><th>Metric</th><th>Value</th></tr>";
      html += "<tr><td>Total Size</td><td>" + String(psramSize / 1024) + " KB</td></tr>";
      html += "<tr><td>Free</td><td>" + String(freePsram / 1024) + " KB</td></tr>";
      html += "<tr><td>Used</td><td>" + String(usedPsram / 1024) + " KB</td></tr>";
      html += "<tr><td>Min Free Ever</td><td>" + String(minFreePsram / 1024) + " KB</td></tr>";
      html += "<tr><td>Largest Free Block</td><td>" + String(maxPsramBlock / 1024) + " KB</td></tr>";
      html += "</table>";
      
      html += "<div class='meter'>";
      html += "<div class='meter-fill " + psramClass + "' style='width: " + String(psramPercent) + "%;'></div>";
      html += "<div class='meter-text'>" + String(psramPercent) + "% used</div>";
      html += "</div>";
      
      // SD Card status
      html += "<h2>SD Card</h2>";
      String sdStatus = sd_ready ? "<span class='good'>Connected</span>" : "<span class='error'>Disconnected</span>";
      html += "<p>Status: " + sdStatus + "</p>";
      
      if (sd_ready && xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        uint8_t cardType = SD_MMC.cardType();
        String cardTypeStr = "Unknown";
        if (cardType == CARD_MMC) cardTypeStr = "MMC";
        else if (cardType == CARD_SD) cardTypeStr = "SDSC";
        else if (cardType == CARD_SDHC) cardTypeStr = "SDHC";
        else if (cardType == CARD_NONE) cardTypeStr = "None";
        
        uint64_t totalBytes = SD_MMC.totalBytes();
        uint64_t usedBytes = SD_MMC.usedBytes();
        uint64_t freeBytes = totalBytes - usedBytes;
        
        int sdUsedPercent = totalBytes > 0 ? (int)(usedBytes * 100.0f / totalBytes) : 0;
        String sdClass = sdUsedPercent > 90 ? "error-bg" : (sdUsedPercent > 70 ? "warn-bg" : "good-bg");
        
        html += "<table><tr><th>Metric</th><th>Value</th></tr>";
        html += "<tr><td>Card Type</td><td>" + cardTypeStr + "</td></tr>";
        html += "<tr><td>Total Size</td><td>" + String(totalBytes / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Free</td><td>" + String(freeBytes / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Used</td><td>" + String(usedBytes / (1024 * 1024)) + " MB</td></tr>";
        html += "<tr><td>Image Count</td><td>" + String(image_count) + "</td></tr>";
        html += "</table>";
        
        html += "<div class='meter'>";
        html += "<div class='meter-fill " + sdClass + "' style='width: " + String(sdUsedPercent) + "%;'></div>";
        html += "<div class='meter-text'>" + String(sdUsedPercent) + "% used</div>";
        html += "</div>";
        
        xSemaphoreGive(sd_mutex);
      }
      
      // WiFi Status
      html += "<h2>WiFi Status</h2>";
      html += "<table><tr><th>Metric</th><th>Value</th></tr>";
      html += "<tr><td>Mode</td><td>Access Point</td></tr>";
      html += "<tr><td>SSID</td><td>" + String(WIFI_AP_SSID) + "</td></tr>";
      html += "<tr><td>IP Address</td><td>" + WiFi.softAPIP().toString() + "</td></tr>";
      html += "<tr><td>Connected Clients</td><td>" + String(WiFi.softAPgetStationNum()) + "</td></tr>";
      html += "</table>";
      
      // System Info
      html += "<h2>System</h2>";
      html += "<table><tr><th>Metric</th><th>Value</th></tr>";
      html += "<tr><td>Uptime</td><td>" + String(millis() / 1000) + " seconds</td></tr>";
      html += "<tr><td>Version</td><td>1.0.7</td></tr>";
      html += "<tr><td>Build Date</td><td>2025-06-15 04:13:12 UTC</td></tr>";
      html += "<tr><td>CPU Frequency</td><td>" + String(ESP.getCpuFreqMHz()) + " MHz</td></tr>";
      html += "</table>";
      
      // Slideshow Info
      html += "<h2>Slideshow</h2>";
      html += "<table><tr><th>Metric</th><th>Value</th></tr>";
      html += "<tr><td>Status</td><td>" + String(slideshow_active ? "<span class='good'>Active</span>" : "Inactive") + "</td></tr>";
      html += "<tr><td>Interval</td><td>" + String(slideshow_interval / 1000.0, 1) + " seconds</td></tr>";
      html += "<tr><td>Current Image</td><td>" + String(current_image + 1) + " of " + String(image_count) + "</td></tr>";
      html += "<tr><td>Controls</td><td><a href='/slideshow/" + String(slideshow_active ? "stop" : "start") + "' style='color:#3b82f6;'>" + 
              (slideshow_active ? "Stop Slideshow" : "Start Slideshow") + "</a></td></tr>";
      html += "</table>";
      
      // Recent Logs
      html += "<h2>Recent Logs</h2>";
      html += "<pre>";
      std::vector<String> logs = Logger.getRecentLogs();
      int logCount = min(10, (int)logs.size());
      for (int i = logs.size() - logCount; i < logs.size(); i++) {
        String logLine = logs[i];
        
        // Color-code log lines
        if (logLine.indexOf("[ERROR]") >= 0 || logLine.indexOf("[CRITICAL]") >= 0) {
          html += "<span class='error'>" + logLine + "</span>\n";
        } else if (logLine.indexOf("[WARN]") >= 0) {
          html += "<span class='warn'>" + logLine + "</span>\n";
        } else {
          html += logLine + "\n";
        }
      }
      html += "</pre>";
      
      html += "<p><a href='/'>Back to Gallery</a></p>";
      
      html += "</body></html>";
      
      server.send(200, "text/html", html);
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
        image_list[localImageCount] = (char *)malloc(fullPath.length() + 1);
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

// Helper function to clear the LCD screen
void clearLCDScreen() {
  // Create a black buffer of reasonable size to avoid memory issues
  const int bufferWidth = 400;  // Half of LCD_H_RES
  const int bufferHeight = 16;  // Small height to save memory

  // Calculate buffer size
  int bufferSize = bufferWidth * bufferHeight;

  // Allocate from PSRAM if available (much larger and safer)
  uint16_t *blackBuffer = (uint16_t *)ps_malloc(bufferSize * sizeof(uint16_t));

  if (blackBuffer == nullptr) {
    // Fallback to smaller buffer if PSRAM allocation fails
    bufferSize = 100;
    blackBuffer = (uint16_t *)malloc(bufferSize * sizeof(uint16_t));
  }

  if (blackBuffer) {
    // Fill buffer with black color (0x0000)
    memset(blackBuffer, 0, bufferSize * sizeof(uint16_t));

    // Fill screen in horizontal strips
    for (int y = 0; y < LCD_V_RES; y += bufferHeight) {
      int height = min(bufferHeight, LCD_V_RES - y);

      // For each strip, fill horizontally if needed
      for (int x = 0; x < LCD_H_RES; x += bufferWidth) {
        int width = min(bufferWidth, LCD_H_RES - x);
        lcd.lcd_draw_bitmap(x, y, x + width, y + height, blackBuffer);
      }

      // Feed watchdog periodically
      if (y % 128 == 0) {
        feed_watchdog();
      }
    }

    // Free buffer
    free(blackBuffer);
  } else {
    Logger.error("Failed to allocate memory for screen clearing");
  }
}

// Display image with orientation support and screen clearing
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

    // Clear screen with black color
    clearLCDScreen();

    bool result = render_jpeg_file(image_list[index]);

    // Release JPEG mutex
    xSemaphoreGive(jpeg_mutex);

    return result;
  },
                                            "JPEG Rendering");

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
          const char *filename = strrchr(image_list[index], '/');
          if (filename) {
            filename++;  // Skip slash
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

// More robust emergency SD recovery function
bool emergency_sd_recovery() {
  Logger.info("Performing comprehensive emergency SD card recovery");

  // End any existing SD session
  SD_MMC.end();

  // Wait for full discharge and card reset
  delay(2000);

  // Reset pins individually with careful timing
  const uint8_t sdPins[] = {
    SDMMC_CLK_PIN, SDMMC_CMD_PIN,
    SDMMC_D0_PIN, SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN
  };

  // First set all pins to input mode
  for (int i = 0; i < 6; i++) {
    pinMode(sdPins[i], INPUT);
  }
  delay(300);

  // Now drive them low to discharge
  for (int i = 0; i < 6; i++) {
    pinMode(sdPins[i], OUTPUT);
    digitalWrite(sdPins[i], LOW);
    delay(50);  // Short delay between pins
  }
  delay(500);

  // Release pins in reverse order with pull-ups
  for (int i = 5; i >= 0; i--) {
    pinMode(sdPins[i], INPUT_PULLUP);
    delay(50);
  }
  delay(500);

  // Reset SD host controller
  Logger.info("Reinitializing SD host controller");

  // Configure with explicit pins
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN,
                 SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);

  // Try ultra-conservative initialization
  bool success = SD_MMC.begin("/sdcard", false, false, 80000);  // Even lower speed (80KHz)

  if (!success) {
    Logger.warn("First SD recovery attempt failed, trying with different settings");
    delay(1000);

    // Try 1-bit mode with slightly higher speed
    success = SD_MMC.begin("/sdcard", true, false, 125000);
  }

  if (success) {
    Logger.info("SD card recovery successful");
    sd_ready = true;

    // Test a simple write operation
    File testFile = SD_MMC.open("/recovery_test.txt", FILE_WRITE);
    if (testFile) {
      testFile.print("RECOVERY");
      testFile.close();

      // Very small delay to ensure SD card operation completes
      delay(100);

      // Read back to verify
      testFile = SD_MMC.open("/recovery_test.txt", FILE_READ);
      if (testFile) {
        char buf[10] = { 0 };
        int bytesRead = testFile.read((uint8_t *)buf, 8);
        testFile.close();

        if (bytesRead == 8 && strcmp(buf, "RECOVERY") == 0) {
          Logger.info("SD card test write successful");
        } else {
          Logger.warn("SD card write test failed, but continuing");
        }
      }

      // Clean up test file
      SD_MMC.remove("/recovery_test.txt");
    }

    // Ensure images directory exists
    if (!SD_MMC.exists("/images")) {
      if (SD_MMC.mkdir("/images")) {
        Logger.info("Created /images directory");
      }
    }

    return true;
  } else {
    Logger.error("All SD card recovery attempts failed");
    sd_ready = false;
    return false;
  }
}

// Function to recover SD card specifically for uploads
bool recover_sd_card_for_upload() {
  Logger.info("Performing special SD recovery for uploads...");

  // First end any active SD session
  SD_MMC.end();

  // Wait for a complete discharge of the SD card interface
  delay(1000);

  // Reset pins one by one with timing gaps
  pinMode(SDMMC_CLK_PIN, INPUT);
  pinMode(SDMMC_CMD_PIN, INPUT);
  pinMode(SDMMC_D0_PIN, INPUT);
  pinMode(SDMMC_D1_PIN, INPUT);
  pinMode(SDMMC_D2_PIN, INPUT);
  pinMode(SDMMC_D3_PIN, INPUT);
  delay(100);

  // Set to outputs and drive low to discharge
  pinMode(SDMMC_CLK_PIN, OUTPUT);
  digitalWrite(SDMMC_CLK_PIN, LOW);
  pinMode(SDMMC_CMD_PIN, OUTPUT);
  digitalWrite(SDMMC_CMD_PIN, LOW);
  pinMode(SDMMC_D0_PIN, OUTPUT);
  digitalWrite(SDMMC_D0_PIN, LOW);
  pinMode(SDMMC_D1_PIN, OUTPUT);
  digitalWrite(SDMMC_D1_PIN, LOW);
  pinMode(SDMMC_D2_PIN, OUTPUT);
  digitalWrite(SDMMC_D2_PIN, LOW);
  pinMode(SDMMC_D3_PIN, OUTPUT);
  digitalWrite(SDMMC_D3_PIN, LOW);
  delay(200);

  // Reset to inputs with pullups
  pinMode(SDMMC_CLK_PIN, INPUT_PULLUP);
  pinMode(SDMMC_CMD_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D0_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D1_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D2_PIN, INPUT_PULLUP);
  pinMode(SDMMC_D3_PIN, INPUT_PULLUP);
  delay(200);

  // Configure SD MMC pins
  SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN,
                 SDMMC_D1_PIN, SDMMC_D2_PIN, SDMMC_D3_PIN);

  // Try to initialize with ultra-conservative parameters
  bool success = SD_MMC.begin("/sdcard", false, false, 100000);  // 1-bit mode, 100KHz speed

  if (success) {
    // Test if SD card is working by writing a small test file
    File testFile = SD_MMC.open("/sd_test.txt", FILE_WRITE);
    if (testFile) {
      testFile.println("SD recovery test");
      testFile.close();
      SD_MMC.remove("/sd_test.txt");

      Logger.info("SD card recovery successful");
      sd_ready = true;
      return true;
    } else {
      Logger.error("SD card recovery - cannot write test file");
      sd_ready = false;
      return false;
    }
  } else {
    Logger.error("SD card recovery failed");
    sd_ready = false;
    return false;
  }
}



// Enhanced JPEG drawing callback with proper centering
int jpeg_draw_callback(JPEGDRAW *pDraw) {
  if (!lcd_ready) return 0;

  // Get image dimensions
  int imgWidth = jpeg.getWidth();
  int imgHeight = jpeg.getHeight();

  // Use the global rotation flag
  bool needsRotation = image_needs_rotation;

  // LCD dimensions
  // LCD_H_RES is typically 800
  // LCD_V_RES is typically 1280

  if (!needsRotation) {
    // No rotation needed - standard centering
    // Calculate position to center the image on screen
    int centerX = (LCD_H_RES - imgWidth) / 2;
    int centerY = (LCD_V_RES - imgHeight) / 2;

    // Ensure we don't go below 0 (important for very large images)
    centerX = max(0, centerX);
    centerY = max(0, centerY);

    // Draw the MCU block on the LCD
    int x = pDraw->x + centerX;
    int y = pDraw->y + centerY;

    // Check boundaries - don't draw outside screen
    if (x >= 0 && y >= 0 && x + pDraw->iWidth <= LCD_H_RES && y + pDraw->iHeight <= LCD_V_RES) {
      lcd.lcd_draw_bitmap(x, y, x + pDraw->iWidth, y + pDraw->iHeight, (uint16_t *)pDraw->pPixels);
    }
  } else {
    // For rotation, we need to handle a special case
    // After rotation, imgWidth becomes imgHeight and imgHeight becomes imgWidth
    int rotatedWidth = imgHeight;
    int rotatedHeight = imgWidth;

    // Calculate position to center the rotated image on the physical screen
    int centerX = (LCD_H_RES - rotatedWidth) / 2;
    int centerY = (LCD_V_RES - rotatedHeight) / 2;

    // Ensure centering values are non-negative
    centerX = max(0, centerX);
    centerY = max(0, centerY);

    // Calculate the position of this block in the rotated image
    int rotX = centerX + pDraw->y;                               // y becomes x after rotation
    int rotY = centerY + (imgWidth - pDraw->x - pDraw->iWidth);  // Adjust Y for 90° rotation

    // Create a buffer for the rotated block
    int rotWidth = pDraw->iHeight;
    int rotHeight = pDraw->iWidth;

    // Allocate buffer for rotated data
    uint16_t *rotatedBlock = (uint16_t *)malloc(rotWidth * rotHeight * sizeof(uint16_t));
    if (!rotatedBlock) {
      Logger.error("Failed to allocate memory for rotation");
      return 0;
    }

    // Rotate the block 90 degrees clockwise
    uint16_t *pixels = (uint16_t *)pDraw->pPixels;
    for (int y = 0; y < pDraw->iHeight; y++) {
      for (int x = 0; x < pDraw->iWidth; x++) {
        // For 90° rotation: new_x = y, new_y = width - 1 - x
        int newX = y;
        int newY = pDraw->iWidth - 1 - x;
        rotatedBlock[newY * rotWidth + newX] = pixels[y * pDraw->iWidth + x];
      }
    }

    // Draw the rotated block if it fits on screen
    if (rotX >= 0 && rotY >= 0 && rotX + rotWidth <= LCD_H_RES && rotY + rotHeight <= LCD_V_RES) {
      lcd.lcd_draw_bitmap(rotX, rotY, rotX + rotWidth, rotY + rotHeight, rotatedBlock);
    } else {
      // Log out-of-bounds issues for debugging
      Logger.warn("Rotated block out of bounds: x=%d, y=%d, w=%d, h=%d",
                  rotX, rotY, rotWidth, rotHeight);
    }

    // Free the buffer
    free(rotatedBlock);
  }

  // Feed watchdog during long operations
  static uint32_t lastFeedTime = 0;
  uint32_t now = millis();
  if (now - lastFeedTime > 5000) {
    feed_watchdog();
    lastFeedTime = now;
  }

  return 1;  // Continue decoding
}

// Completely redesigned file upload handler to avoid crashes
void handle_upload() {
  static File uploadFile;
  static size_t uploadSize = 0;
  static uint32_t lastWdtFeed = 0;
  static bool uploadInProgress = false;
  static const size_t WRITE_BUFFER_SIZE = 512;  // Much smaller buffer for stability
  static uint8_t writeBuffer[WRITE_BUFFER_SIZE];
  static size_t bufferPos = 0;

  HTTPUpload &upload = server.upload();

  // Feed watchdog frequently
  uint32_t now = millis();
  if (now - lastWdtFeed > 1000) {
    feed_watchdog();
    lastWdtFeed = now;
  }

  if (upload.status == UPLOAD_FILE_START) {
    // Reset all variables
    uploadSize = 0;
    bufferPos = 0;
    uploadInProgress = true;

    // Generate safe filename with timestamp
    String safeFilename = upload.filename;
    safeFilename.replace(" ", "_");
    safeFilename.replace("/", "_");

    // Create full path with timestamp
    snprintf(upload_filename, sizeof(upload_filename),
             "/images/img_%u_%s", millis(), safeFilename.c_str());

    Logger.info("Starting new upload: %s", upload.filename.c_str());
    Logger.info("Upload start: %s", upload_filename);
    upload_start_time = millis();

    // Try to recover SD card if not working
    if (!sd_ready) {
      Logger.warn("SD card not ready before upload, attempting recovery");
      emergency_sd_recovery();
    }

    // Take mutex with retry
    int retryCount = 0;
    while (retryCount < 5 && !xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000))) {
      Logger.warn("Failed to get SD mutex for upload, retry %d", retryCount + 1);
      feed_watchdog();
      retryCount++;
    }

    if (retryCount >= 5) {
      Logger.error("Failed to get SD mutex after 5 retries");
      uploadInProgress = false;
      return;
    }

    // Make sure images directory exists
    if (!SD_MMC.exists("/images")) {
      if (!SD_MMC.mkdir("/images")) {
        Logger.warn("Failed to create images directory");
      }
    }

    // Delete any existing file with the same name first
    if (SD_MMC.exists(upload_filename)) {
      SD_MMC.remove(upload_filename);
    }

    // Open file for writing
    uploadFile = SD_MMC.open(upload_filename, FILE_WRITE);

    if (!uploadFile) {
      Logger.error("Failed to open file for writing: %s", upload_filename);
      xSemaphoreGive(sd_mutex);
      uploadInProgress = false;
      return;
    }

    // Make sure file is empty
    uploadFile.seek(0);

    Logger.info("File opened successfully for upload");
  } else if (upload.status == UPLOAD_FILE_WRITE && uploadInProgress) {
    // Process upload data in small chunks
    // We'll accumulate data in the writeBuffer and write when it's full

    size_t remaining = upload.currentSize;
    size_t position = 0;

    while (remaining > 0) {
      // How many bytes can we copy to the buffer
      size_t bytesToCopy = min(remaining, WRITE_BUFFER_SIZE - bufferPos);

      // Copy bytes to write buffer
      memcpy(writeBuffer + bufferPos, upload.buf + position, bytesToCopy);
      bufferPos += bytesToCopy;
      position += bytesToCopy;
      remaining -= bytesToCopy;

      // If buffer is full or we just added the last bytes, write to SD
      if (bufferPos == WRITE_BUFFER_SIZE || remaining == 0) {
        if (uploadFile) {
          size_t bytesWritten = uploadFile.write(writeBuffer, bufferPos);

          if (bytesWritten != bufferPos) {
            Logger.error("SD Write error: %d of %d bytes written", bytesWritten, bufferPos);
          }

          // Update counters
          uploadSize += bytesWritten;

          // Reset buffer
          bufferPos = 0;

          // Periodic flush to ensure data is written
          static uint32_t lastFlush = 0;
          if (millis() - lastFlush > 2000) {
            uploadFile.flush();
            lastFlush = millis();
          }
        } else {
          Logger.error("Upload file not open during write");
          uploadInProgress = false;
          break;
        }
      }

      // Feed watchdog in long loops
      if (position % 16384 == 0) {
        feed_watchdog();
      }
    }
  } else if (upload.status == UPLOAD_FILE_END && uploadInProgress) {
    // Write any remaining data
    if (bufferPos > 0 && uploadFile) {
      uploadFile.write(writeBuffer, bufferPos);
    }

    // Finalize file
    if (uploadFile) {
      // Final flush and close
      uploadFile.flush();
      uploadFile.close();

      // Log upload statistics
      float duration = (millis() - upload_start_time) / 1000.0;
      float speed = uploadSize / (1024.0 * duration);
      Logger.info("Upload complete: %s, %u bytes, %.1f KB/s",
                  upload_filename, uploadSize, speed);

      // Release SD mutex
      xSemaphoreGive(sd_mutex);

      // Wait for SD card operations to complete
      delay(200);

      // After successful upload - refresh image list and display
      if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
        scan_images();

        // Find our newly uploaded image
        int newImageIndex = -1;
        for (int i = 0; i < image_count; i++) {
          if (strcmp(image_list[i], upload_filename) == 0) {
            newImageIndex = i;
            break;
          }
        }

        // Display the new image
        if (newImageIndex >= 0) {
          current_image = newImageIndex;
          display_image(current_image);
        } else {
          Logger.warn("Uploaded image not found in scan");
        }

        xSemaphoreGive(sd_mutex);
      }
    } else {
      // No file, but still need to release mutex if we got this far
      xSemaphoreGive(sd_mutex);
    }

    // Reset upload state
    uploadInProgress = false;
    bufferPos = 0;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Logger.error("Upload aborted");

    // Close file if open
    if (uploadFile) {
      uploadFile.close();

      // Try to delete partial file
      if (SD_MMC.exists(upload_filename)) {
        SD_MMC.remove(upload_filename);
      }
    }

    // Release mutex if upload was in progress
    if (uploadInProgress) {
      xSemaphoreGive(sd_mutex);
    }

    // Reset state
    uploadInProgress = false;
    bufferPos = 0;
  }
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
  bool success = SD_MMC.begin("/sdcard", false, false, 250000);  // 1-bit mode, 250KHz

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
        char buf[5] = { 0 };
        size_t bytesRead = testFile.read((uint8_t *)buf, 4);
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
  Logger.info("Date: 2025-06-14 13:35:06 UTC");
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
  if (SD_MMC.begin("/sdcard", false, false, 250000)) {  // 1-bit mode, 250 KHz speed
    uint8_t cardType = SD_MMC.cardType();
    const char *typeStr = "UNKNOWN";
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
      testFile.print("A");  // Just a single character for simplicity
      testFile.flush();
      testFile.close();
      delay(100);  // Allow filesystem to settle

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
