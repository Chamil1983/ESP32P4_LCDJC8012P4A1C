/**
 * ESP32-S3-Touch-LCD-5_B HD Image Viewer - WiFi First Initialization
 * 
 * - WiFi initialized FIRST to prevent memory allocation issues
 * - Following exact sequence from original TFT_Display_v1.1.ino
 * - Fixed memory allocation order to prevent crashes
 * - Proper SD card initialization sequence
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>
#include <JPEGDEC.h>
#include <vector>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_display_panel.hpp"
#include "lvgl_v8_port.h"
#include "waveshare_sd_card.h"
#include "lvgl.h"

// **CRITICAL: Add missing namespace declarations**
using namespace esp_panel::drivers;
using namespace esp_panel::board;

// Display configuration for ESP32-S3-Touch-LCD-5_B
#define SCREEN_WIDTH 1024
#define SCREEN_HEIGHT 600

// WiFi AP configuration
#define WIFI_AP_SSID "ESP32-ImageViewer"
#define WIFI_AP_PASSWORD "12345678"
#define WIFI_AP_IP IPAddress(192, 168, 4, 1)
#define WIFI_AP_GATEWAY IPAddress(192, 168, 4, 1)
#define WIFI_AP_SUBNET IPAddress(255, 255, 255, 0)
#define WIFI_CHANNEL 6

// Display optimization settings - FIXED for realistic buffer sizes
#define ENABLE_PSRAM_BUFFERS true
#define SHOW_ORIGINAL_RESOLUTION true
#define ENABLE_IMAGE_SCROLLING true
#define AUTO_SLIDESHOW_INTERVAL 8000
#define RANDOM_PREVIEW_MODE false

// Performance optimization settings
#define CPU_FREQ_MHZ 240
#define ENABLE_WIFI_POWER_SAVE false

// FIXED: Realistic image buffer structure for ESP32-S3
typedef struct {
    uint8_t *imageBuffer;
    size_t bufferSize;
    int imageWidth; // Actual image width
    int imageHeight; // Actual image height
    int displayWidth; // Display area width (1024)
    int displayHeight; // Display area height (530)
    int scrollX; // Current scroll X position
    int scrollY; // Current scroll Y position
    volatile bool bufferReady;
    SemaphoreHandle_t bufferMutex;
} RealisticImageBuffer_t;

// Global variables
Board *board = nullptr;
LCD *lcd = nullptr;
Touch *touch = nullptr;
WebServer server(80);
std::vector<String> imageFiles;
int currentImageIndex = 0;
bool isImageLoaded = false;
bool lvglInitialized = false;
bool sdCardInitialized = false;
bool wifiConnected = false;
bool apActive = false;
bool randomPreviewMode = RANDOM_PREVIEW_MODE;

// LVGL objects
lv_obj_t *mainScreen = nullptr;
lv_obj_t *imgContainer = nullptr;
lv_obj_t *imgViewer = nullptr;
lv_obj_t *btnPrev = nullptr;
lv_obj_t *btnNext = nullptr;
lv_obj_t *btnDelete = nullptr;
lv_obj_t *btnRandom = nullptr;
lv_obj_t *statusLabel = nullptr;
lv_obj_t *wifiLabel = nullptr;
lv_style_t styleBtn;
lv_style_t styleBtn_danger;

// FIXED: Realistic image handling
static lv_img_dsc_t imgDsc;
static RealisticImageBuffer_t imageBuffer;
static JPEGDEC jpegDec;

// File upload handling
static File uploadFile;
static String uploadFilename;
static bool uploadInProgress = false;
static size_t uploadTotalSize = 0;
static size_t uploadCurrentSize = 0;

// Performance metrics
static uint32_t lastDrawTime = 0;
static uint32_t totalDrawTime = 0;
static uint32_t frameCount = 0;
static uint32_t lastSlideChangeTime = 0;

// Function declarations
void initRealisticImageBuffer();
void releaseRealisticImageBuffer();
void optimizeSystemPerformance();
void configureWiFiAntiInterference();
bool initWiFi();
bool initSDCard();
void scanSDCardImages();
bool isJpegFile(const String &filename);
void setupWebServer();
void createUI();
void updateStatusLabel();
void showNextImage();
void showPrevImage();
void showImageByIndex(int index);
bool decodeAndDisplayImageRealistic(const String &filename);
int realisticJpegDrawCallback(JPEGDRAW *pDraw);
void WiFiEventCallback(WiFiEvent_t event, WiFiEventInfo_t info);
void showRandomImage();
void deleteCurrentImage();
void updateAutoSlideshow();

// **ENHANCED: WiFi anti-interference configuration**
void configureWiFiAntiInterference() {
    Serial.println("Configuring WiFi anti-interference...");
    
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    if (ENABLE_WIFI_POWER_SAVE) {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    } else {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
    
    esp_wifi_set_max_tx_power(78);
    Serial.println("WiFi anti-interference configuration complete");
}

// **CRITICAL: Initialize WiFi FIRST to prevent memory allocation issues**
bool initWiFi() {
    Serial.println("Initializing WiFi with anti-interference...");
    
    // Set WiFi mode before any other operations
    WiFi.mode(WIFI_MODE_AP);
    delay(100); // Give WiFi time to initialize
    
    // Configure anti-interference settings
    configureWiFiAntiInterference();
    
    // Set up event handler
    WiFi.onEvent(WiFiEventCallback);
    
    // Configure static IP
    if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET)) {
        Serial.println("AP Static IP configuration failed");
        return false;
    }
    
    // Start AP
    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_CHANNEL)) {
        Serial.println("AP setup failed");
        return false;
    }
    
    // Wait for AP to be ready
    delay(500);
    
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.printf("WiFi channel: %d\n", WIFI_CHANNEL);
    Serial.println("WiFi AP initialized with anti-interference");
    
    apActive = true;
    wifiConnected = true;
    return true;
}

// WiFi event handler
void WiFiEventCallback(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case WIFI_EVENT_AP_START:
            Serial.println("AP Started");
            wifiConnected = true;
            break;
        case WIFI_EVENT_AP_STOP:
            Serial.println("AP Stopped");
            wifiConnected = false;
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            Serial.println("Client connected to AP");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            Serial.println("Client disconnected from AP");
            break;
        default:
            break;
    }
}

// Optimize system performance
void optimizeSystemPerformance() {
    Serial.println("Optimizing system performance...");
    setCpuFrequencyMhz(CPU_FREQ_MHZ);
    Serial.printf("CPU frequency set to %d MHz\n", getCpuFrequencyMhz());
    
    esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = CPU_FREQ_MHZ,
        .min_freq_mhz = CPU_FREQ_MHZ,
        .light_sleep_enable = false
    };
    esp_pm_configure(&pm_config);
    
    if (psramFound()) {
        Serial.println("PSRAM detected - optimizing settings");
        Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());
        Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
        
        // Configure PSRAM for optimal LCD performance
        heap_caps_malloc_extmem_enable(1024);
    } else {
        Serial.println("WARNING: PSRAM not found - using internal RAM only");
    }
    
    Serial.println("System performance optimization complete");
}

// FIXED: Initialize realistic image buffer with proper size limits
void initRealisticImageBuffer() {
    Serial.println("Initializing realistic image buffer...");
    
    // Calculate display area (excluding UI)
    imageBuffer.displayWidth = SCREEN_WIDTH;
    imageBuffer.displayHeight = SCREEN_HEIGHT - 70; // Reserve space for UI
    
    // FIXED: Use realistic buffer size - max 2048x2048 for large images
    // This allows for images up to 2048x2048 pixels = 8MB in RGB565
    size_t maxImageSize = 2048 * 2048 * 2; // 8MB maximum
    imageBuffer.bufferSize = maxImageSize;
    imageBuffer.bufferReady = false;
    imageBuffer.scrollX = 0;
    imageBuffer.scrollY = 0;
    
    // Create synchronization primitive
    imageBuffer.bufferMutex = xSemaphoreCreateMutex();
    if (!imageBuffer.bufferMutex) {
        Serial.println("ERROR: Failed to create buffer mutex!");
        return;
    }
    
    // Check available PSRAM
    size_t psramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    Serial.printf("Available PSRAM: %u bytes\n", psramFree);
    
    if (psramFree < imageBuffer.bufferSize) {
        // Reduce buffer size to fit available PSRAM
        imageBuffer.bufferSize = (psramFree * 70) / 100; // Use 70% of available PSRAM
        Serial.printf("Reduced buffer size to fit PSRAM: %u bytes\n", imageBuffer.bufferSize);
    }
    
    // Allocate image buffer with PSRAM preference
    if (ENABLE_PSRAM_BUFFERS && psramFree > imageBuffer.bufferSize) {
        imageBuffer.imageBuffer = (uint8_t *)heap_caps_aligned_alloc(32, imageBuffer.bufferSize,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (imageBuffer.imageBuffer) {
            Serial.printf("Allocated %u bytes in PSRAM\n", imageBuffer.bufferSize);
        }
    }
    
    if (!imageBuffer.imageBuffer) {
        // Fallback to internal memory with smaller buffer
        imageBuffer.bufferSize = SCREEN_WIDTH * SCREEN_HEIGHT * 2; // Just screen size
        imageBuffer.imageBuffer = (uint8_t *)heap_caps_aligned_alloc(32, imageBuffer.bufferSize,
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (imageBuffer.imageBuffer) {
            Serial.printf("Allocated %u bytes in internal RAM\n", imageBuffer.bufferSize);
        }
    }
    
    if (!imageBuffer.imageBuffer) {
        Serial.println("ERROR: Failed to allocate image buffer!");
        if (imageBuffer.bufferMutex) {
            vSemaphoreDelete(imageBuffer.bufferMutex);
            imageBuffer.bufferMutex = nullptr;
        }
        return;
    }
    
    // Clear buffer
    memset(imageBuffer.imageBuffer, 0, imageBuffer.bufferSize);
    
    // Calculate maximum image dimensions that fit in buffer
    size_t maxPixels = imageBuffer.bufferSize / 2; // RGB565 = 2 bytes per pixel
    size_t maxDimension = (size_t)sqrt(maxPixels);
    
    Serial.printf("Realistic image buffer initialized successfully\n");
    Serial.printf("Buffer size: %u bytes, Max image: %ux%u pixels\n",
                  imageBuffer.bufferSize, maxDimension, maxDimension);
}

// Release image buffer
void releaseRealisticImageBuffer() {
    if (imageBuffer.imageBuffer) {
        heap_caps_free(imageBuffer.imageBuffer);
        imageBuffer.imageBuffer = nullptr;
    }
    
    if (imageBuffer.bufferMutex) {
        vSemaphoreDelete(imageBuffer.bufferMutex);
        imageBuffer.bufferMutex = nullptr;
    }
}

// FIXED: Realistic JPEG decoder callback with proper bounds checking
int realisticJpegDrawCallback(JPEGDRAW *pDraw) {
    if (!imageBuffer.imageBuffer) {
        return 0;
    }
    
    // Get image buffer
    uint16_t *destBuffer = (uint16_t *)(imageBuffer.imageBuffer);
    if (!destBuffer) return 0;
    
    // Calculate destination offset for current image dimensions
    int destOffset = (pDraw->y * imageBuffer.imageWidth) + pDraw->x;
    
    // Strict bounds checking
    if (destOffset < 0 ||
        (destOffset + pDraw->iWidth * pDraw->iHeight) > (imageBuffer.bufferSize / 2)) {
        return 0; // Out of bounds
    }
    
    // Copy pixel data at original resolution
    uint16_t *src = pDraw->pPixels;
    uint16_t *dest = destBuffer + destOffset;
    
    for (int y = 0; y < pDraw->iHeight; y++) {
        if ((pDraw->y + y) >= imageBuffer.imageHeight) break;
        
        for (int x = 0; x < pDraw->iWidth; x++) {
            if ((pDraw->x + x) >= imageBuffer.imageWidth) break;
            dest[y * imageBuffer.imageWidth + x] = src[y * pDraw->iWidth + x];
        }
    }
    
    return 1;
}

// FIXED: Enhanced JPEG decoding with realistic buffer management
bool decodeAndDisplayImageRealistic(const String &filename) {
    if (!sdCardInitialized || filename.isEmpty()) {
        Serial.println("SD card not available or empty filename");
        return false;
    }
    
    uint32_t startTime = millis();
    Serial.printf("Loading realistic image: %s\n", filename.c_str());
    
    // Ensure buffer is available
    if (!imageBuffer.imageBuffer) {
        Serial.println("Image buffer not available");
        return false;
    }
    
    // Open JPEG file
    String path = "/" + filename;
    File jpegFile = SD.open(path.c_str(), "r");
    if (!jpegFile) {
        Serial.println("Failed to open JPEG file");
        return false;
    }
    
    size_t fileSize = jpegFile.size();
    if (fileSize == 0) {
        Serial.println("Empty JPEG file");
        jpegFile.close();
        return false;
    }
    
    // Initialize JPEG decoder
    if (!jpegDec.open(jpegFile, realisticJpegDrawCallback)) {
        Serial.println("Failed to initialize JPEG decoder");
        jpegFile.close();
        return false;
    }
    
    // Get original image dimensions
    int originalWidth = jpegDec.getWidth();
    int originalHeight = jpegDec.getHeight();
    Serial.printf("Original JPEG dimensions: %d x %d (%u bytes)\n",
                  originalWidth, originalHeight, fileSize);
    
    // Calculate maximum dimensions that fit in buffer
    size_t maxPixels = imageBuffer.bufferSize / 2; // RGB565
    size_t maxDimension = (size_t)sqrt(maxPixels);
    
    // Determine scaling needed
    int scale = 0;
    imageBuffer.imageWidth = originalWidth;
    imageBuffer.imageHeight = originalHeight;
    
    // Apply scaling if image is too large for buffer
    if (originalWidth > maxDimension || originalHeight > maxDimension) {
        if (originalWidth > maxDimension * 8 || originalHeight > maxDimension * 8) {
            scale = 3; // JPEG_SCALE_EIGHTH
            imageBuffer.imageWidth = originalWidth / 8;
            imageBuffer.imageHeight = originalHeight / 8;
        } else if (originalWidth > maxDimension * 4 || originalHeight > maxDimension * 4) {
            scale = 2; // JPEG_SCALE_QUARTER
            imageBuffer.imageWidth = originalWidth / 4;
            imageBuffer.imageHeight = originalHeight / 4;
        } else if (originalWidth > maxDimension * 2 || originalHeight > maxDimension * 2) {
            scale = 1; // JPEG_SCALE_HALF
            imageBuffer.imageWidth = originalWidth / 2;
            imageBuffer.imageHeight = originalHeight / 2;
        }
    }
    
    // Final check if scaled image fits in buffer
    size_t requiredSize = imageBuffer.imageWidth * imageBuffer.imageHeight * 2;
    if (requiredSize > imageBuffer.bufferSize) {
        Serial.printf("Image still too large after scaling: %u > %u\n", requiredSize, imageBuffer.bufferSize);
        jpegDec.close();
        jpegFile.close();
        return false;
    }
    
    Serial.printf("Using scale: %d, Final dimensions: %dx%d\n",
                  scale, imageBuffer.imageWidth, imageBuffer.imageHeight);
    
    // Clear image buffer
    memset(imageBuffer.imageBuffer, 0, requiredSize);
    
    // Configure JPEG decoder
    jpegDec.setPixelType(RGB565_LITTLE_ENDIAN);
    jpegDec.setMaxOutputSize(512); // Conservative MCU buffer size
    
    // Reset scroll position
    imageBuffer.scrollX = 0;
    imageBuffer.scrollY = 0;
    
    // Setup image descriptor for LVGL
    imgDsc.header.always_zero = 0;
    imgDsc.header.w = imageBuffer.imageWidth;
    imgDsc.header.h = imageBuffer.imageHeight;
    imgDsc.data_size = requiredSize;
    imgDsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    imgDsc.data = imageBuffer.imageBuffer;
    
    // Decode JPEG
    bool decodeSuccess = (jpegDec.decode(0, 0, scale) == 1);
    jpegDec.close();
    jpegFile.close();
    
    if (!decodeSuccess) {
        Serial.println("JPEG decode failed");
        return false;
    }
    
    // Calculate performance metrics
    uint32_t decodeTime = millis() - startTime;
    lastDrawTime = decodeTime;
    totalDrawTime += decodeTime;
    frameCount++;
    lastSlideChangeTime = millis();
    
    Serial.printf("JPEG decoded successfully in %u ms\n", decodeTime);
    
    // Update LVGL display
    if (lvglInitialized && imgViewer) {
        if (lvgl_port_lock(100)) {
            lv_img_set_src(imgViewer, &imgDsc);
            
            // Center the image or enable scrolling if larger than display
            if (imageBuffer.imageWidth > imageBuffer.displayWidth ||
                imageBuffer.imageHeight > imageBuffer.displayHeight) {
                // Calculate centering position
                int centerX = (imageBuffer.imageWidth - imageBuffer.displayWidth) / 2;
                int centerY = (imageBuffer.imageHeight - imageBuffer.displayHeight) / 2;
                lv_obj_set_pos(imgViewer, -centerX, -centerY);
                
                // Enable scrolling on container
                lv_obj_add_flag(imgContainer, LV_OBJ_FLAG_SCROLLABLE);
                lv_obj_set_scroll_dir(imgContainer, LV_DIR_ALL);
                Serial.printf("Image larger than display - scrolling enabled\n");
            } else {
                // Center smaller images
                lv_obj_center(imgViewer);
                lv_obj_clear_flag(imgContainer, LV_OBJ_FLAG_SCROLLABLE);
            }
            
            lvgl_port_unlock();
        }
    }
    
    isImageLoaded = true;
    updateStatusLabel();
    
    return true;
}

// Show image by specific index (for web interface)
void showImageByIndex(int index) {
    if (index < 0 || index >= imageFiles.size()) {
        Serial.printf("Invalid image index: %d\n", index);
        return;
    }
    
    currentImageIndex = index;
    decodeAndDisplayImageRealistic(imageFiles[currentImageIndex]);
}

// Initialize SD card (unchanged as requested)
bool initSDCard() {
    Serial.println("Initializing SD card...");
    
    auto expander = board->getIO_Expander();
    if (!expander) {
        Serial.println("ERROR: IO expander not available");
        return false;
    }
    
    expander->getBase()->pinMode(SD_CS, OUTPUT);
    expander->getBase()->digitalWrite(SD_CS, LOW);
    delay(50);
    
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI);
    
    uint32_t spiFreqs[] = {25000000, 16000000, 10000000, 4000000};
    bool mounted = false;
    
    for (uint8_t i = 0; i < sizeof(spiFreqs) / sizeof(spiFreqs[0]) && !mounted; i++) {
        SPI.setFrequency(spiFreqs[i]);
        delay(50);
        Serial.printf("Trying SD card at %d Hz...\n", spiFreqs[i]);
        mounted = SD.begin(SD_SS);
        if (mounted) {
            Serial.printf("SD card initialized at %d Hz\n", spiFreqs[i]);
            break;
        }
    }
    
    if (!mounted) {
        Serial.println("SD Card Mount Failed");
        return false;
    }
    
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return false;
    }
    
    Serial.printf("SD Card Type: %s\n",
                  cardType == CARD_MMC ? "MMC" :
                  cardType == CARD_SD ? "SDSC" :
                  cardType == CARD_SDHC ? "SDHC" : "UNKNOWN");
    
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    
    return true;
}

// Check if file is JPEG
bool isJpegFile(const String &filename) {
    String lower = filename;
    lower.toLowerCase();
    return lower.endsWith(".jpg") || lower.endsWith(".jpeg");
}

// Scan SD card for images
void scanSDCardImages() {
    imageFiles.clear();
    Serial.println("Scanning for JPEG files on SD card...");
    
    File root = SD.open("/");
    if (!root || !root.isDirectory()) {
        Serial.println("Failed to open root directory");
        return;
    }
    
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String filename = file.name();
            size_t fileSize = file.size();
            if (isJpegFile(filename) && fileSize > 0) {
                imageFiles.push_back(filename);
                Serial.printf("Found image: %s (%u bytes)\n", filename.c_str(), fileSize);
            }
        }
        file.close();
        file = root.openNextFile();
    }
    
    root.close();
    Serial.printf("Found %d valid JPEG images\n", imageFiles.size());
    updateStatusLabel();
}

// Create LVGL UI with scrolling support
void createUI() {
    if (!lvgl_port_lock(-1)) {
        Serial.println("Failed to lock LVGL");
        return;
    }
    
    mainScreen = lv_scr_act();
    
    // Setup button styles
    lv_style_init(&styleBtn);
    lv_style_set_radius(&styleBtn, 10);
    lv_style_set_bg_color(&styleBtn, lv_color_hex(0x2196F3));
    lv_style_set_bg_opa(&styleBtn, LV_OPA_COVER);
    lv_style_set_text_color(&styleBtn, lv_color_white());
    
    lv_style_init(&styleBtn_danger);
    lv_style_set_radius(&styleBtn_danger, 10);
    lv_style_set_bg_color(&styleBtn_danger, lv_color_hex(0xDC3545));
    lv_style_set_bg_opa(&styleBtn_danger, LV_OPA_COVER);
    lv_style_set_text_color(&styleBtn_danger, lv_color_white());
    
    // Create scrollable image container
    imgContainer = lv_obj_create(mainScreen);
    lv_obj_set_size(imgContainer, SCREEN_WIDTH, SCREEN_HEIGHT - 70);
    lv_obj_align(imgContainer, LV_ALIGN_TOP_MID, 0, 5);
    lv_obj_set_style_bg_color(imgContainer, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_pad_all(imgContainer, 0, LV_PART_MAIN);
    lv_obj_set_scroll_dir(imgContainer, LV_DIR_ALL);
    
    // Create image viewer
    imgViewer = lv_img_create(imgContainer);
    lv_obj_center(imgViewer);
    
    // Create navigation buttons
    btnPrev = lv_btn_create(mainScreen);
    lv_obj_add_style(btnPrev, &styleBtn, 0);
    lv_obj_set_size(btnPrev, 80, 40);
    lv_obj_align(btnPrev, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_t *prevLabel = lv_label_create(btnPrev);
    lv_label_set_text(prevLabel, "Prev");
    lv_obj_center(prevLabel);
    
    btnNext = lv_btn_create(mainScreen);
    lv_obj_add_style(btnNext, &styleBtn, 0);
    lv_obj_set_size(btnNext, 80, 40);
    lv_obj_align(btnNext, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_t *nextLabel = lv_label_create(btnNext);
    lv_label_set_text(nextLabel, "Next");
    lv_obj_center(nextLabel);
    
    // Create random button
    btnRandom = lv_btn_create(mainScreen);
    lv_obj_add_style(btnRandom, &styleBtn, 0);
    lv_obj_set_size(btnRandom, 80, 40);
    lv_obj_align(btnRandom, LV_ALIGN_BOTTOM_MID, -90, -10);
    lv_obj_t *randomLabel = lv_label_create(btnRandom);
    lv_label_set_text(randomLabel, "Random");
    lv_obj_center(randomLabel);
    
    // Create delete button
    btnDelete = lv_btn_create(mainScreen);
    lv_obj_add_style(btnDelete, &styleBtn_danger, 0);
    lv_obj_set_size(btnDelete, 80, 40);
    lv_obj_align(btnDelete, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_t *deleteLabel = lv_label_create(btnDelete);
    lv_label_set_text(deleteLabel, "Delete");
    lv_obj_center(deleteLabel);
    
    // Create status label
    statusLabel = lv_label_create(mainScreen);
    lv_obj_align(statusLabel, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_set_style_text_color(statusLabel, lv_color_white(), 0);
    lv_label_set_text(statusLabel, "Loading images...");
    
    // Create WiFi status label
    wifiLabel = lv_label_create(mainScreen);
    lv_obj_align(wifiLabel, LV_ALIGN_TOP_RIGHT, -10, 10);
    lv_obj_set_style_text_color(wifiLabel, lv_color_white(), 0);
    lv_label_set_text(wifiLabel, "WiFi: Active");
    
    // Set up button event handlers
    lv_obj_add_event_cb(btnPrev, [](lv_event_t *e) { showPrevImage(); }, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(btnNext, [](lv_event_t *e) { showNextImage(); }, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(btnRandom, [](lv_event_t *e) { showRandomImage(); }, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(btnDelete, [](lv_event_t *e) { deleteCurrentImage(); }, LV_EVENT_CLICKED, NULL);
    
    lvgl_port_unlock();
}

// Update status label
void updateStatusLabel() {
    if (!lvglInitialized || !statusLabel) return;
    
    if (lvgl_port_lock(10)) {
        String status;
        if (!sdCardInitialized) {
            status = "SD card not available";
        } else if (imageFiles.empty()) {
            status = "No JPEG images found";
        } else {
            status = "Image " + String(currentImageIndex + 1) + "/" +
                    String(imageFiles.size()) + ": " +
                    imageFiles[currentImageIndex];
            if (isImageLoaded) {
                status += String(" (") + String(imageBuffer.imageWidth) + "x" +
                         String(imageBuffer.imageHeight) + ")";
            }
            
            if (frameCount > 0) {
                float avgTime = (float)totalDrawTime / frameCount;
                status += String(" ") + String(avgTime, 1) + "ms";
            }
        }
        lv_label_set_text(statusLabel, status.c_str());
        lvgl_port_unlock();
    }
}

// Show next image
void showNextImage() {
    if (imageFiles.empty()) return;
    
    currentImageIndex = (currentImageIndex + 1) % imageFiles.size();
    decodeAndDisplayImageRealistic(imageFiles[currentImageIndex]);
}

// Show previous image
void showPrevImage() {
    if (imageFiles.empty()) return;
    
    currentImageIndex = (currentImageIndex - 1 + imageFiles.size()) % imageFiles.size();
    decodeAndDisplayImageRealistic(imageFiles[currentImageIndex]);
}

// Show random image
void showRandomImage() {
    if (imageFiles.empty()) return;
    
    if (imageFiles.size() > 1) {
        int newIndex;
        do {
            newIndex = esp_random() % imageFiles.size();
        } while (newIndex == currentImageIndex && imageFiles.size() > 1);
        currentImageIndex = newIndex;
    } else {
        currentImageIndex = 0;
    }
    
    decodeAndDisplayImageRealistic(imageFiles[currentImageIndex]);
}

// Update auto slideshow
void updateAutoSlideshow() {
    if (randomPreviewMode) {
        showRandomImage();
    } else {
        showNextImage();
    }
    
    lastSlideChangeTime = millis();
}

// Delete current image
void deleteCurrentImage() {
    if (imageFiles.empty() || currentImageIndex < 0 || currentImageIndex >= imageFiles.size()) {
        return;
    }
    
    String filename = imageFiles[currentImageIndex];
    String fullPath = "/" + filename;
    
    if (SD.remove(fullPath)) {
        Serial.printf("Deleted: %s\n", filename.c_str());
        imageFiles.erase(imageFiles.begin() + currentImageIndex);
        
        if (imageFiles.empty()) {
            currentImageIndex = -1;
        } else {
            if (currentImageIndex >= imageFiles.size()) {
                currentImageIndex = imageFiles.size() - 1;
            }
            
            if (currentImageIndex >= 0) {
                decodeAndDisplayImageRealistic(imageFiles[currentImageIndex]);
            }
        }
        
        updateStatusLabel();
    }
}

// Setup web server with image selection capability (EXACT HTML from original)
void setupWebServer() {
    // Root page with image selection
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html",
            "<!DOCTYPE html>"
            "<html>"
            "<head>"
            "<title>ESP32 HD Image Viewer - Realistic Resolution</title>"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
            "<style>"
            "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
            ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }"
            ".button { background: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }"
            ".button:hover { background: #45a049; }"
            ".danger { background: #f44336; }"
            ".danger:hover { background: #da190b; }"
            ".status { margin: 10px 0; padding: 10px; background: #e7f3ff; border-radius: 5px; }"
            ".progress { width: 100%; background: #ddd; border-radius: 5px; margin: 10px 0; }"
            ".progress-bar { width: 0%; height: 20px; background: #4CAF50; border-radius: 5px; transition: width 0.3s; }"
            ".image-list { margin: 20px 0; }"
            ".image-item { padding: 10px; margin: 5px 0; background: #f9f9f9; border-radius: 5px; cursor: pointer; }"
            ".image-item:hover { background: #e9e9e9; }"
            ".current { background: #d4edda; border: 2px solid #28a745; }"
            "</style>"
            "</head>"
            "<body>"
            "<div class=\"container\">"
            "<h1>ESP32 HD Image Viewer - Realistic Resolution</h1>"
            "<div class=\"status\" id=\"status\">Ready</div>"
            ""
            "<h3>Upload JPEG Image</h3>"
            "<input type=\"file\" id=\"fileInput\" accept=\".jpg,.jpeg\" />"
            "<button class=\"button\" onclick=\"uploadFile()\">Upload</button>"
            "<div class=\"progress\"><div class=\"progress-bar\" id=\"progressBar\"></div></div>"
            ""
            "<h3>Display Controls</h3>"
            "<button class=\"button\" onclick=\"controlDisplay('next')\">Next Image</button>"
            "<button class=\"button\" onclick=\"controlDisplay('prev')\">Previous Image</button>"
            "<button class=\"button\" onclick=\"controlDisplay('random')\">Random Image</button>"
            "<button class=\"button danger\" onclick=\"controlDisplay('delete')\">Delete Current</button>"
            ""
            "<h3>Available Images (Click to Display)</h3>"
            "<div class=\"image-list\" id=\"imageList\">Loading...</div>"
            "</div>"
            ""
            "<script>"
            "function updateStatus(message, isError) {"
            "  isError = isError || false;"
            "  const status = document.getElementById('status');"
            "  status.textContent = message;"
            "  status.style.background = isError ? '#f8d7da' : '#d4edda';"
            "}"
            ""
            "function updateProgress(percent) {"
            "  document.getElementById('progressBar').style.width = percent + '%';"
            "}"
            ""
            "function uploadFile() {"
            "  const fileInput = document.getElementById('fileInput');"
            "  const file = fileInput.files[0];"
            "  if (!file) {"
            "    updateStatus('Please select a file', true);"
            "    return;"
            "  }"
            ""
            "  const formData = new FormData();"
            "  formData.append('file', file);"
            ""
            "  updateStatus('Uploading...');"
            "  updateProgress(0);"
            ""
            "  fetch('/upload', {"
            "    method: 'POST',"
            "    body: formData"
            "  })"
            "  .then(response => response.text())"
            "  .then(result => {"
            "    updateStatus('Upload successful: ' + result);"
            "    updateProgress(100);"
            "    setTimeout(function() { updateProgress(0); loadImageList(); }, 2000);"
            "  })"
            "  .catch(error => {"
            "    updateStatus('Upload failed: ' + error, true);"
            "    updateProgress(0);"
            "  });"
            "}"
            ""
            "function controlDisplay(action) {"
            "  updateStatus('Sending command: ' + action);"
            "  fetch('/control?action=' + action)"
            "  .then(response => response.text())"
            "  .then(result => {"
            "    updateStatus('Command executed: ' + result);"
            "    if (action !== 'delete') setTimeout(loadImageList, 1000);"
            "  })"
            "  .catch(error => updateStatus('Command failed: ' + error, true));"
            "}"
            ""
            "function selectImage(index) {"
            "  updateStatus('Loading image ' + (index + 1));"
            "  fetch('/select?index=' + index)"
            "  .then(response => response.text())"
            "  .then(result => {"
            "    updateStatus('Image loaded: ' + result);"
            "    loadImageList();"
            "  })"
            "  .catch(error => updateStatus('Failed to load image: ' + error, true));"
            "}"
            ""
            "function loadImageList() {"
            "  fetch('/list')"
            "  .then(response => response.json())"
            "  .then(data => {"
            "    const imageList = document.getElementById('imageList');"
            "    if (data.images.length === 0) {"
            "      imageList.innerHTML = '<p>No images found</p>';"
            "      return;"
            "    }"
            ""
            "    let html = '';"
            "    data.images.forEach(function(image, index) {"
            "      const isCurrentClass = index === data.current ? 'current' : '';"
            "      html += '<div class=\"image-item ' + isCurrentClass + '\" onclick=\"selectImage(' + index + ')\">';"
            "      html += (index + 1) + '. ' + image;"
            "      html += '</div>';"
            "    });"
            "    imageList.innerHTML = html;"
            "    updateStatus('Found ' + data.images.length + ' images, current: ' + (data.current + 1));"
            "  })"
            "  .catch(error => {"
            "    updateStatus('Failed to load image list: ' + error, true);"
            "  });"
            "}"
            ""
            "// Load image list on page load"
            "window.onload = loadImageList;"
            ""
            "// Auto-refresh every 5 seconds"
            "setInterval(loadImageList, 5000);"
            "</script>"
            "</body>"
            "</html>"
        );
    });

    // API endpoints for image management
    server.on("/list", HTTP_GET, []() {
        String json = "{\"images\":[";
        for (size_t i = 0; i < imageFiles.size(); i++) {
            if (i > 0) json += ",";
            json += "\"" + imageFiles[i] + "\"";
        }
        json += "],\"current\":" + String(currentImageIndex) + "}";
        server.send(200, "application/json", json);
    });

    server.on("/control", HTTP_GET, []() {
        String action = server.arg("action");
        String response = "Unknown action";
        
        if (action == "next") {
            showNextImage();
            response = "Next image loaded";
        } else if (action == "prev") {
            showPrevImage();
            response = "Previous image loaded";
        } else if (action == "random") {
            showRandomImage();
            response = "Random image loaded";
        } else if (action == "delete") {
            deleteCurrentImage();
            response = "Image deleted";
        }
        
        server.send(200, "text/plain", response);
    });

    server.on("/select", HTTP_GET, []() {
        int index = server.arg("index").toInt();
        if (index >= 0 && index < imageFiles.size()) {
            currentImageIndex = index;
            decodeAndDisplayImageRealistic(imageFiles[currentImageIndex]);
            server.send(200, "text/plain", "Image " + String(index + 1) + " loaded");
        } else {
            server.send(400, "text/plain", "Invalid image index");
        }
    });

    // File upload handler
    server.on("/upload", HTTP_POST, []() {
        server.send(200, "text/plain", "Upload complete");
    }, []() {
        HTTPUpload& upload = server.upload();
        static File uploadFile;
        
        if (upload.status == UPLOAD_FILE_START) {
            String filename = "/" + upload.filename;
            uploadFile = SD.open(filename, FILE_WRITE);
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (uploadFile) {
                uploadFile.write(upload.buf, upload.currentSize);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (uploadFile) {
                uploadFile.close();
                scanSDCardImages(); // Refresh image list
            }
        }
    });

    server.begin();
}

// **CRITICAL: Follow WiFi-FIRST sequence to prevent memory allocation crashes**
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ESP32-S3-Touch-LCD-5_B HD Image Viewer - WiFi First Initialization");
    Serial.println("=================================================================");
    
    // **STEP 1: Initialize WiFi FIRST to prevent memory allocation issues**
    Serial.println("Step 1: Initializing WiFi FIRST...");
    wifiConnected = initWiFi();
    if (wifiConnected) {
        Serial.println("✓ WiFi initialized successfully");
    } else {
        Serial.println("✗ WiFi initialization failed!");
    }
    
    // **STEP 2: Optimize system performance**
    Serial.println("Step 2: Optimizing system performance...");
    optimizeSystemPerformance();
    
    // **STEP 3: Initialize board**
    Serial.println("Step 3: Initializing board...");
    board = new Board();
    if (!board->init()) {
        Serial.println("✗ Board initialization failed!");
        return;
    }
    Serial.println("✓ Board initialized successfully");
    
    // **STEP 4: Initialize LCD**
    Serial.println("Step 4: Initializing LCD...");
    lcd = board->getLCD();
    if (!lcd->init()) {
        Serial.println("✗ LCD initialization failed!");
        return;
    }
    Serial.println("✓ LCD initialized successfully");
    
    // **STEP 5: Initialize touch**
    Serial.println("Step 5: Initializing touch...");
    touch = board->getTouch();
    if (touch && !touch->init()) {
        Serial.println("⚠ Touch initialization failed!");
        touch = nullptr;
    } else {
        Serial.println("✓ Touch initialized successfully");
    }
    
    // **STEP 6: Begin panel devices**
    Serial.println("Step 6: Beginning panel devices...");
    if (!board->begin()) {
        Serial.println("✗ Panel begin failed!");
        return;
    }
    Serial.println("✓ Panel devices started successfully");
    
    // **STEP 7: Initialize LVGL**
    Serial.println("Step 7: Initializing LVGL...");
    if (!lvgl_port_init(lcd, touch)) {
        Serial.println("✗ LVGL initialization failed!");
        return;
    }
    lvglInitialized = true;
    Serial.println("✓ LVGL initialized successfully");
    
    // **STEP 8: Initialize SD card**
    Serial.println("Step 8: Initializing SD card...");
    sdCardInitialized = initSDCard();
    if (sdCardInitialized) {
        Serial.println("✓ SD card initialized successfully");
        scanSDCardImages();
    } else {
        Serial.println("⚠ SD card initialization failed!");
    }
    
    // **STEP 9: Initialize image buffer**
    Serial.println("Step 9: Initializing image buffer...");
    initRealisticImageBuffer();
    Serial.println("✓ Image buffer initialized successfully");
    
    // **STEP 10: Create UI**
    Serial.println("Step 10: Creating UI...");
    createUI();
    Serial.println("✓ UI created successfully");
    
    // **STEP 11: Setup web server (WiFi already initialized)**
    if (wifiConnected) {
        Serial.println("Step 11: Setting up web server...");
        setupWebServer();
        Serial.println("✓ Web server started successfully");
    }
    
    // **STEP 12: Load first image if available**
    if (sdCardInitialized && !imageFiles.empty()) {
        Serial.println("Step 12: Loading first image...");
        currentImageIndex = 0;
        if (decodeAndDisplayImageRealistic(imageFiles[currentImageIndex])) {
            Serial.println("✓ First image loaded successfully");
        }
    }
    
    Serial.println("=================================================================");
    Serial.println("Setup complete - Image viewer ready!");
    Serial.println("=================================================================");
    
    if (wifiConnected) {
        Serial.printf("WiFi AP: %s\n", WIFI_AP_SSID);
        Serial.printf("Password: %s\n", WIFI_AP_PASSWORD);
        Serial.printf("Web interface: http://%s\n", WiFi.softAPIP().toString().c_str());
    }
    
    Serial.println("System Status:");
    Serial.printf("- WiFi: %s\n", wifiConnected ? "✓ OK" : "✗ FAILED");
    Serial.printf("- Board: %s\n", board ? "✓ OK" : "✗ FAILED");
    Serial.printf("- LCD: %s\n", lcd ? "✓ OK" : "✗ FAILED");
    Serial.printf("- Touch: %s\n", touch ? "✓ OK" : "✗ NOT AVAILABLE");
    Serial.printf("- LVGL: %s\n", lvglInitialized ? "✓ OK" : "✗ FAILED");
    Serial.printf("- SD Card: %s\n", sdCardInitialized ? "✓ OK" : "✗ FAILED");
    Serial.printf("- Images found: %d\n", imageFiles.size());
    
    Serial.println("=================================================================");
}

void loop() {
    // Handle web server requests
    if (wifiConnected) {
        server.handleClient();
    }
    
    // Auto slideshow functionality
    if (AUTO_SLIDESHOW_INTERVAL > 0 && !imageFiles.empty() && 
        (millis() - lastSlideChangeTime) > AUTO_SLIDESHOW_INTERVAL) {
        updateAutoSlideshow();
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}
