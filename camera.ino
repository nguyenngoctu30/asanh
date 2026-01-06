#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <PubSubClient.h>
#include <HTTPClient.h>

// ==================== CONFIGURATION ====================
// WiFi credentials
const char* ssid = "Tiem Tra Hoa FPT";
const char* password = "79797979";

// MQTT Broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32cam/capture";
const char* mqtt_client_id = "ESP32CAM_Client";
// Status topic for connection and upload messages
const char* mqtt_status_topic = "esp32cam/status";

// Server URL - Using HTTP
const char* serverUrl = "http://vista-workplace-toe-buildings.trycloudflare.com/upload";
 https://having-medications-king-concerts.trycloudflare.com  
// Upload settings
const int MAX_UPLOAD_RETRIES = 3;
const int UPLOAD_TIMEOUT = 30000;  // 30 seconds
const int CONNECT_TIMEOUT = 15000; // 15 seconds

// WiFi client with larger buffer
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

boolean takeNewPhoto = false;
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000;

// ==================== CAMERA PINS ====================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
// Flash (LED) pin (AI-Thinker modules commonly use GPIO4)
#define FLASH_GPIO_NUM     4

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n🚀 ESP32-CAM Starting...");
  // Initialize flash pin
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  // Default OFF (assume HIGH turns LED on on most boards; change if inverted)
  digitalWrite(FLASH_GPIO_NUM, LOW);
  
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Initialize WiFi
  setupWiFi();
  
  // Setup MQTT
  setupMQTT();
  
  // Initialize Camera
  setupCamera();
  
  Serial.println("✅ System ready!");
  Serial.println("==================================\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Periodic WiFi health check
  if (millis() - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    checkWiFiHealth();
    lastWiFiCheck = millis();
  }
  
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  // Handle photo capture request
  if (takeNewPhoto) {
    Serial.println("\n📸 ========== CAPTURE REQUEST ==========");
    unsigned long startTime = millis();
    
    captureAndUploadPhoto();
    
    takeNewPhoto = false;
    unsigned long totalTime = millis() - startTime;
    Serial.printf("🕒 Total operation time: %lu ms\n", totalTime);
    Serial.println("========================================\n");
  }
  
  delay(50);
}

// ==================== WiFi FUNCTIONS ====================
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  
  // Increase WiFi power for better signal
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  Serial.println("📡 Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.printf("📍 IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("📶 Signal: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("\n❌ WiFi Failed! Restarting...");
    delay(2000);
    ESP.restart();
  }
}

void checkWiFiHealth() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected! Reconnecting...");
    WiFi.reconnect();
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("✅ WiFi reconnected");
    } else {
      Serial.println("❌ WiFi reconnection failed, restarting...");
      ESP.restart();
    }
  }
}

// ==================== MQTT FUNCTIONS ====================
void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  mqttClient.setKeepAlive(60);
  reconnectMQTT();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("📨 MQTT message on [%s]: ", topic);
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  if (message == "CAPTURE" || message == "capture") {
    Serial.println("✅ Capture command accepted");
    takeNewPhoto = true;
    if (mqttClient.connected()) {
      mqttClient.publish(mqtt_status_topic, "capture_received");
    }
  }
}

void reconnectMQTT() {
  int attempts = 0;
  
  while (!mqttClient.connected() && attempts < 5) {
    Serial.printf("🔌 MQTT connecting (attempt %d/5)...", attempts + 1);
    
    if (mqttClient.connect(mqtt_client_id)) {
      Serial.println(" ✅ Connected");
      mqttClient.subscribe(mqtt_topic);
      Serial.printf("📡 Subscribed to: %s\n", mqtt_topic);
      // Publish online status
      if (mqttClient.connected()) {
        mqttClient.publish(mqtt_status_topic, "online");
      }
      return;
    } else {
      Serial.printf(" ❌ Failed (rc=%d)\n", mqttClient.state());
      delay(3000);
      attempts++;
    }
  }
  
  if (!mqttClient.connected()) {
    Serial.println("❌ MQTT connection failed, restarting ESP32...");
    delay(1000);
    // If still connected (rare), publish offline; otherwise just restart
    if (mqttClient.connected()) {
      mqttClient.publish(mqtt_status_topic, "offline");
    }
    ESP.restart();
  }
}

// ==================== CAMERA FUNCTIONS ====================
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // Use lower resolution for more reliable uploads
  if (psramFound()) {
    Serial.println("✅ PSRAM detected");
    config.frame_size = FRAMESIZE_VGA;  // 640x480 (smaller than SVGA)
    config.jpeg_quality = 14;           // Slightly higher quality but smaller
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("⚠️ No PSRAM, using internal RAM");
    config.frame_size = FRAMESIZE_CIF;  // 400x296 (even smaller)
    config.jpeg_quality = 16;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    delay(1000);
    ESP.restart();
  }

  Serial.println("✅ Camera initialized");
  
  // Warm up camera
  for(int i = 0; i < 3; i++) {
    camera_fb_t * fb = esp_camera_fb_get();
    if(fb) {
      Serial.printf("Warm-up capture %d: %d bytes\n", i+1, fb->len);
      esp_camera_fb_return(fb);
      delay(100);
    }
  }
  Serial.println("✅ Camera warmed up");
}

// ==================== ENHANCED UPLOAD FUNCTIONS ====================
bool testServerConnection() {
  Serial.println("🔍 Testing server connection...");
  
  HTTPClient http;
  http.setTimeout(10000);
  http.setReuse(false);
  
  // Try to connect to server root first
  String testUrl = String(serverUrl);
  testUrl.replace("/upload", "");
  
  if (http.begin(wifiClient, testUrl)) {
    int httpCode = http.GET();
    Serial.printf("Server test: HTTP %d\n", httpCode);
    http.end();
    return (httpCode > 0);
  }
  return false;
}

bool uploadImageBuffer(uint8_t* buffer, size_t length) {
  HTTPClient http;
  
  // Configure timeouts
  http.setTimeout(UPLOAD_TIMEOUT);
  http.setConnectTimeout(CONNECT_TIMEOUT);
  http.setReuse(false);
  
  // Add chunked encoding for better reliability
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("Connection", "close");
  http.addHeader("User-Agent", "ESP32-CAM");
  
  Serial.printf("📤 Uploading %d bytes...\n", length);
  
  unsigned long uploadStart = millis();
  
  if (http.begin(wifiClient, serverUrl)) {
    // Use chunked transfer encoding
    int httpCode = http.POST(buffer, length);
    unsigned long uploadTime = millis() - uploadStart;
    
    if (httpCode > 0) {
      Serial.printf("📬 HTTP Response: %d (Time: %lums)\n", httpCode, uploadTime);
      
      if (httpCode == HTTP_CODE_OK) {
        String response = http.getString();
        Serial.printf("✅ Upload successful! Response: %s\n", response.c_str());
        http.end();
        return true;
      } else {
        Serial.printf("⚠️ Server returned: %d\n", httpCode);
        String response = http.getString();
        if (response.length() > 0) {
          Serial.printf("Response: %s\n", response.c_str());
        }
      }
    } else {
      Serial.printf("❌ HTTP Error: %d (%s)\n", httpCode, http.errorToString(httpCode).c_str());
      
      // Enhanced error diagnostics
      switch(httpCode) {
        case -1: Serial.println("   → Connection timeout/refused"); break;
        case -2: Serial.println("   → Invalid server response"); break;
        case -3: 
          Serial.println("   → Send payload failed - Network issue");
          Serial.println("   → Check: WiFi signal, server reachability, payload size");
          break;
        case -4: Serial.println("   → Connection lost"); break;
        case -11: Serial.println("   → Read timeout"); break;
      }
    }
    
    http.end();
  } else {
    Serial.println("❌ HTTP begin() failed");
  }
  
  return false;
}

// ==================== IMPROVED CAPTURE & UPLOAD ====================
void captureAndUploadPhoto() {
  unsigned long t_start, t_capture;
  // Turn on flash and give it a moment to stabilize
  digitalWrite(FLASH_GPIO_NUM, HIGH);
  delay(120);
  
  // Clear any previous frame
  camera_fb_t * fb_temp = esp_camera_fb_get();
  if (fb_temp) {
    esp_camera_fb_return(fb_temp);
    delay(200); // Longer delay for camera stabilization
  }
  
  // Capture photo
  t_start = millis();
  Serial.println("📷 Capturing image...");
  
  camera_fb_t * fb = esp_camera_fb_get();
  t_capture = millis();
  
  if (!fb) {
    Serial.println("❌ Camera capture failed!");
    // Turn off flash before exiting
    digitalWrite(FLASH_GPIO_NUM, LOW);
    return;
  }
  
  Serial.printf("✅ Captured: %d bytes in %lu ms\n", fb->len, (t_capture - t_start));
  Serial.printf("📊 Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("📶 WiFi RSSI: %d dBm\n", WiFi.RSSI());

  // Enhanced WiFi verification
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected, aborting upload");
    esp_camera_fb_return(fb);
    digitalWrite(FLASH_GPIO_NUM, LOW);
    return;
  }

  // Test server connection first
  if (!testServerConnection()) {
    Serial.println("❌ Server connection test failed!");
    esp_camera_fb_return(fb);
    digitalWrite(FLASH_GPIO_NUM, LOW);
    return;
  }

  // Upload with enhanced retry mechanism
  bool uploadSuccess = false;
  
  for (int attempt = 1; attempt <= MAX_UPLOAD_RETRIES && !uploadSuccess; attempt++) {
    Serial.printf("\n🔄 Upload attempt %d/%d\n", attempt, MAX_UPLOAD_RETRIES);
    Serial.printf("🌐 Target: %s\n", serverUrl);
    
    // Enhanced DNS check
    Serial.print("🔍 DNS resolution... ");
    IPAddress serverIP;
    String hostname = String(serverUrl);
    hostname.replace("http://", "");
    hostname.replace("https://", "");
    int slashIndex = hostname.indexOf('/');
    if (slashIndex > 0) {
      hostname = hostname.substring(0, slashIndex);
    }
    
    if (WiFi.hostByName(hostname.c_str(), serverIP)) {
      Serial.printf("✅ %s -> %s\n", hostname.c_str(), serverIP.toString().c_str());
    } else {
      Serial.println("❌ DNS resolution failed!");
      delay(2000);
      continue;
    }
    
    // Attempt upload
    uploadSuccess = uploadImageBuffer(fb->buf, fb->len);
    
    // Wait before retry with increasing delay
    if (!uploadSuccess && attempt < MAX_UPLOAD_RETRIES) {
      int retryDelay = 2000 * attempt; // 2s, 4s, etc.
      Serial.printf("⏳ Retry in %ds...\n", retryDelay/1000);
      delay(retryDelay);
      
      // Re-check WiFi before retry
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("🔄 WiFi lost, reconnecting...");
        WiFi.reconnect();
        delay(3000);
      }
    }
  }
  
  // Final status report
  if (uploadSuccess) {
    Serial.println("\n🎉 UPLOAD SUCCESSFUL!");
    if (mqttClient.connected()) {
      mqttClient.publish(mqtt_status_topic, "upload_ok");
    }
  } else {
    Serial.println("\n❌ ALL UPLOAD ATTEMPTS FAILED");
    if (mqttClient.connected()) {
      mqttClient.publish(mqtt_status_topic, "upload_failed");
    }
    Serial.println("🔧 System Status:");
    Serial.printf("  • WiFi: %d (%s)\n", WiFi.status(), WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    Serial.printf("  • Signal: %d dBm\n", WiFi.RSSI());
    Serial.printf("  • Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("  • Image Size: %d bytes\n", fb->len);
    Serial.printf("  • PSRAM: %s\n", psramFound() ? "Yes" : "No");
    
    // Suggest solutions based on error pattern
    Serial.println("\n💡 Suggested fixes:");
    Serial.println("  1. Check server URL and endpoint");
    Serial.println("  2. Improve WiFi signal strength");
    Serial.println("  3. Reduce image quality/size");
    Serial.println("  4. Check server logs for errors");
    Serial.println("  5. Verify server accepts POST requests");
  }
  
  // Always release buffer
  esp_camera_fb_return(fb);
  // Turn off flash after done
  digitalWrite(FLASH_GPIO_NUM, LOW);
  Serial.println("✅ Camera buffer released");
  
  // Force garbage collection
  if (ESP.getFreeHeap() < 100000) {
    Serial.println("🧹 Low memory, cleaning up...");
    delay(100);
  }

}
