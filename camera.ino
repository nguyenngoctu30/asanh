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
const char* ssid = "677 5G";
const char* password = "10101010";

// MQTT Broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32cam/capture";
const char* mqtt_client_id = "ESP32CAM_Client";
const char* mqtt_status_topic = "esp32cam/status";

// Server URL - Using HTTP
const char* serverUrl = "http://having-medications-king-concerts.trycloudflare.com";

// Upload settings
const int MAX_UPLOAD_RETRIES = 3;
const int UPLOAD_TIMEOUT = 60000;  // 60 seconds
const int CONNECT_TIMEOUT = 30000; // 30 seconds

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
#define FLASH_GPIO_NUM     4

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n\n🚀 ESP32-CAM Starting...");
  
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  setupWiFi();
  setupMQTT();
  setupCamera();
  
  Serial.println("✅ System ready!");
  Serial.println("==================================\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  if (millis() - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    checkWiFiHealth();
    lastWiFiCheck = millis();
  }
  
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

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

  // Giảm kích thước ảnh để upload dễ dàng hơn
  if (psramFound()) {
    Serial.println("✅ PSRAM detected");
    config.frame_size = FRAMESIZE_VGA;   // 640x480 (nhỏ hơn)
    config.jpeg_quality = 15;            // Chất lượng vừa phải
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("⚠️ No PSRAM, using internal RAM");
    config.frame_size = FRAMESIZE_QVGA;  // 320x240 (rất nhỏ)
    config.jpeg_quality = 18;
    config.fb_count = 1;
  }

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

// ==================== UPLOAD FUNCTIONS ====================
// Phương pháp 1: Raw Binary Upload với optimizations
bool uploadRawBinary(uint8_t* buffer, size_t length) {
  HTTPClient http;
  
  // Set timeouts
  http.setTimeout(60000);  // 60 seconds
  http.setConnectTimeout(30000);  // 30 seconds
  
  // CRITICAL: Keep connection alive and reuse
  http.setReuse(true);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
  Serial.printf("📤 Uploading RAW: %d bytes\n", length);
  
  if (!http.begin(wifiClient, serverUrl)) {
    Serial.println("❌ HTTP begin failed");
    return false;
  }
  
  // Minimal headers for ngrok compatibility
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("Content-Length", String(length));
  http.addHeader("User-Agent", "ESP32-CAM/1.0");
  http.addHeader("Accept", "*/*");
  
  Serial.println("📡 Sending POST request...");
  
  // Send request
  int httpCode = http.POST(buffer, length);
  
  Serial.printf("📬 HTTP Response Code: %d\n", httpCode);
  
  if (httpCode > 0) {
    String response = http.getString();
    Serial.printf("Response: %s\n", response.c_str());
    
    http.end();
    return (httpCode == HTTP_CODE_OK || httpCode == 200);
  } else {
    Serial.printf("❌ HTTP Error: %d (%s)\n", httpCode, http.errorToString(httpCode).c_str());
    
    // Detailed error info
    switch(httpCode) {
      case HTTPC_ERROR_CONNECTION_REFUSED:
        Serial.println("   → Connection refused by server");
        break;
      case HTTPC_ERROR_SEND_HEADER_FAILED:
        Serial.println("   → Failed to send headers");
        break;
      case HTTPC_ERROR_SEND_PAYLOAD_FAILED:
        Serial.println("   → Failed to send payload");
        Serial.println("   → This often means: connection closed during upload");
        break;
      case HTTPC_ERROR_NOT_CONNECTED:
        Serial.println("   → Not connected");
        break;
      case HTTPC_ERROR_CONNECTION_LOST:
        Serial.println("   → Connection lost during transfer");
        break;
      case HTTPC_ERROR_READ_TIMEOUT:
        Serial.println("   → Read timeout");
        break;
    }
    
    http.end();
  }
  
  return false;
}

// Phương pháp 2: Chunked Upload (cho ảnh lớn)
bool uploadChunked(uint8_t* buffer, size_t length) {
  const size_t CHUNK_SIZE = 4096;  // 4KB chunks
  
  HTTPClient http;
  http.setTimeout(UPLOAD_TIMEOUT);
  http.setConnectTimeout(CONNECT_TIMEOUT);
  http.setReuse(false);
  
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("Transfer-Encoding", "chunked");
  
  Serial.printf("📤 Uploading CHUNKED: %d bytes\n", length);
  
  if (!http.begin(wifiClient, serverUrl)) {
    Serial.println("❌ HTTP begin failed");
    return false;
  }
  
  // Gửi từng chunk
  WiFiClient* stream = http.getStreamPtr();
  
  if (!stream->connect(http.getStreamPtr()->remoteIP(), 80)) {
    Serial.println("❌ Stream connection failed");
    http.end();
    return false;
  }
  
  // Send HTTP headers manually
  stream->println("POST /upload HTTP/1.1");
  stream->printf("Host: %s\r\n", "having-medications-king-concerts.trycloudflare.com");
  stream->println("Content-Type: image/jpeg");
  stream->println("Connection: close");
  stream->printf("Content-Length: %d\r\n", length);
  stream->println();
  
  // Send body in chunks
  size_t sent = 0;
  Serial.print("Progress: ");
  
  while (sent < length) {
    size_t toSend = min(CHUNK_SIZE, length - sent);
    size_t written = stream->write(buffer + sent, toSend);
    
    if (written != toSend) {
      Serial.println("\n❌ Write failed");
      http.end();
      return false;
    }
    
    sent += written;
    int progress = (sent * 100) / length;
    Serial.printf("%d%% ", progress);
    
    delay(10);  // Small delay between chunks
    
    // Check connection
    if (!stream->connected()) {
      Serial.println("\n❌ Connection lost during upload");
      http.end();
      return false;
    }
  }
  
  Serial.println("100% ✓");
  
  // Wait for response
  unsigned long timeout = millis();
  while (stream->available() == 0) {
    if (millis() - timeout > 10000) {
      Serial.println("⏱️ Response timeout");
      http.end();
      return false;
    }
    delay(10);
  }
  
  // Read response
  String response = "";
  while (stream->available()) {
    char c = stream->read();
    response += c;
  }
  
  Serial.printf("📬 Response:\n%s\n", response.c_str());
  
  http.end();
  
  // Check if response contains success
  return (response.indexOf("200 OK") > 0 || response.indexOf("success") > 0);
}

// Phương pháp 3: TCP Direct Upload (bypass HTTPClient)
bool uploadDirect(uint8_t* buffer, size_t length) {
  WiFiClient client;
  
  // Parse hostname from URL
  String hostname = "having-medications-king-concerts.trycloudflare.com";
  int port = 80;
  String path = "/upload";
  
  Serial.printf("📤 Connecting to %s:%d\n", hostname.c_str(), port);
  
  // Set timeouts BEFORE connecting
  client.setTimeout(60000);  // 60 second timeout
  
  if (!client.connect(hostname.c_str(), port)) {
    Serial.println("❌ Connection failed");
    return false;
  }
  
  Serial.println("✅ Connected, sending request...");
  
  // Build HTTP request manually
  String headers = "";
  headers += "POST " + String(path) + " HTTP/1.1\r\n";
  headers += "Host: " + String(hostname) + "\r\n";
  headers += "Content-Type: image/jpeg\r\n";
  headers += "Content-Length: " + String(length) + "\r\n";
  headers += "Connection: close\r\n";
  headers += "\r\n";
  
  // Send headers first
  Serial.println("📤 Sending headers...");
  if (client.print(headers) == 0) {
    Serial.println("❌ Failed to send headers");
    client.stop();
    return false;
  }
  
  // Wait a bit for headers to be sent
  delay(100);
  
  // Check if still connected
  if (!client.connected()) {
    Serial.println("❌ Connection lost after headers");
    client.stop();
    return false;
  }
  
  // Send image data in VERY small chunks with delays
  Serial.printf("📤 Sending %d bytes in chunks...\n", length);
  
  size_t sent = 0;
  const size_t CHUNK_SIZE = 512;  // Smaller chunks (512 bytes)
  unsigned long sendStart = millis();
  
  while (sent < length) {
    // Check connection before each chunk
    if (!client.connected()) {
      Serial.printf("❌ Connection lost at %d bytes\n", sent);
      client.stop();
      return false;
    }
    
    size_t toSend = min(CHUNK_SIZE, length - sent);
    
    // Send chunk
    size_t written = client.write(buffer + sent, toSend);
    
    if (written == 0) {
      Serial.printf("❌ Write failed at %d bytes\n", sent);
      Serial.printf("   Connection status: %d\n", client.connected());
      Serial.printf("   Available for write: %d\n", client.availableForWrite());
      client.stop();
      return false;
    }
    
    sent += written;
    
    // Flush every chunk
    client.flush();
    
    // Progress report every 5KB
    if (sent % 5120 == 0 || sent == length) {
      Serial.printf("  📊 Sent: %d/%d bytes (%d%%)\n", sent, length, (sent * 100) / length);
    }
    
    // Important: longer delay between chunks for ngrok
    delay(20);  // 20ms delay between chunks
  }
  
  unsigned long sendTime = millis() - sendStart;
  Serial.printf("✅ All data sent in %lu ms\n", sendTime);
  
  // Final flush
  client.flush();
  delay(100);
  
  // Wait for response
  Serial.println("⏳ Waiting for server response...");
  unsigned long timeout = millis();
  while (!client.available() && millis() - timeout < 20000) {
    if (!client.connected()) {
      Serial.println("❌ Connection closed by server (no response)");
      client.stop();
      return false;
    }
    delay(50);
  }
  
  if (!client.available()) {
    Serial.println("⏱️ Response timeout");
    client.stop();
    return false;
  }
  
  // Read response
  Serial.println("📬 Server response:");
  String response = "";
  
  // Read status line
  if (client.available()) {
    String statusLine = client.readStringUntil('\n');
    Serial.println(statusLine);
    response += statusLine;
  }
  
  // Read headers
  while (client.available()) {
    String line = client.readStringUntil('\n');
    Serial.println(line);
    response += line + "\n";
    
    if (line.length() <= 1) break;  // Empty line = end of headers
  }
  
  // Read body
  delay(50);  // Wait for body
  if (client.available()) {
    String body = client.readString();
    Serial.printf("Body: %s\n", body.c_str());
    response += body;
  }
  
  client.stop();
  
  // Check success
  bool success = (response.indexOf("200 OK") >= 0 || 
                  response.indexOf("\"success\":true") >= 0 ||
                  response.indexOf("successfully") >= 0);
  
  if (success) {
    Serial.println("✅ Upload confirmed successful!");
  } else {
    Serial.println("⚠️ Upload may have failed - check response above");
  }
  
  return success;
}

// ==================== CAPTURE & UPLOAD ====================
void captureAndUploadPhoto() {
  // Turn on flash
  digitalWrite(FLASH_GPIO_NUM, HIGH);
  delay(120);
  
  // Clear previous frame
  camera_fb_t * fb_temp = esp_camera_fb_get();
  if (fb_temp) {
    esp_camera_fb_return(fb_temp);
    delay(200);
  }
  
  // Capture photo
  Serial.println("📷 Capturing image...");
  unsigned long t_start = millis();
  
  camera_fb_t * fb = esp_camera_fb_get();
  unsigned long t_capture = millis();
  
  if (!fb) {
    Serial.println("❌ Camera capture failed!");
    digitalWrite(FLASH_GPIO_NUM, LOW);
    return;
  }
  
  Serial.printf("✅ Captured: %d bytes in %lu ms\n", fb->len, (t_capture - t_start));
  Serial.printf("📊 Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("📶 WiFi RSSI: %d dBm\n", WiFi.RSSI());

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected");
    esp_camera_fb_return(fb);
    digitalWrite(FLASH_GPIO_NUM, LOW);
    return;
  }

  // Try upload methods in order
  bool uploadSuccess = false;
  
  for (int attempt = 1; attempt <= MAX_UPLOAD_RETRIES && !uploadSuccess; attempt++) {
    Serial.printf("\n🔄 Upload attempt %d/%d\n", attempt, MAX_UPLOAD_RETRIES);
    
    // Try Method 1 first (HTTPClient - simplest)
    Serial.println("Method: HTTPClient POST");
    uploadSuccess = uploadRawBinary(fb->buf, fb->len);
    
    // If failed, try Direct TCP as backup
    if (!uploadSuccess) {
      Serial.println("\nTrying backup method: Direct TCP");
      uploadSuccess = uploadDirect(fb->buf, fb->len);
    }
    
    if (!uploadSuccess && attempt < MAX_UPLOAD_RETRIES) {
      int retryDelay = 2000 * attempt;
      Serial.printf("⏳ Retry in %ds...\n", retryDelay/1000);
      delay(retryDelay);
      
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("🔄 WiFi lost, reconnecting...");
        WiFi.reconnect();
        delay(3000);
      }
    }
  }
  
  // Final status
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
  }
  
  esp_camera_fb_return(fb);
  digitalWrite(FLASH_GPIO_NUM, LOW);
  Serial.println("✅ Camera buffer released\n");
}
