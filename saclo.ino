#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <MPU6050_tockn.h>
#include <JQ6500_Serial.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <WiFi.h>

// WiFi configuration
const char* ssid = "677 5G";
const char* password = "10101010";

// ThingSpeak configuration
#define THINGSPEAK_SERVER "api.thingspeak.com"
#define THINGSPEAK_PORT 80
#define THINGSPEAK_CHANNEL_ID "2860387"
#define THINGSPEAK_WRITE_API_KEY "N8JMGBB3FI2IPD7B"
#define THINGSPEAK_UPDATE_INTERVAL 15000 

#define MQTT_SERVER "485dac0774f84a84af21f6b392c46069.s1.eu.hivemq.cloud"
#define MQTT_PORT 8883
#define MQTT_CLIENT_ID "ESP32_Sensor_Client"
#define MQTT_TOPIC_SENSORS "sensors/data"
#define MQTT_TOPIC_WARNINGS "sensors/warnings"
#define MQTT_TOPIC_MOISTURE_THRESHOLD "sensors/moisture_threshold"
#define MQTT_TOPIC_VIBRATION_THRESHOLD "sensors/vibration_threshold"
#define MQTT_TOPIC_TILT_THRESHOLD "sensors/tilt_threshold"
#define MQTT_TOPIC_RAIN_THRESHOLD "sensors/rain_threshold"
const char* mqtt_user = "tu_mqtt";
const char* mqtt_password = "#30032003Tu";

#define SOIL_MOISTURE_PIN 34
#define RAIN_SENSOR_PIN 36  // Chân analog cho cảm biến mưa
const int DRY_VALUE = 4095;
const int WET_VALUE = 1500;
const int RAIN_DRY_VALUE = 4095;   // Giá trị khi không mưa
const int RAIN_WET_VALUE = 1500;   // Giá trị khi mưa nhiều
const int SAMPLE_DELAY = 500;  
#define EEPROM_SIZE 16  // Tăng kích thước EEPROM để lưu thêm ngưỡng mưa
#define EEPROM_ADDR_MOISTURE 0
#define EEPROM_ADDR_VIBRATION 4
#define EEPROM_ADDR_TILT 8
#define EEPROM_ADDR_RAIN 12  // Địa chỉ EEPROM cho ngưỡng mưa
#define INITIAL_SKIP_COUNT 5

// Object declarations
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
MPU6050 mpu6050(Wire);
HardwareSerial mp3Serial(1);
JQ6500_Serial mp3(mp3Serial);
LiquidCrystal_I2C lcd(0x27, 20, 4);
WiFiClientSecure espClient;
WiFiClient wifiClient;
PubSubClient mqttClient(espClient);

// State variables
float prevX = 0, prevY = 0, prevZ = 0;
float prevAngleX = 0.0, prevAngleY = 0.0;
int rawSoilValue = 0;
int moisturePercent = 0;
int rawRainValue = 0;
int rainPercent = 0;
float moistureThreshold = 50.0;
float vibrationThreshold = 7.0;
float tiltThreshold = 15.0;
float rainThreshold = 30.0;  // Ngưỡng mưa (%)
char warningMessage[256] = "";
unsigned long lastScrollTime = 0;
unsigned long lastThingSpeakUpdate = 0;
int scrollIndex = 0;
bool isScrolling = false;
int readingCount = 0;
unsigned long lastVibrationWarning = 0;
unsigned long lastTiltWarning = 0;
unsigned long lastMoistureWarning = 0;
unsigned long lastRainWarning = 0;
int currentWarningCode = 0;
String currentWarningText = "";
bool hasNewWarning = false;

// Biến cho tính toán thời gian mưa
bool isRaining = false;

// Save threshold to EEPROM
void saveThresholdToEEPROM(float threshold, int address) {
    EEPROM.put(address, threshold);
    EEPROM.commit();
    Serial.print(F("Lưu ngưỡng vào EEPROM tại địa chỉ "));
    Serial.print(address);
    Serial.print(F(": "));
    Serial.println(threshold);
}

// Load threshold from EEPROM
float loadThresholdFromEEPROM(int address, float defaultValue) {
    float threshold;
    EEPROM.get(address, threshold);
    if (isnan(threshold) || threshold < 0) {
        threshold = defaultValue;
        saveThresholdToEEPROM(threshold, address);
    }
    Serial.print(F("Tải ngưỡng từ EEPROM tại địa chỉ "));
    Serial.print(address);
    Serial.print(F(": "));
    Serial.println(threshold);
    return threshold;
}

// MQTT callback
void callback(char* topic, byte* payload, unsigned int length) {
    char message[16];
    length = min(length, sizeof(message) - 1);
    strncpy(message, (char*)payload, length);
    message[length] = '\0';
    float newThreshold = atof(message);

    if (strcmp(topic, MQTT_TOPIC_MOISTURE_THRESHOLD) == 0) {
        if (newThreshold >= 0 && newThreshold <= 100) {
            moistureThreshold = newThreshold;
            saveThresholdToEEPROM(moistureThreshold, EEPROM_ADDR_MOISTURE);
            Serial.print(F("Cập nhật ngưỡng độ ẩm từ MQTT: "));
            Serial.println(moistureThreshold);
        }
    } else if (strcmp(topic, MQTT_TOPIC_VIBRATION_THRESHOLD) == 0) {
        if (newThreshold >= 0 && newThreshold <= 20) {
            vibrationThreshold = newThreshold;
            saveThresholdToEEPROM(vibrationThreshold, EEPROM_ADDR_VIBRATION);
            Serial.print(F("Cập nhật ngưỡng rung động từ MQTT: "));
            Serial.println(vibrationThreshold);
        }
    } else if (strcmp(topic, MQTT_TOPIC_TILT_THRESHOLD) == 0) {
        if (newThreshold >= 0 && newThreshold <= 90) {
            tiltThreshold = newThreshold;
            saveThresholdToEEPROM(tiltThreshold, EEPROM_ADDR_TILT);
            Serial.print(F("Cập nhật ngưỡng góc nghiêng từ MQTT: "));
            Serial.println(tiltThreshold);
        }
    } else if (strcmp(topic, MQTT_TOPIC_RAIN_THRESHOLD) == 0) {
        if (newThreshold >= 0 && newThreshold <= 100) {
            rainThreshold = newThreshold;
            saveThresholdToEEPROM(rainThreshold, EEPROM_ADDR_RAIN);
            Serial.print(F("Cập nhật ngưỡng mưa từ MQTT: "));
            Serial.println(rainThreshold);
        }
    }
}

// Connect to MQTT
void connectMQTT() {
    if (!mqttClient.connected()) {
        Serial.print(F("Đang kết nối với MQTT..."));
        if (mqttClient.connect(MQTT_CLIENT_ID, mqtt_user, mqtt_password)) {
            Serial.println(F("Đã kết nối"));
            mqttClient.subscribe(MQTT_TOPIC_MOISTURE_THRESHOLD);
            mqttClient.subscribe(MQTT_TOPIC_VIBRATION_THRESHOLD);
            mqttClient.subscribe(MQTT_TOPIC_TILT_THRESHOLD);
            mqttClient.subscribe(MQTT_TOPIC_RAIN_THRESHOLD);
        } else {
            Serial.print(F("Thất bại, rc="));
            Serial.println(mqttClient.state());
        }
    }
}

// Publish sensor data via MQTT
void publishSensorData(float x, float y, float z, float angleX, float angleY, int moisture, int rain) {
    if (!mqttClient.connected()) return;

    char json[256];
    snprintf(json, sizeof(json), "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"angleX\":%.2f,\"angleY\":%.2f,\"moisture\":%d,\"rain\":%d}",
             x, y, z, angleX, angleY, moisture, rain);
    mqttClient.publish(MQTT_TOPIC_SENSORS, json);
}

// Publish warning via MQTT
void publishWarning(const char* warning) {
    if (!mqttClient.connected()) return;
    mqttClient.publish(MQTT_TOPIC_WARNINGS, warning);
}

// Publish sensor data and warnings to ThingSpeak
void publishToThingSpeak(float x, float y, float z, float angleX, float angleY, int moisture, int warningCode, int rain) {
    if (WiFi.status() != WL_CONNECTED) return;

    WiFiClient client;
    if (!client.connect(THINGSPEAK_SERVER, THINGSPEAK_PORT)) {
        Serial.println(F("Kết nối ThingSpeak thất bại"));
        return;
    }

    String url = "/update?api_key=";
    url += THINGSPEAK_WRITE_API_KEY;
    url += "&field1=" + String(x, 2);
    url += "&field2=" + String(y, 2);
    url += "&field3=" + String(z, 2);
    url += "&field4=" + String(angleX, 2);
    url += "&field5=" + String(angleY, 2);
    url += "&field6=" + String(moisture);
    url += "&field7=" + String(warningCode);
    url += "&field8=" + String(rain);  // Thêm dữ liệu mưa

    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + THINGSPEAK_SERVER + "\r\n" +
                 "Connection: close\r\n\r\n");

    client.stop();
    Serial.println(F("Đã gửi dữ liệu đến ThingSpeak"));
}

// Chuyển đổi milliseconds sang chuỗi thời gian
// Note: rain time tracking removed as requested.

// Display data on LCD
void displayLCD(float x, float y, float z, float angleX, float angleY, int moisture, int rain, char* warning) {
    // Update the LCD with parameters only (no title). LCD is 20x4.
    lcd.clear();

    // Line 0: Moisture and Rain percentages
    String line0 = "Doam:" + String(moisture) + "% ";
    line0 += "Mua:" + String(rain) + "%";
    while (line0.length() < 20) line0 += ' ';
    lcd.setCursor(0, 0);
    lcd.print(line0);

    // Line 1: ADXL345 readings (X,Y)
    String line1 = "ADXL X:" + String(x, 2) + " Y:" + String(y, 2);
    while (line1.length() < 20) line1 += ' ';
    lcd.setCursor(0, 1);
    lcd.print(line1);

    // Line 2: MPU angles
    String line2 = "MPU: X " + String(angleX, 1) + ",Y " + String(angleY, 1);
    while (line2.length() < 20) line2 += ' ';
    lcd.setCursor(0, 2);
    lcd.print(line2);

    // Line 3: warning (short or scrolling)
    String warnStr = String(warning);
    if (warnStr.length() == 0) {
        // Clear line 3
        lcd.setCursor(0, 3);
        String blank = "";
        while (blank.length() < 20) blank += ' ';
        lcd.print(blank);
        isScrolling = false;
    } else if (warnStr.length() <= 20) {
        String line = warnStr;
        while (line.length() < 20) line += ' ';
        lcd.setCursor(0, 3);
        lcd.print(line);
        isScrolling = false;
    } else {
        isScrolling = true;
        scrollIndex = 0;
        char buf[21];
        strncpy(buf, warning, 20);
        buf[20] = '\0';
        lcd.setCursor(0, 3);
        lcd.print(buf);
    }
}

// Scroll warning message on LCD
void scrollWarning(char* warning) {
    if (!isScrolling) return;
    size_t len = strlen(warning);
    if (len <= 20) {
        // Nothing to scroll
        isScrolling = false;
        return;
    }

    if (millis() - lastScrollTime >= 300) {
        char buf[21];
        size_t avail = len - scrollIndex;
        if (avail >= 20) {
            // Enough chars remaining
            strncpy(buf, warning + scrollIndex, 20);
        } else {
            // Fewer than 20 chars remain: copy remainder and pad with spaces
            strncpy(buf, warning + scrollIndex, avail);
            for (size_t i = avail; i < 20; ++i) buf[i] = ' ';
        }
        buf[20] = '\0';

        lcd.setCursor(0, 3);
        lcd.print(buf);

        scrollIndex++;
        if (scrollIndex > len - 20) {
            scrollIndex = 0;
        }
        lastScrollTime = millis();
    }
}

// Check vibration
void checkVibration(float deltaX, float deltaY, float deltaZ, float x, float y, float z, float angleX, float angleY, int moisture, int rain) {
    if (millis() - lastVibrationWarning < 1000) return;
    
    if (deltaX > vibrationThreshold || deltaY > vibrationThreshold || deltaZ > vibrationThreshold) {
        snprintf(warningMessage, sizeof(warningMessage), "Rung dong manh (>%.1fm/s^2)!", vibrationThreshold);
        
        Serial.print(F("CẢNH BÁO: Phát hiện rung động mạnh (>"));
        Serial.print(vibrationThreshold);
        Serial.println(F("m/s^2)!"));
        
        publishWarning("Phát hiện rung động mạnh!");
        currentWarningCode = 1;
        hasNewWarning = true;
    publishToThingSpeak(x, y, z, angleX, angleY, moisture, currentWarningCode, rain);
        lastThingSpeakUpdate = millis();
        
        if (mp3.getStatus() != MP3_STATUS_PLAYING) {
            mp3.playFileByIndexNumber(3);
        }
        lastVibrationWarning = millis();
    }
}

// Check tilt
void checkTilt(float angleX, float angleY, float x, float y, float z, int moisture, int rain) {
    if (millis() - lastTiltWarning < 1000) return;
    
    float angleXChange = abs(angleX - prevAngleX);
    float angleYChange = abs(angleY - prevAngleY);
    
    if (angleXChange > tiltThreshold || angleYChange > tiltThreshold) {
        snprintf(warningMessage, sizeof(warningMessage), "Thay doi goc nghieng (>%.1f°)!", tiltThreshold);
        
        Serial.print(F("CẢNH BÁO: Thay đổi góc nghiêng đột ngột (>"));
        Serial.print(tiltThreshold);
        Serial.println(F("°)!"));
        
        publishWarning("Thay đổi góc nghiêng đột ngột!");
        currentWarningCode = 2;
        hasNewWarning = true;
    publishToThingSpeak(x, y, z, angleX, angleY, moisture, currentWarningCode, rain);
        lastThingSpeakUpdate = millis();
        
        if (mp3.getStatus() != MP3_STATUS_PLAYING) {
            mp3.playFileByIndexNumber(2);
        }
        lastTiltWarning = millis();
    }
}

// Check moisture
void checkMoisture(int moisturePercent, float x, float y, float z, float angleX, float angleY, int rain) {
    if (millis() - lastMoistureWarning < 1000) return;
    
    if (moisturePercent > moistureThreshold) {
        snprintf(warningMessage, sizeof(warningMessage), "Dat qua am (>%d%%)!", (int)moistureThreshold);
        
        Serial.print(F("CẢNH BÁO: Đất quá ẩm (>"));
        Serial.print(moistureThreshold);
        Serial.println(F("%)!"));
        
        publishWarning("Đất quá ẩm!");
        currentWarningCode = 3;
        hasNewWarning = true;
    publishToThingSpeak(x, y, z, angleX, angleY, moisturePercent, currentWarningCode, rain);
        lastThingSpeakUpdate = millis();
        
        if (mp3.getStatus() != MP3_STATUS_PLAYING) {
            mp3.playFileByIndexNumber(1);
        }
        lastMoistureWarning = millis();
    }
}

// Check rain
void checkRain(int rainPercent, float x, float y, float z, float angleX, float angleY, int moisture) {
    if (millis() - lastRainWarning < 1000) return;
    
    if (rainPercent > rainThreshold) {
        snprintf(warningMessage, sizeof(warningMessage), "Tranh mua (>%d%%)!", (int)rainThreshold);
        
        Serial.print(F("CẢNH BÁO: Trời mưa (>"));
        Serial.print(rainThreshold);
        Serial.println(F("%)!"));
        
        publishWarning("Trời mưa - Cảnh báo sạt lở!");
        currentWarningCode = 4;
        hasNewWarning = true;
    publishToThingSpeak(x, y, z, angleX, angleY, moisture, currentWarningCode, rainPercent);
        lastThingSpeakUpdate = millis();
        
        if (mp3.getStatus() != MP3_STATUS_PLAYING) {
            mp3.playFileByIndexNumber(4);  // Phát cảnh báo mưa
        }
        lastRainWarning = millis();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Hệ thống giám sát sạt lở đất - Tối ưu hóa"));
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    pinMode(RAIN_SENSOR_PIN, INPUT);

    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    moistureThreshold = loadThresholdFromEEPROM(EEPROM_ADDR_MOISTURE, 50.0);
    vibrationThreshold = loadThresholdFromEEPROM(EEPROM_ADDR_VIBRATION, 7.0);
    tiltThreshold = loadThresholdFromEEPROM(EEPROM_ADDR_TILT, 15.0);
    rainThreshold = loadThresholdFromEEPROM(EEPROM_ADDR_RAIN, 30.0);

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("He thong giam sat");
    lcd.setCursor(0, 1);
    lcd.print("Khoi dong...");
    delay(1000);
    lcd.clear();

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // Tăng tốc độ I2C lên 400kHz

    // Initialize ADXL345
    if (!accel.begin()) {
        Serial.println(F("Không tìm thấy ADXL345!"));
        while (1) delay(1000);
    }
    accel.setRange(ADXL345_RANGE_16_G);
    Serial.println(F("Đã khởi tạo ADXL345"));

    // Initialize MPU6050
    mpu6050.begin();
    Serial.println(F("Đang hiệu chỉnh MPU6050..."));
    mpu6050.calcGyroOffsets(true);
    Serial.println(F("Đã khởi tạo MPU6050"));

    // Initialize JQ6500 MP3
    mp3Serial.begin(9600, SERIAL_8N1, 25, 26);
    mp3.reset();
    delay(500);
    mp3.setVolume(30);
    mp3.setLoopMode(MP3_LOOP_NONE);
    Serial.println(F("Đã khởi tạo JQ6500"));

    // Initialize WiFi
    WiFi.begin(ssid, password);
    Serial.print(F("Đang kết nối WiFi..."));
    unsigned long wifiTimeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 10000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("\nĐã kết nối WiFi"));
        Serial.print(F("IP: "));
        Serial.println(WiFi.localIP());
        
        // Initialize MQTT
        espClient.setInsecure();
        mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
        mqttClient.setCallback(callback);
        connectMQTT();
    } else {
        Serial.println(F("\nKhông kết nối được WiFi - Tiếp tục chạy offline"));
    }

    // Initialize LED
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    
    Serial.println(F("Hệ thống sẵn sàng!"));
}

void loop() {
    // MQTT loop - không chặn nếu không kết nối
    if (WiFi.status() == WL_CONNECTED) {
        if (!mqttClient.connected()) {
            connectMQTT();
        }
        mqttClient.loop();
    }

    hasNewWarning = false;

    // Read ADXL345
    sensors_event_t event;
    if (accel.getEvent(&event)) {
        float currentX = event.acceleration.x;
        float currentY = event.acceleration.y;
        float currentZ = event.acceleration.z;
        float deltaX = abs(currentX - prevX);
        float deltaY = abs(currentY - prevY);
        float deltaZ = abs(currentZ - prevZ);
        
        if (readingCount >= INITIAL_SKIP_COUNT) {
            checkVibration(deltaX, deltaY, deltaZ, currentX, currentY, currentZ, prevAngleX, prevAngleY, moisturePercent, rainPercent);
        }
        prevX = currentX;
        prevY = currentY;
        prevZ = currentZ;
    }

    // Read MPU6050
    mpu6050.update();
    float angleX = mpu6050.getAngleX();
    float angleY = mpu6050.getAngleY();
    
    if (abs(angleX) > 180 || abs(angleY) > 180) {
        angleX = prevAngleX;
        angleY = prevAngleY;
    }
    
    if (readingCount >= INITIAL_SKIP_COUNT) {
        checkTilt(angleX, angleY, prevX, prevY, prevZ, moisturePercent, rainPercent);
    }
    prevAngleX = angleX;
    prevAngleY = angleY;

    // Read soil moisture
    rawSoilValue = analogRead(SOIL_MOISTURE_PIN);
    moisturePercent = constrain(map(rawSoilValue, DRY_VALUE, WET_VALUE, 1023, 0), 0, 100);
    
    // Read rain sensor
    rawRainValue = analogRead(RAIN_SENSOR_PIN);
    rainPercent = constrain(map(rawRainValue, RAIN_DRY_VALUE, RAIN_WET_VALUE, 0, 1023), 0, 100);
    
    // Rain time tracking removed
    
    if (readingCount >= INITIAL_SKIP_COUNT) {
        checkMoisture(moisturePercent, prevX, prevY, prevZ, prevAngleX, prevAngleY, rainPercent);
        checkRain(rainPercent, prevX, prevY, prevZ, prevAngleX, prevAngleY, moisturePercent);
    }

    // Update LCD
    displayLCD(prevX, prevY, prevZ, prevAngleX, prevAngleY, moisturePercent, rainPercent, warningMessage);
    scrollWarning(warningMessage);

    // Publish sensor data via MQTT (nhanh, không chặn)
    if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
        publishSensorData(prevX, prevY, prevZ, prevAngleX, prevAngleY, moisturePercent, rainPercent);
    }

    // Publish to ThingSpeak periodically
    if (!hasNewWarning && millis() - lastThingSpeakUpdate >= THINGSPEAK_UPDATE_INTERVAL) {
        if (WiFi.status() == WL_CONNECTED) {
            publishToThingSpeak(prevX, prevY, prevZ, prevAngleX, prevAngleY, moisturePercent, currentWarningCode, rainPercent);
        }
        lastThingSpeakUpdate = millis();
        warningMessage[0] = '\0';
        currentWarningCode = 0;
        currentWarningText = "";
    }

    readingCount++;
    delay(SAMPLE_DELAY);
}