#include <esp_now.h>
#include <WiFi.h>
//2c:bc:bb:92:64:b4
uint8_t receiverMAC[] = {0x2C, 0xBC, 0xBB, 0x92, 0x64, 0xB4};// Replace with receiver ESP32 MAC address
#define EYE_SENSOR_PIN 4  // GPIO for eye blink sensor
#define FAIL_LED 2  // GPIO for failure indication LED (change as needed)

esp_now_peer_info_t peerInfo;
volatile int blinkCount = 0;
volatile unsigned long lastBlinkTime = 0;
const unsigned long debounceDelay = 300;  // 300ms debounce delay

void IRAM_ATTR blinkDetected() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastBlinkTime > debounceDelay) { // Ignore rapid signals
        lastBlinkTime = currentTime;
        blinkCount++;
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send Status: ");
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Success");
        digitalWrite(FAIL_LED, LOW);  // Turn OFF LED on success
    } else {
        Serial.println("Failed");
        digitalWrite(FAIL_LED, HIGH); // Turn ON LED on failure
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    pinMode(FAIL_LED, OUTPUT);
    digitalWrite(FAIL_LED, LOW); // Keep LED OFF initially

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    pinMode(EYE_SENSOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(EYE_SENSOR_PIN), blinkDetected, FALLING);
}

void loop() {
    if (blinkCount > 0 && millis() - lastBlinkTime > 2000) { // Process blinks every 2 sec
        Serial.print("Sending Blink Count: ");
        Serial.println(blinkCount);

        esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&blinkCount, sizeof(blinkCount));
        
        blinkCount = 0;  // Reset after sending
        lastBlinkTime = millis();
    }
}
