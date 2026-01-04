#include <WiFi.h>
#include <esp_now.h>

const int buzzerPin = 9;  // Buzzer connected to pin 9

typedef struct struct_message {
    bool fallDetected;
} struct_message;

struct_message data;

void onReceive(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
    memcpy(&data, incomingData, sizeof(data));
    if (data.fallDetected) {
        Serial.println("Fall Detected! Activating Buzzer...");
        digitalWrite(buzzerPin, HIGH);
        delay(5000);
        digitalWrite(buzzerPin, LOW);
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    esp_now_register_recv_cb(onReceive);
}

void loop() {
    // No need for anything here, ESP-NOW handles incoming messages
}
