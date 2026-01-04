#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h>

#define ENA 12
#define IN1 13
#define IN2 14
#define IN3 25
#define IN4 26
#define ENB 27

#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 19

#define BLUE_LED 4    // LED for Joystick mode
#define YELLOW_LED 2  // LED for Bluetooth mode

BluetoothSerial SerialBT;
Servo myServo;

typedef struct {
    int x;
    int y;
    int button;
} JoystickData;

JoystickData joystickData;
bool joystickActive = false;
bool objectDetected = false;
bool reversing = false;

void rotateMotor(int leftSpeed, int rightSpeed) {
    analogWrite(ENA, abs(rightSpeed));
    analogWrite(ENB, abs(leftSpeed));

    digitalWrite(IN1, rightSpeed > 0);
    digitalWrite(IN2, rightSpeed <= 0);
    digitalWrite(IN3, leftSpeed > 0);
    digitalWrite(IN4, leftSpeed <= 0);
}

void stopMotor() {
    rotateMotor(0, 0);
}

int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    int duration = pulseIn(ECHO_PIN, HIGH, 30000);
    int distance = duration * 0.034 / 2;
    return distance;
}

void checkAndAvoidObstacle() {
    int distance = getDistance();

    if (distance > 0 && distance <= 30) {   
        Serial.println("🚨 Obstacle detected! Reversing...");
        stopMotor();
        objectDetected = true;
        reversing = true;

        // Reverse in a straight line
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, 150);  // Adjust speed if needed
        analogWrite(ENB, 150);

        delay(1000); // Move backward for 1 second (~30 cm)

        stopMotor();
        Serial.println("✅ Reversed. Waiting for next command...");
        reversing = false;
        objectDetected = false;
    }
}

// ESP-NOW Data Receive Callback
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&joystickData, incomingData, sizeof(joystickData));
    joystickActive = true;

    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);

    int x = joystickData.x;
    int y = joystickData.y;
    int button = joystickData.button;

    int center = 2048;
    int threshold = 500;

    if (!reversing) {
        if (y > center + threshold) {
            if (!objectDetected) {
                rotateMotor(100, 100);
                Serial.println("🎮 Joystick: Moving Forward");
            }
        } else if (y < center - threshold) {
            rotateMotor(-100, -100);
            Serial.println("🎮 Joystick: Moving Backward");
        } else if (x > center + threshold) {
            rotateMotor(100, -100);
            Serial.println("🎮 Joystick: Turning Right");
        } else if (x < center - threshold) {
            rotateMotor(-100, 100);
            Serial.println("🎮 Joystick: Turning Left");
        } else {
            stopMotor();
            Serial.println("🎮 Joystick: Stopped");
        }
    }
}

void setup() {
    Serial.begin(115200);
    SerialBT.begin("Joystick(RC CAR)");
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);

    myServo.attach(SERVO_PIN);
    myServo.write(90);
}

void loop() {
    if (!joystickActive && SerialBT.available()) {  
        String command = SerialBT.readString();
        command.toLowerCase();
        command.trim();

        Serial.print("Received Command: ");
        Serial.println(command);

        digitalWrite(BLUE_LED, LOW);
        digitalWrite(YELLOW_LED, HIGH);

        if (command == "right") {
            rotateMotor(100, -100);
        } else if (command == "left") {
            rotateMotor(-100, 100);
        } else if (command == "backward") {
            rotateMotor(-100, -100);
        } else if (command == "forward") {
            if (!objectDetected) {
                rotateMotor(100, 100);
            }
        } else if (command == "stop") {
            stopMotor();
        }
    }

    checkAndAvoidObstacle();  

    joystickActive = false;  
}
