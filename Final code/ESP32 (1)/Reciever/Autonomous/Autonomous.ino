#include <esp_now.h>         // ESP-NOW library for wireless communication
#include <WiFi.h>            // WiFi library (needed for ESP-NOW)
#include <ESP32Servo.h>      // Servo motor library
#include <BluetoothSerial.h> // Bluetooth communication library

// Motor driver pins
#define ENA 12  // Speed control for right motor
#define IN1 13  // Right motor forward
#define IN2 14  // Right motor backward
#define IN3 25  // Left motor forward
#define IN4 26  // Left motor backward
#define ENB 27  // Speed control for left motor

// Ultrasonic sensor pins
#define TRIG_PIN 5   // Ultrasonic sensor trigger pin
#define ECHO_PIN 18  // Ultrasonic sensor echo pin

// Servo motor pin
#define SERVO_PIN 19 // Servo motor to scan for obstacles

// LED indicator pins
#define RED_LED_PIN 33   // Red LED for Bluetooth control   
#define GREEN_LED_PIN 32 // Green LED for Blink control

// Bluetooth object
BluetoothSerial SerialBT;   

// Servo object for scanning obstacles
Servo myServo;   

// Variables to store sensor and control data
int blinkCount = 0;          // Stores the blink count received from ESP-NOW
bool motorStarted = false;   // Flag to track if the motor is running
bool obstacleDetected = false; // Flag to disable Bluetooth control when an object is detected

// Function to rotate motors at specific speeds
void rotateMotor(int leftSpeed, int rightSpeed) {
    analogWrite(ENA, abs(rightSpeed)); // Set speed for right motor
    analogWrite(ENB, abs(leftSpeed));  // Set speed for left motor

    // Control right motor direction
    digitalWrite(IN1, rightSpeed > 0);
    digitalWrite(IN2, rightSpeed <= 0);

    // Control left motor direction
    digitalWrite(IN3, leftSpeed > 0);
    digitalWrite(IN4, leftSpeed <= 0);
}

// Function to stop the motors
void stopMotor() {
    rotateMotor(0, 0);
}

// Function to measure distance using the ultrasonic sensor
int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    int duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration * 0.034 / 2; // Convert time to distance (cm)
    return distance;
}

// Function to check for obstacles and avoid them
void checkAndAvoidObstacle() {
    int distance = getDistance();

    if (distance > 0 && distance <= 30) { // If an obstacle is within 30 cm
        Serial.println("Obstacle Detected! Stopping and Scanning...");
        stopMotor();
        obstacleDetected = true; // Disable Bluetooth control

        // Scan the right side
        myServo.write(0);
        delay(500);
        int rightDistance = getDistance();

        // Scan the left side
        myServo.write(180);
        delay(500);
        int leftDistance = getDistance();

        // Reset servo to center
        myServo.write(90);
        delay(500);

        // Choose direction with more space
        if (rightDistance > leftDistance && rightDistance > 30) {
            Serial.println("Turning Right (More Space Available)");
            rotateMotor(100, -100); // Turn right
            delay(1000);
        } 
        else if (leftDistance > rightDistance && leftDistance > 30) {
            Serial.println("Turning Left (More Space Available)");
            rotateMotor(-100, 100); // Turn left
            delay(1000);
        } 
        else {
            Serial.println("No clear path! Reversing...");
            rotateMotor(-100, -100); // Move backward
            delay(1000);
            Serial.println("Turning Right after Reversing");
            rotateMotor(100, -100); // Turn right
            delay(1000);
        }

        Serial.println("Resuming Forward Movement");
        rotateMotor(100, 100);
        delay(1000);
        obstacleDetected = false; // Re-enable Bluetooth control
    }
}

// Function to process blink count received from ESP-NOW
void processBlinkCount() {
    Serial.print("Processing Blink Count: ");
    Serial.println(blinkCount);

    int speedLimit = 150; // Adjust PWM value to match 20 km/h

    if (blinkCount == 2) {
        Serial.println("Moving Forward");
        motorStarted = true;
        rotateMotor(speedLimit, speedLimit);
    } 
    else if (blinkCount == 3) {
        Serial.println("Turning Right");
        rotateMotor(speedLimit, -speedLimit);
        delay(1000);
        Serial.println("Continuing Forward");
        rotateMotor(speedLimit, speedLimit);
    } 
    else if (blinkCount == 4) {
        Serial.println("Turning Left");
        rotateMotor(-speedLimit, speedLimit);
        delay(1000);
        Serial.println("Continuing Forward");
        rotateMotor(speedLimit, speedLimit);
    } 
    else if (blinkCount == 5) {
        Serial.println("Stopping");
        motorStarted = false;
        stopMotor();
    }
}


// ESP-NOW data receive callback function
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    memcpy(&blinkCount, data, sizeof(blinkCount));
    Serial.print("Received Blink Count: ");
    Serial.println(blinkCount);
    processBlinkCount();
}

// Bluetooth control function
void bluetoothControl() {
    if (SerialBT.available()) {
        char command = SerialBT.read();
        Serial.print("Received Bluetooth Command: ");
        Serial.println(command);

        if (command == 'F') rotateMotor(100, 100);  // Move forward
        if (command == 'B') rotateMotor(-100, -100); // Move backward
        if (command == 'L') rotateMotor(-100, 100);  // Turn left
        if (command == 'R') rotateMotor(100, -100);  // Turn right
        if (command == 'S') stopMotor();             // Stop motor
    }
}

// Setup function
void setup() {
    Serial.begin(115200);
    SerialBT.begin("Blink(RC CAR)"); // Initialize Bluetooth
    WiFi.mode(WIFI_STA);           // Set ESP32 to station mode (needed for ESP-NOW)

    // Set motor driver pins as output
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Set LED pins as output
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);

    // Initialize LEDs to be OFF
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    esp_now_register_recv_cb(onDataRecv); // Register callback for ESP-NOW messages

    // Attach the servo motor to the specified pin
    myServo.attach(SERVO_PIN);
}

// Main loop function
void loop() {
    if (motorStarted) {
        checkAndAvoidObstacle();
    }

    // Bluetooth Commands
    bluetoothControl();

    // LED control based on Bluetooth and Blink control states
    if (SerialBT.connected()) {
        digitalWrite(RED_LED_PIN, HIGH);   // Red light on for Bluetooth control
        digitalWrite(GREEN_LED_PIN, LOW);
    } else if (blinkCount == 1) {
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, HIGH); // Green light on for Blink control
    }
}
