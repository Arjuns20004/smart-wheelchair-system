#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SoftwareSerial.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
SoftwareSerial gsm(10, 11);  // A7672S (TX, RX)

const char phoneNumber[] = "+919345636893";
unsigned long lastFallTime = 0;
const int fallDelay = 10000;  // 10 seconds between alerts

const int buzzerPin = 9;

void setup() {
  Serial.begin(9600);
  gsm.begin(115200);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  if (!accel.begin()) {
    Serial.println("ADXL345 not detected!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  delay(5000);
  gsm.println("AT");
  delay(1000);
  gsm.println("AT+CMGF=1");  // Text mode
  delay(1000);
  gsm.println("AT+CSCS=\"GSM\"");
  delay(1000);

  // Power on GPS
  gsm.println("AT+CGNSPWR=1");
  delay(2000);
}

void loop() {
  float filteredZ = getFilteredZ();

  unsigned long currentTime = millis();
  if (filteredZ < 0 && currentTime - lastFallTime > fallDelay) {
    Serial.println("Fall Detected!");

    digitalWrite(buzzerPin, HIGH);
    delay(5000);
    digitalWrite(buzzerPin, LOW);

    String gpsLink = getGPSLocation();
    sendSMS("Fall Detected! Please check.\nLocation: " + gpsLink);

    lastFallTime = currentTime;
  }

  delay(500);
}

float getFilteredZ() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sum += event.acceleration.z;
    delay(10);
  }
  return sum / 10;
}

String getGPSLocation() {
  gsm.println("AT+CGNSINF");
  delay(2000);

  String gpsData = "";
  while (gsm.available()) {
    gpsData = gsm.readStringUntil('\n');
    if (gpsData.startsWith("+CGNSINF:")) {
      break;
    }
  }

  // Parse GPS
  int commaIndex1 = gpsData.indexOf(',', 2);
  for (int i = 0; i < 2; i++) commaIndex1 = gpsData.indexOf(',', commaIndex1 + 1); // Skip to Latitude
  int latStart = commaIndex1 + 1;
  int latEnd = gpsData.indexOf(',', latStart);
  String latitude = gpsData.substring(latStart, latEnd);

  int lonStart = latEnd + 1;
  int lonEnd = gpsData.indexOf(',', lonStart);
  String longitude = gpsData.substring(lonStart, lonEnd);

  if (latitude.length() > 5 && longitude.length() > 5) {
    return "https://maps.google.com/?q=" + latitude + "," + longitude;
  } else {
    return "Location not available";
  }
}

void sendSMS(String message) {
  gsm.println("AT+CMGF=1");
  delay(1000);
  gsm.println("AT+CMGS=\"" + String(phoneNumber) + "\"");
  delay(1000);
  gsm.print(message);
  delay(500);
  gsm.write(26); // CTRL+Z
  delay(5000);
}
