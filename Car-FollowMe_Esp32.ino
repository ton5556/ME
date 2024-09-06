#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPSPlus.h>

// Constants and pin definitions
#define DEVICE_NAME "ESP32_CAR"  // BLE Device name

// GPS setup for ESP32-C3
HardwareSerial nss(1);  // Use HardwareSerial, adjust pins accordingly
TinyGPSPlus gps;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Motor pins
#define MOTOR_A_EN_PIN 21  // Choose an available GPIO pin
#define MOTOR_B_EN_PIN 22  // Choose an available GPIO pin
#define MOTOR_A_IN_1_PIN 23  // Choose an available GPIO pin
#define MOTOR_A_IN_2_PIN 24  // Choose an available GPIO pin
#define MOTOR_B_IN_1_PIN 25  // Choose an available GPIO pin
#define MOTOR_B_IN_2_PIN 26  // Choose an available GPIO pin

#define LED_BUILTIN 2

// Structure to hold GPS coordinates
struct GeoLoc {
  float lat;
  float lon;
};

GeoLoc phoneLoc;  // To store the location received via BLE

void setup() {
  // Setup serial for debugging
  Serial.begin(115200);

  // GPS initialization
  nss.begin(9600, SERIAL_8N1, 16, 17); // Adjust TX/RX pins as per ESP32-C3

  // Motor pins
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize BLE
  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();

  // Create a BLE service
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x180A));

  // Create a BLE characteristic
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         BLEUUID((uint16_t)0x2A57),
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x180A));
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE server started, awaiting connections...");

  // Initialize compass
  if (!mag.begin()) {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  displayCompassDetails();
}

void loop() {
  // Update location if available
  GeoLoc carLoc = checkGPS();
  if (carLoc.lat != 0 && carLoc.lon != 0) {
    float distance = geoDistance(carLoc, phoneLoc);
    float turn = geoBearing(carLoc, phoneLoc) - geoHeading();

    if (distance > 3.0) {  // Arbitrary threshold to start/stop following
      drive(distance, turn);
    } else {
      stop();
    }
  }
}

void processReceivedData(String data) {
  // Assuming data format is "lat,lon"
  int commaIndex = data.indexOf(',');
  if (commaIndex != -1) {
    String latStr = data.substring(0, commaIndex);
    String lonStr = data.substring(commaIndex + 1);

    phoneLoc.lat = latStr.toFloat();
    phoneLoc.lon = lonStr.toFloat();
  }
}

GeoLoc checkGPS() {
  GeoLoc loc = {0, 0};
  while (nss.available()) {
    gps.encode(nss.read());
    if (gps.location.isUpdated()) {
      loc.lat = gps.location.lat();
      loc.lon = gps.location.lng();
    }
  }
  return loc;
}

float geoDistance(GeoLoc from, GeoLoc to) {
  // Calculate the distance between two locations using the Haversine formula
  const float R = 6371000; // Earth radius in meters
  float lat1 = radians(from.lat);
  float lat2 = radians(to.lat);
  float dLat = radians(to.lat - from.lat);
  float dLon = radians(to.lon - from.lon);

  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1) * cos(lat2) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c; // Distance in meters
}

float geoBearing(GeoLoc from, GeoLoc to) {
  float y = sin(to.lon - from.lon) * cos(to.lat);
  float x = cos(from.lat) * sin(to.lat) -
            sin(from.lat) * cos(to.lat) * cos(to.lon - from.lon);
  float bearing = atan2(y, x);
  return degrees(bearing);
}

float geoHeading() {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  if (heading < 0)
    heading += 2 * PI;
  return degrees(heading);
}

void drive(float distance, float turn) {
  int speed = map(distance, 0, 100, 0, 255); // Map distance to speed
  if (turn > 10) {
    // Turn right
    analogWrite(MOTOR_A_EN_PIN, speed);
    digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN_2_PIN, LOW);
    digitalWrite(MOTOR_B_IN_1_PIN, LOW);
    digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  } else if (turn < -10) {
    // Turn left
    analogWrite(MOTOR_B_EN_PIN, speed);
    digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_B_IN_2_PIN, LOW);
    digitalWrite(MOTOR_A_IN_1_PIN, LOW);
    digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  } else {
    // Move forward
    analogWrite(MOTOR_A_EN_PIN, speed);
    analogWrite(MOTOR_B_EN_PIN, speed);
    digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN_2_PIN, LOW);
    digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
    digitalWrite(MOTOR_B_IN_2_PIN, LOW);
  }
}

void stop() {
  analogWrite(MOTOR_A_EN_PIN, 0);
  analogWrite(MOTOR_B_EN_PIN, 0);
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void displayCompassDetails() {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
}
