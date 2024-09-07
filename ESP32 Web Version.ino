#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ESP32Servo.h>
#include "./TinyGPS.h"
#include "./CoolerDefinitions.h"

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

WebServer server(80);

// GPS
TinyGPS gps;

// Lid
Servo lidServo;
CoolerLid lidState = CLOSED;

// Master Enable
bool enabled = false;

// Serial components
HardwareSerial GPSSerial(1);  // Use UART1 for GPS

/* Compass */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Function prototypes
GeoLoc checkGPS();
GeoLoc gpsdump(TinyGPS &gps);
bool feedgps();
void displayCompassDetails();
float geoBearing(struct GeoLoc &a, struct GeoLoc &b);
float geoDistance(struct GeoLoc &a, struct GeoLoc &b);
float geoHeading();
void setServo(int pos);
void setSpeedMotorA(int speed);
void setSpeedMotorB(int speed);
void setSpeed(int speed);
void stop();
void drive(int distance, float turn);
void driveTo(struct GeoLoc &loc, int timeout);
void setupCompass();

// Web server route handlers
void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Cooler Robot Control</h1>";
  html += "<p><a href='/togglelid'>Toggle Lid</a></p>";
  html += "<p><a href='/toggleenable'>Toggle Enable</a></p>";
  html += "<form action='/drive' method='get'>";
  html += "Latitude: <input type='text' name='lat'><br>";
  html += "Longitude: <input type='text' name='lon'><br>";
  html += "<input type='submit' value='Drive To'>";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleToggleLid() {
  switch (lidState) {
    case OPENED:
      setServo(SERVO_LID_CLOSE);
      lidState = CLOSED;
      break;
    case CLOSED:
      setServo(SERVO_LID_OPEN);
      lidState = OPENED;
      break;
  }
  server.send(200, "text/plain", "Lid toggled");
}

void handleToggleEnable() {
  enabled = !enabled;
  if (!enabled) {
    stop();
  }
  server.send(200, "text/plain", enabled ? "Enabled" : "Disabled");
}

void handleDrive() {
  if (server.hasArg("lat") && server.hasArg("lon")) {
    float lat = server.arg("lat").toFloat();
    float lon = server.arg("lon").toFloat();
    GeoLoc destination;
    destination.lat = lat;
    destination.lon = lon;
    driveTo(destination, GPS_STREAM_TIMEOUT);
    server.send(200, "text/plain", "Driving to destination");
  } else {
    server.send(400, "text/plain", "Missing latitude or longitude");
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize GPS serial
  GPSSerial.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/togglelid", handleToggleLid);
  server.on("/toggleenable", handleToggleEnable);
  server.on("/drive", handleDrive);
  server.begin();
  
  // Compass setup
  setupCompass();

  // Motor pins
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  lidServo.setPeriodHertz(50);
  lidServo.attach(SERVO_PIN, 500, 2400);
}

void loop() {
  server.handleClient();
}

// Implement the remaining functions (checkGPS, gpsdump, feedgps, etc.) here
// These functions can remain largely the same as in the previous version

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;
  
  return coolerLoc;
}

// The rest of the functions (gpsdump, feedgps, displayCompassDetails, etc.) 
// should be implemented here, similar to the previous version.
// Remember to use GPSSerial instead of nss in these functions.
