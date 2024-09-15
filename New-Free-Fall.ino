#include <Wire.h>
#include <VL53L0X.h>
#include <LiquidCrystal_I2C.h>

// Define I2C addresses for the ToF 10120 sensors
#define ToF_Sensor1_Address 0x29  // Example I2C address for ToF 10120 sensor 1
#define ToF_Sensor2_Address 0x30  // Example I2C address for ToF 10120 sensor 2

#define Ledred 13
#define Ledgreen 10

TwoWire lcdWire;
VL53L0X vl53Sensor;  // VL53L0X sensor for height measurement
Adafruit_VL53L0X tofSensor1; // ToF 10120 sensor 1
Adafruit_VL53L0X tofSensor2; // ToF 10120 sensor 2
LiquidCrystal_I2C lcd(0x27, 16, 2);

volatile bool measurementComplete = false;
volatile bool firstSensorTriggered = false;
volatile bool secondSensorTriggered = false;
volatile bool bothSensorsTriggered = false;

double g = 9.81;  // Gravitational acceleration
float triggerDistance = 0.10;  // Set trigger distance to 10 cm
double t;  // Time for free fall

void setup() {
  pinMode(Ledred, OUTPUT);
  pinMode(Ledgreen, OUTPUT);

  Serial.begin(9600);

  lcd.begin(16, 2, &lcdWire);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Free Fall");
  lcd.setCursor(0, 1);
  lcd.print("Height (m):");
  delay(3000);

  digitalWrite(Ledgreen, HIGH);

  Wire.begin();
  
  // Initialize VL53L0X
  vl53Sensor.init();
  vl53Sensor.setTimeout(500);
  
  // Initialize ToF 10120 sensors
  tofSensor1.begin(ToF_Sensor1_Address);
  tofSensor2.begin(ToF_Sensor2_Address);
}

void loop() {
  // Measure the height using VL53L0X
  int height_mm = vl53Sensor.readRangeSingleMillimeters();
  float height_m = height_mm / 1000.0;  // Convert mm to meters

  static unsigned long lastUpdateTime = 0;
  static const unsigned long updateInterval = 500;  // Update interval in ms
  unsigned long currentTime = millis();

  if (currentTime - lastUpdateTime >= updateInterval) {
    if (!measurementComplete) {
      updateLCD(height_m);  // Update the LCD with height measurement
    }
    lastUpdateTime = currentTime;
  }

  // Measure distances with ToF 10120 sensors
  int distance1_mm = tofSensor1.readRange();
  int distance2_mm = tofSensor2.readRange();

  float distance1_m = distance1_mm / 1000.0;
  float distance2_m = distance2_mm / 1000.0;

  // Check for ToF 10120 sensor triggers
  if (distance1_m <= triggerDistance && !firstSensorTriggered) {
    firstSensorTriggered = true;
    digitalWrite(Ledred, HIGH);
    digitalWrite(Ledgreen, LOW);
  }

  if (distance2_m <= triggerDistance && firstSensorTriggered && !secondSensorTriggered) {
    secondSensorTriggered = true;
    measurementComplete = true;
    bothSensorsTriggered = true;
    digitalWrite(Ledred, LOW);
    digitalWrite(Ledgreen, HIGH);
  }

  if (measurementComplete) {
    // Calculate the time of free fall using the height and gravitational acceleration
    t = sqrt((2 * height_m) / g);  // Time (t) from height (height_m)

    lcd.clear();  // Clear the display only when measurement is complete
    lcd.setCursor(0, 0);
    lcd.print("Time (s):");
    lcd.setCursor(10, 0);
    lcd.print(t, 4);  // Display the calculated time t

    if (bothSensorsTriggered) {
      lcd.setCursor(0, 1);
      lcd.print("Height (m):");
      lcd.setCursor(10, 1);
      lcd.print(height_m, 4);  // Display the height measured by VL53L0X

      delay(2000);  // Delay after both sensors triggered
      bothSensorsTriggered = false;  // Reset the flag
    }

    // Reset flags for next measurement
    measurementComplete = false;
    firstSensorTriggered = false;
    secondSensorTriggered = false;
    digitalWrite(Ledgreen, HIGH);  // Reset LED state
  }
}

void updateLCD(float height_m) {
  lcd.setCursor(0, 1);
  lcd.print("Height (m):");
  lcd.setCursor(10, 1);
  lcd.print(height_m, 4);

  Serial.print("Height (m): ");
  Serial.println(height_m, 4);
}
