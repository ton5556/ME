#include <Wire.h>
#include <VL53L0X.h>
#include <LiquidCrystal_I2C.h>

#define IR_Sensor1 2
#define IR_Sensor2 3
#define Ledred 13
#define Ledgreen 10
#define MAX_DISTANCE 600

TwoWire lcdWire; 
VL53L0X sensor; 
LiquidCrystal_I2C lcd(0x27, 16, 2); 

volatile unsigned long startMicros = 0;
volatile unsigned long endMicros = 0;

volatile bool measurementComplete = false;
volatile bool firstSensorTriggered = false;
volatile bool secondSensorTriggered = false;
volatile bool bothSensorsTriggered = false;

double g = 9.81; 
double t;
//double t_avg;

void setup() {
  pinMode(IR_Sensor1, INPUT);
  pinMode(IR_Sensor2, INPUT);
     
  lcd.begin(16, 2, &lcdWire); 
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Free Fall");
  lcd.setCursor(0, 1);
  lcd.print("Height (m):");
  delay(3000);

  digitalWrite(Ledgreen , HIGH);

  Wire.begin(); 
  sensor.init();
  sensor.setTimeout(500);
  
  attachInterrupt(digitalPinToInterrupt(IR_Sensor1), ISR_IR_Sensor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_Sensor2), ISR_IR_Sensor2, CHANGE);
}

void loop() {
  
  static unsigned long lastUpdateTime = 0;
  static const unsigned long updateInterval = 500; // Update interval in milliseconds form 500
  
  unsigned long currentTime = millis();
  
  int distance_mm = sensor.readRangeSingleMillimeters();
  float distance_m = distance_mm / 1000.0; 
  float distance_m_fig = distance_m - 0.03;
  
  if (currentTime - lastUpdateTime >= updateInterval) {
    if (!measurementComplete) {
      updateLCD(distance_m_fig);
    }
    lastUpdateTime = currentTime;
  }

  t = sqrt((2 * distance_m_fig) / g);
  //t_avg = t +- 0.01;

  if (measurementComplete) {
    double free_fall_time = (endMicros - startMicros);

    double sqrt_time = sqrt(free_fall_time);
    double unrooted_time = sqrt_time - 0.0832;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Time (s):");
    lcd.setCursor(10, 0);
    lcd.print(t, 4);
    
    if (bothSensorsTriggered) {
      lcd.setCursor(0, 1);
      lcd.print("Height (m):");
      lcd.setCursor(10, 1);
      lcd.print(distance_m_fig ,4);
      
      delay(2000); // Delay after both sensors triggered
      bothSensorsTriggered = false; // Reset the flag
    }
    
    measurementComplete = false;
    firstSensorTriggered = false;
    secondSensorTriggered = false;
    digitalWrite(Ledgreen , HIGH); // Reset LED state
  }
}

void ISR_IR_Sensor1() {
  if (digitalRead(IR_Sensor1) == HIGH) {
    startMicros = micros();
    firstSensorTriggered = true;
    digitalWrite(Ledred , HIGH);
    digitalWrite(Ledgreen , LOW);
    if (firstSensorTriggered && secondSensorTriggered) {
      bothSensorsTriggered = true;
    }
  }
}

void ISR_IR_Sensor2() {
  if (digitalRead(IR_Sensor2) == LOW) {
    if (firstSensorTriggered) {
      endMicros = micros();
      measurementComplete = true;
      digitalWrite(Ledred , LOW);
      digitalWrite(Ledgreen , HIGH);
      secondSensorTriggered = true;
      if (firstSensorTriggered && secondSensorTriggered) {
        bothSensorsTriggered = true;
      }
    }
  }
}

void updateLCD(float distance_m_fig) {
  lcd.setCursor(0, 1);
  lcd.print("Height (m):");
  lcd.setCursor(10, 1);
  lcd.print(distance_m_fig ,4);

  Serial.print("Height (m):");
  Serial.println(distance_m_fig, 4);
}
