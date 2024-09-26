#include "main.h"
#include "LiquidCrystal_I2C_STM32.h"  // STM32-specific I2C LCD library
#include "vl53l0x_api.h"               // VL53L0X API for STM32
#include <math.h>

#define XSHUT_ToF_Sensor1 GPIO_PIN_4  // Pin for ToF_Sensor1
#define XSHUT_ToF_Sensor2 GPIO_PIN_5  // Pin for ToF_Sensor2
#define XSHUT_VL53L0X GPIO_PIN_6      // Pin for VL53L0X (height sensor)

#define Ledred GPIO_PIN_13
#define Ledgreen GPIO_PIN_10

I2C_HandleTypeDef hi2c1;  // I2C handle for communication
VL53L0X_Dev_t vl53Sensor;  // VL53L0X device handle
VL53L0X_Dev_t tofSensor1;
VL53L0X_Dev_t tofSensor2;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address and dimensions for the LCD

volatile bool measurementComplete = false;
volatile bool firstSensorTriggered = false;
volatile bool secondSensorTriggered = false;
volatile bool bothSensorsTriggered = false;

double g = 9.81;  // Gravitational acceleration
float triggerDistance = 0.10;  // Set trigger distance to 10 cm
double t;  // Time for free fall

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void updateLCD(float height_m);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  // Initialize LCD
  lcd.begin(16, 2, &hi2c1);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Free Fall");
  lcd.setCursor(0, 1);
  lcd.print("Height (m):");
  HAL_Delay(3000);

  HAL_GPIO_WritePin(GPIOB, Ledgreen, GPIO_PIN_SET);  // Green LED ON

  // Initialize VL53L0X sensors (height and ToF)
  // Note: VL53L0X initialization is specific to the ST API, more details below

  // Turn off all sensors initially
  HAL_GPIO_WritePin(GPIOA, XSHUT_ToF_Sensor1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, XSHUT_ToF_Sensor2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, XSHUT_VL53L0X, GPIO_PIN_RESET);

  // Initialize VL53L0X (height sensor)
  HAL_GPIO_WritePin(GPIOA, XSHUT_VL53L0X, GPIO_PIN_SET);
  HAL_Delay(10);
  if (VL53L0X_InitSensor(&vl53Sensor) != VL53L0X_ERROR_NONE) {
    Error_Handler();  // Error handling if VL53L0X fails to initialize
  }

  // Initialize ToF_Sensor1
  HAL_GPIO_WritePin(GPIOA, XSHUT_ToF_Sensor1, GPIO_PIN_SET);
  HAL_Delay(10);
  if (VL53L0X_InitSensor(&tofSensor1) != VL53L0X_ERROR_NONE) {
    Error_Handler();  // Error handling if ToF_Sensor1 fails to initialize
  }

  // Initialize ToF_Sensor2
  HAL_GPIO_WritePin(GPIOA, XSHUT_ToF_Sensor2, GPIO_PIN_SET);
  HAL_Delay(10);
  if (VL53L0X_InitSensor(&tofSensor2) != VL53L0X_ERROR_NONE) {
    Error_Handler();  // Error handling if ToF_Sensor2 fails to initialize
  }

  // Main loop
  while (1) {
    float height_m = VL53L0X_GetMeasurement(&vl53Sensor) / 1000.0;  // Height in meters

    static unsigned long lastUpdateTime = 0;
    static const unsigned long updateInterval = 500;  // Update interval in ms
    unsigned long currentTime = HAL_GetTick();

    if (currentTime - lastUpdateTime >= updateInterval) {
      if (!measurementComplete) {
        updateLCD(height_m);  // Update the LCD with height measurement
      }
      lastUpdateTime = currentTime;
    }

    // Measure distances with ToF 10120 sensors
    float distance1_m = VL53L0X_GetMeasurement(&tofSensor1) / 1000.0;
    float distance2_m = VL53L0X_GetMeasurement(&tofSensor2) / 1000.0;

    // Check for ToF 10120 sensor triggers
    if (distance1_m <= triggerDistance && !firstSensorTriggered) {
      firstSensorTriggered = true;
      HAL_GPIO_WritePin(GPIOB, Ledred, GPIO_PIN_SET);  // Red LED ON
      HAL_GPIO_WritePin(GPIOB, Ledgreen, GPIO_PIN_RESET);  // Green LED OFF
    }

    if (distance2_m <= triggerDistance && firstSensorTriggered && !secondSensorTriggered) {
      secondSensorTriggered = true;
      measurementComplete = true;
      bothSensorsTriggered = true;
      HAL_GPIO_WritePin(GPIOB, Ledred, GPIO_PIN_RESET);  // Red LED OFF
      HAL_GPIO_WritePin(GPIOB, Ledgreen, GPIO_PIN_SET);  // Green LED ON
    }

    if (measurementComplete) {
      // Calculate the time of free fall using the height and gravitational acceleration
      t = sqrt((2 * height_m) / g);  // Time (t) from height (height_m)

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Time (s):");
      lcd.setCursor(10, 0);
      lcd.print(t, 4);  // Display the calculated time t

      if (bothSensorsTriggered) {
        lcd.setCursor(0, 1);
        lcd.print("Height (m):");
        lcd.setCursor(10, 1);
        lcd.print(height_m, 4);  // Display the height measured by VL53L0X

        HAL_Delay(2000);  // Delay after both sensors triggered
        bothSensorsTriggered = false;  // Reset the flag
      }

      // Reset flags for the next measurement
      measurementComplete = false;
      firstSensorTriggered = false;
      secondSensorTriggered = false;
      HAL_GPIO_WritePin(GPIOB, Ledgreen, GPIO_PIN_SET);  // Reset LED state
    }
  }
}

void updateLCD(float height_m) {
  lcd.setCursor(0, 1);
  lcd.print("Height (m):");
  lcd.setCursor(10, 1);
  lcd.print(height_m, 4);

  printf("Height (m): %.4f\n", height_m);
}
