// CoolerDefinitions.h

// Pin variables (adjust these according to your ESP32 wiring)
#define SERVO_PIN 13
#define GPS_TX_PIN 16
#define GPS_RX_PIN 17
#define MOTOR_A_EN_PIN 25
#define MOTOR_B_EN_PIN 26
#define MOTOR_A_IN_1_PIN 27
#define MOTOR_A_IN_2_PIN 14
#define MOTOR_B_IN_1_PIN 12
#define MOTOR_B_IN_2_PIN 13

// If one motor tends to spin faster than the other, add offset
#define MOTOR_A_OFFSET 20
#define MOTOR_B_OFFSET 0

// You must then add your 'Declination Angle' to the compass, which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 13° 24' E (Positive), which is ~13 Degrees, or (which we need) 0.23 radians
#define DECLINATION_ANGLE 0.23f

// The offset of the mounting position to true north
// It would be best to run the /examples/magsensor sketch and compare to the compass on your smartphone
#define COMPASS_OFFSET 0.0f

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18

// Number of changes in movement to timeout for GPS waypoints
// Keeps the cooler from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45

// PWM write for servo locations
#define SERVO_LID_OPEN 20
#define SERVO_LID_CLOSE 165

// Definitions (don't edit these)
struct GeoLoc {
  float lat;
  float lon;
};

enum CoolerLid {
  OPENED,
  CLOSED
};
