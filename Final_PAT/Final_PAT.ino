#include <Arduino_LSM9DS1.h>
#include <Servo.h>
#include "PPMEncoder2.h"

#define OUT_PIN 5

// all the pin numbers for the buttons and the servos
const int resetButtonPin = 6;
const int panServoPin = 11;
const int tiltServoPin = 10;
const bool tiltReverse = false;
const bool panReverse = false;
const float tiltGain = 1.8;
const float panGain = 1.5;
const int calibratingSamples = 600;

const int averageOf = 10; // amount of gyro values it will take the average from
const int buttonHold = 40; // loop sequences it needs to go through with the user holding down the button for it to reset

Servo tilt_servo;
Servo pan_servo;
float tilt_servo_pos = 0; // tilt servo position value
float pan_servo_pos = 0; // pan servo position value

float gx, gy, gz; // gyroscope output values
float gx_cal, gy_cal, gz_cal; // gyroscope calibration values to remove the drift

float ax, ay, az; // accelerometer output values

// angles based on gyro outputs (gyroscope values are added along with a consideration of elapsed_time)
float gyroX = 0.0;
float gyroZ = 0.0;

float accTilt;

// calculated angles using the gyro and accelerometer angle values (with plane axis names)
float tilt = 0;
float pan = 0;

float rounded_tilt; // tilt formatted with averages
float rounded_pan; // pan formatted with averages

float ppm_tilt = 1100;
float ppm_pan = 1100;

float tilt_total = 0; // total of tilt_values so I don't have to calculate it every time
float pan_total = 0; // total of pan_values so I don't have to calculate it every time
float tilt_values[averageOf]; // list for figuring out the average of the tilt values and removing the noise
float pan_values[averageOf]; // list for figuring out the average of the pan values and removing the noise

float tilt_center = 0; // center of the tilt gets assigned when reseting the gyro with the reset button
float pan_center = 0; // center of the pan gets assigned when reseting the gyro with the reset button

int rounding_value_i = 0; // loops through 0,1,2,3,4... until averageOf to know where to replace the current gyro readings with the old ones in the roll and tilt values list
int button_count = 0; // counts sequences button was down

long i = 0;

unsigned long previousTime = 0;                                               // Variable for time and angle calculation for the gyroscope angle calculations

void setup() {
  // setup all the pins and hardware
  pinMode(resetButtonPin, INPUT_PULLUP);
  tilt_servo.attach(tiltServoPin);
  pan_servo.attach(panServoPin);

  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize LSM9DS1 sensor!");
    while (1);
  }

  ppmEncoder.begin(OUT_PIN, 6, true);
  pinMode(OUT_PIN, OUTPUT);

  // Calibrating the gyroscope offset error value and subtracting it from the gyro output every loop sequence to reduce drift
  Serial.println();
  Serial.println("Beginning Calibration. Keep head tracker stationary");      // Print out this text
  delay(2000);                                                                // Give the user time to read the message
  Serial.print("Calibrating");                                                // Print out this text
  int cal_int = 0;
  while (cal_int < calibratingSamples) {
    if (!IMU.gyroscopeAvailable()) continue;
    if (cal_int % 200 == 0) Serial.print(".");                                // Print a dot on the LCD every 200 readings
    IMU.readGyroscope(gx, gy, gz);                                            // Read the raw gyro data from the IMU
    gx_cal += gx;                                                             // Add the gyro x-axis offset to the gyro_x_cal variable
    gy_cal += gy;                                                             // Add the gyro x-axis offset to the gyro_x_cal variable
    gz_cal += gz;                                                             // Add the gyro x-axis offset to the gyro_x_cal variable
    cal_int++;
  }
  Serial.println();

  // Divide the gyro calibration values by 2000 to get the avarage offset
  gx_cal /= calibratingSamples;
  gy_cal /= calibratingSamples;
  gz_cal /= calibratingSamples;
}

void loop() {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  // Calculate time since the last loop execution
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - previousTime) / 1000000.0;               // Convert to seconds
  previousTime = currentTime;                                                 // Setting the previous Time to the current

  // Read gyroscope and accelerometer data
  IMU.readGyroscope(gx, gy, gz);
  IMU.readAcceleration(ax, ay, az);

  // Subtracting the gyro offset values from the raw ones for more accurate values and no drift
  gx -= gx_cal; // tilt
  gy -= gy_cal; // roll
  gz -= gz_cal; // pan

  // transfer pitch to pan.
  gz += gy * sin((rounded_tilt + tilt_center) * (PI / 180));

  accTilt = atan2(ay, az) * 180.0 / PI;

  // Calculate angles from the gyro
  if(i == 0){
    gyroX = accTilt;
  } else {
    gyroX -= gx * elapsedTime;                                                // Integrate angular velocity to get angle
  }
  gyroZ -= gz * elapsedTime;                                                  // Integrate angular velocity to get angle

  // X = tilt; Y = roll; Z = pan;

  // tracks the button for long enough and if he/she did then the gyro center gets reset
  int buttonValue = digitalRead(resetButtonPin);
  if (buttonValue == 0) {
    if (button_count == buttonHold) {
      gyroCenter();

      delay(300);
    }
    button_count++;
  } else {
    button_count = 0;
  }
  
  tilt = gyroX * 0.9996 + accTilt * 0.0004 - tilt_center;
  //tilt = accTilt - tilt_center;
  //roll = gyroY * 0.9996 + accRoll * 0.0004 - roll_center;
  pan = gyroZ - pan_center; 

  // keeping the tilt value in the -180 to 180 range
  while(true){
    if(tilt < -180) {
      tilt += 360;
    } else if (tilt > 180) {
      tilt -= 360;
    } else {
      break;
    }
  }
  // keeping the pan value in the -180 to 180 range
  while(true){
    if(pan < -180) {
      pan += 360;
    } else if (pan > 180) {
      pan -= 360;
    } else {
      break;
    }
  }

  average();                                                                  // Smoothing out the tilt and pan values

  tilt_servo_pos = rounded_tilt * tiltGain + 90;                              // Multiplying the tilt by the gain (the servo's sensitivity and how much it will turn in comparison to your head) and adding 90 to make it the center
  pan_servo_pos = rounded_pan * panGain + 90;                                 // Multiplying the pan by the gain (the servo's sensitivity and how much it will turn in comparison to your head) and adding 90 to make it the center

  // Setting the ranges for the servo. 170 on the tilt. 180 on the pan
  if(tilt_servo_pos < 5) { tilt_servo_pos = 5; } else if(tilt_servo_pos > 175) { tilt_servo_pos = 175; }
  if(pan_servo_pos < 0) { pan_servo_pos = 0; } else if(pan_servo_pos > 180) { pan_servo_pos = 180; }

  // Reversing the values if needed
  if(tiltReverse){
    tilt_servo_pos = map(tilt_servo_pos, 0, 180, 180, 0);
  }
  if(panReverse){
    pan_servo_pos = map(pan_servo_pos, 0, 180, 180, 0);
  }
  
  // Making the values readable to the transmitter by setting them to a specific range
  ppm_tilt = map(tilt_servo_pos, 5, 175, 600, 1600);
  ppm_pan = map(pan_servo_pos, 0, 180, 600, 1600);

  // Sending the formated IMU angles as PPM values to the flysky transmitter
  ppmEncoder.setChannel(0, ppm_pan);
  ppmEncoder.setChannel(1, ppm_tilt);

  i++;                                                                        // add the loop index
}

void average() {
  // changing the total by subtracting the oldest and adding the newest that way we don't have to calculate the total every time
  tilt_total -= tilt_values[rounding_value_i];
  tilt_total += tilt;
  tilt_values[rounding_value_i] = tilt;                                       // replacing the oldest tilt value with the current one

  // changing the total by subtracting the oldest and adding the newest that way we don't have to calculate the total every time
  pan_total -= pan_values[rounding_value_i];
  pan_total += pan;
  pan_values[rounding_value_i] = pan;                                         // replacing the oldest roll value with the current one

  // average of tilt and roll values
  rounded_tilt = tilt_total / averageOf;
  rounded_pan = pan_total / averageOf;

  rounding_value_i++;
  rounding_value_i = rounding_value_i % averageOf;
}

void resetArrays() {
  tilt_total = 0;
  pan_total = 0;
  for(int i = 0; i < averageOf; i++){
    tilt_values[i] = 0;
    pan_values[i] = 0;
  }
  rounded_tilt = 0;
  rounded_pan = 0;
}

void gyroCenter(){
  tilt_center = rounded_tilt + tilt_center;
  pan_center = rounded_pan + pan_center;

  tilt_servo_pos = 0;
  pan_servo_pos = 0;
  i = 0;
  button_count = 0;

  resetArrays();

  Serial.println("reset");
}