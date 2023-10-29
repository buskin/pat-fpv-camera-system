#include <Arduino_LSM9DS1.h>
#include <Servo.h>

Servo tilt_servo;
Servo pan_servo;
float tilt_servo_pos = 0;
float pan_servo_pos = 0;

// all the pin numbers for the buttons and the servos
const int resetButtonPin = 2;
const int tiltServoPin = 4;
const int panServoPin = 3;

const int averageOf = 10; // amount of gyro values it will take the average from
const int buttonHold = 10; // loop sequinces it needs to go through with the user holding down the button for it to reset

int i = 0; // loops through 0,1,2,3,4... until averageOf to know where to replace the current gyro readings with the old ones in the roll and tilt values list
int button_count = 0; // variable for counting how much sequences did it loop through with the user holding down the button (activates reset if equals to buttonHold and starts at zero again if you unpress it while the counting)

float tilt_total = 0; // total of tilt_values so I don't have to calculate it every time
float roll_total = 0; // total of roll_values so I don't have to calculate it every time
float pan_total = 0; // total of pan_values so I don't have to calculate it every time
float tilt_values[averageOf]; // list for figuring out the average of the tilt values and removing the noise
float roll_values[averageOf]; // list for figuring out the average of the roll values and removing the noise
float pan_values[averageOf]; // list for figuring out the average of the pan values and removing the noise

float tilt_center = 0; // center of the tilt gets assigned when reseting the gyro with the reset button
float roll_center = 0; // center of the roll gets assigned when reseting the gyro with the reset button
float pan_center = 0; // center of the pan gets assigned when reseting the gyro with the reset button


float tilt;
float roll;
float pan;
float rounded_tilt;
float rounded_roll;
float rounded_pan;
//float filtered_tilt;
//float filtered_roll;
//float filtered_pan;

float tilt2;
float roll2;
float pan2;

unsigned long previousMillis = 0;

bool isFirstCycle = true;

void setup() {
  pinMode(resetButtonPin, INPUT_PULLUP);
  tilt_servo.attach(tiltServoPin);
  pan_servo.attach(panServoPin);

  // Initialize the IMU sensor
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    exit(0); // Terminate the program
  }

  // Serial communication setup
  Serial.begin(9600);
  while (!Serial);
  
  //calibrateMagnetometer();

  Serial.println("IMU initialized");

  delay(300);
}

void loop() {
  // Read sensor data
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    unsigned long currentMillis = millis();
    float dt = (currentMillis - previousMillis) / 1000.0;  // Convert to seconds

    // Read accelerometer data
    IMU.readAcceleration(ax, ay, az);
    // Read gyroscope data
    IMU.readGyroscope(gx, gy, gz);
    // Read magnetometer data
    IMU.readMagneticField(mx, my, mz);

    // tracks if the user held the button for long enough and if he/she did then the gyro center gets reset
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

    // Calculate tilt and roll
    tilt = atan2(ay, az) * 180.0 / PI - tilt_center;
    roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI - roll_center;
    pan = atan2(-my, mx) * 180 / PI - pan_center;

    tilt2 += gx * dt;
    roll2 += gy * dt;
    pan2 += gz * dt;
    previousMillis = currentMillis;

    if(tilt < -180) {
      tilt += 360;
    } else if (tilt > 180) {
      tilt -= 360;
    }

    if(roll < -180) {
      roll += 360;
    } else if (roll > 180) {
      roll -= 360;
    }
    
    if(pan < -180) {
      pan += 360;
    } else if (pan > 180) {
      pan -= 360;
    }

    // smoothing out the roll and tilt values
    // rounded_tilt
    // rounded_roll
    average();

    // rounded tilt and roll based on the center value
    //filtered_tilt = rounded_tilt - tilt_center;
    //filtered_roll = rounded_roll - roll_center;

    // Print out the gyro values in the serial monitor
    Serial.print("Tilt: ");
    Serial.print(tilt2);
    Serial.print(" | Roll: ");
    Serial.print(pan2);
    Serial.print(" | Pan: ");
    Serial.println(pan2);
    // Serial.print(" | Button: ");
    // Serial.println(buttonValue);

    // Setting the position in the range of movement
    tilt_servo_pos = rounded_tilt + 90;
    pan_servo_pos = rounded_pan + 90;

    if(tilt_servo_pos < 0) { tilt_servo_pos = 0; } else if(tilt_servo_pos > 180) { tilt_servo_pos = 180; }
    if(pan_servo_pos < 0) { pan_servo_pos = 0; } else if(pan_servo_pos > 180) { pan_servo_pos = 180; }

    //tilt_servo.write(tilt_servo_pos);
    //pan_servo.write(pan_servo_pos);

    // if (isFirstCycle == true){
    //   pan_servo.write(0);
    //   tilt_servo.write(180);
    //   Serial.println("move");
    // }

    isFirstCycle = false;
  }
}

void average() {
  // changing the total by subtracting the oldest and adding the newest that way we don't have to calculate the total every time
  tilt_total -= tilt_values[i];
  tilt_total += tilt;
  // replacing the oldest tilt value with the current one
  tilt_values[i] = tilt;

  // changing the total by subtracting the oldest and adding the newest that way we don't have to calculate the total every time
  roll_total -= roll_values[i];
  roll_total += roll;
  // replacing the oldest roll value with the current one 
  roll_values[i] = roll;

  // changing the total by subtracting the oldest and adding the newest that way we don't have to calculate the total every time
  pan_total -= pan_values[i];
  pan_total += pan;
  // replacing the oldest roll value with the current one 
  pan_values[i] = pan;

  // average of tilt and roll values
  rounded_tilt = tilt_total / averageOf;
  rounded_roll = roll_total / averageOf;
  rounded_pan = pan_total / averageOf;

  i++;
  i = i % averageOf;
}

void resetArrays() {
  tilt_total = 0;
  roll_total = 0;
  pan_total = 0;
  for(int i = 0; i < averageOf; i++){
    tilt_values[i] = 0;
    roll_values[i] = 0;
    pan_values[i] = 0;
  }
  rounded_tilt = 0;
  rounded_roll = 0;
  rounded_pan = 0;
}

void gyroCenter(){
  tilt_center = rounded_tilt + tilt_center;
  roll_center = rounded_roll + roll_center;
  pan_center = rounded_pan + pan_center;

  tilt_servo_pos = 0;
  pan_servo_pos = 0;
  i = 0;
  button_count = 0;

  resetArrays();

  Serial.println("reset");
}