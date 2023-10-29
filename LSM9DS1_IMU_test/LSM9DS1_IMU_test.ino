#include <Arduino_LSM9DS1.h>

// Variables for gyroscope data
float gx, gy, gz;
float gx_cal, gy_cal, gz_cal;

// Variables for time and angle calculation
unsigned long previousTime = 0;
float elapsedTime, dt;
float angleX = 0.0;  // Initial angle
float angleY = 0.0;
float angleZ = 0.0;

long loop_timer;
long i = 0;
float angleCoefficient = 180.0 / 155.0;

void setup() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize LSM9DS1 sensor!");
    while (1);
  }

  Serial.println("Beginning Calibration. Keep head tracker stationary");
  delay(1000);
  Serial.print("Calibrating");
  for (int cal_int = 0; cal_int < 2000; cal_int++) {  //Run this code 2000 times
    if (cal_int % 200 == 0) Serial.print(".");        //Print a dot on the LCD every 125 readings
    IMU.readGyroscope(gx, gy, gz);
    gx_cal += gx;  //Add the gyro x-axis offset to the gyro_x_cal variable
    gy_cal += gy;  //Add the gyro x-axis offset to the gyro_x_cal variable
    gz_cal += gz;  //Add the gyro x-axis offset to the gyro_x_cal variable
    delay(2);
  }
  Serial.println();

  gx_cal /= 2000;  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gy_cal /= 2000;  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gz_cal /= 2000;  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  Serial.begin(9600);
}

void loop() {
  // Read gyroscope data
  IMU.readGyroscope(gx, gy, gz);

  // Calculate time since the last loop execution
  unsigned long currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
  previousTime = currentTime;

  gx -= gx_cal;
  gy -= gy_cal;
  gz -= gz_cal;

  // calc angle
  angleX += gx * elapsedTime;  // Integrate angular velocity to get angle
  angleY += gy * elapsedTime;
  angleZ += gz * elapsedTime;

  // Print the angle
  if (i % 100 == 0) {
    Serial.print("X: ");
    Serial.print(angleX * angleCoefficient);
    Serial.print(" | Y: ");
    Serial.print(angleY * angleCoefficient);
    Serial.print(" | Z: ");
    Serial.println(angleZ * angleCoefficient);
  }

  i++;
  // delay(1);
  wewait(1000);
}

void wewait(long time) {
  while (micros() - loop_timer < time)
  loop_timer = micros();  //Reset the loop timer
}