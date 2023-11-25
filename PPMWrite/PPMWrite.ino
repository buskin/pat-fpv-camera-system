#include "PPMEncoder2.h"

#define OUT_PIN 6

#define SYNC_TIME 3500
#define LOW_TIME 400
#define HIGH_MIN 1000
#define HIGH_MAZ 2000
#define NUM_CHANNELS 6

// frame is 1000 000 microseconds devide by 40 times per second.
// so we repeat our train of PPM pulses 40 times per second
// 40 times per second 40 HZ
const int FRAME_TIME = 20000;


// calculate max time we need to send 8 channels with 2000 value each and 5000 sync pulse
// TIME = 5000 + (2000 + 500) * 8 = 25000



int frame_time_left;


void setup() {
  Serial.begin(9600);
  while (!Serial);

  ppmEncoder.begin(OUT_PIN, 6, true);
  pinMode(OUT_PIN, OUTPUT);

  
}

const unsigned long blinkInterval = 20000;  // Blink interval in microseconds (2 kHz)
unsigned long previousMillis = 0;  // Variable to store the last time LED was updated

void loop() {
  // Check if it's time to toggle the LED
  // unsigned long currentMillis = micros();
  // if (currentMillis - previousMillis >= blinkInterval) {
  //   previousMillis = currentMillis;  // Save the current time
  //   // Toggle the LED state
  //   if (digitalRead(LED_BUILTIN) == HIGH) {
  //     digitalWrite(LED_BUILTIN, LOW);  // Turn off the LED
  //   } else {
  //     digitalWrite(LED_BUILTIN, HIGH);  // Turn on the LED
  //   }
  // }
  // return;

  //600-1600
  ppmEncoder.setChannel(0, 1100);
  ppmEncoder.setChannel(1, 605);
  ppmEncoder.setChannel(2, 1550);
  ppmEncoder.setChannel(3, 1600);
  //ppmEncoder.setChannel(4, 1500);
  //ppmEncoder.setChannel(5, 900);

  //ppmEncoder.write();
  //Serial.println(micros());
}