#include "PPMEncoder2.h"

#define OUT_PIN 5

#define SYNC_TIME 3500
#define LOW_TIME 400
#define HIGH_MIN 1000
#define HIGH_MAZ 2000
#define NUM_CHANNELS 6

// frame is 1000 000 microseconds devide by 40 times per second.
// so we repeat our train of PPM pulses 40 times per second
// 40 times per second 40 HZ
const int FRAME_TIME = 20000;

int ppmValues[NUM_CHANNELS];

// calculate max time we need to send 8 channels with 2000 value each and 5000 sync pulse
// TIME = 5000 + (2000 + 500) * 8 = 25000

int frame_time_left;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  //pinMode(OUT_PIN, OUTPUT);    // sets the digital pin OUT_PIN as output
  ppmEncoder.begin(OUT_PIN, 6, true);
}

void loop() {
  ppmEncoder.setChannel(0, 1000);
  ppmEncoder.setChannel(1, 1100);
  ppmEncoder.setChannel(2, 1200);
  ppmEncoder.setChannel(3, 1300);
  ppmEncoder.setChannel(4, 1700);
  ppmEncoder.setChannel(5, 1900);
}