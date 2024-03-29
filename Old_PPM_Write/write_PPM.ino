// video example which shows how PPM works
// https://youtu.be/0IVPUiYKeek?si=ogZn6SzKZd1FdwJM
#include "PPMEncoder.h"

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
  ppmEncoder.begin(OUT_PIN);
}

void loop() {
  frame_time_left = FRAME_TIME;
  // sync pulse
  //pulse(OUT_PIN, SYNC_TIME);

  // Min value
  ppmEncoder.setChannel(0, 500);
  ppmEncoder.setChannel(0, PPMEncoder::MIN);
  ppmEncoder.setChannelPercent(0, 0);

  // Max value
  ppmEncoder.setChannel(0, 2500);
  ppmEncoder.setChannel(0, PPMEncoder::MAX);
  ppmEncoder.setChannelPercent(0, 100);

  // 1ch
  pulse(OUT_PIN, 2000);
  //Serial.print("LOW ");
  // 2ch
  pulse(OUT_PIN, 2000);
  //Serial.print("HIGH ");
  // 3ch
  pulse(OUT_PIN, 2000);
  //Serial.print("LOW ");
  // 4ch
  pulse(OUT_PIN, 2000);
  //Serial.print("HIGH ");
  // 5ch
  pulse(OUT_PIN, 2000);
  //Serial.print("LOW ");
  // 6ch
  pulse(OUT_PIN, 2000);
  //Serial.print("HIGH ");
  // 7ch
  //pulse(OUT_PIN, 1000);
  // 8ch
  //pulse(OUT_PIN, 1000);

  pulse(OUT_PIN, SYNC_TIME);
  //Serial.print("SYNC DONE : ");
  // wait till end of a frame
  delayMicroseconds(frame_time_left);
  //pulse(OUT_PIN, frame_time_left);
  Serial.print("DONE");
  Serial.println();
}

void pulse(int pin, int time) {
  // stop SIGNAL
  digitalWrite(pin, HIGH);
  // wait delay signal
  delayMicroseconds(LOW_TIME);

  Serial.print("=");
  Serial.print(LOW_TIME);

  // start SIGNAL
  digitalWrite(pin, LOW);  
  // wait SIGNAL
  delayMicroseconds(time);

  Serial.print("_");
  Serial.print(time);

  //digitalWrite(pin, HIGH);
  frame_time_left -= (time + LOW_TIME);
}
