// video example which shows how PPM works
// https://youtu.be/0IVPUiYKeek?si=ogZn6SzKZd1FdwJM
#define OUT_PIN 13

#define SYNC_TIME 5000
#define LOW_TIME 500
#define HIGH_MIN 1000
#define HIGH_MAZ 2000

// frame is 1000 000 microseconds devide by 40 times per second.
// so we repeat our train of PPM pulses 40 times per second
// 40 times per second 40 HZ
const int FRAME_TIME = 1000000 / 40;

// calculate max time we need to send 8 channels with 2000 value each and 5000 sync pulse
// TIME = 5000 + (2000 + 500) * 8 = 25000

int frame_time_left;

void setup() {
  pinMode(OUT_PIN, OUTPUT);    // sets the digital pin 13 as output
}

void loop() {
  frame_time_left = FRAME_TIME;
  // sync pulse
  pulse(OUT_PIN, SYNC_TIME);
  // 1ch
  pulse(OUT_PIN, 1000);
  // 2ch
  pulse(OUT_PIN, 2000);
  // 3ch
  pulse(OUT_PIN, 1000);
  // 4ch
  pulse(OUT_PIN, 2000);
  // 5ch
  pulse(OUT_PIN, 1000);
  // 6ch
  pulse(OUT_PIN, 2000);
  // 7ch
  pulse(OUT_PIN, 1000);
  // 8ch
  pulse(OUT_PIN, 2000);

  // wait till end of a frame
  delayMicroseconds(frame_time_left);
}

void pulse(int pin, int time) {
  // start SIGNAL
  digitalWrite(pin, HIGH);  
  // wait SIGNAL
  delayMicroseconds(time);
  // stop SIGNAL
  digitalWrite(pin, LOW);
  // wait delay signal
  delayMicroseconds(LOW_TIME);
  frame_time_left = frame_time_left - (time + LOW_TIME); 
}
