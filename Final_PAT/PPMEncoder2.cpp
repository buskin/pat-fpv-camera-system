#include "PPMEncoder2.h"
#include "NRF52_MBED_TimerInterrupt.h"
#define DEBUG false
#define TIMER_RATE 10

PPMEncoder2 ppmEncoder;

long time_main;
long time_chill_till;
NRF52_MBED_Timer ITimer(NRF_TIMER_1);
bool ledToggle2 = false;
bool ledValue = false;

bool signalValue = true;
long cnt = 0;

bool line_break = false;
char* line1;
long line2;

void TimerHandler0()
{
  time_main = micros();// / (DEBUG ? 100:1);
  if (time_main < time_chill_till) return;
  ppmEncoder.interrupt();
}

void PPMEncoder2::begin(uint8_t pin, uint8_t ch, bool inverted) {
  if (!inverted) {
    onState = HIGH;
    offState = LOW;
  } else {
    onState = LOW;
    offState = HIGH;
  }

  outputPin = pin;
  numChannels = ch;

  // fill defaults
  for (uint8_t chn = 0; chn < numChannels; chn++) {
    setChannel(chn, 1100);
  }

  pinMode(LED_BUILTIN,  OUTPUT);

  enable();

  ITimer.attachInterruptInterval(TIMER_RATE, TimerHandler0);
}

void PPMEncoder2::setChannel(uint8_t channel, uint16_t value) {
  channels[channel] = value;
}

void PPMEncoder2::enable() {
  enabled = true;
  isSpace = false;
  time_frame_started = 0;
  time_chill_till = 0;
  currentChannel = 0;
  digitalWrite(outputPin, offState);
}

void PPMEncoder2::disable() {
  enabled = false;
}

void PPMEncoder2::interrupt() {
  if (!enabled) return;

  // delay after pulse
  if (isSpace) {
    time_chill_till = time_main + PPM_PULSE_LENGTH_uS;
    #if DEBUG
      line1 = " LO-";
      line2 = PPM_PULSE_LENGTH_uS;
    #endif
    digitalWrite(outputPin, offState);
  } else {
    if (currentChannel == 0) {
      time_frame_started = time_main;
      #if DEBUG
        line_break = true;
      #endif
      ledValue = !ledValue;
      digitalWrite(LED_BUILTIN, ledValue);
    }
    if (currentChannel < numChannels) {
      time_chill_till = time_main + channels[currentChannel];
      #if DEBUG
        line1 = " HI-";
        line2 = channels[currentChannel];
      #endif
      currentChannel++;
    } else {
      long time_chill_till_prev = time_chill_till;
      time_chill_till = time_frame_started + PPM_FRAME_LENGTH_uS - PPM_PULSE_LENGTH_uS;
      #if DEBUG
        line1 = " HSYNC-";
        line2 = time_chill_till - time_chill_till_prev;
      #endif
      currentChannel = 0;
    }
    digitalWrite(outputPin, onState);
  }

  // this is a switch between
  isSpace = !isSpace;
}

void PPMEncoder2::write() {
  if (line_break) {
    Serial.println();
    line_break = false;
  }
  if (line1 != "") {
    Serial.print(line1);
    Serial.print(line2);
    line1 = "";
  }
}
