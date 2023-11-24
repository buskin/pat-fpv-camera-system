#include "api/Print.h"
#include "PPMEncoder2.h"
#include "NRF52_MBED_TimerInterrupt.h"
#define DEBUG false

PPMEncoder2 ppmEncoder;

long time_main;
void TimerHandler0()
{
  time_main += 10;
  if (time_main % 1000000 != 0) return;
  ppmEncoder.interrupt();
}
NRF52_MBED_Timer ITimer(NRF_TIMER_1);

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
    setChannel(chn, 1500);
  }

  pinMode(LED_BUILTIN,  OUTPUT);

  // in DEBUG we run 1000 times longer, so instead of 1500 microseconds we will have 1500 milliseconds
  ITimer.attachInterruptInterval(10 * DEBUG ? 1000 : 1, TimerHandler0);

  enable();
}

void PPMEncoder2::setChannel(uint8_t channel, uint16_t value) {
  channels[channel] = value;
}

void PPMEncoder2::enable() {
  enabled = true;
  isSpace = false;
  time_main = 0;
  time_frame_started = 0;
  time_chill_till = 0;
  currentChannel = 0;
  digitalWrite(outputPin, offState);
}

void PPMEncoder2::disable() {
  enabled = false;
}

void PPMEncoder2::interrupt() {
  if (time_main < time_chill_till) return;

  if (!enabled) return;

  // delay after pulse
  if (isSpace) {
    digitalWrite(outputPin, offState);
    time_chill_till = time_main + PPM_PULSE_LENGTH_uS;
    #if DEBUG
      Serial.print(" LO-");
      Serial.print(PPM_PULSE_LENGTH_uS);
    #endif
  } else {
    if (currentChannel == 0) {
      time_frame_started = time_main;
      #if DEBUG
        Serial.println();
      #endif
    }
    digitalWrite(outputPin, onState);
    if (currentChannel < numChannels) {
      time_chill_till = time_main + channels[currentChannel];
      #if DEBUG
        Serial.print(" HI-");
        Serial.print(channels[currentChannel]);
      #endif
      currentChannel++;
    } else {
      long time_chill_till_prev = time_chill_till;
      time_chill_till = time_frame_started + PPM_FRAME_LENGTH_uS - PPM_PULSE_LENGTH_uS;
      #if DEBUG
        Serial.print(" HSYNC-");
        Serial.print(time_chill_till - time_chill_till_prev);
      #endif
      currentChannel = 0;
    }
  }

  // this is a switch between
  isSpace = !isSpace;
}

