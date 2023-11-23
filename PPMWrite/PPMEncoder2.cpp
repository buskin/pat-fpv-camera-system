#include "PPMEncoder2.h"

#define DEBUG false

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

  enable();
}

void PPMEncoder2::setChannel(uint8_t channel, uint16_t value) {
  channels[channel] = value;
}

void PPMEncoder2::enable() {
  enabled = true;
  isSpace = false;
  time_frame_started = mstime();
  time_chill_till = mstime();
  currentChannel = 0;
  digitalWrite(outputPin, offState);
}

void PPMEncoder2::disable() {
  enabled = false;
}

long PPMEncoder2::mstime() {
  if (DEBUG) 
    return millis();
  else
    return micros();
}


void PPMEncoder2::interrupt() {
  long now = mstime();
  // Serial.print("_");
  // Serial.print(now);
  // Serial.print(" ");
  // Serial.print(time_chill_till);
  // Serial.println();

  if (!enabled) {
    return;
  }

  if (time_chill_till > now) {
    // we wait till that time
    return;
  }
  bool debugPrint = DEBUG;

  // delay after pulse
  if (isSpace) {
    digitalWrite(outputPin, offState);
    time_chill_till = now + PPM_PULSE_LENGTH_uS;
    if (debugPrint) {
      Serial.print(" LO-");
      Serial.print(PPM_PULSE_LENGTH_uS);
    }
  } else {
    if (currentChannel == 0) {
      time_frame_started = now;
      if (debugPrint) {
        Serial.println();
      }
    }
    digitalWrite(outputPin, onState);
    if (currentChannel < numChannels) {
      time_chill_till = now + channels[currentChannel];
      if (debugPrint) {
        Serial.print(" HI-");
        Serial.print(channels[currentChannel]);
      }
      currentChannel++;
    } else {
      long time_chill_till_prev = time_chill_till;
      time_chill_till = time_frame_started + PPM_FRAME_LENGTH_uS - PPM_PULSE_LENGTH_uS;
      if (debugPrint) {
        Serial.print(" HSYNC-");
        Serial.print(time_chill_till - time_chill_till_prev);
      }
      currentChannel = 0;
    }
  }

  // this is a switch between
  isSpace = !isSpace;
}