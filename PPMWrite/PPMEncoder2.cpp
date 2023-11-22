#include "PPMEncoder2.h"

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
  state = true;
  time_frame_started = micros();
  time_chill_till = 0;
  currentChannel = 0;
  digitalWrite(outputPin, offState);
}

void PPMEncoder2::disable() {
  enabled = false;
}

void PPMEncoder2::interrupt() {
  int now = micros();
  if (!enabled) {
    return;
  }

  if (time_chill_till > now) {
    // we wait till that time
    return;
  }

  // delay after pulse
  if (state) {
    digitalWrite(outputPin, offState);
    time_chill_till = now + PPM_PULSE_LENGTH_uS;
    Serial.print(" LO-");
    Serial.print(PPM_PULSE_LENGTH_uS);
  } else {
    digitalWrite(outputPin, onState);
    // we've sent all pulses, send SYNC pulse
    if (currentChannel >= numChannels) {
      currentChannel = 0;
      // we wait till next frame - pulse length (time to stop signal for signal separations)
      int time_chill_till_prev = time_chill_till;
      time_chill_till = time_frame_started + PPM_FRAME_LENGTH_uS - PPM_PULSE_LENGTH_uS;
      // next frame starting exactly in PPM_FRAME_LENGTH_uS;
      time_frame_started = time_frame_started + PPM_FRAME_LENGTH_uS;
      Serial.print(" HI-");
      Serial.print(time_chill_till - time_chill_till_prev);
      Serial.println();
    }
    // we sent current channel, time to send next channel
    else
    {
      time_chill_till = now + channels[currentChannel];
      currentChannel++;
      Serial.print(" HI-");
      Serial.print(channels[currentChannel]);
    }
  }

  // this is a switch between
  state = !state;
}