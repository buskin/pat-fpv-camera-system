#include "PPMEncoder2.h"

PPMEncoder2 ppmEncoder;


void PPMEncoder2::begin(uint8_t pin) {
  begin(pin, PPM_DEFAULT_CHANNELS, false);
}

void PPMEncoder2::begin(uint8_t pin, uint8_t ch) {
  begin(pin, ch, false);
}

void PPMEncoder2::begin(uint8_t pin, uint8_t ch, boolean inverted) {
  // Store on/off-State in variable to avoid another if in timing-critical interrupt
  onState = (inverted) ? HIGH : LOW;
  offState = (inverted) ? LOW : HIGH;
  
  pinMode(pin, OUTPUT);
  digitalWrite(pin, offState);

  enabled = true;
  state = true;
  elapsedUs = 0;
  currentChannel = 0;

  numChannels = ch;
  outputPin = pin;

  for (uint8_t ch = 0; ch < numChannels; ch++) {
    setChannelPercent(ch, 0);
  }

}

void PPMEncoder2::setChannel(uint8_t channel, uint16_t value) {
  channels[channel] = constrain(value, PPMEncoder2::MIN, PPMEncoder2::MAX);
}

void PPMEncoder2::setChannelPercent(uint8_t channel, uint8_t percent) {
  percent = constrain(percent, 0, 100);
  setChannel(channel, map(percent, 0, 100, PPMEncoder2::MIN, PPMEncoder2::MAX));
}

void PPMEncoder2::enable() {
 enabled = true;
}

void PPMEncoder2::disable() {
 enabled = false;
 state = false;
 elapsedUs = 0;
 currentChannel = 0;
 
 digitalWrite(outputPin, offState);
}

void PPMEncoder2::interrupt() {
  if (!enabled) {
    return;
  }

  TCNT1 = 0;

  if (state) {
    digitalWrite(outputPin, onState);
    OCR1A = PPM_PULSE_LENGTH_uS * 2;

  } else {
    digitalWrite(outputPin, offState);

    if (currentChannel >= numChannels) {
      currentChannel = 0;
      elapsedUs = elapsedUs + PPM_PULSE_LENGTH_uS;
      OCR1A = (PPM_FRAME_LENGTH_uS - elapsedUs) * 2;
      elapsedUs = 0;
    } else {
      OCR1A = (channels[currentChannel] - PPM_PULSE_LENGTH_uS) * 2;
      elapsedUs = elapsedUs + channels[currentChannel];

      currentChannel++;
    }
  }

  state = !state;
}

ISR(TIMER1_COMPA_vect) {
  ppmEncoder.interrupt();
}