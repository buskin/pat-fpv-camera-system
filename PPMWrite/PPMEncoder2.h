#ifndef PPMEncoder2_h
#define PPMEncoder2_h


#include "Arduino.h"

#define PPM_DEFAULT_CHANNELS 12
// #define PPM_PULSE_LENGTH_uS 500
// #define PPM_FRAME_LENGTH_uS 22500
#define PPM_PULSE_LENGTH_uS 400
#define PPM_FRAME_LENGTH_uS 20000

class PPMEncoder2 {
public:
  void begin(uint8_t pin, uint8_t ch, bool inverted);
  void setChannel(uint8_t channel, uint16_t value);
  void setChannelPercent(uint8_t channel, uint8_t percent);
  void enable();
  void disable();
  void interrupt();

private:
  long mstime();
  uint8_t outputPin;
  bool enabled;
  bool isSpace;
  long time_frame_started;
  long time_chill_till;
  uint8_t currentChannel;
  uint8_t numChannels;
  uint16_t channels[PPM_DEFAULT_CHANNELS];
  uint8_t onState;
  uint8_t offState;
};

extern PPMEncoder2 ppmEncoder;
#endif