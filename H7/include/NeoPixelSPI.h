#ifndef NeoPixelSPI_h
#define NeoPixelSPI_h

#include "mbed.h"
#include "Arduino.h"

enum LED_TYPES_E {
  LED_TYPES_W, // 单色 LED
  LED_TYPES_RGB,
  LED_TYPES_RGBW,
};

struct LED_CONFIG_S {
  LED_TYPES_E type = LED_TYPES_RGB;
  byte red = 0;
  byte green = 0;
  byte blue = 0;
  byte white = 0;
};

class NeoPixelSPI {
public:
  NeoPixelSPI(mbed::SPI *spi_device, int numberNeoPixels);
  void setup();
  void transfer(LED_CONFIG_S *ledConfigs, int numLEDs);

private:
  void byteToSPI(byte *byteString, byte value);
  int buildLEDMsg(LED_CONFIG_S *ledConfigs, int numLEDs);

  mbed::SPI *spi;
  byte *outputString;
};

#endif
