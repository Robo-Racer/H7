#include "NeoPixelSPI.h"

#define BYTE_TRANSFER_LENGTH 8
#define RGBW_TRANSFER_LENGTH 4
#define HEADER_BITS 30
#define FOOTER_BITS 30
#define FREQUENCY 5700000

NeoPixelSPI::NeoPixelSPI(mbed::SPI *spi_device, int numberNeoPixels) {
  spi = spi_device;
  outputString = (byte *)malloc((numberNeoPixels * RGBW_TRANSFER_LENGTH * BYTE_TRANSFER_LENGTH) + HEADER_BITS + FOOTER_BITS);
}

void NeoPixelSPI::setup() {
  spi->frequency(FREQUENCY);
  spi->format(8, 0);
}

void NeoPixelSPI::transfer(LED_CONFIG_S *ledConfigs, int numLEDs) {
  int length = buildLEDMsg(ledConfigs, numLEDs);
  spi->transfer((byte *)outputString, length, (byte *)0, 0, 0);
}

void NeoPixelSPI::byteToSPI(byte *byteString, byte value) {
  for (int i = 0; i < BYTE_TRANSFER_LENGTH; i++) {
    if (value % 2 == 1) {
      byteString[BYTE_TRANSFER_LENGTH - i - 1] = 0xF0;
    } else {
      byteString[BYTE_TRANSFER_LENGTH - i - 1] = 0xC0;
    }
    value = value >> 1;
  }
}

int NeoPixelSPI::buildLEDMsg(LED_CONFIG_S *ledConfigs, int numLEDs) {
  byte header[HEADER_BITS] = {0};
  byte footer[FOOTER_BITS] = {0};
  int writeIndex = 0;

  for (int i = 0; i < HEADER_BITS; i++) {
    outputString[writeIndex] = header[i];
    writeIndex++;
  }

  for (int i = 0; i < numLEDs; i++) {
    if (ledConfigs[i].type == LED_TYPES_RGB || ledConfigs[i].type == LED_TYPES_RGBW) {
      byteToSPI(&outputString[writeIndex], ledConfigs[i].red);
      writeIndex += 8;
      byteToSPI(&outputString[writeIndex], ledConfigs[i].green);
      writeIndex += 8;
      byteToSPI(&outputString[writeIndex], ledConfigs[i].blue);
      writeIndex += 8;
    }
    if (ledConfigs[i].type == LED_TYPES_W || ledConfigs[i].type == LED_TYPES_RGBW) {
      byteToSPI(&outputString[writeIndex], ledConfigs[i].white);
      writeIndex += 8;
    }
  }

  for (int i = 0; i < FOOTER_BITS; i++) {
    outputString[writeIndex] = footer[i];
    writeIndex++;
  }
  return writeIndex;
}
