// ColorSensors.h

// For geting color values from the Color Sensors and processing data with the color Sensors.
// Color Sensor: https://www.adafruit.com/product/3595
// I2C Multiplexer: https://www.adafruit.com/product/2717?gad_source=1&gclid=CjwKCAiA0PuuBhBsEiwAS7fsNaWjxQqWUw7G_rGpGjiJDOesKRFjJ_IWH0KRpeo2kNod3V09S37HHhoCuSsQAvD_BwE

#ifndef COLORSENSORS_H
#define COLORSENSORS_H

//includes
#include <Arduino.h>
#include "Wire.h"
#include "Adafruit_APDS9960.h"

//defines
#define TCAADDR 0x70
#define COLORSENSOR_SDA 21
#define COLORSENSOR_SCL 22
#define COLORSENSORNUM 5
#define MAXWAITTIME 5

//The color values that the color sensors read
struct Color{
    uint16_t r = 0;
    uint16_t g = 0; 
    uint16_t b = 0; 
    uint16_t a = 0;
};


//initializes the color sensors and the SDA and SCL wire
void initColorSensors();

void colorSensorRead();

//selects the color sensor using the I2C Multiplexer
void colorSensorSelect(uint8_t);

struct Color* getCalibratedColors();

struct Color* getCurrentColors();

void printColors();

void setLineCalibration();

#endif //define COLORSENSORS_H