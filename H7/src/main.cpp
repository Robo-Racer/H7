#include <Arduino.h>
#include "Wire.h"
#include "Adafruit_APDS9960.h"
#include "ColorSensor.h"

#define TCAADDR 0x70
#define COLORSENSOR_SDA -1  //Define Pins for H7
#define COLORSENSOR_SCL -1  //Define Pins for H7

Adafruit_APDS9960 colorSensor[6];
TwoWire colorWire(COLORSENSOR_SDA, COLORSENSOR_SCL);


// put function declarations here:
void tcaselect(uint8_t);


void setup() {
  colorWire.begin();
  Serial.begin(115200);
  while (!Serial);  


  //initialize all of the color sensors
  for(uint8_t i = 0; i <= 5; i++)
    {
    tcaselect(i);
    delay(10);
    colorSensor[i].begin(10, APDS9960_AGAIN_4X, APDS9960_ADDRESS, &colorWire);
    colorSensor[i].enable(true);
    colorSensor[i].enableColor(true);
    }
}

void loop() {
  uint16_t r=0, g=0, b=0, a=0;

  for(uint8_t i = 0; i <= 5; i++)
    { 
    tcaselect(i);
    delay(10);
    if(colorSensor[i].colorDataReady())
      {
      colorSensor[i].getColorData(&r, &g, &b, &a);
      Serial.print(i);
      Serial.print(" R=");
      Serial.print(r);
      Serial.print(" G=");
      Serial.print(g);
      Serial.print(" B=");
      Serial.print(b);
      Serial.print(" A=");
      Serial.println(a);
      }
    
    }
  Serial.println();
  delay(5000);
}

// put function definitions here:
void tcaselect(uint8_t i) {
  if (i > 7) return;
  colorWire.beginTransmission(TCAADDR);
  colorWire.write(1 << i);
  colorWire.endTransmission();  
}