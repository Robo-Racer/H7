// ColorSensors.cpp
#include "ColorSensors.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "Adafruit_APDS9960.h"
#include "Wire.h"

Adafruit_APDS9960   colorSensor[COLORSENSORNUM];
struct Color        currentColors[COLORSENSORNUM];
struct Color        calibratedColors[COLORSENSORNUM];
TwoWire             colorWire(COLORSENSOR_SDA, COLORSENSOR_SCL);


void initColorSensors(){
    colorWire.begin();

    //initialize all of the color sensors
    for(uint8_t i = 0; i <= COLORSENSORNUM-1; i++){
        colorSensorSelect(i);
        delay(10);
        colorSensor[i].begin(10, APDS9960_AGAIN_4X, APDS9960_ADDRESS, &colorWire);
        colorSensor[i].enable(true);
        colorSensor[i].enableColor(true);
    }
}

//gets the color data from all of the color sensors
void colorSensorRead(){
    u_int8_t waitTime = 0;

    for(uint8_t i = 0; i <= COLORSENSORNUM-1; i++){ 
        colorSensorSelect(i);

        //wait for data to be ready
        while(!colorSensor[i].colorDataReady() && waitTime < MAXWAITTIME){
            delayMicroseconds(1);
            waitTime ++;
        }

        colorSensor[i].getColorData( &currentColors[i].r, &currentColors[i].g, 
                                        &currentColors[i].b, &currentColors[i].a );   
    }

}


void colorSensorSelect(uint8_t selectNum) {
  if (selectNum > 7) return;
  colorWire.beginTransmission(TCAADDR);
  colorWire.write(1 << selectNum);
  colorWire.endTransmission();  
}


struct Color* getCalibratedColors(){
    return calibratedColors;
}


struct Color* getCurrentColors(){
    return currentColors;
}

void colorLedOn(boolean on, Adafruit_NeoPixel strip, int numPixel){
    int color = 0;
    if(on){
        color = 255;
    }

    for (int i = 0; i < numPixel; i++) {
        strip.setPixelColor(i, strip.Color(color, color, color)); // set to white
    }
    strip.show(); // update the display
}


//used for debuging with color values
void printColors(){
    struct Color colorValue;

    colorSensorRead();
    Serial.println("Last Read Colors:");
    for(uint8_t i = 0; i <= COLORSENSORNUM-1; i++){ 
        Serial.print(i);
        Serial.print(" R=");
        Serial.print(currentColors[i].r);
        Serial.print(" G=");
        Serial.print(currentColors[i].g);
        Serial.print(" B=");
        Serial.print(currentColors[i].b);
        Serial.print(" A=");
        Serial.println(currentColors[i].a);
    }
    Serial.println();

    Serial.println("Calibrated Colors:");
    for(uint8_t i = 0; i <= COLORSENSORNUM-1; i++){
        Serial.print(i);
        Serial.print(" R=");
        Serial.print(calibratedColors[i].r);
        Serial.print(" G=");
        Serial.print(calibratedColors[i].g);
        Serial.print(" B=");
        Serial.print(calibratedColors[i].b);
        Serial.print(" A=");
        Serial.println(calibratedColors[i].a);
    }
    Serial.println();
}


void setLineCalibration(){
    u_int8_t waitTime = 0;

    for(uint8_t i = 0; i <= COLORSENSORNUM-1; i++){ 
        colorSensorSelect(i);

        //wait for data to be ready
        while(!colorSensor[i].colorDataReady() && waitTime < MAXWAITTIME){
            delayMicroseconds(1);
            waitTime ++;
        }

        colorSensor[i].getColorData( &calibratedColors[i].r, &calibratedColors[i].g, 
                                     &calibratedColors[i].b, &calibratedColors[i].a );

    }
}
