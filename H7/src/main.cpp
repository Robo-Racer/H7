#include <Arduino.h>

#include "Adafruit_APDS9960.h"
#include "ColorSensorArray.h"
#include "PIDServoControl.h"
#include "Wire.h"


/////////////////////////////////////
//Function Declarations
/////////////////////////////////////


void setup() {
  Serial.begin(115200);
  while (!Serial); 

  initColorSensors(); 
  
  delay(100);
  setLineCalibration();
}


void loop() {
  printColors();
  printError();
  delay(1000);
}


/////////////////////////////////////
//Function Definitions
/////////////////////////////////////
