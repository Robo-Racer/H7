// PIDServoControl.cpp
#include "PIDServoControl.h"
#include "ColorSensors.h"
#include "Adafruit_APDS9960.h"
#include <Arduino.h>

int16_t servoPrevError = 0;
int16_t servoP = 0;
int16_t servoI = 0;
int16_t servoD = 0;


//PID constants
int16_t servoKp = 1;
int16_t servoKi = 0;
int16_t servoKd = 0;


//Caluclates error values for the difference in ambient light between the left and right side
//a positive totalError value means that the line has moved more to the left
int16_t _calculateLineError(){
    struct Color* currentColors     = getCurrentColors();
    struct Color* calibratedColors  = getCalibratedColors();

    int16_t leftError   =   (currentColors[3].a - calibratedColors[3].a) + 
                    (currentColors[4].a - calibratedColors[4].a) +
                    (currentColors[5].a - calibratedColors[5].a);

    int16_t rightError  =   (currentColors[2].a - calibratedColors[2].a) + 
                    (currentColors[1].a - calibratedColors[1].a) +
                    (currentColors[0].a - calibratedColors[0].a);
    
    int16_t totalError = leftError - rightError;
    
    return totalError;
}


//https://howtomechatronics.com/tutorials/arduino/arduino-brushless-motor-control-tutorial-esc-bldc/
int16_t calculatePIDAngleChange( ){
    
    int16_t totalError = _calculateLineError();
    Serial.println("ColorSensor Line Error:");
    Serial.println(totalError);
    //PID equation values
    servoP = totalError;
    servoI += totalError;
    servoD = totalError - servoPrevError;
    servoPrevError = totalError;

    //set the angle change and limit the max values
    int16_t angleChange = servoP*servoKp + servoI*servoKi + servoD*servoKd;
    if(angleChange > 90){
        angleChange = 90;
    }
    else if (angleChange < -90){
        angleChange = -90;
    }

    return angleChange;
}


//used for debuging error values
void printLineError(){
    int16_t totalError = _calculateLineError();
    Serial.print("Total Line Error=");
    Serial.println(totalError);
    Serial.println();
}

