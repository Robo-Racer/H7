// PIDServoControl.cpp
#include "PIDServoControl.h"
#include "ColorSensorArray.h"
#include "Adafruit_APDS9960.h"
#include <Arduino.h>

int16_t    leftError   = 0;
int16_t    rightError  = 0;
int16_t    totalError  = 0;

//PID equation values
int16_t prevError = 0;
int16_t P = 0;
int16_t I = 0;
int16_t D = 0;

//PID constants
int16_t Kp = 0;
int16_t Ki = 0;
int16_t Kd = 0;

//Caluclates error values for the difference in ambient light between the left and right side
//a positive totalError value means that the line has moved more to the left
int16_t calculateError(){
    struct Color* currentColors     = getCurrentColors();
    struct Color* calibratedColors  = getCalibratedColors();

    leftError   =   (currentColors[3].a - calibratedColors[3].a) + 
                    (currentColors[4].a - calibratedColors[4].a);//+
                  //(currentColors[5].a - calibratedColors[5].a);

    rightError  =   (currentColors[2].a - calibratedColors[2].a) + 
                    (currentColors[1].a - calibratedColors[1].a);//+
                  //(currentColors[0].a - calibratedColors[0].a);
    
    totalError  = leftError - rightError;
    
    return totalError;
}

int16_t calculatePIDAngleChange(){
    P = totalError;
    I += totalError;
    D = totalError - prevError;
    prevError = totalError;

    int16_t angleChange = P*Kp + I*Ki + D*Kd;
}


//used for debuging error values
void printError(){
    calculateError();
    Serial.print("Left Error=");
    Serial.println(leftError);
    Serial.print("Right Error=");
    Serial.println(rightError);
    Serial.print("Total Error=");
    Serial.println(totalError);
    Serial.println();
}

