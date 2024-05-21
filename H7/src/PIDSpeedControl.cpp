// PIDSpeedControl.cpp
#include "PIDSpeedControl.h"
#include "ColorSensors.h"
#include "Adafruit_APDS9960.h"
#include <Arduino.h>


//PID equation values
int16_t motorPrevError = 0;
int16_t motorP = 0;
int16_t motorI = 0;
int16_t motorD = 0;

//PID constants
int16_t motorKp = 0;
int16_t motorKi = 0;
int16_t motorKd = 0;

//Calculates error values for the difference in ambient light between the left and right side
//a positive totalError value means that the line has moved more to the left
int16_t _calculateSpeedError(float currentSpeed, float targetSpeed){
    
    int16_t totalError = currentSpeed - targetSpeed;
    
    return totalError;
}

//https://howtomechatronics.com/tutorials/arduino/arduino-brushless-motor-control-tutorial-esc-bldc/
int16_t calculatePIDSpeedChange(float currentSpeed, float targetSpeed){
    int16_t totalError = _calculateSpeedError();

    motorP = totalError;
    motorI += totalError;
    motorD = totalError - motorPrevError;
    motorPrevError = totalError;

    //set the motor change and limit the max values
    int16_t motorChange = motorP*motorKp + motorI*motorKi + motorD*motorKd;
    if(motorChange > 90){
        motorChange = 90;
    }
    else if (motorChange < -90){
        motorChange = -90;
    }

    return motorChange;//slowstart speed + motor change should give the speed?
}


//used for debuging error values
void printSpeedError(){
    int16_t totalError = _calculateSpeedError();
    Serial.print("Total Error=");
    Serial.println(totalError);
    Serial.println();
}

