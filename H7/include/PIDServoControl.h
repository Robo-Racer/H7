// PIDServoControl.h

// For calculating the servos position using a PID equation
//

#ifndef PIDSERVOCONTROL_H
#define PIDSERVOCONTROL_H

//includes
#include <Arduino.h>

//calculates error for the PID equation
int16_t _calculateLineError();

int16_t calculatePIDAngleChange(int error);

//prints the error values
void printLineError();

#endif //define PIDSERVOCONTROL_H