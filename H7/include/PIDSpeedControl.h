// PIDSpeedControl.h

// For calculating the speed using a PID equation
//

#ifndef PIDSPEEDCONTROL_H
#define PIDSPEEDCONTROL_H

//includes
#include <Arduino.h>

//calculates error for the PID equation
int16_t _calculateSpeedError();

int16_t calculatePIDSpeedChange();

//prints the error values
void printSpeedError();

#endif //define PIDSPEEDCONTROL_H