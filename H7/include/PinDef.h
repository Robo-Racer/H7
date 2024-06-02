// PinDef.h
//
// Used to store the values for pin defines to make it easier to reference
//


#ifndef PINDEF_H
#define PINDEF_H

//DEFINES
int espRx = LEDB + PA_10 + 1;
int espTx = LEDB + PA_9 + 1;
int motorPin = LEDB + PD_4 + 1;//GPIO 2?
int servoPin = LEDB + PE_3 + 1;//GPIO 4?
int hallPin = LEDB + PG_3 + 1;//GPIO 5
int stopPin1 = LEDB + PC_15 + 1;//GPIO1
int stopPin2 = LEDB + PG_10 + 1;//GPIO6

//COMMUNICATION ENUMS
enum messageHeader{
    COMM_ERR = -1,
    STOP = 0, 
    START = 1,
    READYTOSTART = 2,
    DATA = 3,
};

//COMMUNICATION ENUMS
enum dataHeader{
    DATA_ERR = -1,
    SPEED = 0,
    DISTANCE = 1,
};


#endif //define PINDEF_H