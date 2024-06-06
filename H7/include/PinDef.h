//
// Used to store the values for pin defines to make it easier to reference
//

#ifndef PINDEF_H
#define PINDEF_H

#define NUMCOLORPIXELS 24 // number of LEDs

//DEFINES
int espRx = LEDB + PA_10 + 1;
int espTx = LEDB + PA_9 + 1;
int motorPin = LEDB + PD_4 + 1;//GPIO 2?
int servoPin = LEDB + PE_3 + 1;//GPIO 4?
int hallPin = LEDB + PG_3 + 1;//GPIO 5
int uart1Tx = LEDB + PA_9 + 1;
int uart1Rx = LEDB + PA_10 + 1;
int stopPin1 = LEDB + PC_15 + 1;//GPIO1
int stopPin2 = LEDB + PG_10 + 1;//GPIO6
int ultrasonicRx = LEDB + PI_9 + 1;//UART0
int ultrasonicTx = LEDB + PA_0 + 1;//UART0
int openMVRX = LEDB + PG_9 + 1;;//UART2  serial 2 RX
int openMVTX = LEDB + PG_14 + 1;//UART2  serial 2 TX
int neoPin = LEDB + 6 + 1;

//COMMUNICATION ENUMS
enum messageHeader{
    COMM_ERR = -1,
    STOP = 1, 
    START = 2,
    READYTOSTART = 3,
    DATA = 4,
};

//COMMUNICATION ENUMS
enum dataHeader{
    DATA_ERR = -1,
    SPEED = 1,
    DISTANCE = 2,
};


#endif //define PINDEF_H