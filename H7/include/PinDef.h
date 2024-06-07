//
// Used to store the values for pin defines to make it easier to reference
//

#ifndef PINDEF_H
#define PINDEF_H

#define NUMCOLORPIXELS 24 // number of LEDs

//DEFINES
int espRx = LEDB + PA_10 + 1;
int espTx = LEDB + PA_9 + 1;
int motorPin = LEDB + PE_3 + 1;//GPIO 4
int servoPin = LEDB + PD_4 + 1;//GPIO 2
int hallPin = LEDB + PG_3 + 1;//GPIO 5
int uart1Tx = LEDB + PA_9 + 1;
int uart1Rx = LEDB + PA_10 + 1;
int stopPin1 = LEDB + PG_10 + 1;//GPIO6
int stopPin2 = LEDB + PD_5 + 1;//GPIO3
int ultrasonicRx = LEDB + PI_9 + 1;//UART0
int ultrasonicTx = LEDB + PA_0 + 1;//UART0
int openMVRX = LEDB + PG_9 + 1;;//UART2  serial 2 RX
int openMVTX = LEDB + PG_14 + 1;//UART2  serial 2 TX
int neoPin = LEDB + PG_10 + 1;//GPIO 6

//COMMUNICATION ENUMS
enum messageHeader{
    COMM_ERR = -1,
    STOP_ESP = 1, 
    START_ESP = 2,
    READYTOSTART = 3,
    DATA = 4,
};

//COMMUNICATION ENUMS
enum dataHeader{
    DATA_ERR = -1,
    SPEED = 1,
    DISTANCE = 2,
    TIME = 3,
};


#endif //define PINDEF_H