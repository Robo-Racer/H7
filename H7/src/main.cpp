#include <Arduino.h>
#include <Servo.h>
#include "PinDef.h"
#include "Portenta_H7_TimerInterrupt.h"
#include "PIDSpeedControl.h"
#include "PIDServoControl.h"
#include <string>

#include <iostream>
using namespace std;

// Servo Globals
//const int servoPin = 164;     // Change this to the desired GPIO pin
// breakoutPin pin = GPIO_2;
const int motorLowSpeed = 1550; // Lowest speed forward
Servo myMotor;
Servo myServo;


// RPM Globals
//const int microsecToSec = 1000000;
const float metersPerRotation = 0.126937324787;
volatile float targetSpeed = 0;
volatile float speedMPS = 0; 
volatile float rps = 0;
volatile int rotations = 0;
volatile int targetPWM = 1500;//neutral
volatile bool stop = false;

int is_interrupt = 0;
int rpm_time = 0;
int distance = 0;


// Portenta_H7 OK       : TIM1, TIM4, TIM7, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17
Portenta_H7_Timer ITimer0(TIM15);

//interupt functions
void count_rotation();
void tetherStop();
void get_speed();


//functions
string get_substring(int occurance);
void serial_get_data();
int slow_start(float targetSpeed);
void speed_control(int speed);


void setup() {
    Serial.begin(115200);
    while (!Serial);

    //setting up UART with  ESP32-S3
    Serial1.begin(9600, SERIAL_8N1);
    
    //setting up the emergency tether stop
    pinMode(stopPin1, INPUT);
    pinMode(stopPin2, OUTPUT);
    digitalWrite (stopPin2, LOW);
    attachInterrupt(stopPin1, tetherStop, RISING); 

    //setting up the hall effect sensor to count rotations
    pinMode(hallPin, INPUT);
    attachInterrupt(hallPin, count_rotation, FALLING);  

    // execute get_speed every 500ms
    if (ITimer0.attachInterruptInterval(500000, get_speed))
    {
      Serial.print(F("Starting ITimer0 OK"));
    }
    else{
      Serial.println(F("Failed to start ITimer0"));
    }

    // Attaches the servo to the specified pin
    myServo.attach(servoPin); 
    Serial.println("Init Servo");
    myServo.write(90); // Straight Starting signal
    delay(1000);

    // Attaches the motor to the specified pin
    myMotor.attach(motorPin); 
    Serial.println("Init Motor");
    myMotor.writeMicroseconds(1500); // Neutral Starting signal
    delay(1000);

    delay(1000);
    Serial.println("Start? (Press y): \n");
    while(1){
        if(Serial1.available() > 0) {
            char c = Serial1.read();
            Serial.println(c);
            if (c == 'y'){
                break;
            }
        }
    }

}


void loop() {
  bool running = true;
  targetSpeed = 5.0;//set the target speed in m/s
  //targetPWM = slow_start(targetSpeed);
  targetPWM -= 8;
  targetPWM = 1560;
  if(targetPWM < 1550){
    targetPWM = 1550;
  }

  while(running && !stop){
    //myMotor.writeMicroseconds(targetPWM);'
    myServo.write(120);
    delay(2000);
    myServo.write(60);
    delay(2000);
    Serial.print("PWM: ");
    Serial.println(targetPWM);
    Serial.print("Rotations Per Second: ");
    Serial.println(rps);
    Serial.print("Speed m/s: ");
    Serial.println(speedMPS);
    //delay(1000);
  }

  myMotor.writeMicroseconds(1500);
}


void count_rotation() {
    rotations ++;
}


void get_speed(){
  rps = rotations*2;
  speedMPS = rps*metersPerRotation;
  rotations = 0;

  //changes the target PWM based on the new speed
  if (speedMPS > targetSpeed && targetPWM > 1550)
  {
    targetPWM --;
  }
  else if (speedMPS < targetSpeed && targetPWM < 2000){
    targetPWM ++;
  }
}

/*String get_substring(String strIn, int occuranceNum){
  String subString = "";
  int found = 0;

  for(int i=0; i<(int)strIn.length(); i++){
    // If cur char is not del, then append it to the cur "word", otherwise
      // you have completed the word, print it, and start a new word.
      if(strIn[i] == ' '){
        found ++;
      }
  
      if (found == occuranceNum){
        subString += strIn[i];
      } else if(found > occuranceNum){
        break;
      }
  }

  return subString;
}*/

void process_data(){
  String headerStr;
  String recievedMessage;
  dataHeader recievedHeader;

  headerStr = Serial.readStringUntil(' ');
  recievedHeader = (dataHeader)(headerStr.toInt());
  
  switch(recievedHeader){
    case SPEED:
      break;

    case DISTANCE:
      break;

    default:
      break;
    
  }

}


void serial_get_data(){

    String recievedMessage;
    String headerStr;
    while(Serial1.available() > 0){
      if(Serial1.available() > 0){
        headerStr = Serial.readStringUntil(' ');
      }
      
      commHeader recievedHeader = (commHeader)(headerStr.toInt());

      switch(recievedHeader){
        case COMM_ERR:
          recievedMessage = Serial.readStringUntil('\n'); //clear the buffer
          break;

        case STOP:
          stop = true;
          recievedMessage = Serial.readStringUntil('\n'); //clear the buffer
          break;

        case START:
          stop = false;
          recievedMessage = Serial.readStringUntil('\n'); //clear the buffer
          break;

        case READYTOSTART:
          recievedMessage = Serial.readStringUntil('\n'); //clear the buffer
          break;

        case DATA:
          string varName = get_substring(1);
          process_data();
          break;

        /*default:
          break;*/
      }

    }
}


//slowly ramps up the PWM until the speed passes the target speed. This gets the target PWM value for a given speed.
int slow_start(float targetSpeed){
  int setSpeed = motorLowSpeed;

  while(speedMPS < targetSpeed){
    myMotor.writeMicroseconds(setSpeed);
    delay(1000);
    setSpeed += 5;
    Serial.print("PWM: ");
    Serial.println(setSpeed);
    Serial.print("Rotations Per Second: ");
    Serial.println(rps);
    Serial.print("Speed m/s: ");
    Serial.println(speedMPS);
  }

  return setSpeed;
}

void tetherStop(){
  stop = true;

}

//Distance per 1 Axle Rotation: 0.127 meters

//RPM -> M/S
/*
(X Rotation / 1 Minute) * (1 Minute / 60 Seconds) * (0.127 Meters / 1 Rotation) = Z m/s

Z m/s * (60 s / minute) * (1 Rotation / 0.127 Meters) = X rpm

RPM -> pwm number???
*/