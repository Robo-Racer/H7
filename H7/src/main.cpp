#include <Arduino.h>
#include <Servo.h>
#include "PinDef.h"
#include "Portenta_H7_TimerInterrupt.h"
#include "PIDSpeedControl.h"
#include "PIDServoControl.h"
#include <string>


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

bool setupError = false;
String errorMessage = " ";

// Portenta_H7 OK       : TIM1, TIM4, TIM7, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17
Portenta_H7_Timer ITimer0(TIM1);

//interupt functions
void count_rotation();
void tetherStop();
void get_speed();


//functions
messageHeader serial_get_message();
void serial_send_message(messageHeader mHeader, dataHeader dHeader, String data);
int slow_start(float targetSpeed);


void setup() {
  Serial.begin(115200);
  while (!Serial);

  //setting up UART with  ESP32-S3
  Serial1.begin(9600, SERIAL_8N1);
  delay(2000);
  
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
    Serial.println("Timer0 interupt initialized");
  }
  else{
    errorMessage += "ITimer0 startup Error\n";
    setupError = true;
    Serial.println("Timer0 interupt failed to initialized");
  }

  // Attaches the servo to the specified pin
  myServo.attach(servoPin); 
  delay(500);
  if(myServo.attached()){
    Serial.println("Servo initialized");
    myServo.write(90); // Straight starting signal
  } else{
    errorMessage += "Servo setup error\n";
    setupError = true;
  }
  
  // Attaches the motor to the specified pin
  myMotor.attach(motorPin); 
  delay(500);
  if(myMotor.attached()){
    Serial.println("Motor initialized");
    myMotor.writeMicroseconds(1500); // Neutral Starting signal
  } else{
    errorMessage += "Motor setup error";
    setupError = true;
  }
  
}


void loop() {
  bool running = true;
  bool waitingForEsp = true;
  messageHeader recievedMessageType;
  String message = " ";

  // wait for ESP32's start message.
  Serial.println("Waiting for the ESP32 to start.");
  while (waitingForEsp){
    if(Serial1.available() > 0){
      recievedMessageType = serial_get_message();
      Serial.print("Message type: ");
      Serial.println(recievedMessageType);
      
      if(setupError == false){
        if(recievedMessageType == START){
          waitingForEsp = false;
        } else if(recievedMessageType == READYTOSTART){
          Serial.println("Send ready to start.");
          serial_send_message(READYTOSTART, DATA_ERR, message);
        }
      } else{
        Serial.println("Robot setup error please restart");
        serial_send_message(DATA, DATA_ERR, errorMessage);
      }
      
    }
  }
  Serial.println("ESP32 start confirmed");
  
  
  targetSpeed = 4.5;//set the target speed in m/s
  targetPWM = slow_start(targetSpeed);
  targetPWM -= 8;
  targetPWM = 1560;
  if(targetPWM < 1550){
    targetPWM = 1550;
  }

  while(running && !stop){
    myMotor.writeMicroseconds(targetPWM);
    Serial.print("PWM: ");
    Serial.println(targetPWM);
    Serial.print("Rotations Per Second: ");
    Serial.println(rps);
    Serial.print("Speed m/s: ");
    Serial.println(speedMPS);
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


void process_data(){
  String headerStr;
  String recievedMessage;
  dataHeader recievedHeader;

  headerStr = Serial1.readStringUntil(' ');
  recievedHeader = (dataHeader)(headerStr.toInt());
  
  switch(recievedHeader){
    case SPEED:
      recievedMessage = Serial1.readStringUntil('\n'); //clear the buffer
      break;

    case DISTANCE:
      recievedMessage = Serial1.readStringUntil('\n'); //clear the buffer
      break;

    default:
      break;
    
  }

}


messageHeader serial_get_message(){

  String recievedMessage;
  String headerStr;
  messageHeader recievedHeader;

  while(Serial1.available() > 0){
    if(Serial1.available() > 0){
      headerStr = Serial1.readStringUntil(' ');
    }
    
    recievedHeader = (messageHeader)(headerStr.toInt());

    switch(recievedHeader){
      case COMM_ERR:
        recievedMessage = Serial1.readStringUntil('\n'); //clear the buffer
        break;

      case STOP:
        stop = true;
        recievedMessage = Serial1.readStringUntil('\n'); //clear the buffer
        break;

      case START:
        stop = false;
        recievedMessage = Serial1.readStringUntil('\n'); //clear the buffer
        break;

      case READYTOSTART:
        recievedMessage = Serial1.readStringUntil('\n'); //clear the buffer
        break;

      case DATA:
        process_data();
        break;

      default:
        recievedMessage = Serial1.readStringUntil('\n'); //clear the buffer
        break;
    }

  }

  return recievedHeader;
}


void serial_send_message(messageHeader mHeader, dataHeader dHeader, String data){
  switch(mHeader){
    case COMM_ERR:
      Serial1.print(mHeader);
      Serial1.print(" ");
      Serial1.println(data);
      break;

    case DATA:
      Serial1.print(mHeader);
      Serial1.print(" ");
      Serial1.print(dHeader);
      Serial1.print(" ");
      Serial1.println(data);
      break;

    default:
      Serial1.print(mHeader);
      Serial1.println(" ");
      break;

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