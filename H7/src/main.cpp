#include <Arduino.h>
#include <Servo.h>
#include "PinDef.h"
#include "Portenta_H7_TimerInterrupt.h"
#include "PIDSpeedControl.h"
#include "PIDServoControl.h"
#include "ultrasonicsensor.h"
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
bool setupError = false;
String errorMessage = " ";
const int threshold = 10; // Example threshold value, adjust based on testing
// Portenta_H7 OK       : TIM1, TIM4, TIM7, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17
Portenta_H7_Timer ITimer0(TIM15);
UltrasonicSensor ultrasonicSensor(ultrasonicRx, ultrasonicTx);


//interupt functions
void count_rotation();
void tetherStop();

void get_speed();

//functions
messageHeader serial_get_message();
void serial_send_message(messageHeader mHeader, dataHeader dHeader, String data);
void get_RPS();
void handle_openMV_input();
//functions
string get_substring(int occurance);
void serial_get_data();
int slow_start(float targetSpeed);
void speed_control(int speed);


void setup() {
    bool waitingForEsp = true;
    messageHeader recievedMessageType;

    Serial.begin(115200);
    while (!Serial);

    //setting up UART with  ESP32-S3
    Serial1.begin(9600, SERIAL_8N1);
    Serial2.begin(115200); // UART for OpenMV communication
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
      Serial.print(F("Starting ITimer0"));
    }
    else{
      errorMessage += "ITimer0 startup Error\n";
      setupError = true;
      Serial.println(F("Failed to start ITimer0"));
    }

    // Attaches the servo to the specified pin
    myServo.attach(servoPin); 
    delay(500);
    if(myServo.attached()){
      Serial.println("Init Servo");
      myServo.write(90); // Straight starting signal
    } else{
      errorMessage += "Servo setup error\n";
      setupError = true;
    }
    

    // Attaches the motor to the specified pin
    myMotor.attach(motorPin); 
    if(myMotor.attached()){
      Serial.println("Init Motor");
      myMotor.writeMicroseconds(1500); // Neutral Starting signal
      delay(1000);
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


  while (waitingForEsp){//wait for ESP32 start message
    if(Serial1.available() > 0){
      recievedMessageType = serial_get_message();
      
      if(setupError == false){
        if(recievedMessageType == START){
          waitingForEsp = false;
        } else if(recievedMessageType == READYTOSTART){
          serial_send_message(READYTOSTART, DATA_ERR, message);
        }
      } else{
        serial_send_message(DATA, DATA_ERR, errorMessage);
      }
      
    }
  }
  

  while (running && !stop) {
      handle_openMV_input();
      // Update and check distance from ultrasonic sensor
      ultrasonicSensor.update();
      float distance = ultrasonicSensor.getDistance();
      if (distance < 20) { // If an object is detected within 20 cm
          myMotor.writeMicroseconds(1500); // Immediately stop the motor
          stop = true;
      }
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
      Serial.print("Distance (cm): ");
      Serial.println(distance);

  }
}

  //delay(2000);  // Wait for 1 second before repeating


void speed_control(int speed){
  int onTime = 5;
  int offTime= 20;
  int cyclesForTime = (int)((5000)/(((speed * onTime)+offTime))+1); //cycles to get to 2 seconds (adds an additional cycle so slightly over)
  for(int i = 0; i < cyclesForTime; i++){
    myMotor.writeMicroseconds(motorLowSpeed); 
    delay((speed * onTime));

    myMotor.writeMicroseconds(1500); // Neutral again.
    delay((offTime));
  }
}

// Function to handle OpenMV input
void handle_openMV_input() {
    if (Serial2.available() > 0) {
        // int error = Serial2.parseInt(); // Read the error value from OpenMV
        // int servoAngle = map(error, -45, 45, 0, 180); // Map error to servo angle range
        // myServo.write(servoAngle);
        
        // Read the error value
        int error = Serial2.parseInt(); // read the error value from OpenMV
        // pass the error value as the current error to the PID calculation
        int servoAngleChange = calculatePIDAngleChange(error); // Read the error value from OpenMV.

        // Adjust the servo angle
        int servoAngle = map(servoAngleChange, -90, 90, 0, 180); // map the angle change to the servo angle range
        myServo.write(servoAngle); // Adjust the motor speed based on the error.
        
        // Adjust motor speed based on error
        targetPWM = (abs(error) > threshold) ? 1550 : 1600;
        myMotor.writeMicroseconds(targetPWM);

        Serial.print("Error: ");
        Serial.println(error);
        Serial.print("Servo Angle: ");
        Serial.println(servoAngle);
        Serial.print("Motor PWM: ");
        Serial.println(targetPWM);
    }

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

  void get_RPS(){
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

}

  //Distance per 1 Axle Rotation: 0.127 meters

  //RPM -> M/S
  /*
  (X Rotation / 1 Minute) * (1 Minute / 60 Seconds) * (0.127 Meters / 1 Rotation) = Z m/s

  Z m/s * (60 s / minute) * (1 Rotation / 0.127 Meters) = X rpm

  RPM -> pwm number???
  */