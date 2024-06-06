#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Servo.h>
#include <string>
#include "ColorSensors.h"
#include "PinDef.h"
#include "Portenta_H7_TimerInterrupt.h"
#include "PIDSpeedControl.h"
#include "PIDServoControl.h"
#include "ultrasonicsensor.h"
#include "mbed.h"
#include "NeoPixelSPI.h"
#include <iostream>
using namespace std;


// Motor Global Vars
const int motorLowSpeed = 1550; // Lowest speed forward
Servo myMotor;

// Servo Global Vars
Servo myServo;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, neoPin, NEO_GRB + NEO_KHZ800);


// RPM Globals
const float metersPerRotation = 0.126937324787;
volatile float targetSpeed = 0;
volatile float speedMPS = 0; 
volatile float rps = 0;
volatile int rotations = 0;
volatile int targetPWM = 1500;//neutral
volatile bool stop = false;

int distance = 0;
const int threshold = 10; // Example threshold value, adjust based on testing

bool setupError = false;
String errorMessage = " ";


// Portenta_H7 OK       : TIM1, TIM4, TIM7, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17
Portenta_H7_Timer ITimer0(TIM1);
UltrasonicSensor ultrasonicSensor(ultrasonicRx, ultrasonicTx);
// SPI pins for Portenta H7
SPI spi(PC_3, NC, PI_1); // MOSI, MISO (not used), SCLK
NeoPixelSPI neoPixelSpi(&spi, NUMPIXELS);
LED_CONFIG_S leds[NUMPIXELS];



//interupt functions
void count_rotation();
void tetherStop();
void get_speed();

//functions
void handle_openMV_input();
messageHeader serial_get_message();
void serial_send_message(messageHeader mHeader, dataHeader dHeader, String data);
int slow_start(float targetSpeed);



void setup() {
  Serial.begin(115200); // Setting up serial with computer for error messages
  Serial1.begin(115200); // UART for OpenMV communication
  Serial3.begin(9600, SERIAL_8N1); // Setting up UART with ESP32-S3
  delay(2000);


  // Initialize the NeoPixel library
  neoPixelSpi.setup();

  // Turn off all LEDs
  turnOffAllLEDs();
  delay(1000); // Delay 1 second

  // Turn the first strip red
  turnOnLEDs(0, 8, 255, 0, 0, 0);
  delay(2000); // Delay 2 seconds

  // Turn on the second light strip to red
  turnOnLEDs(8, 16, 255, 0, 0, 0);
  delay(3000); // Delay 3 seconds

  // Turn on the third light strip to green
  turnOnLEDs(16, 24, 0, 255, 0, 0);


  //Validate communication with ESP and OpenMV
  if(!Serial3){
    errorMessage += "ESP32 communication setup error\n";
    setupError = true;
    Serial.println("ESP32 communication setup error");
  }
  if(!Serial3){
    errorMessage += "OpenMV communication setup error\n";
    setupError = true;
    Serial.println("OpenMV communication setup error");
  }

  //setting up the emergency tether stop
  pinMode(stopPin1, INPUT);
  pinMode(stopPin2, OUTPUT);
  digitalWrite (stopPin2, LOW);
  attachInterrupt(stopPin1, tetherStop, RISING); 

  //setting up the hall effect sensor to count rotations
  pinMode(hallPin, INPUT);
  attachInterrupt(hallPin, count_rotation, FALLING);  

  //initialize all 6 of the color sensors
  initColorSensors();

  strip.begin(); // initialize the NeoPixel library
  colorLedOn(false, strip, NUMCOLORPIXELS); // initialize all pixels off

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
  targetSpeed = 4.5;//set the target speed in m/s

  // wait for ESP32's start message.
  Serial.println("Waiting for the ESP32 to start.");
  /*while (waitingForEsp){

    if(Serial3.available() > 0){
      recievedMessageType = serial_get_message();
      Serial.print("Message type: ");
      Serial.println(recievedMessageType);
      delay(100);
      
      if(setupError == false){
        if(recievedMessageType == START){
          waitingForEsp = false;
        } else if(recievedMessageType == READYTOSTART){
          Serial.println("Got ready to start.");
          serial_send_message(READYTOSTART, DATA_ERR, message);
        }
      } else{
        Serial.println("Robot setup error please restart");
        serial_send_message(DATA, DATA_ERR, errorMessage);
      }
    }

  }*/
  Serial.println("ESP32 start confirmed");
  colorLedOn(true, strip, NUMCOLORPIXELS);
  delay(500);
  setLineCalibration();
  
  //targetSpeed = 4.5;//set the target speed in m/s
  targetPWM = 1500;//slow_start(targetSpeed);
  targetPWM -= 8;
  if(targetPWM < 1550){
    targetPWM = 1550;
  }

  while (running && !stop) {
      int16_t colorError = calculatePIDAngleChange();
      //handle_openMV_input();
      // Update and check distance from ultrasonic sensor
      /*ultrasonicSensor.update();
      float distance = ultrasonicSensor.getDistance();
      if (distance < 20) { // If an object is detected within 20 cm
          Serial.println("Object detected");
          myMotor.writeMicroseconds(1500); // Immediately stop the motor
          stop = true;
      }
      //myMotor.writeMicroseconds(targetPWM);
      Serial.print("PWM: ");
      Serial.println(targetPWM);
      Serial.print("Rotations Per Second: ");
      Serial.println(rps);
      Serial.print("Speed m/s: ");
      Serial.println(speedMPS);*/
      if(Serial3.available() > 0){
        recievedMessageType = serial_get_message();
        Serial.print("Message type: ");
        Serial.println(recievedMessageType);
      }
  }

  myMotor.writeMicroseconds(1500);

}

// Function to handle OpenMV input
void handle_openMV_input() {
    if (Serial1.available() > 0) {
        // int error = Serial2.parseInt(); // Read the error value from OpenMV
        // int servoAngle = map(error, -45, 45, 0, 180); // Map error to servo angle range
        // myServo.write(servoAngle);
        
        // Read the error value
        int error = Serial1.parseInt(); // read the error value from OpenMV
        // pass the error value as the current error to the PID calculation
        if(error > 90 && error <= 180){
          int anglediff = error - 135;
          error = 135 - anglediff;
        } else{
          int anglediff = error - 45;
          error = 45 - anglediff;
        }
        error = error-90;
        int servoAngleChange = calculatePIDAngleChange(error); // Read the error value from OpenMV.
        // Adjust the servo angle
        int servoAngle = map(servoAngleChange, -90, 90, 180, 0); // map the angle change to the servo angle range
        myServo.write(servoAngle); // Adjust the motor speed based on the error.

        Serial.println("Set Angle:");
        Serial.println(servoAngle);
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


void process_data(){
  String headerStr;
  String recievedMessage;
  dataHeader recievedHeader;

  headerStr = Serial3.readStringUntil(',');
  recievedHeader = (dataHeader)(headerStr.toInt());
  
  switch(recievedHeader){
    case SPEED:
      recievedMessage = Serial3.readStringUntil('\n'); //clear the buffer
      break;

    case DISTANCE:
      recievedMessage = Serial3.readStringUntil('\n'); //clear the buffer
      break;

    default:
      break;
    
  }

}


messageHeader serial_get_message(){

  String recievedMessage;
  String headerStr;
  messageHeader recievedHeader;

  while(Serial3.available() > 0){
    if(Serial3.available() > 0){
      headerStr = Serial3.readStringUntil(',');
    }
    
    recievedHeader = (messageHeader)(headerStr.toInt());

    switch(recievedHeader){
      case COMM_ERR:
        recievedMessage = Serial3.readStringUntil('\n'); //clear the buffer
        break;

      case STOP:
        stop = true;
        recievedMessage = Serial3.readStringUntil('\n'); //clear the buffer
        break;

      case START:
        stop = false;
        recievedMessage = Serial3.readStringUntil('\n'); //clear the buffer
        break;

      case READYTOSTART:
        recievedMessage = Serial3.readStringUntil('\n'); //clear the buffer
        break;

      case DATA:
        process_data();
        break;

      default:
        recievedMessage = Serial3.readStringUntil('\n'); //clear the buffer
        break;
    }

  }

  return recievedHeader;
}

void serial_send_message(messageHeader mHeader, dataHeader dHeader, String data){
  String SerMessage = "";
  switch(mHeader){
    case COMM_ERR:
      Serial3.print(mHeader);
      Serial3.print(" ");
      Serial3.println(data);
      break;

    case DATA:
      Serial3.print(mHeader);
      Serial3.print(" ");
      Serial3.print(dHeader);
      Serial3.print(" ");
      Serial3.println(data);
      break;

    default:
      SerMessage = mHeader + ",";
      Serial3.print(mHeader);
      Serial3.println(",");
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


void turnOnLEDs(int start, int end, byte red, byte green, byte blue, byte white) {
  for (int i = start; i < end; i++) {
    leds[i].red = red;
    leds[i].green = green;
    leds[i].blue = blue;
    leds[i].white = white;
  }
  neoPixelSpi.transfer(leds, NUMPIXELS);
}

void turnOffAllLEDs() {
  for (int i = 0; i < NUMPIXELS; i++) {
    leds[i].red = 0;
    leds[i].green = 0;
    leds[i].blue = 0;
    leds[i].white = 0;
  }
  neoPixelSpi.transfer(leds, NUMPIXELS);
}
