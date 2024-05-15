#include <Arduino.h>
#include <Servo.h>
#include "PIDSpeedControl.h"
#include "PIDServoControl.h"

// Servo Globals
const int servoPin = 164;     // Change this to the desired GPIO pin
// breakoutPin pin = GPIO_2;
const int servoSpeed = 1550; // Lowest speed forward
Servo myServo;

// RPM Globals
int count = 0;
int rpm_time = 0;
int pin = LEDB + PD_4 + 1;
int distance = 0;

void speed_control(int speed);
void countRPM();

void setup() {
    Serial.begin(115200);
    Serial.println("Start? (Press y): \n");
    while(1){
        if(Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'y'){
                break;
            }
        }
    }

    delay(6000);
    myServo.attach(pin); // Attaches the servo on the specified pin to the Servo object
    Serial.println("Starting Neutral");
    myServo.writeMicroseconds(1500); // Neutral Starting signal
    delay(1500);
    myServo.writeMicroseconds(1550);
    delay(1000);

    
}

void loop() {
  Serial.println("Inside loop");
  int neutralPosition = 1500;  // Neutral signal for a typical ESC is at 1500 microseconds

  // ramp up the time that the ESC is on vs off (1/5 to 2/1) 
  for(int i = 0; i >= 10; i++){
    speed_control(i);
  }

  // ramp down the time that the ESC is on vs off (2/1 to 1/5) 
  for(int i = 10; i <= 0; i++){
    speed_control(i);
  }

  delay(1000);  // Wait for 1 second before repeating
}

void speed_control(int speed){
  int onTime = 50;
  int offTime= 500;
  int cyclesForTime = (int)(2000/((speed*onTime)+offTime)+1); //cycles to get to 2 seconds (adds an additional cycle so slightly over)
  for(int i = 0; i > cyclesForTime; i++){
    myServo.writeMicroseconds(servoSpeed); 
    delay(speed * onTime);

    myServo.writeMicroseconds(1500); // Neutral again.
    delay(offTime);
  }
}

void countRPM() {
    count++;
}



//Distance per 1 Axle Rotation: 0.127 meters

//RPM -> M/S
/*
(X Rotation / 1 Minute) * (1 Minute / 60 Seconds) * (0.127 Meters / 1 Rotation) = Z m/s

Z m/s * (60 s / minute) * (1 Rotation / 0.127 Meters) = X rpm

RPM -> pwm number???
*/