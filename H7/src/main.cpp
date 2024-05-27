#include <Arduino.h>
#include <Servo.h>
#include "Portenta_H7_TimerInterrupt.h"
#include "PIDSpeedControl.h"
#include "PIDServoControl.h"
#include "ultrasonicsensor.h"

// Servo Globals
//const int servoPin = 164;     // Change this to the desired GPIO pin
// breakoutPin pin = GPIO_2;
const int motorLowSpeed = 1550; // Lowest speed forward
Servo myMotor;
Servo myServo;



// Define the pins for the ultrasonic sensor
#define TRIG_PIN 10//replace with the actual pin number on Portenta H7
#define ECHO_PIN 11//replace with the actual pin number on Portenta H7

// Define the other pins based on the connections specified
#define VCC_PIN 3.3V // Use 3.3V  pin on Portenta H7,replaces the actual pin number on Portenta H7
#define GND_PIN GND // Use GND pin on Portenta H7, replaces the actual pin number on Portenta H7
#define SERIAL_TX_PIN PA_2 // Replace with the actual TX pin number on Portenta H7 (e.g., Serial1 TX)
#define SERIAL_RX_PIN PA_3 // Replace with the actual RX pin number on Portenta H7 (e.g., Serial1 RX)
#define ANALOG_PIN A0 // Replace with the actual analog input pin number on Portenta H7

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

int espRx = LEDB + PA_10 + 1;
int espTx = LEDB + PA_9 + 1;

int motorPin = LEDB + PD_4 + 1;//GPIO 2?
int servoPin = LEDB + PE_3 + 1;//GPIO 4?
int hallPin = LEDB + PG_3 + 1;//GPIO 5
int stopPin1 = LEDB + PC_15 + 1;//GPIO1
int stopPin2 = LEDB + PG_10 + 1;//GPIO6
int distance = 0;


// Portenta_H7 OK       : TIM1, TIM4, TIM7, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17
Portenta_H7_Timer ITimer0(TIM15);

//interupt functions
void count_rotation();
void tetherStop();
void get_RPS();




//functions
int slow_start(float targetSpeed);
void speed_control(int speed);



// Instantiate the ultrasonic sensor
UltrasonicSensor ultrasonicSensor(TRIG_PIN, ECHO_PIN);



void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial1.begin(9600, SERIAL_8N1);
    
    pinMode(stopPin1, INPUT);
    pinMode(stopPin2, OUTPUT);
    digitalWrite (stopPin2, LOW);
    attachInterrupt(stopPin1, tetherStop, RISING);  //attaching the interrupt and declaring the variables, one of the interrupt pins on Nano is D2, and has to be declared as 0 here

    pinMode(hallPin, INPUT);
    attachInterrupt(hallPin, count_rotation, FALLING);  //attaching the interrupt and declaring the variables, one of the interrupt pins on Nano is D2, and has to be declared as 0 here

     // execute getRPS every 500ms
    if (ITimer0.attachInterruptInterval(500000, get_RPS))
    {
      Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(millis());
    }
    else
      Serial.println(F("Can't set ITimer0. Select another freq. or timer"));


    // Initialize the ultrasonic sensor
    ultrasonicSensor.init();
    // Set up Vcc and GND for the sensor
    //pinMode(VCC_PIN, OUTPUT);
   // digitalWrite(VCC_PIN, HIGH);
    // GND_PIN doesn't need pinMode, just ensure proper connection

    // Set up analog input for the sensor
    //pinMode(ANALOG_PIN, INPUT);

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

    myServo.attach(servoPin); // Attaches the servo on the specified pin to the Servo object
    Serial.println("Starting Neutral");
    myServo.write(90); // Neutral Starting signal
    delay(1000);

    myMotor.attach(motorPin); // Attaches the servo on the specified pin to the Servo object
    Serial.println("Starting Neutral");
    myMotor.writeMicroseconds(1500); // Neutral Starting signal
    delay(1000);

}

void loop() {
    bool running = true;
    targetSpeed = 5.0; // Set the target speed in m/s

    // Check for obstacles
    ultrasonicSensor.checkObstacle();

    if (stop) {
        myMotor.writeMicroseconds(1500); // Stop the motor
    } else {
        // Proceed with normal operations
        targetPWM -= 8;
        targetPWM = 1560;
        if (targetPWM < 1550) {
            targetPWM = 1550;
        }

        while (running && !stop) {
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
        }
    }
}

  //Serial.println(digitalRead(stopPin1));

  // ramp up the time that the ESC is on vs off (1/5 to 2/1) 
  /*for(int i = 4; i <= 4; i++){
    rotations = 0;
    rps = 0;
    Serial.print("speed control: ");
    Serial.println(i);
    speed_control(i);

    Serial.print("Rotations Per Second: ");
    Serial.println(rps);
    Serial.print("Speed m/s: ");
    Serial.println(speedMPS);
  }*/


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

void count_rotation() {
    rotations ++;
}

void get_RPS(){
  rps = rotations*2;
  speedMPS = rps*metersPerRotation;
  rotations = 0;

  if (speedMPS > targetSpeed && targetPWM > 1550)
  {
    targetPWM --;
  }
  else if (speedMPS < targetSpeed && targetPWM < 2000){
    targetPWM ++;
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