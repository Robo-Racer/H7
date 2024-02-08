#include <Arduino.h>
#include "UltrasonicSensors_JSN.h" // JSN-SR04T sensor
#include "UltrasonicSensors_Taidacent.h" // Taidacent sensor
#include "ObstacleDetection.h" //obstacle detection class
#include "ObstacleAvoidance.h"


//https://docs.arduino.cc/resources/pinouts/ABX00042-full-pinout.pdf


// Pins for the JSN-SR04T sensor
const int triggerPinJSN = D0;// Use D0 directly because the Arduino IDE defines the pin number
const int echoPinJSN = D1;//Use D1 as echo pin

// Pins for the Taidacent sensor
const int triggerPinTaidacent = D2;
const int pwmPinTaidacent = D3;

// Create instances of the sensor classes
UltrasonicSensor_JSN ultrasonicJSN(triggerPinJSN, echoPinJSN);
UltrasonicSensor_Taidacent ultrasonicTaidacent(triggerPinTaidacent, pwmPinTaidacent);

ObstacleDetection obstacleDetection(&ultrasonicJSN, &ultrasonicTaidacent, 50); // 假设阈值为50cm


//Define motor control pins
int motorPin1 = 5; // Example pin number
int motorPin2 = 6; // Example pin number

// Instantiate the obstacle avoidance class
ObstacleAvoidance obstacleAvoidance(&obstacleDetection, motorPin1, motorPin2);

void setup() {
    Serial.begin(9600);
    ultrasonicJSN.begin();            // Initialize JSN-SR04T sensor
    ultrasonicTaidacent.begin();      // Initialize Taidacent sensor
    obstacleDetection.begin(); //Initialize obstacle detection
}

void loop() {
    long distanceJSN = ultrasonicJSN.readDistance();
    float distanceTaidacent = ultrasonicTaidacent.readDistance();

    Serial.print("JSN-SR04T Distance: ");
    Serial.print(distanceJSN);
    Serial.println(" cm");

    Serial.print("Taidacent Distance: ");
    Serial.print(distanceTaidacent);
    Serial.println(" cm");

    // Add obstacle avoidance decision logic based on sensor readings here
    obstacleDetection.detectObstacles(); // Detect obstacles and handle them
    delay(100); // Delay a bit to avoid spamming the serial output
}
