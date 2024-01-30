// UltrasonicSensors_1.cpp
#include "UltrasonicSensors_1.h"
#include <Arduino.h>


//For detecting any objects. blind zone distance of 3cm, super low, it has a detection range of 3cm-4.5m
//https://www.amazon.com/JSN-SR04T-Integrated-Transducer-Ultrasonic-Waterproof/dp/B0C73XZRFQ/ref=sr_1_5?crid=L86R36BB1MQ1&keywords=waterproof+ultrasonic+sensor&qid=1700536333&sprefix=waterproof+ultras%2Caps%2C166&sr=8-5

UltrasonicSensor::UltrasonicSensor(int triggerPin, int echoPin) {
    _triggerPin = triggerPin;
    _echoPin = echoPin;
}

void UltrasonicSensor::begin() {
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

long UltrasonicSensor::readDistance() {
    // Clear the trigger pin
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);
    
    // Sets the trigger pin to HIGH state for 10 microseconds
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_triggerPin, LOW);
    
    // Reads the echo pin, returns the sound wave travel time in microseconds
    long duration = pulseIn(_echoPin, HIGH);
    
    // Calculating the distance
    long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    return distance;
}
