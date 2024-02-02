// UltrasonicSensors_1.cpp
#include "UltrasonicSensors_JSN.h"
#include <Arduino.h>


//For detecting any objects. blind zone distance of 3cm, super low, it has a detection range of 3cm-4.5m
//https://www.amazon.com/JSN-SR04T-Integrated-Transducer-Ultrasonic-Waterproof/dp/B0C73XZRFQ/ref=sr_1_5?crid=L86R36BB1MQ1&keywords=waterproof+ultrasonic+sensor&qid=1700536333&sprefix=waterproof+ultras%2Caps%2C166&sr=8-5

UltrasonicSensor_JSN::UltrasonicSensor_JSN(int triggerPin, int echoPin) {
    _triggerPin = triggerPin;
    _echoPin = echoPin;
}

void UltrasonicSensor_JSN::begin() {
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

long UltrasonicSensor_JSN::readDistance() {
    // Clear the trigger pin
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);
    
    // Sets the trigger pin to HIGH state for 10 ms
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_triggerPin, LOW);
    
    // Reads the echo pin, returns the sound wave travel time in ms
    long duration = pulseIn(_echoPin, HIGH);//Record the round-trip time of the sound wave in ms
    // Calculate distance
    // 0.034 is the speed of sound (in cm/ms) divided by 2 (the forward and return sound waves)
    // At room temperature, the speed of sound is about 348 m/s, which is 0.034 cm/us when converted into cm/ms
    // Calculating the distance
    long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    return distance;
}
