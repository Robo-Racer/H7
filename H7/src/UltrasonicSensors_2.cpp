// UltrasonicSensors_2.cpp

//blind zone distance of 20cm, with a working range of 25cm-4m, however it is smaller and is similar to what you find in most cars.
//https://www.amazon.com/Taidacent-Waterproof-Ultra-Precision-Ultrasonic-Detection/dp/B082D96911/ref=sr_1_4?crid=L86R36BB1MQ1&keywords=waterproof%2Bultrasonic%2Bsensor&qid=1700536405&sprefix=waterproof%2Bultras%2Caps%2C166&sr=8-4&th=1


#include "UltrasonicSensors_2.h"
#include <Arduino.h>

UltrasonicSensor_2::UltrasonicSensor_2(int triggerPin, int pwmPin) {
    _triggerPin = triggerPin;
    _pwmPin = pwmPin;
}

void UltrasonicSensor_2::begin() {
    pinMode(_triggerPin, OUTPUT);
    pinMode(_pwmPin, INPUT);
}

float UltrasonicSensor_2::readDistance() {
    // Trigger the sensor
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_triggerPin, LOW);

    // Read the pulse width to measure distance
    unsigned long pulseWidth = pulseIn(_pwmPin, HIGH);

    // Calculate the distance based on the pulse width
    // At room temperature, the speed of sound is approximately 348 m/s
    // Hence, the distance is pulseWidth (in microseconds) divided by 57.5 to get centimeters
    float distance = pulseWidth / 57.5;

    return distance;
}
