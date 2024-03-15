// UltrasonicSensors_2.cpp

//blind zone distance of 20cm, with a working range of 25cm-4m, however it is smaller and is similar to what you find in most cars.


#include "UltrasonicSensors_Taidacent.h"
#include <Arduino.h>

UltrasonicSensor_Taidacent::UltrasonicSensor_Taidacent(int triggerPin, int pwmPin) {
    _triggerPin = triggerPin;
    _pwmPin = pwmPin;
}

void UltrasonicSensor_Taidacent::begin() {
    pinMode(_triggerPin, OUTPUT);
    pinMode(_pwmPin, INPUT);
}

float UltrasonicSensor_Taidacent::readDistance() {
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
