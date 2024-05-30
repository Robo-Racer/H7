#include "ultrasonicsensor.h"

UltrasonicSensor::UltrasonicSensor(int rxPin, int txPin) : rxPin(rxPin), txPin(txPin), distance(0), duration(0) {}

void UltrasonicSensor::begin() {
    pinMode(rxPin, OUTPUT);
    pinMode(txPin, INPUT);
}

void UltrasonicSensor::sendPing() {
    digitalWrite(rxPin, LOW);
    delayMicroseconds(2);
    digitalWrite(rxPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(rxPin, LOW);
    duration = pulseIn(txPin, HIGH);
    distance = (duration / 2.0) * 0.0343; // Convert to cm
}

float UltrasonicSensor::getDistance() {
    sendPing();
    return distance;
}

void UltrasonicSensor::update() {
    distance = getDistance();
}
