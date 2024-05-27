#include "ultrasonicsensor.h"

// Define a threshold distance in cm for obstacle detection
#define OBSTACLE_DISTANCE_THRESHOLD 20

extern Servo myMotor;
extern int stopPin2;
extern volatile bool stop;

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
    this->distance = 0.0;
}

void UltrasonicSensor::init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float UltrasonicSensor::getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    distance = (duration * 0.034) / 2;
    return distance;
}

void UltrasonicSensor::checkObstacle() {
    distance = getDistance();
    if (distance <= OBSTACLE_DISTANCE_THRESHOLD) {
        Serial.println("Obstacle detected! Stopping the car.");
        stop = true;
        myMotor.writeMicroseconds(1500); // Neutral signal to stop the motor
        digitalWrite(stopPin2, HIGH); // Additional stop signal if needed
    } else {
        stop = false;
        digitalWrite(stopPin2, LOW); // Clear stop signal if obstacle is removed
    }
}
