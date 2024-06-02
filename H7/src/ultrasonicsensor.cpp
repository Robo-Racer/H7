#include "ultrasonicsensor.h"

// Constructor for the UltrasonicSensor class.
// Initializes the class with transmit (txPin) and receive (rxPin) pins.
UltrasonicSensor::UltrasonicSensor(int rxPin, int txPin) : rxPin(rxPin), txPin(txPin), distance(0), duration(0) {}

// Sets up the pin modes for the ultrasonic sensor: rxPin as OUTPUT, txPin as INPUT.
void UltrasonicSensor::begin() {
    pinMode(rxPin, OUTPUT);
    pinMode(txPin, INPUT);
}

// Sends an ultrasonic ping by toggling the rxPin.
// Measures the duration for which the ping signal is high, and calculates the distance.
void UltrasonicSensor::sendPing() {
    digitalWrite(rxPin, LOW);  // Ensure the pin is low before starting a new pulse
    delayMicroseconds(2);      // Wait for 2 microseconds
    digitalWrite(rxPin, HIGH); // Send a high pulse
    delayMicroseconds(10);     // Wait for 10 microseconds
    digitalWrite(rxPin, LOW);  // End the pulse
    duration = pulseIn(txPin, HIGH);  // Measure the time for echo to return

    // Calculate distance: (duration/2) * speed of sound in air (0.0343 cm/us)
    // Distance = (Time in microseconds / 2) * (Speed of sound in air (34300 cm/s) / 1000000 us/s)
    distance = (duration / 2.0) * 0.0343; // Calculate distance in centimeters based on the time measured
}

// Returns the distance measured by the sensor.
// It triggers a new measurement each time it's called.
float UltrasonicSensor::getDistance() {
    sendPing();  // Send a new ping to measure distance
    return distance;  // Return the measured distance
}

// Updates the distance measurement by performing a new distance measurement.
void UltrasonicSensor::update() {
    distance = getDistance();  // Store the newly measured distance
}
