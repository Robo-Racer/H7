#include "MB1260.h"

MB1260::MB1260(int triggerPin, int echoPin) {
    _triggerPin = triggerPin;
    _echoPin = echoPin;
}

void MB1260::begin() {
    pinMode(_triggerPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

long MB1260::readDistance() {
    // Ensure the trigger pin is low for a clean high pulse
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);
    
    // Trigger the measurement
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10); // 10 microseconds high pulse
    digitalWrite(_triggerPin, LOW);
    
    // Measure the length of the echo signal
    long duration = pulseIn(_echoPin, HIGH);
    
    // Calculate the distance based on the duration and the speed of sound.
    // The speed of sound is approximately 344 meters per second or 0.034 cm/μs at room temperature.
    // Since the sound has to travel to the object and back, we divide by 2.
    long distance = duration * 0.034 / 2;
    
    return distance;
}
