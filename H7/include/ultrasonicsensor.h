#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(int rxPin, int txPin);
    void begin();
    float getDistance(); // Returns the distance in cm
    void update(); // Update sensor reading

private:
    int rxPin;
    int txPin;
    float distance;
    long duration;
    void sendPing();
};

#endif // ULTRASONIC_SENSOR_H
