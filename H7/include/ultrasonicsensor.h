#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(int trigPin, int echoPin);
    void init();
    float getDistance();
    void checkObstacle();

private:
    int trigPin;
    int echoPin;
    float distance;
};

#endif // ULTRASONICSENSOR_H
