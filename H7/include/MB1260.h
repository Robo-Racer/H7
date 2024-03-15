#ifndef MB1260_H
#define MB1260_H

#include <Arduino.h>

class MB1260 {
  public:
    MB1260(int triggerPin, int echoPin); // Constructor
    void begin(); // Initialize sensor
    long readDistance(); // Read distance

  private:
    int _triggerPin, _echoPin; // Trigger and echo pins
};

#endif
