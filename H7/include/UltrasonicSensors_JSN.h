// UltrasonicSensors_JSN.h

//For detecting any objects. blind zone distance of 3cm, super low, it has a detection range of 3cm-4.5m
//https://www.amazon.com/JSN-SR04T-Integrated-Transducer-Ultrasonic-Waterproof/dp/B0C73XZRFQ/ref=sr_1_5?crid=L86R36BB1MQ1&keywords=waterproof+ultrasonic+sensor&qid=1700536333&sprefix=waterproof+ultras%2Caps%2C166&sr=8-5


#ifndef ULTRASONICSENSORS_JSN_H
#define ULTRASONICSENSORS_JSN_H

class UltrasonicSensor_JSN {
public:
    UltrasonicSensor_JSN(int triggerPin, int echoPin); // Constructor
    void begin();                                  // Initialize sensor
    long readDistance();                           // Read distance from sensor

private:
    int _triggerPin;
    int _echoPin;
};

#endif
