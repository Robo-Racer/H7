// UltrasonicSensors_2.h

//blind zone distance of 20cm, with a working range of 25cm-4m, however it is smaller and is similar to what you find in most cars.
//https://www.amazon.com/Taidacent-Waterproof-Ultra-Precision-Ultrasonic-Detection/dp/B082D96911/ref=sr_1_4?crid=L86R36BB1MQ1&keywords=waterproof%2Bultrasonic%2Bsensor&qid=1700536405&sprefix=waterproof%2Bultras%2Caps%2C166&sr=8-4&th=1


#ifndef ULTRASONICSENSORS_2_H
#define ULTRASONICSENSORS_2_H

class UltrasonicSensor_2 {
public:
    UltrasonicSensor_2(int triggerPin, int pwmPin); // Constructor
    void begin();                                   // Initialize sensor
    float readDistance();                           // Read distance from sensor

private:
    int _triggerPin;
    int _pwmPin;
};

#endif
