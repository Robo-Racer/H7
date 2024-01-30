#include <Arduino.h>
#include "UltrasonicSensors_1.h" // JSN-SR04T sensor
#include "UltrasonicSensors_2.h" // Taidacent sensor

// Pins for the JSN-SR04T sensor
const int triggerPinJSN = 7;
const int echoPinJSN = 6;

// Pins for the Taidacent sensor
const int triggerPinTaidacent = 9;
const int pwmPinTaidacent = 8;

// Create instances of the sensor classes
UltrasonicSensor ultrasonicJSN(triggerPinJSN, echoPinJSN);
UltrasonicSensor_2 ultrasonicTaidacent(triggerPinTaidacent, pwmPinTaidacent);

void setup() {
    Serial.begin(9600);
    ultrasonicJSN.begin();            // Initialize JSN-SR04T sensor
    ultrasonicTaidacent.begin();      // Initialize Taidacent sensor
}

void loop() {
    long distanceJSN = ultrasonicJSN.readDistance();
    float distanceTaidacent = ultrasonicTaidacent.readDistance();

    Serial.print("JSN-SR04T Distance: ");
    Serial.print(distanceJSN);
    Serial.println(" cm");

    Serial.print("Taidacent Distance: ");
    Serial.print(distanceTaidacent);
    Serial.println(" cm");

    // Add obstacle avoidance decision logic based on sensor readings here

    delay(100); // Delay a bit to avoid spamming the serial output
}
