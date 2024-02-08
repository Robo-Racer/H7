// ObstacleDetection.h
#ifndef ObstacleDetection_h
#define ObstacleDetection_h

#include <Arduino.h>
#include "UltrasonicSensors_JSN.h" 
#include "UltrasonicSensors_Taidacent.h" 

class ObstacleDetection {
  private:
    UltrasonicSensor_JSN *ultrasonicJSN;
    UltrasonicSensor_Taidacent *ultrasonicTaidacent;
    long thresholdDistance; //Threshold distance, if an obstacle is detected below this distance, it is considered that there is an obstacle

  public:
    // constuctor
    ObstacleDetection(UltrasonicSensor_JSN *jsnSensor, UltrasonicSensor_Taidacent *taidacentSensor, long threshold) {
      ultrasonicJSN = jsnSensor;
      ultrasonicTaidacent = taidacentSensor;
      thresholdDistance = threshold;
    }

    //Initialize sensor
    void begin() {
      ultrasonicJSN->begin();
      ultrasonicTaidacent->begin();
    }

    // Detect obstacles
    void detectObstacles() {
      long distanceJSN = ultrasonicJSN->readDistance();
      float distanceTaidacent = ultrasonicTaidacent->readDistance();

      if (distanceJSN <= thresholdDistance || distanceTaidacent <= thresholdDistance) {
        Serial.println("Obstacle Detected!");
      } else {
        Serial.println("No Obstacle Detected.");
      }
    }
};

#endif
