#ifndef LASERSENSOR_H
#define LASERSENSOR_H

#include <webots/DistanceSensor.hpp>

class LaserSensor
{
private:
    webots::DistanceSensor* _webotsSensor = nullptr;
    bool _status;

public:
    LaserSensor() : _status(false) {}

    bool init(webots::DistanceSensor* webotsSensor);
    
    float getDistance();
    bool isReady();
};

#endif