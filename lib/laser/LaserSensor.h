#ifndef LASERSENSOR_H
#define LASERSENSOR_H

#include <webots/DistanceSensor.hpp>

class LaserSensor
{
private:
    webots::DistanceSensor *_webotsSensor = nullptr;
    bool _status;

    static const int MIN_DISTANCE = 2;
    static const int MAX_DISTANCE = 400;

public:
    LaserSensor() : _status(false) {}

    bool init(webots::DistanceSensor *webotsSensor);

    double getDistance();
    bool isReady();
};

#endif