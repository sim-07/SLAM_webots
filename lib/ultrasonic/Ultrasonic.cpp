#include "Ultrasonic.h"
#include <iostream>

bool Ultrasonic::init(webots::DistanceSensor* webotsUS)
{
    std::cout << "Init ultrasonic in Webots" << std::endl;

    if (webotsUS == nullptr) {
        _status = false;
        return false;
    }

    _webotsUS = webotsUS;
    
    _webotsUS->enable(32); 
    
    _status = true;
    return true;
}

float Ultrasonic::getDistance()
{
    if (!_status || _webotsUS == nullptr) return -1.0f;

    double distanceInMeters = _webotsUS->getValue();
    float distanceInCm = static_cast<float>(distanceInMeters * 100.0);

    if (distanceInCm < MIN_DISTANCE) return static_cast<float>(MIN_DISTANCE);
    if (distanceInCm > MAX_DISTANCE) return static_cast<float>(MAX_DISTANCE);

    return distanceInCm;
}

bool Ultrasonic::test() 
{
    return _status;
}