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

double Ultrasonic::getDistance()
{
    if (!_status || _webotsUS == nullptr) return -1.0f;

    double distance = _webotsUS->getValue() * 100.0;

    if (distance <= MIN_DISTANCE || distance >= MAX_DISTANCE) return 0.0f;

    return distance;
}

bool Ultrasonic::test() 
{
    return _status;
}