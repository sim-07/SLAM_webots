#include "LaserSensor.h"
#include <iostream>

bool LaserSensor::init(webots::DistanceSensor* webotsSensor) {
    std::cout << "Init laser in Webots" << std::endl;

    if (webotsSensor == nullptr) {
        _status = false;
        return false;
    }

    _webotsSensor = webotsSensor;
    
    _webotsSensor->enable(32); 
    
    _status = true;
    return true;
}

double LaserSensor::getDistance() {
    if (!_status || _webotsSensor == nullptr) return -1.0f;

    double distance = _webotsSensor->getValue() * 100.0;

    if (distance <= MIN_DISTANCE || distance >= MAX_DISTANCE) return 0.0f;

    return distance;
}

bool LaserSensor::isReady() {
    return _status;
}