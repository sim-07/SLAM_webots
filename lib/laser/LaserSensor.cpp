#include "LaserSensor.h"
#include <iostream>

bool LaserSensor::init(webots::DistanceSensor* webotsSensor) {
    std::cout << "Init laser in Webots" << std::endl;

    if (webotsSensor == nullptr) {
        _status = false;
        return false;
    }

    _webotsSensor = webotsSensor;
    
    _webotsSensor->enable(50); 
    
    _status = true;
    return true;
}

float LaserSensor::getDistance() {
    if (!_status || _webotsSensor == nullptr) return -1.0f;

    double distanceInMeters = _webotsSensor->getValue();

    return static_cast<float>(distanceInMeters * 100.0);
}

bool LaserSensor::isReady() {
    return _status;
}