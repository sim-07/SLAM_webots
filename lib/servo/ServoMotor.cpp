#include "ServoMotor.h"
#include <cmath>
#include <algorithm>
#include <iostream>

void ServoMotor::init(webots::Motor* webotsServo) {
    std::cout << "Init servo in Webots" << std::endl;

    _webotsServo = webotsServo;
    _currentAngle = MIN_ANGLE;

    if (_webotsServo != nullptr) {
        _webotsServo->setVelocity(1.5);
        
        _webotsServo->setPosition(MIN_ANGLE);
    }
}

void ServoMotor::moveToAngleFast(double angle) {
    if (_webotsServo == nullptr) return;

    angle = std::clamp(angle, MIN_ANGLE, MAX_ANGLE);
    _currentAngle = angle;

    _webotsServo->setPosition(angle);
}