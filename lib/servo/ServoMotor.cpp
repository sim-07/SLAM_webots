#include "ServoMotor.h"
#include <cmath>
#include <algorithm>
#include <iostream>

void ServoMotor::init(webots::Motor* webotsServo) {
    std::cout << "Init servo in Webots" << std::endl;

    _webotsServo = webotsServo;

    if (_webotsServo != nullptr) {
        _webotsServo->setVelocity(VELOCITY);
        _webotsServo->setPosition(MIN_ANGLE);
    } else {
        std::cout << "Servo nullptr" << std::endl;
    }
}

void ServoMotor::moveToAngleFast(double angle) {
    if (_webotsServo == nullptr) return;

    angle = std::clamp(angle, MIN_ANGLE, MAX_ANGLE);
    
    _webotsServo->setPosition(angle);
    _currentAngle = angle;
}