#include "Motor.h"

void Motor::init(webots::Motor* webotsMotor) {
    _webotsMotor = webotsMotor;
}

void Motor::setPower(int power) {
    _currentPower = power;
    if (_webotsMotor == nullptr) return;

    double velocity = (static_cast<double>(power) / 255.0) * MAX_VELOCITY;

    _webotsMotor->setVelocity(velocity);
}

void Motor::motorStop() {
    _currentPower = 0;
    if (_webotsMotor != nullptr) {
        _webotsMotor->setVelocity(0.0);
    }
}