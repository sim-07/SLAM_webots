#include "Gyroscope.h"

void Gyroscope::init(webots::Gyro *gyro) {
    _webotsGyro = gyro;
}

void Gyroscope::update(int timeStep) {
    _timeStep = timeStep;

    if (_webotsGyro) {
        double z = _webotsGyro->getValues()[2];

        _currAngle += z * (_timeStep / 1000.0);
        _currAngle = normAngle(_currAngle);
    }
}

double Gyroscope::normAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double Gyroscope::getCurrAngle() {
    return _currAngle;
}