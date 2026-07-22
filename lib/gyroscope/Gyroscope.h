#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <webots/Gyro.hpp>
#include <cmath>

class Gyroscope
{
private:
    webots::Gyro *_webotsGyro;
    int _timeStep;
    double _currAngle = 0.0;

    double normAngle(double angle);

public:    
    void init(webots::Gyro *gyro);
    void update(int timeStep);
    double getCurrAngle();
};

#endif