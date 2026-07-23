#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <webots/Motor.hpp>
#include <cstdint>

class ServoMotor
{
private:
    webots::Motor *_webotsServo = nullptr;
    double _currentAngle = MIN_ANGLE;

public:
    ServoMotor() : _currentAngle(1.4835) {}

    void init(webots::Motor *webotsServo);

    void moveToAngleFast(double angle);
    double getAngle() const { return _currentAngle; }

    static constexpr double MIN_ANGLE = -1.4835; // -85 gradi
    static constexpr double MAX_ANGLE = 1.4835;

    static constexpr double VELOCITY = 1.5;
};

#endif