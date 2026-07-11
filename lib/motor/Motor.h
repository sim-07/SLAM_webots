#ifndef MOTOR_H
#define MOTOR_H

#include <webots/Motor.hpp>

class Motor {

    private:
        webots::Motor* _webotsMotor = nullptr;
        int _currentPower;
        const double MAX_VELOCITY = 10.0;
        

    public:
        Motor() : _currentPower(0) {}
        
        void init(webots::Motor* webotsMotor);
        void setPower(int power);
        void motorStop();
        int getPower() { return _currentPower; }
};

#endif