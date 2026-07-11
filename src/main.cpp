#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>

#include <RobotMovements.h>
#include <Explorer.h>
#include <Navigator.h>
#include <Connection.h>

#include "Common.h"

int main(int argc, char **argv)
{

    webots::Robot *robot = new webots::Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    RobotMovements rb;
    Navigator nav;
    Explorer explorer;
    Connection conn;
    ServoMotor servo;
    LaserSensor ls;
    Ultrasonic ultrasonic;

    Encoder leftEnc;
    Encoder rightEnc;

    Motor leftMotor;
    Motor rightMotor;

    webots::Motor *leftMotorWebots = robot->getMotor("left_wheel_motor");
    webots::Motor *rightMotorWebots = robot->getMotor("right_wheel_motor");

    leftMotorWebots->setPosition(INFINITY);
    rightMotorWebots->setPosition(INFINITY);
    leftMotorWebots->setVelocity(0.0);
    rightMotorWebots->setVelocity(0.0);

    leftMotor.init(leftMotorWebots);
    rightMotor.init(rightMotorWebots);

    webots::PositionSensor *leftEncoderWebots = robot->getPositionSensor("left_wheel_sensor");
    webots::PositionSensor *rightEncoderWebots = robot->getPositionSensor("right_wheel_sensor");

    if (leftEncoderWebots)
        leftEncoderWebots->enable(timeStep);
    if (rightEncoderWebots)
        rightEncoderWebots->enable(timeStep);

    leftEnc.init(leftEncoderWebots);
    rightEnc.init(rightEncoderWebots);

    webots::DistanceSensor *webotsLaser = robot->getDistanceSensor("laser_sensor");
    webots::DistanceSensor *webotsUS = robot->getDistanceSensor("ultrasonic_sensor");

    webots::Motor *webotsServo = robot->getMotor("servo_motor");

    

    bool isLsOk = ls.init(webotsLaser);

    if (!isLsOk) {
        std::cout << "Problem with laser" << std::endl;
    }

    bool isUltrasonicOk = ultrasonic.init(webotsUS);
    if (!isUltrasonicOk) {
        std::cout << "Problem with ultrasonic" << std::endl;
    }

    servo.init(webotsServo);

    rb.init(&nav, &leftEnc, &rightEnc, &leftMotor, &rightMotor);

    explorer.init(&nav, &rb, &servo, &ls, &ultrasonic);

    conn.init(nav, explorer, rb);

    while (robot->step(timeStep) != -1)
    {
        explorer.update();
        rb.update();
    }

    delete robot;
    return 0;
}

