#ifndef ROBOTMOVEMENTS_H
#define ROBOTMOVEMENTS_H

#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include "Encoder.h"
#include "Navigator.h"
#include "Motor.h"

#include "../../src/Common.h"

static const uint8_t MOTOR_POWER = 150;
static const uint8_t MIN_MOTOR_POWER = 30;
static const float TURN_TOLERANCE = 0.1;
static const float POWER_REDUCE_CONSTANT = 100;

// enum NavStatus {
//     SUCCESS = 0,
//     OBSTACLE = 1,
//     FAILED = 2
// };

enum RbState
{
    IDLE_RB = 0,
    MOVING_STRAIGHT = 1,
    TURNING = 2,
    FOLLOWING = 3,
    COMPLETED_ROUTE = 4,
    GOTO = 5,
    PROBLEM = 6
};

class RobotMovements
{
private:
    Navigator *_nav;

    Motor *_leftMotor;
    Motor *_rightMotor;

    Encoder *_leftEnc;
    Encoder *_rightEnc;

    RbState _currentState = IDLE_RB;

    Route _currentRoute;

    int _currIndexRoute = 0;

    Pos _nextCell;
    Pos _destination;

    float _avgStraight = 0;
    float _avgTurn = 0;
    float _targetDis = 0;
    double _targetRad = 0;

    int _encProblem = 0;

    double normAngle(double angle);

    void goTo();

    const float _wheelDistance = 0;

public:
    RobotMovements(double wheelDistance)
        : _wheelDistance(wheelDistance)
    {
    }

    void init(Navigator *n,
              Encoder *le,
              Encoder *re,
              Motor *lm,
              Motor *rm);
    void goStraight();
    void turn();
    void stop();
    void setCurrentState(RbState currState);
    RbState getCurrentState() { return _currentState; };
    void update();
    void setRoute(Route &route);
    void followPath();
    void setDestination(int16_t x, int16_t y);
};

#endif
