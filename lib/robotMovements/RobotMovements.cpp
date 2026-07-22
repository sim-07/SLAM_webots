#include <cmath>

#include "RobotMovements.h"
#include <iostream>

void RobotMovements::init(Navigator *n,
                          Encoder *le,
                          Encoder *re,
                          Motor *lm,
                          Motor *rm)
{
    std::cout << "Init rb in Webots" << std::endl;

    _nav = n;

    _leftMotor = lm;
    _rightMotor = rm;

    _leftEnc = le;
    _rightEnc = re;

    stop();
}

const float Kp = 10; // da calibrare
const int UNIT = 10; // dimensioni cella in cm

void RobotMovements::update()
{
    switch (_currentState)
    {
    case GOTO:
        goTo();
        break;

    case FOLLOWING:
        followPath();
        break;

    case MOVING_STRAIGHT:
        goStraight();
        break;

    case TURNING:
        turn();
        break;

    case COMPLETED_ROUTE:
        stop();
        _currIndexRoute = 0;
        _leftEnc->reset();
        _rightEnc->reset();
        _avgTurn = 0;
        break;

    default:
        break;
    }
}

void RobotMovements::goTo()
{
    Route r = _nav->calcRoute({_destination.x, _destination.y});

    if (r.numSteps > 0)
    {
        setCurrentState(FOLLOWING);
        _currentRoute = r;
    } else {
        setCurrentState(COMPLETED_ROUTE);
    }
}

void RobotMovements::setDestination(int16_t x, int16_t y)
{
    std::cout << "Destination rb: " << x << ":" << y << std::endl;
    _destination = {x, y};
}

void RobotMovements::setRoute(Route &route)
{
    _currentRoute = route;
    _currIndexRoute = 0;
}

void RobotMovements::setCurrentState(RbState currState)
{
    std::cout << "Current state rb: " << currState << std::endl;
    _currentState = currState;
}

void RobotMovements::goStraight()
{

    float dL = abs(_leftEnc->getCurrDistance());
    float dR = abs(_rightEnc->getCurrDistance());

    _avgStraight = (dL + dR) / 2.0;

    if (_avgStraight < abs(_targetDis))
    {

        std::cout << "Moving forward in rb: _avgStraight: " << _avgStraight << " abs(_targetDis): " << abs(_targetDis) << std::endl;

        float error = dL - dR;

        if (std::abs(error) > 1.1)
        {
            std::cout << "Correcting error in goStraight(), error: " << error << std::endl;

            float corr = error * Kp;

            _leftMotor->setPower(MOTOR_POWER - corr);
            _rightMotor->setPower(MOTOR_POWER + corr);
        }
        else
        {
            _leftMotor->setPower(MOTOR_POWER);
            _rightMotor->setPower(MOTOR_POWER);
        }
    }
    else
    {
        std::cout << "Finished moving forward" << std::endl;
        stop();
        _leftEnc->reset();
        _rightEnc->reset();
        // Pos newPos = _nav->calcFinalCell(_nav->getPos(), _nav->getDir(), _avgStraight);
        // _nav->setCurrPos(newPos);
        _nav->setCurrPos({_currentRoute.route[_currIndexRoute].x, _currentRoute.route[_currIndexRoute].y});
        _targetDis = 0;
        _avgStraight = 0;
        setCurrentState(FOLLOWING);
    }
}

void RobotMovements::stop()
{
    _leftMotor->motorStop();
    _rightMotor->motorStop();
}

void RobotMovements::turn()
{

    double turnDis = std::abs((_wheelDistance * _targetRad) / 2.0f); // totale distanza che le ruote devono percorrere
    int dir = (_targetRad > 0) ? -1 : 1;

    float abs_dis_l = std::abs(_leftEnc->getCurrDistance());
    float abs_dis_r = std::abs(_rightEnc->getCurrDistance());

    _avgTurn = (abs_dis_l + abs_dis_r) / 2;

    if (turnDis - _avgTurn > TURN_TOLERANCE)
    {
        float powerReducer = POWER_REDUCE_CONSTANT / (turnDis - _avgTurn);
        float motorPower = MOTOR_POWER - powerReducer > MIN_MOTOR_POWER ? MOTOR_POWER - powerReducer : MIN_MOTOR_POWER;

        std::cout << "Turning in rb: _avgTurn: " << _avgTurn << " | turnDis: " << turnDis << " | motorPower: " << motorPower << std::endl;

        float error = abs_dis_l - abs_dis_r;

        if (std::abs(error) > 0.1)
        {
            std::cout << "Correcting error in turn(), error: " << error << std::endl;

            float corr = error * Kp;

            _leftMotor->setPower((motorPower - corr) * dir);
            _rightMotor->setPower((motorPower + corr) * -dir);
        }
        else
        {
            _leftMotor->setPower(motorPower * dir);
            _rightMotor->setPower(motorPower * -dir);
        }
    }
    else
    {
        stop();
        _leftEnc->reset();
        _rightEnc->reset();
        double realAngle = normAngle(_nav->getDir() + (_avgTurn / (_wheelDistance / 2.0f)) * -dir);
        _nav->setDir(realAngle);
        std::cout << "Finished turning, _avgTurn: " << _avgTurn << std::endl;
        _targetRad = 0;
        _avgTurn = 0;
        setCurrentState(FOLLOWING);
    }

    if (_leftEnc->getCurrDistance() == 0 || _rightEnc->getCurrDistance() == 0)
    {
        _encProblem++;
    }

    if (_encProblem > 3)
    {
        std::cout << "Enc problem" << std::endl;

        setCurrentState(PROBLEM);
        stop();
        _encProblem = 0;
    }
}

void RobotMovements::followPath()
{
    std::cout << "Inside followPath() rb" << std::endl;
    if (_currentRoute.numSteps == 1 && _currentRoute.turnAngle > 0)
    {
        std::cout << "_currentRoute.numSteps == 1 && _currentRoute.turnAngle > 0 in rb" << std::endl;
        if (std::abs(normAngle(_currentRoute.turnAngle - _nav->getDir())) < TURN_TOLERANCE)
        {
            std::cout << "COMPLETED_ROUTE" << std::endl;
            setCurrentState(COMPLETED_ROUTE);
            return;
        }
        else
        {
            stop();
            _leftEnc->reset();
            _rightEnc->reset();
            _avgTurn = 0;
            _targetRad = _currentRoute.turnAngle;
            _currIndexRoute++;
            setCurrentState(TURNING);
            return;
        }
    }

    // if (_nav->getPos().x == _currentRoute.route[_currentRoute.route.size() - 1].x && _nav->getPos().y == _currentRoute.route[_currentRoute.route.size() - 1].y) {
    //     std::cout << "Completed route rb" << std::endl;
    //     std::cout << "COMPLETED_ROUTE" << std::endl;
    //     setCurrentState(COMPLETED_ROUTE);
    //     return;
    // }

    if (_currentRoute.route.size() > _currIndexRoute)
    {
        _nextCell = _currentRoute.route[_currIndexRoute];
        std::cout << "Robot in: " << _nav->getPos().x << ":" << _nav->getPos().y << ". Next cell is: " << _nextCell.x << ":" << _nextCell.y << std::endl;
    }
    else
    {
        std::cout << "Completed route rb" << std::endl;
        std::cout << "COMPLETED_ROUTE" << std::endl;
        setCurrentState(COMPLETED_ROUTE);
        return;
    }

    Pos currPos = _nav->getPos();
    double currDir = _nav->getDir();

    if (_nextCell.x == currPos.x && _nextCell.y == currPos.y)
    {
        _currIndexRoute++;
        return;
    }

    double absAngle = atan2(_nextCell.y - currPos.y, _nextCell.x - currPos.x);
    double turnAngle = absAngle - currDir;

    turnAngle = normAngle(turnAngle);

    if (std::abs(turnAngle) > 0.02)
    {
        stop();
        _leftEnc->reset();
        _rightEnc->reset();
        _avgTurn = 0;
        _targetRad = turnAngle;
        setCurrentState(TURNING);

        return;
    }

    // va dritto finché non c'è una curva
    bool isDiagonal = _nextCell.x != currPos.x && _nextCell.y != currPos.y;
    float straightDis = 0;

    std::cout << "Before for in followPath() rb: " << "_currIndexRoute: " << _currIndexRoute << " _currentRoute.route.size(): " << _currentRoute.route.size() << std::endl;

    for (int i = _currIndexRoute; i < _currentRoute.route.size(); i++)
    {
        _currIndexRoute = i;

        if (i == _currentRoute.route.size() - 1)
        {
            straightDis += isDiagonal ? 1.4142135f : 1.0f;
            break;
        }

        double absAngleS = atan2(_currentRoute.route[i + 1].y - _currentRoute.route[i].y, _currentRoute.route[i + 1].x - _currentRoute.route[i].x);
        double diffStart = absAngleS - absAngle; // quanto cambia in base all'inizio (se è dritto l'angolo rimane lo stesso per tutto il tragitto)

        diffStart = normAngle(diffStart);

        straightDis += isDiagonal ? 1.4142135f : 1.0f;

        if (abs(diffStart) > 0.1)
        {
            std::cout << "Found an angle at: " << _currentRoute.route[i].x << ":" << _currentRoute.route[i].y << std::endl;

            break;
        }
    }

    _leftEnc->reset();
    _rightEnc->reset();
    _avgStraight = 0;
    _targetDis = straightDis * UNIT;

    std::cout << "Moving forward, _targetDis: " << _targetDis << std::endl;

    setCurrentState(MOVING_STRAIGHT);

    return;
}

double RobotMovements::normAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}
