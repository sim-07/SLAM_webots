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
    Route r = _nav->calcRoute(_destination.x, _destination.y);

    if (r.numSteps > 0)
    {
        setCurrentState(FOLLOWING);
        _currentRoute = r;
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
    std::cout << "Inside goStraight() rb. _targetDis: " << _targetDis << std::endl;

    if (_avgStraight < abs(_targetDis))
    {
        float dL = abs(_leftEnc->getCurrDistance());
        float dR = abs(_rightEnc->getCurrDistance());

        _avgStraight = (dL + dR) / 2.0;

        float error = dL - dR;

        if (std::abs(error) > 1.1)
        {
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
        stop();
        _leftEnc->reset();
        _rightEnc->reset();
        _targetDis = 0;
        _avgStraight = 0;
        _nav->setCurrPos(_currentRoute.route[_currIndexRoute].x, _currentRoute.route[_currIndexRoute].y); // TODO controllare
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

    double turnDis = abs((_wheelDistance * _targetRad) / 2.0f); // totale distanza che le ruote devono percorrere
    int dir = (_targetRad > 0) ? -1 : 1;

    if (_avgTurn < turnDis)
    {
        std::cout << "Turning in rb: _avgTurn: " << _avgTurn << " turnDis " << turnDis << std::endl;

        float abs_dis_l = abs(_leftEnc->getCurrDistance());
        float abs_dis_r = abs(_rightEnc->getCurrDistance());

        // std::cout << "abs_dis_l: " << abs_dis_l << std::endl;
        // std::cout << "abs_dis_r: " << abs_dis_r << std::endl;

        _avgTurn = (abs_dis_l + abs_dis_r) / 2;

        float error = abs_dis_l - abs_dis_r;

        if (std::abs(error) > 0.1)
        {
            float corr = error * Kp;

            _leftMotor->setPower((MOTOR_POWER - corr) * dir);
            _rightMotor->setPower((MOTOR_POWER + corr) * -dir);
        }
        else
        {
            _leftMotor->setPower(MOTOR_POWER * dir);
            _rightMotor->setPower(MOTOR_POWER * -dir);
        }
    }
    else
    {
        std::cout << "Finished turning" << std::endl;

        stop();
        _leftEnc->reset();
        _rightEnc->reset();
        _nav->setDir(normAngle(_nav->getDir() + _targetRad));
        _targetRad = 0;
        _avgTurn = 0;
        setCurrentState(FOLLOWING);
    }

    if (_leftEnc->getCurrDistance() == 0 || _rightEnc->getCurrDistance() == 0)
    {
        _encProblem++;
    }

    if (_encProblem > 2)
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
        if (std::abs(normAngle(_currentRoute.turnAngle - _nav->getDir())) < 0.05)
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

    if (abs(turnAngle) > 0.1)
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
    bool isDiagonal = _currentRoute.route[_currIndexRoute + 1].x != _currentRoute.route[_currIndexRoute].x && _currentRoute.route[_currIndexRoute + 1].y != _currentRoute.route[_currIndexRoute].y;
    float straightDis = 0;

    for (int i = _currIndexRoute; i < _currentRoute.route.size() - 1; i++)
    {
        std::cout << "_currIndexRoute in for in followPath() rb before incrementing: " << _currIndexRoute << std::endl;

        double absAngleS = atan2(_currentRoute.route[i + 1].y - _currentRoute.route[i].y, _currentRoute.route[i + 1].x - _currentRoute.route[i].x);
        double diffStart = absAngleS - absAngle; // quanto cambia in base all'inizio (se è dritto l'angolo rimane lo stesso per tutto il tragitto)

        diffStart = normAngle(diffStart);

        if (abs(diffStart) > 0.1)
        {
            std::cout << "Found an angle at: " << _currentRoute.route[i].y << ":" << _currentRoute.route[i].x << std::endl;

            break;
        }

        straightDis += isDiagonal ? 1.4142135f : 1.0f;
        std::cout << "straightDis is (end of followPath() in rb): " << straightDis << std::endl;

        _currIndexRoute++;
    }

    _leftEnc->reset();
    _rightEnc->reset();
    _avgStraight = 0;
    _targetDis = straightDis * UNIT;
    setCurrentState(MOVING_STRAIGHT);

    std::cout << "_targetDis is (end of followPath() in rb): " << _targetDis << std::endl;

    return;
}

double RobotMovements::normAngle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}
