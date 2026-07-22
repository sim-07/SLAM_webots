#ifndef EXPLORER_H
#define EXPLORER_H

#include "Navigator.h"
#include <LaserSensor.h>
#include <Ultrasonic.h>
#include <ServoMotor.h>
#include <RobotMovements.h>
#include <cstring>

#include "../../src/Common.h"

#include <set>

static const double SCAN_PRECISION = 0.0872; // ogni quanti gradi scansiona (5)
static const int MAX_BORDERS_FOUND = 3;

enum ExpState
{
    IDLE = 0,
    START_EXPLORE = 1,
    SCAN = 2,
    MOVE_TO_FRONTIER = 3,
    FOLLOWING_F = 4,
    COMPLETED = 5,
};

enum DelayState
{
    INITSTATE,
    AWAITING,
    TIMEPASSED
};

class Explorer
{

private:
    void searchObstacles();
    void scan();
    double normAngle(double angle);
    void findBorder();
    Pos findClosestBorder(Pos currPos);
    Pos calcCoordinates(Pos currPos, float dis, double totAngle);

    void deleteBorder(Pos closestBorder);

    Pos _firstPos;
    Navigator *_nav;
    RobotMovements *_rb;
    ServoMotor *_servo;
    LaserSensor *_laser;
    Ultrasonic *_ultrasonic;
    ExpState _currentState = IDLE;

    int _attemptsFindBorders = 3;

    std::vector<Pos> _bordersToExplore = {{0, 0}}; // Cella iniziale messa come sconosciuta, così all'inizio farà una scansione a 360 gradi (si gira su se stesso se la frontiera ha e stesse coordinate della posizione corrente)

    volatile bool _scanning = false;
    double _currScanPoint;

    int _dirServo = 1;
    int _delay = 0;
    int _bordersExplored = 0;
    DelayState _currDelayState;

public:
    void init(Navigator *n, RobotMovements *r, ServoMotor *s, LaserSensor *l, Ultrasonic *u);
    void explore(Navigator &nav);
    void stopExploring();
    void update();
    void setCurrentState(ExpState currState);
};

#endif