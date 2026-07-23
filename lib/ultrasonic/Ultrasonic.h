#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <webots/DistanceSensor.hpp>
#include <cstdint>

class Ultrasonic
{
private:
    webots::DistanceSensor *_webotsUS = nullptr;
    bool _status = false;

    bool test();

    static const int MIN_DISTANCE = 2;
    static const int MAX_DISTANCE = 400;

public:
    // Costruttore di default
    Ultrasonic() : _status(false) {}

    bool init(webots::DistanceSensor *webotsUS);
    double getDistance();
};

#endif