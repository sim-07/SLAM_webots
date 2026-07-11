#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <webots/DistanceSensor.hpp>
#include <cstdint>

static const uint8_t MIN_DISTANCE = 2;
static const int MAX_DISTANCE = 400;

class Ultrasonic
{
    private:
        webots::DistanceSensor* _webotsUS = nullptr;
        bool _status = false;
        
        bool test();

    public:
        // Costruttore di default
        Ultrasonic() : _status(false) {}

        bool init(webots::DistanceSensor* webotsUS);
        float getDistance();
};

#endif