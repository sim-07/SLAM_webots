#ifndef ENCODER_H
#define ENCODER_H

#include <webots/PositionSensor.hpp>

class Encoder {

    private:
        const float _wheelRadius = 0;
        float _offsetRad = 0.0f;

        webots::PositionSensor* _webotsSensor = nullptr;


    public:
        Encoder(float wheelRadius)
            : _wheelRadius(wheelRadius) {}

        bool init(webots::PositionSensor* webotsSensor);
        void reset();
        
        float getCurrDistance();
};

#endif