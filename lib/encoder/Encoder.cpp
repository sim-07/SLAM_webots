#include "Encoder.h"
#include <cmath>

bool Encoder::init(webots::PositionSensor* webotsSensor) {
    _webotsSensor = webotsSensor;

    if (_webotsSensor == nullptr) {
        return false;
    }

    return true;
}

void Encoder::reset() {
    if (_webotsSensor != nullptr) {
        _offsetRad = _webotsSensor->getValue();
    }
}

float Encoder::getCurrDistance() { // cm
    _webotsSensor->getValue();

    float relRad = _webotsSensor->getValue() - _offsetRad;

    return relRad * _wheelRadius;
}