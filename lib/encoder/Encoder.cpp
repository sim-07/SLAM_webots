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
        _offsetDis = _webotsSensor->getValue();
    }
}

float Encoder::getCurrDistance() { // cm
    float relRad = _webotsSensor->getValue() - _offsetDis;

    return relRad * _wheelRadius;
}