#include <Arduino.h>
#include "emergency_brake.h"

void beginEmergencyBrake() {
    pinMode(button_pin, INPUT_PULLUP);
}

bool getEmergencyBrake() {
    return digitalRead(button_pin) == LOW;
}
