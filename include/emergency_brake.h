#pragma once

#include <Arduino.h>

const int button_pin = 7;

void beginEmergencyBrake();
bool getEmergencyBrake();
