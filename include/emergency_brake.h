#pragma once

#include <Arduino.h>
#include "pin_config.h"

const int button_pin = Pins::BRAKE;

void beginEmergencyBrake();
bool getEmergencyBrake();
