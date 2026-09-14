#pragma once

#include <Arduino.h>

void beginBluetooth();
void updateBluetooth(uint8_t *packet, size_t packetSize);
