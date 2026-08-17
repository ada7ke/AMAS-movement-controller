#pragma once

#include <Arduino.h>

class PressureCollector {
public:
    PressureCollector(HardwareSerial &serialPort);

    uint16_t pressures[48] = {0};
    uint32_t goodPackets = 0;
    uint32_t badChecksums = 0;

    bool update();

private:
    HardwareSerial &serial;
    uint8_t packet[99];
    int packetIndex = 0;

    bool processPacket();
};

void beginPressureSensors();
void updatePressureSensors();

bool pressureDataReady();
void clearPressureDataReady();

uint16_t *getLeftPressures();
uint16_t *getRightPressures();