#include <Arduino.h>
#include "pressure_sensor.h"

const int LEFT_RX_PIN = 0;
const int LEFT_TX_PIN = 1;
const int RIGHT_RX_PIN = 20;
const int RIGHT_TX_PIN = 21;

const int COLLECTOR_BAUD = 115200;

HardwareSerial LeftSerial(0);
HardwareSerial RightSerial(1);

PressureCollector leftCollector(LeftSerial);
PressureCollector rightCollector(RightSerial);

bool newLeftData = false;
bool newRightData = false;

PressureCollector::PressureCollector(HardwareSerial &serialPort)
: serial(serialPort) {}

bool PressureCollector::processPacket() {
    uint8_t checksum = 0;

    for (int i = 0; i < 98; i++) {
        checksum += packet[i];
    }

    if (checksum != packet[98]) {
        badChecksums++;
        return false;
    }

    for (int i = 0; i < 48; i++) {
        uint8_t high = packet[2 + i * 2];
        uint8_t low = packet[3 + i * 2];

        pressures[i] = (high << 8) | low;
    }

    goodPackets++;

    return true;
}

bool PressureCollector::update() {
    bool newPacket = false;

    while (serial.available()) {
        uint8_t byte = serial.read();

        if (packetIndex == 0) {
            if (byte == 0x40) {
                packet[packetIndex++] = byte;
            }

            continue;
        }

        packet[packetIndex++] = byte;

        if (packetIndex == 99) {
            if (processPacket()) {
                newPacket = true;
            }

            packetIndex = 0;
        }
    }

    return newPacket;
}

void beginPressureSensors() {
    LeftSerial.begin(COLLECTOR_BAUD, SERIAL_8N1, LEFT_RX_PIN, -1);
    RightSerial.begin(COLLECTOR_BAUD, SERIAL_8N1, RIGHT_RX_PIN, -1);
}

void updatePressureSensors() {
    if (leftCollector.update()) {
        newLeftData = true;
    }

    if (rightCollector.update()) {
        newRightData = true;
    }
}

bool pressureDataReady() {
    return newLeftData && newRightData;
}

void clearPressureDataReady() {
    newLeftData = false;
    newRightData = false;
}

uint16_t *getLeftPressures() {
    return leftCollector.pressures;
}

uint16_t *getRightPressures() {
    return rightCollector.pressures;
}