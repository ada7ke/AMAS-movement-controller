#include <Arduino.h>
#include "encoder.h"
#include "pressure_sensor.h"

const int PC_BAUD = 921600;

void sendData() {
    uint8_t packet[201];

    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 1;

    int index = 3;

    uint16_t *leftPressures = getLeftPressures();
    uint16_t *rightPressures = getRightPressures();

    for (int i = 0; i < 48; i++) {
        uint16_t value = leftPressures[i];

        packet[index++] = value & 0xFF;
        packet[index++] = (value >> 8) & 0xFF;
    }

    for (int i = 0; i < 48; i++) {
        uint16_t value = rightPressures[i];

        packet[index++] = value & 0xFF;
        packet[index++] = (value >> 8) & 0xFF;
    }

    int16_t position = getEncoderPosition();
    int16_t angle = getEncoderAngle();

    packet[index++] = position & 0xFF;
    packet[index++] = (position >> 8) & 0xFF;

    packet[index++] = angle & 0xFF;
    packet[index++] = (angle >> 8) & 0xFF;

    packet[index++] = getEncoderCenterFound() ? 1 : 0;

    uint8_t checksum = 0;

    for (int i = 0; i < 200; i++) {
        checksum += packet[i];
    }

    packet[200] = checksum;

    Serial.write(packet, sizeof(packet));
}

void setup() {
    Serial.begin(PC_BAUD);
    delay(1500);

    beginEncoder();
    beginPressureSensors();
}

void loop() {
    updateEncoder();
    updatePressureSensors();

    if (pressureDataReady()) {
        sendData();
        clearPressureDataReady();
    }

    // static unsigned long timer = 0;
    // if (millis() - timer >= 10) {
    //     timer = millis();

    //     testlog();
    //     printf("\n");
    // }
}