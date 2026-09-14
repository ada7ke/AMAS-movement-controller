#include <Arduino.h>
#include "bluetooth.h"
#include "encoder.h"
#include "emergency_brake.h"
#include "pressure_sensor.h"

void encoderlog() {
    printf("angle: %03d | ", getEncoderAngle());
    printf("A:%04u B:%04u C:%04u | ", getEncoderAnalogA(), getEncoderAnalogB(), getEncoderAnalogC());
    printf("A:%u B:%u C:%u | ", getEncoderStripeA(), getEncoderStripeB(), getEncoderStripeC());
    printf("center: %s | ", getEncoderCenterFound() ? "T" : "F");
}

void brakelog() {
    printf("brake:%s\n", getEmergencyBrake() ? "T" : "F");
}

void pressurelog() {
    printf("_left: ");
    for (int i = 0; i < 48; i++) {
        printf("%03u ", getLeftPressures()[i]);
    }
    printf("\nright: ");
    for (int i = 0; i < 48; i++) {
        printf("%03u ", getRightPressures()[i]);
    }
    printf("\n");
}

void testlog() {

    //encoderlog();
    //brakelog();
    pressurelog();
}

void sendData() {
    uint8_t packet[207];

    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 1 | (getEmergencyBrake() ? 0x80 : 0);

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

    uint16_t analogAReading = getEncoderAnalogA();
    uint16_t analogBReading = getEncoderAnalogB();
    uint16_t analogCReading = getEncoderAnalogC();

    packet[index++] = analogAReading & 0xFF;
    packet[index++] = (analogAReading >> 8) & 0xFF;
    packet[index++] = analogBReading & 0xFF;
    packet[index++] = (analogBReading >> 8) & 0xFF;
    packet[index++] = analogCReading & 0xFF;
    packet[index++] = (analogCReading >> 8) & 0xFF;

    uint8_t checksum = 0;
    for (int i = 0; i < 206; i++) {
        checksum += packet[i];
    }
    packet[206] = checksum;

    Serial.write(packet, sizeof(packet));
    updateBluetooth(packet, sizeof(packet));
}

void setup() {
    Serial.begin(921600);
    delay(1500);

    beginBluetooth();
    beginEncoder();
    beginPressureSensors();
    beginEmergencyBrake();
}

void loop() {
    updateEncoder();
    updatePressureSensors();

    static unsigned long timer = 0;
    static unsigned long diagnosticTimer = 0;
    if (millis() - timer >= 50) {
        timer = millis();

        sendData();

        //testlog();

    }

}