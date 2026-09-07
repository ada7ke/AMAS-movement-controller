#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "encoder.h"
#include "emergency_brake.h"
#include "pressure_sensor.h"

#define BLE_DEVICE_NAME "AMAS_Pedal_Controller"
#define BLE_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *bleServer = nullptr;
BLECharacteristic *bleTx = nullptr;

bool bleConnected = false;
bool oldBleConnected = false;

class MyBLEServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *server) {
        bleConnected = true;
        printf("bluetooth connected\n");
    }

    void onDisconnect(BLEServer *server) {
        bleConnected = false;
        printf("bluetooth disconnected\n");
    }
};

void beginBluetooth() {
    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(247);

    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new MyBLEServerCallbacks());

    BLEService *service = bleServer->createService(BLE_SERVICE_UUID);

    bleTx = service->createCharacteristic(BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    bleTx->addDescriptor(new BLE2902());

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMaxPreferred(0x12);

    BLEDevice::startAdvertising();

    printf("bluetooth advertising as %s\n", BLE_DEVICE_NAME);
}

void sendData() {
    uint8_t packet[207];

    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 1 | (getEmergencyBrake() ? 0x80 : 0);

    int index = 3;

    uint16_t *leftPressures = getLeftPressures();
    uint16_t *rightPressures = getRightPressures();

    // printf("\nleft pressures: ");
    // for (int i = 0; i < 48; i++) {
    //     printf("%d ", leftPressures[i]);
    // }

    // printf("\nright pressures: ");
    // for (int i = 0; i < 48; i++) {
    //     printf("%d ", rightPressures[i]);
    // }
    // printf("\n");

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

    uint16_t sensorAReading = getEncoderSensorA();
    uint16_t sensorBReading = getEncoderSensorB();
    uint16_t sensorCReading = getEncoderSensorC();

    packet[index++] = sensorAReading & 0xFF;
    packet[index++] = (sensorAReading >> 8) & 0xFF;
    packet[index++] = sensorBReading & 0xFF;
    packet[index++] = (sensorBReading >> 8) & 0xFF;
    packet[index++] = sensorCReading & 0xFF;
    packet[index++] = (sensorCReading >> 8) & 0xFF;

    uint8_t checksum = 0;
    for (int i = 0; i < 206; i++) {
        checksum += packet[i];
    }
    packet[206] = checksum;

    Serial.write(packet, sizeof(packet));

    if (bleConnected) {
        bleTx->setValue(packet, sizeof(packet));
        bleTx->notify();
    }
}

void setup() {
    Serial.begin(921600);
    delay(1500);

    // beginBluetooth();
    beginEncoder();
    beginPressureSensors();
    beginEmergencyBrake();
}

void loop() {
    updateEncoder();
    updatePressureSensors();

    static unsigned long timer = 0;
    static unsigned long diagnosticTimer = 0;
    if (millis() - timer >= 10) {
        timer = millis();

        sendData();
        
        //encoderlog();
        //pressurelog();
        //brakelog();

    }

    // if (!bleConnected && oldBleConnected) {
    //     delay(500);
    //     bleServer->startAdvertising();
    //     printf("bluetooth advertising restarted\n");
    //     oldBleConnected = false;
    // }

    // if (bleConnected && !oldBleConnected) {
    //     oldBleConnected = true;
    // }
}