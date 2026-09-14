#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wifi.h>
#include "bluetooth.h"

#define BLE_DEVICE_NAME "AMAS_Pedal_Controller"
#define BLE_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer *bleServer = nullptr;
NimBLECharacteristic *bleTx = nullptr;

volatile bool bleConnected = false;

unsigned long lastBleNotification = 0;
const unsigned long BLE_NOTIFICATION_INTERVAL = 50;

class MyBLEServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        bleConnected = true;
        printf("bluetooth connected\n");
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        bleConnected = false;
        printf("bluetooth disconnected, reason=%d\n", reason);
    }
};


void beginBluetooth() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    printf("wifi disabled\n");

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(247);

    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(new MyBLEServerCallbacks());

    bleServer->advertiseOnDisconnect(true);

    NimBLEService *service =
        bleServer->createService(BLE_SERVICE_UUID);

    bleTx = service->createCharacteristic(BLE_TX_UUID, NIMBLE_PROPERTY::NOTIFY);

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData scanResponse;

    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->enableScanResponse(true);
    scanResponse.setName(BLE_DEVICE_NAME);
    advertising->setScanResponseData(scanResponse);

    NimBLEDevice::startAdvertising();

    printf("bluetooth advertising as %s\n", BLE_DEVICE_NAME);
}


void updateBluetooth(uint8_t *packet, size_t packetSize) {
    if (bleConnected && bleTx != nullptr && millis() - lastBleNotification >= BLE_NOTIFICATION_INTERVAL) {
        lastBleNotification = millis();

        bleTx->setValue(packet, packetSize);
        bleTx->notify();
    }
}