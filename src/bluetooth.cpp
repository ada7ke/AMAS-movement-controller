#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
