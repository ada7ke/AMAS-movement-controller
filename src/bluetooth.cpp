#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wifi.h>
#include <atomic>
#include "bluetooth.h"

#define BLE_DEVICE_NAME "AMAS_Pedal_Controller"
#define BLE_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer *bleServer = nullptr;
NimBLECharacteristic *bleTx = nullptr;

volatile bool bleConnected = false;

unsigned long lastBleNotification = 0;
const unsigned long BLE_NOTIFICATION_INTERVAL = 50;
constexpr uint16_t CONN_INTERVAL = 6; // 1.25 ms units: 6 = 7.5 ms, 12 = 15 ms.
static std::atomic<uint16_t> blePeer{BLE_HS_CONN_HANDLE_NONE};
static std::atomic<bool> requestInterval{false};

class MyBLEServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        bleConnected = true;
        blePeer = connInfo.getConnHandle();
        printf("bluetooth connected\n");
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        bleConnected = false;
        blePeer = BLE_HS_CONN_HANDLE_NONE;
        printf("bluetooth disconnected, reason=%d\n", reason);
    }

};

class MyBLETxCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t value) override {
        // start negotiation after the client enables notifications.
        if (value & 1) requestInterval = true;
    }
};
static MyBLETxCallbacks bleTxCallbacks;

static void updateConnectionInterval() {
    static unsigned long lastRequestMs = 0;
    static uint8_t attemptsLeft = 0;
    if (requestInterval.exchange(false)) {
        lastRequestMs = millis();
        attemptsLeft = 3;
    }
    const uint16_t handle = blePeer.load();
    if (handle == BLE_HS_CONN_HANDLE_NONE || attemptsLeft == 0 || millis() - lastRequestMs < 1000) return;
    lastRequestMs = millis();
    --attemptsLeft;
    const auto info = bleServer->getPeerInfoByHandle(handle);
    if (info.getConnInterval() != CONN_INTERVAL) {
        bleServer->updateConnParams(handle, CONN_INTERVAL, CONN_INTERVAL,
                                   info.getConnLatency(), info.getConnTimeout());
    }
}

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
    bleTx->setCallbacks(&bleTxCallbacks);

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
    updateConnectionInterval();
    if (bleConnected && bleTx != nullptr && millis() - lastBleNotification >= BLE_NOTIFICATION_INTERVAL) {
        lastBleNotification = millis();

        bleTx->setValue(packet, packetSize);
        bleTx->notify();
    }
}
