#include <Arduino.h>

HardwareSerial TestSerial(1);

void setup() {
    Serial.begin(115200);
    delay(1000);

    TestSerial.begin(115200, SERIAL_8N1, 21, 20);

    printf("loopback test started\n");
}

void loop() {
    TestSerial.write(0x40);
    TestSerial.write(0x12);
    TestSerial.write(0x34);

    delay(10);

    int available = TestSerial.available();

    printf("available: %d\n", available);

    while (TestSerial.available()) {
        uint8_t byte = TestSerial.read();
        printf("%02X ", byte);
    }

    printf("\n");

    delay(500);
}