#include <Arduino.h>

const int TEST_PIN = 10;

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(TEST_PIN, OUTPUT);
}

void loop() {
    digitalWrite(TEST_PIN, HIGH);
    printf("GPIO6 HIGH\n");
    delay(2000);

    digitalWrite(TEST_PIN, LOW);
    printf("GPIO6 LOW\n");
    delay(2000);
}