#include <Arduino.h> 

HardwareSerial TestSerial(1); 

void setup() { 
    Serial.begin(115200); 
    delay(1000); 

    TestSerial.begin(115200, SERIAL_8N1, 0, -1); 

    printf("sensor UART test started"); 
} 

void loop() { 
    int available = TestSerial.available(); 
    if (available > 0) { 
        printf("available: %d", available); 
        while (TestSerial.available()) { 
            uint8_t byte = TestSerial.read(); 
            printf(" %02X", byte); 
        } 
        printf("\n"); 
    } 
    delay(10); 
}