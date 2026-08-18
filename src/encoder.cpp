#include <Arduino.h>
#include "encoder.h"

namespace {
    const int BLACK_THRESHOLD_A = 1500;
    const int WHITE_THRESHOLD_A = 500;

    const int BLACK_THRESHOLD_B = 1500;
    const int WHITE_THRESHOLD_B = 500;

    const int BLACK_THRESHOLD_C = 3000;
    const int WHITE_THRESHOLD_C = 500;
}

TCRT5000::TCRT5000(int id, int aP, int dP, int blackThreshold, int whiteThreshold)
: id(id), aP(aP), dP(dP),
  blackThreshold(blackThreshold), whiteThreshold(whiteThreshold),
  stripeState(false) {}

void TCRT5000::reset() {
    stripeState = false;
}

void TCRT5000::begin() {
    pinMode(aP, INPUT);
    pinMode(dP, INPUT);
    reset();
}

int TCRT5000::getaR() const {
    return analogRead(aP);
}

int TCRT5000::getdR() const {
    return digitalRead(dP);
}

bool TCRT5000::getStripe() { // true if black
    int aR = getaR();

    if (!stripeState && aR > blackThreshold) {
        stripeState = true;
    } else if (stripeState && aR < whiteThreshold) {
        stripeState = false;
    }

    return stripeState;
}

bool TCRT5000::getState() const {
    return stripeState;
}

String TCRT5000::getColor() const {
    return stripeState ? "black" : "white";
}

void TCRT5000::log() {
    int aR = getaR();

    printf("| %04d %s |",
        aR, getColor()
    );
}

TCRT5000 sensorA(1, A4, 8, BLACK_THRESHOLD_A, WHITE_THRESHOLD_A);
TCRT5000 sensorB(2, A3, 9, BLACK_THRESHOLD_B, WHITE_THRESHOLD_B);
TCRT5000 sensorC(3, A2, 10, BLACK_THRESHOLD_C, WHITE_THRESHOLD_C);

bool lastAStripe = false;
bool lastBStripe = false;
bool lastCStripe = false;
bool centerFound = false;
int positionCount = 0;
int angle = 0;

void updateCenter() {
    bool CStripe = sensorC.getStripe();

    if (CStripe && !lastCStripe) {
        centerFound = true;
        positionCount = 0;
        angle = 0;
        printf("CENTER RESET\n");
    }

    lastCStripe = CStripe;
}

void updatePosition() {
    if (!centerFound) return;

    bool AStripe = sensorA.getStripe();
    bool BStripe = sensorB.getStripe();

    if (AStripe != lastAStripe) {
        positionCount += AStripe == BStripe ? 1 : -1;
    }

    if (BStripe != lastBStripe) {
        positionCount += AStripe != BStripe ? 1 : -1;
    }

    lastAStripe = AStripe;
    lastBStripe = BStripe;
}

void testlog() {
    printf("A:%d B:%d | ", sensorA.getState(), sensorB.getState());

    printf("pos: %03d ", positionCount);
    printf("angle: %03d |", angle);

    sensorA.log();
    sensorB.log();
    sensorC.log();

    printf("| center: %s", sensorC.getColor().c_str());
}

void demolog() {
    printf("A:%d B:%d C: %d| ", sensorA.getState(), sensorB.getState(), sensorC.getState());
    printf("angle: %03d |", angle);
}

void beginEncoder() {
    sensorA.begin();
    sensorB.begin();
    sensorC.begin();

    lastAStripe = sensorA.getStripe();
    lastBStripe = sensorB.getStripe();
    lastCStripe = sensorC.getStripe();
}

void updateEncoder() {
    updateCenter();
    updatePosition();

    angle = -(positionCount * 2);
}

int getEncoderSensorA() {
    return sensorA.getaR();
}

int getEncoderSensorB() {
    return sensorB.getaR();
}

int getEncoderSensorC() {
    return sensorC.getaR();
}

int getEncoderPosition() {
    return positionCount;
}

int getEncoderAngle() {
    return angle;
}

bool getEncoderCenterFound() {
    return centerFound;
}