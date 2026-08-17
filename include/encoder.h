#pragma once

#include <Arduino.h>

class TCRT5000 {
private:
    int id;
    int aP;
    int dP;

    int blackThreshold;
    int whiteThreshold;

    bool stripeState;

public:
    TCRT5000(int id, int aP, int dP, int blackThreshold, int whiteThreshold);

    void reset();
    void begin();

    int getaR() const;
    int getdR() const;

    bool getStripe();
    bool getState() const;
    String getColor() const;

    void log();
};

void beginEncoder();
void updateEncoder();

int getEncoderPosition();
int getEncoderAngle();
bool getEncoderCenterFound();

void testlog();
void demolog();