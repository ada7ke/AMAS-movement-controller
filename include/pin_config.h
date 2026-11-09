#pragma once

#include <Arduino.h>

#define XIAO_ESP32C3 1
#define ESP32C3_SUPERMINI 2

#ifndef PIN_PROFILE
#define PIN_PROFILE ESP32C3_SUPERMINI // change pin configuration here
#endif

namespace Pins {
#if PIN_PROFILE == XIAO_ESP32C3
    constexpr int ENCODER_A_ANALOG = A2;
    constexpr int ENCODER_A_DIGITAL = 8;
    constexpr int ENCODER_B_ANALOG = A1;
    constexpr int ENCODER_B_DIGITAL = 9;
    constexpr int ENCODER_C_ANALOG = A0;
    constexpr int ENCODER_C_DIGITAL = 10;
    constexpr int LEFT_PRESSURE_RX = 21;
    constexpr int LEFT_PRESSURE_TX = -1; // receive only
    constexpr int RIGHT_PRESSURE_RX = 20;
    constexpr int RIGHT_PRESSURE_TX = -1; // receive only
    constexpr int BRAKE = 7;
#elif PIN_PROFILE == ESP32C3_SUPERMINI
    constexpr int ENCODER_A_ANALOG = A4;
    constexpr int ENCODER_A_DIGITAL = 8;
    constexpr int ENCODER_B_ANALOG = A3;
    constexpr int ENCODER_B_DIGITAL = 9;
    constexpr int ENCODER_C_ANALOG = A2;
    constexpr int ENCODER_C_DIGITAL = 10;
    constexpr int LEFT_PRESSURE_RX = 0;
    constexpr int LEFT_PRESSURE_TX = 1;
    constexpr int RIGHT_PRESSURE_RX = 20;
    constexpr int RIGHT_PRESSURE_TX = 21;
    constexpr int BRAKE = 7;
#else
#error "Unknown PIN_PROFILE"
#endif
}
