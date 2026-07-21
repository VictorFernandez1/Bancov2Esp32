#pragma once

#include <Arduino.h>

bool initNtcSensor();

uint16_t readRawAdc();

float convertRawToVoltage(uint16_t rawAdc);

bool calculateRt(float voltageMeasured, float& rtOhms);

bool calculateTemperatureC(float rtOhms, float& temperatureC);

float readReliableVoltage();
