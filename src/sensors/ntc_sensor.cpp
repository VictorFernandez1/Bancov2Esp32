#include "sensors/ntc_sensor.h"

#include <math.h>

namespace {

constexpr uint8_t NTC_ADC_PIN = 3;
constexpr float VCC_VOLTAGE = 3.28F;
constexpr float FIXED_RESISTOR_OHMS = 100000.0F;
constexpr float ADC_MAX_COUNT = 4095.0F;
constexpr float MIN_VALID_VOLTAGE = 0.01F;
constexpr float BETA_PARAMETER = 3775.0F;
constexpr float NOMINAL_RESISTANCE_OHMS = 88500.0F;
constexpr float NOMINAL_TEMPERATURE_C = 25.0F;
constexpr float KELVIN_OFFSET = 273.15F;

}

bool initNtcSensor() {
    analogReadResolution(12);
    pinMode(NTC_ADC_PIN, INPUT);
    return true;
}

uint16_t readRawAdc() {
    return static_cast<uint16_t>(analogRead(NTC_ADC_PIN));
}

float convertRawToVoltage(uint16_t rawAdc) {
    return (static_cast<float>(rawAdc) / ADC_MAX_COUNT) * 2.8746F + 0.0354F;
}

float readReliableVoltage() {
    const int SAMPLES = 32;
    uint32_t totalMilliVolts = 0;

    for (int i = 0; i < SAMPLES; i++) {
        totalMilliVolts += analogReadMilliVolts(NTC_ADC_PIN);
        delayMicroseconds(100);
    }

    float averageMilliVolts = static_cast<float>(totalMilliVolts) / SAMPLES;
    return averageMilliVolts / 1000.0F;
}

bool calculateRt(float voltageMeasured, float& rtOhms) {
    if (voltageMeasured <= MIN_VALID_VOLTAGE) {
        return false;
    }

    rtOhms = ((VCC_VOLTAGE * FIXED_RESISTOR_OHMS) / voltageMeasured) - FIXED_RESISTOR_OHMS;

    return true;
}

bool calculateTemperatureC(float rtOhms, float& temperatureC) {
    if (rtOhms <= 0.0F) {
        return false;
    }

    const float nominalTemperatureK = NOMINAL_TEMPERATURE_C + KELVIN_OFFSET;
    const float logTerm = logf(rtOhms / NOMINAL_RESISTANCE_OHMS);
    const float inverseTemperatureK =
        (1.0F / nominalTemperatureK) + (logTerm / BETA_PARAMETER);

    if (inverseTemperatureK <= 0.0F) {
        return false;
    }

    temperatureC = (1.0F / inverseTemperatureK) - KELVIN_OFFSET;
    return true;
}
