#include <Arduino.h>
#include "ble/ble_stepper_server.h"
#include "esp_bt.h"
#include "sensors/ntc_sensor.h"

static void temperatureTask(void* /*pvParams*/) {
    initNtcSensor();
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Let BLE stack settle

    constexpr size_t TEMP_WINDOW = 10;
    float tempRing[TEMP_WINDOW] = {0};
    size_t tempIdx = 0;
    size_t tempCount = 0;
    float tempSum = 0.0F;

    while (true) {
        const float voltage = readReliableVoltage();

        float rtOhms = 0.0F;
        if (calculateRt(voltage, rtOhms)) {
            float temperatureC = 0.0F;
            if (calculateTemperatureC(rtOhms, temperatureC)) {
                tempSum -= tempRing[tempIdx];
                tempRing[tempIdx] = temperatureC;
                tempSum += temperatureC;
                tempIdx = (tempIdx + 1) % TEMP_WINDOW;
                if (tempCount < TEMP_WINDOW) tempCount++;

                const float meanTemp = tempSum / static_cast<float>(tempCount);
                publishTemperatureC(meanTemp);
                Serial.printf("Temp: %.2f C (raw: %.2f C), Rt: %.0f Ohms, V: %.3f V\n", meanTemp, temperatureC, rtOhms, voltage);
            } else {
                Serial.printf("Temp calc failed — Rt=%.0f Ohms\n", rtOhms);
            }
        } else {
            Serial.printf("Rt calc failed — V=%.3f V\n", voltage);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(9600);
    // set max tx power to 8.5 dBm
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    initBleStepperServer();

    xTaskCreate(temperatureTask, "temperatureTask", 4096, NULL, 1, NULL);
}

void loop() {
    // All logic is event-driven via BLE callbacks and FreeRTOS tasks
}
