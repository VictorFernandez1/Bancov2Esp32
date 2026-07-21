#include <Arduino.h>
#include "ble/ble_stepper_server.h"
#include "esp_bt.h"

void setup() {
    Serial.begin(9600);
    // set max tx power to 8.5 dBm
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    initBleStepperServer();
}

void loop() {
    // All logic is event-driven via BLE callbacks and FreeRTOS tasks
}
