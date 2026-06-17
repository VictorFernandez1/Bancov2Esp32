#include <Arduino.h>
#include "ble/ble_stepper_server.h"

void setup() {
    Serial.begin(9600);
    initBleStepperServer();
}

void loop() {
    // All logic is event-driven via BLE callbacks and FreeRTOS tasks
}
