#include <Arduino.h>
#include "ble/ble_stepper_server.h"
#include "esp_bt.h"
#include "sensors/ntc_sensor.h"
#include <TMCStepper.h>

// ---- UART to TMC2209 ----
#define UART_RX_PIN 5
#define UART_TX_PIN 6
#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f   // CHANGE to your board's actual sense resistor value

// ---- Enable pin (no STEP/DIR needed in VACTUAL mode) ----
#define EN_PIN 20

HardwareSerial &driverSerial = Serial1;
TMC2209Stepper driver(&driverSerial, R_SENSE, DRIVER_ADDRESS);

const int32_t RUN_VELOCITY = 1800;

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
                //Serial.printf("Temp: %.2f C (raw: %.2f C), Rt: %.0f Ohms, V: %.3f V\n", meanTemp, temperatureC, rtOhms, voltage);
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
    Serial.begin(115200);
    delay(1000);
    // set max tx power to 8.5 dBm
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    initBleStepperServer();

    //xTaskCreate(temperatureTask, "temperatureTask", 4096, NULL, 1, NULL);

    driverSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    driver.begin();
    driver.toff(5);
    driver.rms_current(1200);       // mA, tune to your motor
    driver.microsteps(4);
    driver.en_spreadCycle(false);  // StealthChop
    driver.pwm_autoscale(true);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);     // LOW = driver enabled

    driver.VACTUAL(0);             // make sure it's stopped initially


}

void loop() {
    Serial.println("Moving forward...");
    digitalWrite(EN_PIN, LOW);  // enable driver
    driver.VACTUAL(RUN_VELOCITY);
    delay(2000);                   // run for 2 seconds
    digitalWrite(EN_PIN, HIGH);  // disable driver
    Serial.println("Stopping...");
    driver.VACTUAL(0);
    delay(500);
    digitalWrite(EN_PIN, LOW);  // enable driver
    Serial.println("Moving backward...");
    driver.VACTUAL(-RUN_VELOCITY);
    delay(2000);
    digitalWrite(EN_PIN, HIGH);  // disable driver
    Serial.println("Stopping...");
    driver.VACTUAL(0);
    delay(1000);
    // All logic is event-driven via BLE callbacks and FreeRTOS tasks
}
