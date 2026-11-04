#include "ble_stepper_server.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <TMCStepper.h>

// ----- TMC2209 UART configuration -----
#define UART_RX_PIN 5
#define UART_TX_PIN 6
#define DRIVER_ADDRESS 0b01
//#define ROT_DRIVER_ADDRESS 0b01
#define R_SENSE 0.11f

// ----- Pin configuration -----
#define ROT_STEP_PIN   9    // Rotational motor STEP
#define LIN_STEP_PIN   7    // Linear motor STEP
#define DIR_PIN        21   // Shared direction pin
#define SLEEP_PIN      20   // Shared sleep/enable pin (active LOW: LOW = enabled, HIGH = sleep)
#define OUT_LIMIT_PIN   0   // OUT limit switch (active LOW)
#define IN_LIMIT_PIN    1   // IN limit switch (active LOW)
#define FAN_PIN        2    // Fan control pin (active High)
#define OPTICAL_SENSOR_PIN     4   // Optical position sensor (active LOW)

// ----- Timing -----
#define PULSE_WIDTH_US    70          // 0.3 ms HIGH pulse
#define PULSE_INTERVAL_US_ROTATIONAL 200  //  ms between pulse starts (4 ms LOW after pulse)
#define PULSE_INTERVAL_US_LINEAR  80  //  ms between pulse starts for linear motor (4 ms LOW after pulse)
#define PULSE_INTERVAL_US_HOMING    5000/8  //  ms between pulse starts for rotational homing (4 ms LOW after pulse)

// ----- Linear movement durations -----
#define LINEAR_DURATION_MS 5000
#define HOMING_MAX_DURATION_MS 20000
#define HOMING_RAMP_DURATION_MS 500
#define IN_CLOSING_DURATION_MS 100
#define HOMING_VACTUAL 4000

// ----- Rotational motor sensor limits -----
#define ROTATIONAL_VACTUAL 1000
#define ROTATIONAL_SENSOR_SKIP_DURATION_MS    300  // Ignore sensor for this duration
#define ROTATIONAL_HOMING_ESCAPE_DURATION_MS  500
#define ROTATIONAL_HOMING_SENSOR_SKIP_DURATION_MS  300
#define ROTATIONAL_SENSOR_CLEAR_MAX_DURATION_MS  2000
#define ROTATIONAL_SENSOR_HOME_DURATION_MS  1000
#define ROTATIONAL_MAX_DURATION_MS    3000 // Max duration before error
#define MAX_FLAGS_TO_FIND_HOME  12          // Max flags to find home position

// ----- Position-specific centering steps (indexed 0-11) -----
//static const uint16_t POSITION_DURATION_MS[12] = { 0, 25, 25, 20, 20, 0,  0, 0, 0, 0, 0, 22 }; //Juice-1 con driver A4988

//static const uint16_t POSITION_DURATION_MS[12] = {0, 0,  0,  0, 20, 100,  0,  0,  0, 0,  0,  0 }; //Juice-2
///////////////////////////////////////////       1   2   3   4   5   6   7   8   9  10  11  12
static const uint16_t POSITION_DURATION_MS[12] = {0, 200, 200, 160, 200, 160,160,150, 0, 0, 0,100};   //Juice-1 con driver TMC2209 (same as Juice-2)

// ----- BLE UUIDs (custom 128-bit) -----
#define BLE_SERVICE_UUID     "AA000001-1234-1234-1234-1234567890AA"
#define BLE_CMD_CHAR_UUID    "AA000002-1234-1234-1234-1234567890AA"
#define BLE_STATUS_CHAR_UUID "AA000003-1234-1234-1234-1234567890AA"
#define BLE_TEMP_CHAR_UUID   "AA000004-1234-1234-1234-1234567890AA"

// ----- Global state -----
static volatile bool     motorBusy      = false;
static volatile bool     stopRequested  = false;
static volatile uint32_t pulseIntervalUs = PULSE_INTERVAL_US_ROTATIONAL;
static volatile int32_t vactualMagnitude = 1800;
static volatile uint16_t motorCurrentMa = 1200;
static volatile uint16_t motorMicrosteps = 2;
static volatile uint32_t linearDurationMs = LINEAR_DURATION_MS;
static BLECharacteristic* statusCharacteristic = nullptr;
static BLECharacteristic* temperatureCharacteristic = nullptr;
static volatile bool isClientConnected = false;

HardwareSerial &driverSerial = Serial1;
//HardwareSerial &rotDriverSerial = Serial0;
TMC2209Stepper driver(&driverSerial, R_SENSE, DRIVER_ADDRESS);
//TMC2209Stepper rotdriver(&driverSerial, R_SENSE, ROT_DRIVER_ADDRESS);
TMC2209Stepper rotdriver(&driverSerial, R_SENSE, 0b00); // Using same address for rotational driver

// ----- Task parameter structs -----
struct VactualTaskParams {
    bool dirHigh;
    uint32_t durationMs;
};

struct VactualHomingTaskParams {
    bool dirHigh;
    uint8_t limitPin;
    bool closeAfterLimit;
};

struct RotationalMotorTaskParams {
    uint8_t  stepPin;
    bool     dirHigh;  // true → DIR HIGH, false → DIR LOW
    uint8_t  sensorPin;  // GPIO pin for optical sensor
    uint8_t  position;   // 1-12, selects centering duration from POSITION_DURATION_MS
};

// ============================================================
//  GPIO layer
// ============================================================

static void setupMotorGpio() {
    pinMode(ROT_STEP_PIN,  OUTPUT);
    pinMode(LIN_STEP_PIN,  OUTPUT);
    pinMode(DIR_PIN,       OUTPUT);
    pinMode(SLEEP_PIN,    OUTPUT);
    pinMode(OUT_LIMIT_PIN, INPUT_PULLUP);   // OUT limit switch (active LOW)
    pinMode(IN_LIMIT_PIN, INPUT_PULLUP);    // IN limit switch (active LOW)
    pinMode(OPTICAL_SENSOR_PIN, INPUT_PULLUP);  // Optical sensor (active LOW)
    pinMode(FAN_PIN,      OUTPUT);

    digitalWrite(ROT_STEP_PIN, LOW);
    digitalWrite(LIN_STEP_PIN, LOW);
    digitalWrite(DIR_PIN,      LOW);
    digitalWrite(SLEEP_PIN,    HIGH);   // active LOW → HIGH = sleep/disabled
    digitalWrite(FAN_PIN,     LOW);    // active HIGH → LOW = disabled

    Serial.println("Motor GPIO initialized:");
    Serial.println("ROT_STEP=GPIO9, LIN_STEP=GPIO7, DIR=GPIO21, SLEEP=GPIO20, OUT_LIMIT=GPIO0, IN_LIMIT=GPIO1, OPTICAL=GPIO4, FAN=GPIO2");
}

static void generateStepPulse(uint8_t stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(stepPin, LOW);
}

static void enableMotors()  { digitalWrite(SLEEP_PIN, LOW);  }
static void disableMotors() { digitalWrite(SLEEP_PIN, HIGH); }

static void setupTmcDriver() {
    driverSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    driver.begin();
    driver.toff(5);
    driver.rms_current(motorCurrentMa);
    driver.microsteps(motorMicrosteps);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    driver.VACTUAL(0);

    //rotDriverSerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    rotdriver.begin();
    rotdriver.toff(5);
    rotdriver.rms_current(1200);
    rotdriver.microsteps(2);
    rotdriver.en_spreadCycle(false);
    rotdriver.pwm_autoscale(true);
    rotdriver.VACTUAL(0);
}

// ============================================================
//  Motor task (FreeRTOS — replaces Python threads)
// ============================================================

static void vactualMotorTask(void* pvParams) {
    auto* params = static_cast<VactualTaskParams*>(pvParams);

    enableMotors();
    const int32_t velocity = params->dirHigh ? vactualMagnitude : -vactualMagnitude;
    driver.VACTUAL(velocity);
    bool stopped = false;

    const uint32_t startMs = millis();
    while ((millis() - startMs) < params->durationMs) {
        if (stopRequested) {
            stopped = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    driver.VACTUAL(0);
    disableMotors();

    stopRequested = false;

    if (statusCharacteristic) {
        statusCharacteristic->setValue(stopped ? "STOPPED" : "COMPLETE");
        statusCharacteristic->notify();
    }

    motorBusy = false;
    delete params;
    vTaskDelete(NULL);
}

static void launchVactualMotorTask(bool dirHigh) {
    auto* params = new VactualTaskParams{dirHigh, linearDurationMs};
    stopRequested = false;
    motorBusy = true;
    xTaskCreate(vactualMotorTask, "vactualMotorTask", 2048, params, 1, NULL);
}

static void vactualHomingTask(void* pvParams) {
    auto* params = static_cast<VactualHomingTaskParams*>(pvParams);

    enableMotors();
    const int32_t targetVelocity = params->dirHigh ? HOMING_VACTUAL : -HOMING_VACTUAL;

    bool stopped = false;
    bool limitReached = false;
    const uint32_t startMs = millis();

    while ((millis() - startMs) < HOMING_RAMP_DURATION_MS) {
        if (stopRequested) {
            stopped = true;
            break;
        }
        if (digitalRead(params->limitPin) == LOW) {
            limitReached = true;
            break;
        }

        const uint32_t rampElapsedMs = millis() - startMs;
        const int32_t rampVelocity = static_cast<int32_t>(
            (static_cast<int64_t>(targetVelocity) * rampElapsedMs)
            / HOMING_RAMP_DURATION_MS
        );
        driver.VACTUAL(rampVelocity);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!stopped && !limitReached) {
        driver.VACTUAL(targetVelocity);
    }

    while ((millis() - startMs) < HOMING_MAX_DURATION_MS) {
        if (stopRequested) {
            stopped = true;
            break;
        }
        if (digitalRead(params->limitPin) == LOW) {
            limitReached = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!stopped && limitReached && params->closeAfterLimit) {
        driver.VACTUAL(targetVelocity);
        const uint32_t closeStartMs = millis();
        while ((millis() - closeStartMs) < IN_CLOSING_DURATION_MS) {
            if (stopRequested) {
                stopped = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    driver.VACTUAL(0);
    disableMotors();
    stopRequested = false;

    if (statusCharacteristic) {
        if (stopped) {
            statusCharacteristic->setValue("STOPPED");
        } else if (limitReached) {
            statusCharacteristic->setValue("COMPLETE");
        } else {
            statusCharacteristic->setValue(
                params->dirHigh ? "ERROR: IN limit not reached."
                                : "ERROR: OUT limit not reached."
            );
        }
        statusCharacteristic->notify();
    }

    motorBusy = false;
    delete params;
    vTaskDelete(NULL);
}

static void launchVactualHomingTask(bool dirHigh, uint8_t limitPin, bool closeAfterLimit) {
    auto* params = new VactualHomingTaskParams{dirHigh, limitPin, closeAfterLimit};
    stopRequested = false;
    motorBusy = true;
    xTaskCreate(vactualHomingTask, "vactualHomingTask", 2048, params, 1, NULL);
}

// ============================================================
//  Rotational motor task with position sensor
// ============================================================

static void rotationalMotorTask(void* pvParams) {
    auto* params = static_cast<RotationalMotorTaskParams*>(pvParams);

    enableMotors();
    bool stopped = false;
    bool positionReached = false;
    const int32_t rotationalVelocity = params->dirHigh
        ? ROTATIONAL_VACTUAL
        : -ROTATIONAL_VACTUAL;
    const uint32_t phase2DurationMs =
        ROTATIONAL_MAX_DURATION_MS > ROTATIONAL_SENSOR_SKIP_DURATION_MS
            ? ROTATIONAL_MAX_DURATION_MS - ROTATIONAL_SENSOR_SKIP_DURATION_MS
            : 0;

    // Phase 1: Skip sensor check for the initial duration.
    rotdriver.VACTUAL(rotationalVelocity);
    const uint32_t phase1StartMs = millis();
    while ((millis() - phase1StartMs) < ROTATIONAL_SENSOR_SKIP_DURATION_MS) {
        if (stopRequested) {
            stopped = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Phase 2: Monitor sensor until position reached or maximum duration.
    if (!stopped) {
        const uint32_t phase2StartMs = millis();
        while ((millis() - phase2StartMs) < phase2DurationMs) {
            if (stopRequested) {
                stopped = true;
                break;
            }

            // Check if sensor pin went LOW (position reached)
            if (digitalRead(params->sensorPin) == LOW) {
                positionReached = true;
                // Phase 3: Move at one third speed until sensor goes high.
                rotdriver.VACTUAL(rotationalVelocity / 3);
                const uint32_t phase3StartMs = millis();
                while ((millis() - phase3StartMs) < ROTATIONAL_SENSOR_SKIP_DURATION_MS) {
                    if (stopRequested) {
                        stopped = true;
                        break;
                    }
                    if (digitalRead(params->sensorPin) == HIGH) {
                        // Sensor pin went HIGH again. Move opposite to center.
                        rotdriver.VACTUAL(-rotationalVelocity / 3);
                        const uint32_t centerDurationMs =
                            POSITION_DURATION_MS[params->position - 1];
                        const uint32_t centerStartMs = millis();
                        while ((millis() - centerStartMs) < centerDurationMs) {
                            if (stopRequested) {
                                stopped = true;
                                break;
                            }
                            vTaskDelay(pdMS_TO_TICKS(1));
                        }
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    rotdriver.VACTUAL(0);
    disableMotors();
    stopRequested = false;

    if (statusCharacteristic) {
        if (stopped) {
            statusCharacteristic->setValue("STOPPED");
        } else if (positionReached) {
            statusCharacteristic->setValue("COMPLETE");
        } else {
            statusCharacteristic->setValue("ERROR: DESIRED POSITION NOT REACHED");
        }
        statusCharacteristic->notify();
    }

    motorBusy = false;
    delete params;
    vTaskDelete(NULL);
}

static void launchRotationalMotorTask(uint8_t stepPin, bool dirHigh, uint8_t position) {
    auto* params = new RotationalMotorTaskParams{stepPin, dirHigh, OPTICAL_SENSOR_PIN, position};
    stopRequested = false;
    motorBusy = true;
    xTaskCreate(rotationalMotorTask, "rotationalMotorTask", 2048, params, 1, NULL);
}



static void rotationalHomingTask(void* /*pvParams*/) {
    enableMotors();
    bool stopped = false;
    // Phase 0: Move clockwise for a short duration to ensure we are not
    // already on a flag.
    rotdriver.VACTUAL(-ROTATIONAL_VACTUAL);
    const uint32_t escapeStartMs = millis();
    while ((millis() - escapeStartMs) < ROTATIONAL_HOMING_ESCAPE_DURATION_MS) {
        if (stopRequested) {
            stopped = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    rotdriver.VACTUAL(0);

    bool homeFound = false;
    uint8_t flagCount = 0;  // Count of flag-to-flag transitions

    while (flagCount < MAX_FLAGS_TO_FIND_HOME+1) {

        if (stopRequested) {
            stopped = true;
            break;
        }



        rotdriver.VACTUAL(ROTATIONAL_VACTUAL);
        // Phase 1: Skip sensor for the initial duration.
        const uint32_t skipStartMs = millis();
        while ((millis() - skipStartMs) < ROTATIONAL_HOMING_SENSOR_SKIP_DURATION_MS) {
            if (stopRequested) {
                stopped = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        rotdriver.VACTUAL(0);
        if (stopped) {
            break;
        }



        // Check if HOME flag detected (sensor LOW immediately after skip phase)
        if (digitalRead(OPTICAL_SENSOR_PIN) == LOW) {
            homeFound = true;
            break;
        }

        // Phase 2: Move until we find the next flag (sensor goes LOW)
        bool flagDetected = false;
        const uint32_t searchStartMs = millis();
        rotdriver.VACTUAL(ROTATIONAL_VACTUAL);
        while ((millis() - searchStartMs) < ROTATIONAL_MAX_DURATION_MS) {
            if (stopRequested) {
                stopped = true;
                break;
            }

            if (digitalRead(OPTICAL_SENSOR_PIN) == LOW) {
                flagDetected = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        rotdriver.VACTUAL(0);

        if (stopped) {
            break;
        }

        if (flagDetected) {
            flagCount++;
        }
        else {
            // If we didn't detect a flag, we may have reached the end of the rotation without finding home
            break;
        }
        //100 ms delay between flag checks
        delayMicroseconds(100000);
    }
    rotdriver.VACTUAL(0);

    // If home found, we will move counterclockwise until optical sensor goes HIGH (to ensure we are past the home flag)

    if (homeFound) {

        rotdriver.VACTUAL(ROTATIONAL_VACTUAL / 4);
        const uint32_t clearStartMs = millis();
        while ((millis() - clearStartMs) < ROTATIONAL_SENSOR_CLEAR_MAX_DURATION_MS) {
            if (stopRequested) {
                stopped = true;
                break;
            }

            if (digitalRead(OPTICAL_SENSOR_PIN) == HIGH) {

                break;  // We are past the home flag
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        rotdriver.VACTUAL(0);

       // Now move clockwise for the centering duration.
        rotdriver.VACTUAL(-ROTATIONAL_VACTUAL / 4);
        delayMicroseconds(100);
        const uint32_t centerStartMs = millis();
        while ((millis() - centerStartMs) < ROTATIONAL_SENSOR_HOME_DURATION_MS) {
            if (stopRequested) {
                stopped = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    rotdriver.VACTUAL(0);
    disableMotors();
    stopRequested = false;

    if (statusCharacteristic) {
        if (stopped) {
            statusCharacteristic->setValue("STOPPED");
        } else if (homeFound) {
            statusCharacteristic->setValue("COMPLETE");
        } else {
            statusCharacteristic->setValue("ERROR: HOME not found after max duration or flags.");
        }
        statusCharacteristic->notify();
    }

    motorBusy = false;
    vTaskDelete(NULL);
}

static void launchRotationalHomingTask() {
    stopRequested = false;
    motorBusy = true;
    xTaskCreate(rotationalHomingTask, "rotationalHomingTask", 2048, NULL, 1, NULL);
}



static void launchMoveInHomeTask() {
    launchVactualHomingTask(true, IN_LIMIT_PIN, true);
}

static void launchMoveOutHomeTask() {
    launchVactualHomingTask(false, OUT_LIMIT_PIN, false);
}

// ============================================================
//  BLE helpers
// ============================================================

static void sendStatus(const char* msg) {
    if (statusCharacteristic) {
        statusCharacteristic->setValue(msg);
        statusCharacteristic->notify();
    }
}

void publishTemperatureC(float temperatureC) {
    if (!temperatureCharacteristic || !isClientConnected) {
        return;
    }

    char payload[20] = {0};
    snprintf(payload, sizeof(payload), "%.2f C", temperatureC);

    temperatureCharacteristic->setValue(payload);
    temperatureCharacteristic->notify();
}

// ============================================================
//  BLE callbacks
// ============================================================

class CommandCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) override {
        String cmd = characteristic->getValue().c_str();
        cmd.trim();
        cmd.toUpperCase();

        Serial.printf("Received command: %s\n", cmd.c_str());

        // STOP, configuration updates, and fan commands bypass the busy guard.
        if (cmd == "STOP") {
            stopRequested = true;
            sendStatus("OK");
            return;
        }

        if (cmd.startsWith("SETVACTUAL:")) {
            int32_t val = cmd.substring(11).toInt();
            if (val >= 1 && val <= 10000) {
                vactualMagnitude = val;
                Serial.printf("VACTUAL magnitude set to %ld\n", (long) vactualMagnitude);
                sendStatus("OK");
            } else {
                Serial.printf("SETVACTUAL rejected: %ld (must be 1..10000)\n", (long) val);
                sendStatus("INVALID");
            }
            return;
        }

        if (cmd.startsWith("SETCURRENT:")) {
            int32_t val = cmd.substring(11).toInt();
            if (val >= 100 && val <= 2000) {
                motorCurrentMa = (uint16_t) val;
                driver.rms_current(motorCurrentMa);
                Serial.printf("Motor current set to %u mA\n", motorCurrentMa);
                sendStatus("OK");
            } else {
                Serial.printf("SETCURRENT rejected: %ld (must be 100..2000)\n", (long) val);
                sendStatus("INVALID");
            }
            return;
        }

        if (cmd.startsWith("SETMICROSTEPS:")) {
            int32_t val = cmd.substring(14).toInt();
            if (val >= 0 && val <= 256) {
                motorMicrosteps = (uint16_t) val;
                driver.microsteps(motorMicrosteps);
                Serial.printf("Microsteps set to %u\n", motorMicrosteps);
                sendStatus("OK");
            } else {
                Serial.printf("SETMICROSTEPS rejected: %ld (must be 0..256)\n", (long) val);
                sendStatus("INVALID");
            }
            return;
        }

        if (cmd.startsWith("SETLINEARDURATION:")) {
            uint32_t val = (uint32_t) cmd.substring(18).toInt();
            if (val >= 1 && val <= 60000) {
                linearDurationMs = val;
                Serial.printf("Linear duration set to %lu ms\n", (unsigned long) linearDurationMs);
                sendStatus("OK");
            } else {
                Serial.printf("SETLINEARDURATION rejected: %lu (must be 1..60000)\n", (unsigned long) val);
                sendStatus("INVALID");
            }
            return;
        }

        if (cmd.startsWith("SETINTERVAL:")) {
            int32_t val = cmd.substring(12).toInt();
            if (val > (int32_t)PULSE_WIDTH_US) {
                pulseIntervalUs = (uint32_t)val;
                Serial.printf("Pulse interval set to %u us\n", pulseIntervalUs);
                sendStatus("OK");
            } else {
                Serial.printf("SETINTERVAL rejected: %d (must be > %d)\n", val, PULSE_WIDTH_US);
                sendStatus("INVALID");
            }
            return;
        }

        if (cmd == "FANON") {
            digitalWrite(FAN_PIN, HIGH);
            sendStatus("OK");
            return;
        }

        if (cmd == "FANOFF") {
            digitalWrite(FAN_PIN, LOW);
            sendStatus("OK");
            return;
        }
        //

        if (motorBusy) {
            Serial.println("Motor busy — sending WAIT");
            sendStatus("WAIT");
            return;
        }

        // Movement commands, only accepted if motor is not busy.

        if (cmd == "MOVEIN") {
            sendStatus("OK");
            launchVactualMotorTask(true);                           // Positive VACTUAL = IN
        } else if (cmd == "MOVEOUT") {
            sendStatus("OK");
            launchVactualMotorTask(false);                          // Negative VACTUAL = OUT
        } else if (cmd == "MOVEINHOME") {
            sendStatus("OK");
            launchMoveInHomeTask();  // DIR HIGH = IN, monitored by IN_LIMIT_PIN
        } else if (cmd == "MOVEOUTHOME") {
            sendStatus("OK");
            launchMoveOutHomeTask();  // DIR LOW = OUT, monitored by OUT_LIMIT_PIN
        } else if (cmd.startsWith("MOVECLOCKWISE")) {
            uint8_t pos = 1;
            int colonIdx = cmd.indexOf(':');
            if (colonIdx >= 0) pos = constrain(cmd.substring(colonIdx + 1).toInt(), 1, 12);
            sendStatus("OK");
            launchRotationalMotorTask(ROT_STEP_PIN, false, pos);  // DIR LOW  = CW, sensor-aware
        } else if (cmd.startsWith("MOVECOUNTERCLOCKWISE")) {
            uint8_t pos = 1;
            int colonIdx = cmd.indexOf(':');
            if (colonIdx >= 0) pos = constrain(cmd.substring(colonIdx + 1).toInt(), 1, 12);
            sendStatus("OK");
            launchRotationalMotorTask(ROT_STEP_PIN, true, pos);   // DIR HIGH = CCW, sensor-aware
        }else if (cmd == "ROTATIONALHOMING") {
            sendStatus("OK");
            launchRotationalHomingTask();  // DIR HIGH = CCW, finds HOME flag
        } else if (cmd == "SENSORGPIO5") {
            // Query optical sensor state (always allowed, bypasses busy guard)
            bool sensorLow = digitalRead(OPTICAL_SENSOR_PIN) == LOW;
            sendStatus(sensorLow ? "SENSOR:LOW" : "SENSOR:HIGH");
        } else {
            Serial.printf("Unknown command: %s\n", cmd.c_str());
            sendStatus("INVALID");
        }
    }
};

class ConnectionCallback : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override {
        Serial.println("Client connected.");
        isClientConnected = true;
    }

    void onDisconnect(BLEServer* server) override {
        Serial.println("Client disconnected. Restarting advertising...");
        isClientConnected = false;
        BLEDevice::startAdvertising();
    }
};

// ============================================================
//  Public init
// ============================================================

void initBleStepperServer() {
    setupMotorGpio();
    setupTmcDriver();

    BLEDevice::init("ESP32_STEPPER");


    BLEServer* bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new ConnectionCallback());

    BLEService* service = bleServer->createService(BLE_SERVICE_UUID);

    // Command characteristic — client writes commands
    BLECharacteristic* cmdCharacteristic = service->createCharacteristic(
        BLE_CMD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    cmdCharacteristic->setCallbacks(new CommandCallback());

    // Status characteristic — server notifies client (OK / WAIT / COMPLETE / INVALID)
    statusCharacteristic = service->createCharacteristic(
        BLE_STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    statusCharacteristic->addDescriptor(new BLE2902());

    temperatureCharacteristic = service->createCharacteristic(
        BLE_TEMP_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    temperatureCharacteristic->addDescriptor(new BLE2902());
    temperatureCharacteristic->setValue("0.00 C");

    service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("BLE server started. Device name: ESP32_STEPPER");
    Serial.println("Commands: MOVEIN / MOVEOUT / MOVEINHOME / MOVEOUTHOME / HOMING / MOVECLOCKWISE / MOVECOUNTERCLOCKWISE / FANON / FANOFF / STOP / SETVACTUAL:<value> / SETCURRENT:<mA> / SETMICROSTEPS:<value> / SETLINEARDURATION:<ms> / SETINTERVAL:<us>");
}
