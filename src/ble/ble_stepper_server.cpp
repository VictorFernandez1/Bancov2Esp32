#include "ble_stepper_server.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ----- Pin configuration -----
#define ROT_STEP_PIN   9    // Rotational motor STEP
#define LIN_STEP_PIN   7    // Linear motor STEP
#define DIR_PIN        21   // Shared direction pin
#define SLEEP_PIN      20   // Shared sleep/enable pin (active HIGH)
#define HOME_PIN        0   // Home Pin (digital, weak pull-up)
#define FAN_PIN        10   // Fan control pin (active LOW)
#define OPTICAL_SENSOR_PIN     5   // Optical position sensor (active LOW)

// ----- Timing -----
#define PULSE_WIDTH_US    100          // 0.3 ms HIGH pulse
#define PULSE_INTERVAL_US_DEFAULT 1500  // 5 ms between pulse starts (4 ms LOW after pulse)

// ----- Motor steps -----
#define LINEAR_STEPS      1000
#define HOMING_MAX_STEPS  12000

// ----- Rotational motor sensor limits -----
#define ROTATIONAL_SENSOR_SKIP_STEPS    100  // Ignore sensor for first N steps
#define ROTATIONAL_MAX_STEPS    500           // Max steps before error

// ----- BLE UUIDs (custom 128-bit) -----
#define BLE_SERVICE_UUID     "AA000001-1234-1234-1234-1234567890AA"
#define BLE_CMD_CHAR_UUID    "AA000002-1234-1234-1234-1234567890AA"
#define BLE_STATUS_CHAR_UUID "AA000003-1234-1234-1234-1234567890AA"

// ----- Global state -----
static volatile bool     motorBusy      = false;
static volatile bool     stopRequested  = false;
static volatile uint32_t pulseIntervalUs = PULSE_INTERVAL_US_DEFAULT;
static BLECharacteristic* statusCharacteristic = nullptr;

// ----- Task parameter structs -----
struct MotorTaskParams {
    uint8_t  stepPin;
    bool     dirHigh;  // true → DIR HIGH, false → DIR LOW
    uint16_t steps;
};

struct RotationalMotorTaskParams {
    uint8_t  stepPin;
    bool     dirHigh;  // true → DIR HIGH, false → DIR LOW
    uint8_t  sensorPin;  // GPIO pin for optical sensor
};

// ============================================================
//  GPIO layer
// ============================================================

static void setupMotorGpio() {
    pinMode(ROT_STEP_PIN,  OUTPUT);
    pinMode(LIN_STEP_PIN,  OUTPUT);
    pinMode(DIR_PIN,       OUTPUT);
    pinMode(SLEEP_PIN,    OUTPUT);
    pinMode(HOME_PIN, INPUT_PULLUP);
    pinMode(OPTICAL_SENSOR_PIN, INPUT_PULLUP);  // Optical sensor (active LOW)
    pinMode(FAN_PIN,      OUTPUT);

    digitalWrite(ROT_STEP_PIN, LOW);
    digitalWrite(LIN_STEP_PIN, LOW);
    digitalWrite(DIR_PIN,      LOW);
    digitalWrite(SLEEP_PIN,    LOW);   // active HIGH → LOW = disabled
    digitalWrite(FAN_PIN,     LOW);    // active HIGH → LOW = disabled

    Serial.println("Motor GPIO initialized:");
    Serial.println("ROT_STEP=GPIO7, LIN_STEP=GPIO9, DIR=GPIO21, SLEEP=GPIO20, POSITION=GPIO0, FAN=GPIO10");
}

static void generateStepPulse(uint8_t stepPin) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(stepPin, LOW);
}

static void enableMotors()  { digitalWrite(SLEEP_PIN, HIGH); }
static void disableMotors() { digitalWrite(SLEEP_PIN, LOW);  }

// ============================================================
//  Motor task (FreeRTOS — replaces Python threads)
// ============================================================

static void motorTask(void* pvParams) {
    auto* params = static_cast<MotorTaskParams*>(pvParams);

    enableMotors();
    delayMicroseconds(1000);  // A4988 wake-up time after SLEEP→HIGH
    digitalWrite(DIR_PIN, params->dirHigh ? HIGH : LOW);
    delayMicroseconds(1);     // DIR setup time before first STEP (A4988 requires ≥200 ns)

    bool stopped = false;
    for (uint16_t i = 0; i < params->steps; i++) {
        if (stopRequested) {
            stopped = true;
            break;
        }
        generateStepPulse(params->stepPin);
        delayMicroseconds(pulseIntervalUs - PULSE_WIDTH_US);
    }

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

static void launchMotorTask(uint8_t stepPin, bool dirHigh, uint16_t steps) {
    auto* params = new MotorTaskParams{stepPin, dirHigh, steps};
    stopRequested = false;
    motorBusy = true;
    xTaskCreate(motorTask, "motorTask", 2048, params, 1, NULL);
}

// ============================================================
//  Rotational motor task with position sensor
// ============================================================

static void rotationalMotorTask(void* pvParams) {
    auto* params = static_cast<RotationalMotorTaskParams*>(pvParams);

    enableMotors();
    delayMicroseconds(1000);  // A4988 wake-up time after SLEEP→HIGH
    digitalWrite(DIR_PIN, params->dirHigh ? HIGH : LOW);
    delayMicroseconds(1);     // DIR setup time before first STEP

    bool stopped = false;
    bool positionReached = false;
    uint16_t stepCount = 0;

    // Phase 1: Skip sensor check for first ROTATIONAL_SENSOR_SKIP_STEPS
    for (uint16_t i = 0; i < ROTATIONAL_SENSOR_SKIP_STEPS; i++) {
        if (stopRequested) {
            stopped = true;
            break;
        }
        generateStepPulse(params->stepPin);
        delayMicroseconds(pulseIntervalUs - PULSE_WIDTH_US);
        stepCount++;
    }

    // Phase 2: Monitor sensor until position reached or max steps hit
    if (!stopped) {
        for (uint16_t i = ROTATIONAL_SENSOR_SKIP_STEPS; i < ROTATIONAL_MAX_STEPS; i++) {
            if (stopRequested) {
                stopped = true;
                break;
            }

            // Check if sensor pin went LOW (position reached)
            if (digitalRead(params->sensorPin) == LOW) {
                positionReached = true;
                break;
            }

            generateStepPulse(params->stepPin);
            delayMicroseconds(pulseIntervalUs - PULSE_WIDTH_US);
            stepCount++;
        }
    }

    disableMotors();
    stopRequested = false;

    if (statusCharacteristic) {
        if (stopped) {
            statusCharacteristic->setValue("STOPPED");
        } else if (positionReached) {
            statusCharacteristic->setValue("REACHED_POSITION");
        } else {
            statusCharacteristic->setValue("ERROR: DESIRED POSITION NOT REACHED");
        }
        statusCharacteristic->notify();
    }

    motorBusy = false;
    delete params;
    vTaskDelete(NULL);
}

static void launchRotationalMotorTask(uint8_t stepPin, bool dirHigh) {
    auto* params = new RotationalMotorTaskParams{stepPin, dirHigh, OPTICAL_SENSOR_PIN};
    stopRequested = false;
    motorBusy = true;
    xTaskCreate(rotationalMotorTask, "rotationalMotorTask", 2048, params, 1, NULL);
}

static void homingTask(void* /*pvParams*/) {
    enableMotors();
    delayMicroseconds(1000);  // A4988 wake-up time after SLEEP→HIGH
    digitalWrite(DIR_PIN, HIGH);  // IN direction per updated protocol
    delayMicroseconds(1);         // DIR setup time before first STEP

    bool stopped = false;
    bool homeFound = false;

    for (uint16_t i = 0; i < HOMING_MAX_STEPS; i++) {
        if (stopRequested) {
            stopped = true;
            break;
        }

        if (digitalRead(HOME_PIN) == LOW) {
            homeFound = true;
            break;
        }

        generateStepPulse(LIN_STEP_PIN);
        delayMicroseconds(pulseIntervalUs - PULSE_WIDTH_US);
    }

    disableMotors();
    stopRequested = false;

    if (statusCharacteristic) {
        if (stopped) {
            statusCharacteristic->setValue("STOPPED");
        } else if (homeFound) {
            statusCharacteristic->setValue("REACHED_HOME");
        } else {
            statusCharacteristic->setValue("ERROR: Home position not found.");
        }
        statusCharacteristic->notify();
    }

    motorBusy = false;
    vTaskDelete(NULL);
}

static void launchHomingTask() {
    stopRequested = false;
    motorBusy = true;
    xTaskCreate(homingTask, "homingTask", 2048, NULL, 1, NULL);
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

// ============================================================
//  BLE callbacks
// ============================================================

class CommandCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) override {
        String cmd = characteristic->getValue().c_str();
        cmd.trim();
        cmd.toUpperCase();

        Serial.printf("Received command: %s\n", cmd.c_str());

        // STOP, SETINTERVAL, FANON and FANOFF bypass the busy guard intentionally
        if (cmd == "STOP") {
            stopRequested = true;
            sendStatus("OK");
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
            launchMotorTask(LIN_STEP_PIN, true, LINEAR_STEPS);       // DIR HIGH = IN
        } else if (cmd == "MOVEOUT") {
            sendStatus("OK");
            launchMotorTask(LIN_STEP_PIN, false, LINEAR_STEPS);      // DIR LOW  = OUT
        } else if (cmd == "HOMING") {
            sendStatus("OK");
            launchHomingTask();
        } else if (cmd == "MOVECLOCKWISE") {
            sendStatus("OK");
            launchRotationalMotorTask(ROT_STEP_PIN, false);  // DIR LOW  = CW, sensor-aware
        } else if (cmd == "MOVECOUNTERCLOCKWISE") {
            sendStatus("OK");
            launchRotationalMotorTask(ROT_STEP_PIN, true);   // DIR HIGH = CCW, sensor-aware
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
    }

    void onDisconnect(BLEServer* server) override {
        Serial.println("Client disconnected. Restarting advertising...");
        BLEDevice::startAdvertising();
    }
};

// ============================================================
//  Public init
// ============================================================

void initBleStepperServer() {
    setupMotorGpio();

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

    service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.println("BLE server started. Device name: ESP32_STEPPER");
    Serial.println("Commands: MOVEIN / MOVEOUT / HOMING / MOVECLOCKWISE / MOVECOUNTERCLOCKWISE / FANON / FANOFF / STOP / SETINTERVAL:<us>");
}
