# ESP32 Stepper Motor Controller

This project controls a linear stepper motor, a rotational stepper motor, a
fan, and an NTC temperature sensor from an ESP32-C3 over Bluetooth Low Energy
(BLE). It contains:

- PlatformIO firmware for a **LOLIN C3 Mini**.
- A Python/PySide6 desktop application that discovers and controls the
  firmware with [Bleak](https://github.com/hbldh/bleak).

The firmware advertises as `ESP32_STEPPER`.

## Features

- Linear motor movement in both directions.
- Timed linear movement using the TMC2209 `VACTUAL` velocity mode.
- Linear in/out homing using limit switches.
- Rotational movement with an optical position sensor and selectable positions
  1 through 12.
- Rotational homing.
- Fan control.
- Emergency stop command.
- NTC temperature measurement with a 10-sample moving average.
- Temperature and motor-status notifications over BLE.
- A desktop UI with motor controls, status logging, and a 120-second
  temperature plot.

## Hardware connections

The firmware is configured for the following ESP32-C3 GPIOs:

| Function | GPIO | Electrical behavior |
| --- | ---: | --- |
| Rotational motor STEP | 9 | Step output |
| Linear motor STEP | 7 | Step output |
| Shared direction | 21 | Direction output |
| Shared driver sleep/enable | 20 | Active low |
| Out limit switch | 0 | Active low, internal pull-up |
| In limit switch | 1 | Active low, internal pull-up |
| Fan | 2 | Active high |
| Optical position sensor | 4 | Active low, internal pull-up |
| NTC ADC input | 3 | 12-bit ADC |

The motor driver must be compatible with the configured STEP, DIR, and
active-low SLEEP/ENABLE signals. Verify the wiring and motor-driver current
limits before applying power.

The temperature calculation assumes:

- 3.28 V supply/reference model.
- 100 kOhm fixed resistor.
- NTC beta parameter of 3775.
- Nominal NTC resistance of 88.5 kOhm at 25 °C.

These values are defined in `src/sensors/ntc_sensor.cpp` and may need to be
calibrated for different hardware.

## Firmware setup

### Requirements

- VS Code with the PlatformIO extension, or PlatformIO Core.
- A LOLIN C3 Mini.
- A USB data cable.

### Build and upload

From this directory:

```text
pio run
pio run --target upload
pio device monitor --baud 9600
```

The PlatformIO environment is `lolin_c3_mini`, configured in
`platformio.ini`. The serial monitor prints GPIO initialization, BLE
connection events, received commands, and temperature-calculation results.

After startup, the board initializes BLE and begins advertising. The
temperature task samples once per second and publishes the averaged value to
connected clients.

## Desktop controller

### Requirements

- Python 3.10 or newer (the application uses modern type annotation syntax).
- A computer with Bluetooth Low Energy support.
- `bleak` and `PySide6`.

Install the Python dependencies in a virtual environment:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install bleak PySide6
```

Run the controller:

```powershell
python MotorControllerEspv2.py
```

Click **Connect** to scan for the device named `ESP32_STEPPER`. Once
connected, the motor, fan, sensor, stop, and interval controls are enabled.
The application subscribes to both status and temperature notifications.

On Windows, Bluetooth permissions and an already-paired/available adapter may
be required. If the device is not found, confirm that the ESP32 is powered,
advertising, and not connected exclusively to another BLE client.

### Optional standalone executable

The included `MotorControllerEspv2.spec` is a PyInstaller specification for
building a single windowed executable:

```powershell
python -m pip install pyinstaller
pyinstaller MotorControllerEspv2.spec
```

The resulting executable is named `MotorControllerEsp` and is placed in the
PyInstaller output directory.

## BLE protocol

### Service and characteristics

| Purpose | UUID | Properties |
| --- | --- | --- |
| Service | `AA000001-1234-1234-1234-1234567890AA` | — |
| Commands | `AA000002-1234-1234-1234-1234567890AA` | Write, Write Without Response |
| Status | `AA000003-1234-1234-1234-1234567890AA` | Notify |
| Temperature | `AA000004-1234-1234-1234-1234567890AA` | Read, Notify |

Commands are UTF-8 strings. The firmware trims whitespace and treats commands
case-insensitively.

### Commands

| Command | Description |
| --- | --- |
| `MOVEIN` | Move the linear motor inward using VACTUAL for the configured duration |
| `MOVEOUT` | Move the linear motor outward using VACTUAL for the configured duration |
| `MOVEINHOME` | Ramp to VACTUAL `+4000` over 500 milliseconds, move until the in-limit switch, then continue for 400 milliseconds |
| `MOVEOUTHOME` | Ramp to VACTUAL `-4000` over 500 milliseconds, then move until the out-limit switch |
| `MOVECLOCKWISE:1` through `MOVECLOCKWISE:12` | Move the rotational motor clockwise to a sensor-defined position using VACTUAL ±750 |
| `MOVECOUNTERCLOCKWISE:1` through `MOVECOUNTERCLOCKWISE:12` | Move the rotational motor counterclockwise to a sensor-defined position using VACTUAL ±750 |
| `ROTATIONALHOMING` | Find the rotational home flag using VACTUAL |
| `STOP` | Request cancellation of the active motor task |
| `SETVACTUAL:<value>` | Set the VACTUAL magnitude for regular linear movement (`1..10000`, default `1800`) |
| `SETCURRENT:<mA>` | Set TMC2209 RMS motor current (`100..2000` mA, default `1200`) |
| `SETMICROSTEPS:<value>` | Set TMC2209 microsteps (`0..256`, default `0`) |
| `SETLINEARDURATION:<ms>` | Set regular linear movement duration (`1..60000` ms, default `5000`) |
| `FANON` / `FANOFF` | Enable or disable the fan |
| `SETINTERVAL:<microseconds>` | Set the pulse interval when greater than the 70 µs pulse width |
| `SENSORGPIO5` | Query the optical sensor |

`SENSORGPIO5` is a legacy command name. It reads the optical sensor on GPIO 4,
not GPIO 5. `MOVEIN` and `MOVEOUT` set the configurable signed VACTUAL value
and stop it after the configured duration. `MOVEINHOME` and `MOVEOUTHOME` use
fixed signed VACTUAL values of `+4000` and `-4000`, respectively, using a
linear 500 millisecond ramp from zero, and stop when their limit switch is
reached. Homing has a 20,000 millisecond safety timeout; `MOVEINHOME`
continues inward for 400 milliseconds after reaching the limit. STOP or a
limit switch is honored immediately during the ramp. Rotational positioning
uses a separate TMC2209 at UART address `0b01`, fixed at 1200 mA and
2 microsteps. It uses VACTUAL ±750 for normal positioning and one third of
that speed during positioning sensor clearing and centering. Rotational
homing uses VACTUAL ±1000 for escape and flag search, then VACTUAL ±250 for
sensor clearing and final centering. Its former escape, sensor-skip,
maximum-search, sensor-clear, and centering step values are interpreted as
milliseconds. The existing 1-second and 100 ms waits are preserved.

Movement commands are rejected with `WAIT` while another motor task is
running. `STOP`, fan commands, interval updates, and sensor queries remain
available while a motor is busy.

### Status values

The status characteristic sends text notifications such as:

- `OK` — command accepted.
- `WAIT` — a movement command was received while the motor was busy.
- `COMPLETE` — movement or homing completed.
- `STOPPED` — movement was interrupted by `STOP`.
- `INVALID` — command or interval was invalid.
- `SENSOR:LOW` or `SENSOR:HIGH` — optical sensor state.
- `ERROR: ...` — a limit or rotational position could not be reached.

Temperature notifications are formatted as text such as `23.47 C`.

## Project layout

```text
.
├── platformio.ini
├── MotorControllerEspv2.py
├── MotorControllerEspv2.spec
├── src/
│   ├── main.cpp
│   ├── ble/
│   │   ├── ble_stepper_server.cpp
│   │   └── ble_stepper_server.h
│   └── sensors/
│       ├── ntc_sensor.cpp
│       └── ntc_sensor.h
├── include/
├── lib/
└── test/
```

`main.cpp` starts the BLE server and the FreeRTOS temperature task.
`ble_stepper_server.cpp` owns the BLE protocol, GPIO setup, motor tasks, limit
switch handling, and status notifications. `ntc_sensor.cpp` reads and
converts the NTC signal. The Python application provides the desktop
controller and keeps BLE I/O on a background asyncio thread so the Qt UI
remains responsive.

## Troubleshooting

- **No device found:** check power, BLE availability, and that the device name
  is exactly `ESP32_STEPPER`.
- **Controls remain disabled:** the desktop application enables them only
  after BLE connection succeeds.
- **`WAIT` response:** wait for the current movement to finish or send
  `STOP`.
- **Limit error:** check the relevant active-low switch and its pull-up
  wiring. Homing stops after the firmware's maximum step count.
- **Rotational position error:** check the optical sensor, flag alignment, and
  direction. The sensor is active low on GPIO 4.
- **Temperature calculation failure:** inspect the NTC divider wiring and
  verify the resistance and beta constants match the installed sensor.

## License

No license file is currently included in this repository. Add a license before
distributing the firmware or desktop application.
