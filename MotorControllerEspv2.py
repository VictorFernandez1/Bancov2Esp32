#!/usr/bin/env python3
"""
ESP32 Stepper Motor Controller GUI
PySide6 front-end that connects to the ESP32_STEPPER BLE device and sends
motor commands over BLE.

BLE characteristics (defined in ble_stepper_server.cpp):
  Command (Write):   AA000002-1234-1234-1234-1234567890AA
  Status  (Notify):  AA000003-1234-1234-1234-1234567890AA
"""

import asyncio
import sys
import threading

from bleak import BleakClient, BleakScanner
from PySide6.QtCore import Qt, QObject, Signal
from PySide6.QtWidgets import (
    QApplication, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QSpinBox, QTextEdit, QVBoxLayout, QWidget,
)

# ── BLE constants ──────────────────────────────────────────────────────────────
DEVICE_NAME      = "ESP32_STEPPER"
CMD_CHAR_UUID    = "AA000002-1234-1234-1234-1234567890AA"
STATUS_CHAR_UUID = "AA000003-1234-1234-1234-1234567890AA"


# ── Thread-safe bridge between asyncio worker and Qt UI ───────────────────────

class _Signals(QObject):
    """Signals emitted from the asyncio thread, received in the Qt main thread."""
    status_changed  = Signal(str)   # human-readable status line
    notification    = Signal(str)   # raw notification value from ESP32
    connected       = Signal(bool)  # True = connected, False = disconnected


class BleWorker:
    """
    Manages a BleakClient on a dedicated asyncio event loop running in a
    background daemon thread.  All public methods are safe to call from the
    Qt main thread.
    """

    def __init__(self, signals: _Signals):
        self._signals = signals
        self._loop: asyncio.AbstractEventLoop | None = None
        self._client: BleakClient | None = None
        self._thread = threading.Thread(target=self._run_loop, daemon=True)

    def start(self):
        self._thread.start()

    # ── Public API (called from Qt thread) ────────────────────────────────────

    def connect(self):
        asyncio.run_coroutine_threadsafe(self._connect(), self._loop)

    def disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)

    def send_command(self, cmd: str):
        asyncio.run_coroutine_threadsafe(self._send(cmd), self._loop)

    # ── Internal: runs in the background asyncio thread ───────────────────────

    def _run_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _connect(self):
        self._signals.status_changed.emit(f"Scanning for '{DEVICE_NAME}'…")
        try:
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
        except OSError as e:
            self._signals.status_changed.emit(f"Bluetooth error: {e}")
            self._signals.connected.emit(False)
            return

        if device is None:
            self._signals.status_changed.emit("Device not found. Is the ESP32 powered and advertising?")
            self._signals.connected.emit(False)
            return

        self._signals.status_changed.emit(f"Found {device.address}. Connecting…")
        try:
            self._client = BleakClient(device, disconnected_callback=self._on_ble_disconnect)
            await self._client.connect()
            await self._client.start_notify(STATUS_CHAR_UUID, self._on_notification)
            self._signals.status_changed.emit(f"Connected  ({device.address})")
            self._signals.connected.emit(True)
        except Exception as e:
            self._signals.status_changed.emit(f"Connection failed: {e}")
            self._client = None
            self._signals.connected.emit(False)

    async def _disconnect(self):
        if self._client and self._client.is_connected:
            await self._client.disconnect()

    async def _send(self, cmd: str):
        if self._client and self._client.is_connected:
            await self._client.write_gatt_char(CMD_CHAR_UUID, cmd.encode("utf-8"))

    def _on_notification(self, _sender, data: bytearray):
        self._signals.notification.emit(data.decode("utf-8"))

    def _on_ble_disconnect(self, _client: BleakClient):
        self._client = None
        self._signals.status_changed.emit("Disconnected")
        self._signals.connected.emit(False)


# ── Main window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Stepper Controller")
        self.setMinimumWidth(380)

        self._signals = _Signals()
        self._worker  = BleWorker(self._signals)
        self._worker.start()

        self._signals.status_changed.connect(self._on_status)
        self._signals.notification.connect(self._on_notification)
        self._signals.connected.connect(self._on_connected_changed)

        self._build_ui()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)

        # Connection group
        conn_box    = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_box)

        self._status_label = QLabel("Not connected")
        self._status_label.setAlignment(Qt.AlignCenter)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setFixedHeight(36)
        self._connect_btn.clicked.connect(self._toggle_connect)

        conn_layout.addWidget(self._status_label)
        conn_layout.addWidget(self._connect_btn)
        layout.addWidget(conn_box)

        # Linear motor group
        linear_box    = QGroupBox("Linear Motor")
        linear_layout = QHBoxLayout(linear_box)

        self._btn_in   = self._make_cmd_btn("◀  Move In",   "MOVEIN")
        self._btn_out  = self._make_cmd_btn("▶  Move Out",  "MOVEOUT")
        self._btn_in_home = self._make_cmd_btn("Move In Home",    "MOVEINHOME")
        self._btn_out_home = self._make_cmd_btn("Move Out Home",   "MOVEOUTHOME")
        linear_layout.addWidget(self._btn_in)
        linear_layout.addWidget(self._btn_out)
        linear_layout.addWidget(self._btn_in_home)
        linear_layout.addWidget(self._btn_out_home)
        layout.addWidget(linear_box)

        # Rotational motor group
        rot_box    = QGroupBox("Rotational Motor")
        rot_layout = QHBoxLayout(rot_box)

        self._btn_cw  = self._make_cmd_btn("↻  Clockwise",        "MOVECLOCKWISE")
        self._btn_ccw = self._make_cmd_btn("↺  Counterclockwise", "MOVECOUNTERCLOCKWISE")
        self._btn_rot_home = self._make_cmd_btn("Find Home", "ROTATIONALHOMING")
        rot_layout.addWidget(self._btn_cw)
        rot_layout.addWidget(self._btn_ccw)
        rot_layout.addWidget(self._btn_rot_home)
        layout.addWidget(rot_box)

        # Position sensor group
        sensor_box    = QGroupBox("Position Sensor")
        sensor_layout = QHBoxLayout(sensor_box)
        self._btn_sensor_query = QPushButton("Check Sensor")
        self._btn_sensor_query.setFixedHeight(44)
        self._btn_sensor_query.setEnabled(False)
        self._btn_sensor_query.clicked.connect(lambda: self._worker.send_command("SENSORGPIO5"))
        sensor_layout.addWidget(self._btn_sensor_query)
        layout.addWidget(sensor_box)

        # Fan control group
        fan_box    = QGroupBox("Fan")
        fan_layout = QHBoxLayout(fan_box)
        self._btn_fan_on  = self._make_cmd_btn("Fan On",  "FANON")
        self._btn_fan_off = self._make_cmd_btn("Fan Off", "FANOFF")
        fan_layout.addWidget(self._btn_fan_on)
        fan_layout.addWidget(self._btn_fan_off)
        layout.addWidget(fan_box)

        # Stop button
        self._btn_stop = QPushButton("\u25a0  Stop")
        self._btn_stop.setFixedHeight(44)
        self._btn_stop.setEnabled(False)
        self._btn_stop.setStyleSheet("QPushButton { background-color: #c0392b; color: white; font-weight: bold; }"
                                     "QPushButton:disabled { background-color: #7f8c8d; color: #bdc3c7; }")
        self._btn_stop.clicked.connect(lambda: self._worker.send_command("STOP"))
        layout.addWidget(self._btn_stop)

        # Pulse interval group
        interval_box    = QGroupBox("Pulse Interval")
        interval_layout = QHBoxLayout(interval_box)

        interval_layout.addWidget(QLabel("Interval (\u00b5s):"))
        self._interval_spin = QSpinBox()
        self._interval_spin.setRange(300, 50000)
        self._interval_spin.setValue(1500)
        self._interval_spin.setSingleStep(100)
        self._interval_spin.setEnabled(False)
        interval_layout.addWidget(self._interval_spin)

        self._btn_set_interval = QPushButton("Set")
        self._btn_set_interval.setFixedHeight(36)
        self._btn_set_interval.setEnabled(False)
        self._btn_set_interval.clicked.connect(
            lambda: self._worker.send_command(f"SETINTERVAL:{self._interval_spin.value()}")
        )
        interval_layout.addWidget(self._btn_set_interval)
        layout.addWidget(interval_box)
        # Log
        log_box    = QGroupBox("Log")
        log_layout = QVBoxLayout(log_box)
        self._log  = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFixedHeight(130)
        log_layout.addWidget(self._log)
        layout.addWidget(log_box)

    def _make_cmd_btn(self, label: str, command: str) -> QPushButton:
        btn = QPushButton(label)
        btn.setFixedHeight(44)
        btn.setEnabled(False)
        btn.clicked.connect(lambda: self._worker.send_command(command))
        return btn

    def _command_buttons(self):
        return (self._btn_in, self._btn_out, self._btn_in_home, self._btn_out_home,
                self._btn_cw, self._btn_ccw, self._btn_sensor_query,
                self._btn_fan_on, self._btn_fan_off, self._btn_stop,
                self._btn_set_interval, self._interval_spin, self._btn_rot_home)

    # ── Slots (called in Qt main thread) ─────────────────────────────────────

    def _toggle_connect(self):
        if self._connect_btn.text() == "Connect":
            self._connect_btn.setEnabled(False)
            self._worker.connect()
        else:
            self._worker.disconnect()

    def _on_status(self, msg: str):
        self._status_label.setText(msg)
        self._append_log(f"[status]  {msg}")

    def _on_notification(self, msg: str):
        self._append_log(f"[esp32]   {msg}")

    def _on_connected_changed(self, connected: bool):
        self._connect_btn.setEnabled(True)
        self._connect_btn.setText("Disconnect" if connected else "Connect")
        for btn in self._command_buttons():
            btn.setEnabled(connected)

    def _append_log(self, text: str):
        self._log.append(text)
        self._log.verticalScrollBar().setValue(
            self._log.verticalScrollBar().maximum()
        )

    def closeEvent(self, event):
        self._worker.disconnect()
        super().closeEvent(event)


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
