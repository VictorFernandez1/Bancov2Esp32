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
import re
import sys
import threading
import time
from collections import deque

from bleak import BleakClient, BleakScanner
from PySide6.QtCore import Qt, QObject, QPointF, Signal
from PySide6.QtGui import QPainter, QPen
from PySide6.QtWidgets import (
    QApplication, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QSpinBox, QTextEdit, QVBoxLayout, QWidget,
)

# ── BLE constants ──────────────────────────────────────────────────────────────
DEVICE_NAME      = "ESP32_STEPPER"
CMD_CHAR_UUID    = "AA000002-1234-1234-1234-1234567890AA"
STATUS_CHAR_UUID = "AA000003-1234-1234-1234-1234567890AA"
TEMP_CHAR_UUID   = "AA000004-1234-1234-1234-1234567890AA"


# ── Thread-safe bridge between asyncio worker and Qt UI ───────────────────────

class _Signals(QObject):
    """Signals emitted from the asyncio thread, received in the Qt main thread."""
    status_changed      = Signal(str)   # human-readable status line
    notification        = Signal(str)   # raw notification value from ESP32
    temperature_sample  = Signal(float, float)  # timestamp, temperature C
    connected           = Signal(bool)  # True = connected, False = disconnected


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
        except Exception as e:
            self._signals.status_changed.emit(f"Connection failed: {e}")
            self._client = None
            self._signals.connected.emit(False)
            return

        try:
            await self._client.start_notify(STATUS_CHAR_UUID, self._on_notification)
        except Exception as e:
            self._signals.status_changed.emit(f"Failed to subscribe to status: {e}")

        try:
            await self._client.start_notify(TEMP_CHAR_UUID, self._on_temp_notification)
        except Exception as e:
            self._signals.status_changed.emit(f"Failed to subscribe to temperature: {e}")

        self._signals.status_changed.emit(f"Connected  ({device.address})")
        self._signals.connected.emit(True)

    async def _disconnect(self):
        if self._client and self._client.is_connected:
            await self._client.disconnect()

    async def _send(self, cmd: str):
        if self._client and self._client.is_connected:
            await self._client.write_gatt_char(CMD_CHAR_UUID, cmd.encode("utf-8"))

    def _on_notification(self, _sender, data: bytearray):
        self._signals.notification.emit(data.decode("utf-8"))

    def _on_temp_notification(self, _sender, data: bytearray):
        text = data.decode("utf-8", errors="replace").strip()
        self._signals.notification.emit(text)

        match = re.search(r"[-+]?\d+(?:\.\d+)?", text)
        if match is None:
            return

        try:
            temperature_c = float(match.group(0))
        except ValueError:
            return

        self._signals.temperature_sample.emit(time.monotonic(), temperature_c)

    def _on_ble_disconnect(self, _client: BleakClient):
        self._client = None
        self._signals.status_changed.emit("Disconnected")
        self._signals.connected.emit(False)


# ── Temperature plot widget ───────────────────────────────────────────────────

class TemperaturePlotWidget(QWidget):
    """Custom lightweight plot for the last 120 seconds of temperature data."""

    WINDOW_SECONDS = 120.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self._samples: deque[tuple[float, float]] = deque()
        self.setMinimumHeight(200)

    def add_sample(self, timestamp_s: float, temperature_c: float):
        self._samples.append((timestamp_s, temperature_c))
        self._trim(timestamp_s)
        self.update()

    def _trim(self, now_s: float):
        cutoff = now_s - self.WINDOW_SECONDS
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        rect = self.rect()
        painter.fillRect(rect, Qt.black)

        left = 48
        right = 16
        top = 16
        bottom = 28
        plot_w = max(10, rect.width() - left - right)
        plot_h = max(10, rect.height() - top - bottom)

        y_tick_count = 9
        x_tick_count = 13

        painter.setPen(QPen(Qt.darkGray, 1))
        for i in range(y_tick_count):
            y = top + int((plot_h * i) / (y_tick_count - 1))
            painter.drawLine(left, y, left + plot_w, y)
        for i in range(x_tick_count):
            x = left + int((plot_w * i) / (x_tick_count - 1))
            painter.drawLine(x, top, x, top + plot_h)

        painter.setPen(QPen(Qt.white, 1))
        painter.drawRect(left, top, plot_w, plot_h)

        if len(self._samples) < 2:
            painter.setPen(QPen(Qt.lightGray, 1))
            painter.drawText(left + 8, top + 20, "Waiting for temperature samples...")
            painter.drawText(left, rect.height() - 8, "Time window: last 120 s")
            return

        now_s = self._samples[-1][0]
        self._trim(now_s)

        min_t = min(t for _, t in self._samples)
        max_t = max(t for _, t in self._samples)
        span_t = max(0.5, max_t - min_t)

        margin = max(0.2, span_t * 0.12)
        y_min = min_t - margin
        y_max = max_t + margin
        y_span = max(0.1, y_max - y_min)

        points: list[QPointF] = []
        x_start = now_s - self.WINDOW_SECONDS
        for ts, temp in self._samples:
            x_norm = (ts - x_start) / self.WINDOW_SECONDS
            x_norm = max(0.0, min(1.0, x_norm))
            y_norm = (temp - y_min) / y_span
            y_norm = max(0.0, min(1.0, y_norm))

            x = left + (x_norm * plot_w)
            y = top + ((1.0 - y_norm) * plot_h)
            points.append(QPointF(x, y))

        painter.setPen(QPen(Qt.cyan, 2))
        for i in range(1, len(points)):
            painter.drawLine(points[i - 1], points[i])

        painter.setPen(QPen(Qt.lightGray, 1))
        for i in range(y_tick_count):
            y = top + int((plot_h * i) / (y_tick_count - 1))
            temp_label = y_max - ((y_max - y_min) * i / (y_tick_count - 1))
            painter.drawLine(left - 4, y, left, y)
            painter.drawText(4, y + 4, f"{temp_label:.1f}")

        for i in range(x_tick_count):
            x = left + int((plot_w * i) / (x_tick_count - 1))
            sec_ago = int(self.WINDOW_SECONDS - (self.WINDOW_SECONDS * i / (x_tick_count - 1)))
            painter.drawLine(x, top + plot_h, x, top + plot_h + 4)
            label = "now" if sec_ago == 0 else f"-{sec_ago}s"
            painter.drawText(x - 14, rect.height() - 8, label)


# ── Main window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Stepper Controller")
        self.setMinimumWidth(640)

        self._signals = _Signals()
        self._worker  = BleWorker(self._signals)
        self._worker.start()

        self._signals.status_changed.connect(self._on_status)
        self._signals.notification.connect(self._on_notification)
        self._signals.temperature_sample.connect(self._on_temperature_sample)
        self._signals.connected.connect(self._on_connected_changed)

        self._build_ui()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        top_layout = QVBoxLayout(root)
        top_layout.setSpacing(8)
        top_layout.setContentsMargins(12, 12, 12, 12)

        # ── Main area: controls left, temperature right ─────────────────────
        main_row = QHBoxLayout()
        main_row.setSpacing(10)

        # -- Left column: motor controls --
        left_col = QVBoxLayout()
        left_col.setSpacing(8)

        # Connection group
        conn_box    = QGroupBox("Connection")
        conn_layout = QVBoxLayout(conn_box)
        conn_layout.setContentsMargins(8, 8, 8, 8)

        self._status_label = QLabel("Not connected")
        self._status_label.setAlignment(Qt.AlignCenter)

        self._connect_btn = QPushButton("Connect")
        self._connect_btn.setFixedHeight(32)
        self._connect_btn.clicked.connect(self._toggle_connect)

        conn_layout.addWidget(self._status_label)
        conn_layout.addWidget(self._connect_btn)
        left_col.addWidget(conn_box)

        # Linear motor group
        linear_box    = QGroupBox("Linear Motor")
        linear_layout = QHBoxLayout(linear_box)
        linear_layout.setContentsMargins(8, 8, 8, 8)

        self._btn_in   = self._make_cmd_btn("◀  Move In",   "MOVEIN")
        self._btn_out  = self._make_cmd_btn("▶  Move Out",  "MOVEOUT")
        self._btn_in_home = self._make_cmd_btn("Move In Home",    "MOVEINHOME")
        self._btn_out_home = self._make_cmd_btn("Move Out Home",   "MOVEOUTHOME")
        linear_layout.addWidget(self._btn_in)
        linear_layout.addWidget(self._btn_out)
        linear_layout.addWidget(self._btn_in_home)
        linear_layout.addWidget(self._btn_out_home)
        left_col.addWidget(linear_box)

        # Rotational motor group
        rot_box    = QGroupBox("Rotational Motor")
        rot_layout = QHBoxLayout(rot_box)
        rot_layout.setContentsMargins(8, 8, 8, 8)

        self._btn_cw  = self._make_cmd_btn("↻  Clockwise",        "MOVECLOCKWISE")
        self._btn_ccw = self._make_cmd_btn("↺  Counterclockwise", "MOVECOUNTERCLOCKWISE")
        self._btn_rot_home = self._make_cmd_btn("Find Home", "ROTATIONALHOMING")
        rot_layout.addWidget(self._btn_cw)
        rot_layout.addWidget(self._btn_ccw)
        rot_layout.addWidget(self._btn_rot_home)
        left_col.addWidget(rot_box)

        # Utility row: Sensor + Fan + Stop + Interval
        util_row = QHBoxLayout()
        util_row.setSpacing(8)

        # Position sensor group
        sensor_box    = QGroupBox("Sensor")
        sensor_layout = QVBoxLayout(sensor_box)
        sensor_layout.setContentsMargins(6, 6, 6, 6)
        self._btn_sensor_query = QPushButton("Check")
        self._btn_sensor_query.setFixedHeight(36)
        self._btn_sensor_query.setEnabled(False)
        self._btn_sensor_query.clicked.connect(lambda: self._worker.send_command("SENSORGPIO5"))
        sensor_layout.addWidget(self._btn_sensor_query)
        util_row.addWidget(sensor_box)

        # Fan group
        fan_box    = QGroupBox("Fan")
        fan_layout = QHBoxLayout(fan_box)
        fan_layout.setContentsMargins(6, 6, 6, 6)
        self._btn_fan_on  = self._make_cmd_btn("On",  "FANON")
        self._btn_fan_off = self._make_cmd_btn("Off", "FANOFF")
        fan_layout.addWidget(self._btn_fan_on)
        fan_layout.addWidget(self._btn_fan_off)
        util_row.addWidget(fan_box)

        # Stop button
        stop_box = QGroupBox("Emergency")
        stop_box_layout = QVBoxLayout(stop_box)
        stop_box_layout.setContentsMargins(6, 6, 6, 6)
        self._btn_stop = QPushButton("\u25a0  Stop")
        self._btn_stop.setFixedHeight(36)
        self._btn_stop.setEnabled(False)
        self._btn_stop.setStyleSheet("QPushButton { background-color: #c0392b; color: white; font-weight: bold; }"
                                     "QPushButton:disabled { background-color: #7f8c8d; color: #bdc3c7; }")
        self._btn_stop.clicked.connect(lambda: self._worker.send_command("STOP"))
        stop_box_layout.addWidget(self._btn_stop)
        util_row.addWidget(stop_box)

        # Pulse interval group
        interval_box    = QGroupBox("Interval")
        interval_layout = QVBoxLayout(interval_box)
        interval_layout.setContentsMargins(6, 6, 6, 6)

        interval_layout.addWidget(QLabel("Interval (\u00b5s):"))
        self._interval_spin = QSpinBox()
        self._interval_spin.setRange(300, 50000)
        self._interval_spin.setValue(1500)
        self._interval_spin.setSingleStep(100)
        self._interval_spin.setEnabled(False)
        interval_layout.addWidget(self._interval_spin)

        self._btn_set_interval = QPushButton("Set")
        self._btn_set_interval.setFixedHeight(32)
        self._btn_set_interval.setEnabled(False)
        self._btn_set_interval.clicked.connect(
            lambda: self._worker.send_command(f"SETINTERVAL:{self._interval_spin.value()}")
        )
        interval_layout.addWidget(self._btn_set_interval)
        util_row.addWidget(interval_box)

        left_col.addLayout(util_row)
        main_row.addLayout(left_col, 3)

        # -- Right column: temperature --
        temp_box    = QGroupBox("Temperature")
        temp_layout = QVBoxLayout(temp_box)
        temp_layout.setContentsMargins(8, 8, 8, 8)

        self._temp_label = QLabel("--.- C")
        self._temp_label.setAlignment(Qt.AlignCenter)
        self._temp_label.setStyleSheet("QLabel { font-size: 22px; font-weight: bold; }")
        temp_layout.addWidget(self._temp_label)

        self._temp_plot = TemperaturePlotWidget()
        self._temp_plot.setMinimumHeight(140)
        temp_layout.addWidget(self._temp_plot)
        main_row.addWidget(temp_box, 2)

        top_layout.addLayout(main_row)

        # ── Log at bottom ────────────────────────────────────────────────────
        log_box    = QGroupBox("Log")
        log_layout = QVBoxLayout(log_box)
        self._log  = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setFixedHeight(80)
        log_layout.addWidget(self._log)
        top_layout.addWidget(log_box)

    def _make_cmd_btn(self, label: str, command: str) -> QPushButton:
        btn = QPushButton(label)
        btn.setFixedHeight(36)
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

    def _on_temperature_sample(self, timestamp_s: float, temperature_c: float):
        self._temp_label.setText(f"{temperature_c:.2f} C")
        self._temp_plot.add_sample(timestamp_s, temperature_c)

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
