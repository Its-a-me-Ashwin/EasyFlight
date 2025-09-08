import sys
import struct
import serial
import threading
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

START_BYTE = 0xAA

class TelemetryReader(QtCore.QObject):
    data_received = QtCore.pyqtSignal(dict)

    def __init__(self, port="COM6", baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.running = True
            threading.Thread(target=self.read_loop, daemon=True).start()
        except Exception as e:
            print(f"[ERROR] Could not open serial port: {e}")

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

    def read_loop(self):
        while self.running and self.ser:
            frame = self.read_frame()
            if frame:
                self.data_received.emit(frame)

    def read_frame(self):
        # Sync to start byte
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b[0] == START_BYTE:
                break

        # Read length
        length_bytes = self.ser.read(1)
        if not length_bytes:
            return None
        length = length_bytes[0]

        # Read payload + checksum
        payload = self.ser.read(length + 1)
        if len(payload) < length + 1:
            return None

        data_bytes, checksum = payload[:-1], payload[-1]

        # Verify checksum
        calc = 0
        for x in data_bytes:
            calc ^= x
        if calc != checksum:
            return None

        # Parse fields
        offset = 0

        def get16():
            nonlocal offset
            val = struct.unpack_from("<H", data_bytes, offset)[0]
            offset += 2
            return val

        def get32f():
            nonlocal offset
            val = struct.unpack_from("<f", data_bytes, offset)[0]
            offset += 4
            return val

        channels = [get16() for _ in range(6)]
        roll, pitch, yaw = get32f(), get32f(), get32f()
        roll_rate, pitch_rate, yaw_rate = get32f(), get32f(), get32f()
        roll_out, pitch_out, yaw_out = get32f(), get32f(), get32f()
        bad, lost = get16(), get16()

        return {
            "channels": channels,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "roll_rate": roll_rate, "pitch_rate": pitch_rate, "yaw_rate": yaw_rate,
            "roll_out": roll_out, "pitch_out": pitch_out, "yaw_out": yaw_out,
            "bad": bad, "lost": lost,
        }

# =============================
# GUI
# =============================
class MainWindow(QtWidgets.QWidget):
    def __init__(self, port="COM6", baud=115200):
        super().__init__()
        self.setWindowTitle("Flight Controller Telemetry")
        self.resize(1200, 800)

        layout = QtWidgets.QVBoxLayout(self)

        # === Channel bars ===
        self.channel_plots = []
        ch_layout = QtWidgets.QHBoxLayout()
        for i in range(6):
            plot = pg.PlotWidget(title=f"CH{i+1}")
            plot.setYRange(1000, 2000)
            curve = plot.plot([0, 1], [1500, 1500], pen="w")
            self.channel_plots.append((plot, curve))
            ch_layout.addWidget(plot)
        layout.addLayout(ch_layout)

        # === IMU Angles Plot ===
        self.imu_plot = pg.PlotWidget(title="IMU Angles (deg)")
        self.imu_plot.addLegend()
        self.imu_curves = {
            "roll": self.imu_plot.plot(pen="r", name="Roll"),
            "pitch": self.imu_plot.plot(pen="g", name="Pitch"),
            "yaw": self.imu_plot.plot(pen="b", name="Yaw"),
        }
        layout.addWidget(self.imu_plot)

        # === PID Outputs Plot ===
        self.pid_plot = pg.PlotWidget(title="PID Outputs (normalized)")
        self.pid_plot.addLegend()
        self.pid_plot.setYRange(-1, 1)  # fixed scale for visibility
        self.pid_curves = {
            "roll_out": self.pid_plot.plot(pen="m", name="RollOut"),
            "pitch_out": self.pid_plot.plot(pen="c", name="PitchOut"),
            "yaw_out": self.pid_plot.plot(pen="y", name="YawOut"),
        }
        layout.addWidget(self.pid_plot)

        # Data buffers
        self.data_buffer = {k: [] for k in list(self.imu_curves.keys()) + list(self.pid_curves.keys())}
        self.time_buffer = []

        # Telemetry Reader
        self.reader = TelemetryReader(port, baud)
        self.reader.data_received.connect(self.update_display)
        self.reader.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_display(self, data):
        # Update channel plots
        for i, (plot, curve) in enumerate(self.channel_plots):
            curve.setData([0, 1], [data["channels"][i]] * 2)

        # Append IMU/PID values
        t = QtCore.QTime.currentTime().msecsSinceStartOfDay() / 1000.0
        self.time_buffer.append(t)
        for k in self.data_buffer:
            self.data_buffer[k].append(data[k])

        # Keep buffer small
        if len(self.time_buffer) > 200:
            self.time_buffer.pop(0)
            for k in self.data_buffer:
                self.data_buffer[k].pop(0)

    def update_plot(self):
        for k, curve in self.imu_curves.items():
            curve.setData(self.time_buffer, self.data_buffer[k])
        for k, curve in self.pid_curves.items():
            curve.setData(self.time_buffer, self.data_buffer[k])


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    port = "COM6"
    baud = 115200
    win = MainWindow(port, baud)
    win.show()
    sys.exit(app.exec_())
