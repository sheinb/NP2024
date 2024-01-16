#
# scope.py
#
#  create a basic oscilloscope like interface to control the
#  Teensy spike generator
#
from turtle import bgcolor
from PySide6 import QtCore, QtWidgets, QtSerialPort, QtGui
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from random import randint
import numpy as np
import struct
import ctypes
from pathlib import Path
import glob
import platform
import serial.tools.list_ports
import socket
import sys


data_folder = f"/tmp/"
analysis_folder = f"/shared/enc23/analysis"


class Widget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        self.reset_button = QtWidgets.QPushButton(
            text="Reset Graph", checkable=False, clicked=self.on_reset
        )
        self.connect_button = QtWidgets.QPushButton(
            text="Connect", checkable=True, clicked=self.on_clicked
        )
        self.connect_button.setCheckable(True)
        self.connect_path = QtWidgets.QComboBox()
        self.connect_path.currentIndexChanged.connect(self.set_connect_path)
        self.connect_frame = QtWidgets.QHBoxLayout()
        self.connect_frame.addWidget(self.reset_button)
        self.connect_frame.addWidget(self.connect_button)
        self.connect_frame.addWidget(self.connect_path)

        self.saveanalog_button = QtWidgets.QPushButton(text="Analog", checkable=True)
        self.save_frame = QtWidgets.QHBoxLayout()
        self.save_frame.addWidget(self.saveanalog_button)

        self.stream_path = QtWidgets.QLineEdit()
        self.stream_path.setPlaceholderText("filename")
        self.stream_path.textChanged.connect(self.set_stream_path)

        self.frame_count = 0  # keep track of each receive for updating
        self.update_every = 50

        # initialize the serial port at the end
        self.serial = None

        self.show_params = False
        self.reset_params = False

        self.analog_file = None
        self.filename = "filename"
        self.in_obs = False
        self.obs_id = -1

        self.graphWidget = pg.PlotWidget(width=600, height=200)
        self.graphWidget.setWindowTitle
        self.x = list(np.arange(1000) * 0.5)  # 2 seconds of data
        self.y = list(np.repeat(2048.0, 1000))  # 1000 points
        self.graphWidget.setBackground("k")
        pen = pg.mkPen(color=(255, 0, 0))
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)
        xax = self.graphWidget.getAxis("bottom")
        xax.setTicks([])
        self.graphWidget.setDownsampling(auto=True, mode="peak")
        self.graphWidget.getViewBox().setYRange(0, 4096)

        self.graphWidget.showGrid(x=True, y=True)

        self.inobs_button = QtWidgets.QPushButton(
            text="Obs Sync", checkable=False, clicked=self.set_obs
        )
        self.inobs_button.setEnabled(False)

        self.inobs_button.setFixedSize(QtCore.QSize(120, 25))
        self.inobs_button
        self.inobs_label = QtWidgets.QLabel("Obs:")
        self.inobs_status = QtWidgets.QLabel(" " * 10)
        self.inobs_status.setAlignment(QtCore.Qt.AlignCenter)
        # setting up border and radius
        self.inobs_status.setStyleSheet("border: 0px")
        self.status_frame = QtWidgets.QHBoxLayout()

        self.status_frame.addWidget(self.inobs_button)
        self.status_frame.addWidget(self.inobs_label)
        self.status_frame.addWidget(self.inobs_status)
        self.status_frame.addStretch()

        row = 0
        grid = QtWidgets.QGridLayout(self)
        grid.setColumnMinimumWidth(0, 100)
        grid.addLayout(self.status_frame, row, 0)
        grid.addLayout(self.connect_frame, row, 1)
        row += 1

        grid.addLayout(self.save_frame, row, 0)
        grid.addWidget(self.stream_path, row, 1)
        row += 1

        grid.addWidget(self.graphWidget, row, 0, 1, 2)
        row += 1

        self.serial = QtSerialPort.QSerialPort(
            None, baudRate=234000, readyRead=self.receive
        )

        self.refresh_ports()
        self.obs_start_time = 0

        if platform.node() == "enc1":
            self.ipaddr = "192.168.88.242"
        elif platform.node() == "enc2":
            self.ipaddr = "192.168.88.236"
        elif platform.node() == "enc3":
            self.ipaddr = "192.168.88.238"
        elif platform.node() == "enc4":
            self.ipaddr = "192.168.88.240"

        self.port = 2570

    # Commands for communicating with Rpi over TCP/IP socket (port 2570)
    def send_qpcsh_command(self, cmd):
        # Create a control socket for sending commands
        qpcsh_address = (self.ipaddr, self.port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(qpcsh_address)
        sock.sendall(cmd.encode("UTF-8"))
        result = sock.makefile().readline()
        sock.close()
        return result

    ### Enumerate Serial Ports
    def refresh_ports(self):
        """find serial ports and return default"""
        ports = [
            comport.device
            for comport in serial.tools.list_ports.comports()
            if comport.description.startswith("FeatherS3")
        ]
        self.connect_path.addItems(ports)
        if ports:
            self.connect_path.setCurrentIndex(len(ports) - 1)

    def set_stream_path(self):
        self.filename = self.stream_path.text()

    def set_connect_path(self):
        self.serial.setPortName(self.connect_path.currentText())

    def set_param(self, param, value):
        if self.serial and self.serial.isOpen():
            msg = f"{param}={value}\n"
            self.serial.write(msg.encode())

    def receive(self):
        navail = self.serial.bytesAvailable()
        if navail:
            databytes = self.serial.read(navail)

            if databytes and self.saveanalog_button.isChecked():
                self.analog_file.write(databytes.data())

            for val in databytes.data().decode("utf-8").split("\r\n"):
                if val:
                    fval = float(val)
                    #                    print(val)

                    self.y = self.y[1:] + [fval]
                    self.frame_count += 1
                    if self.frame_count % self.update_every == 0:
                        self.data_line.setData(self.x, self.y)  # Update the data.

    def set_obs(self):
        if self.inobs_button.isChecked():
            self.inobs_button.setText("Stop Recording")
            self.set_param("simulate_sync", 1)
            self.set_param("in_obs", 1)
        else:
            self.set_param("in_obs", 0)
            self.inobs_button.setText("Start Recording")

    def on_reset(self):
        self.graphWidget.getViewBox().setYRange(0, 4096)

    def on_clicked(self):
        checked = self.connect_button.isChecked()
        self.connect_button.setText("Disconnect" if checked else "Connect")
        if checked:
            if not self.serial.isOpen():
                # On Windows we open and close the serial port using PySerial to initialize
                if platform.system() == "Windows":
                    ser = serial.Serial(w.connect_path.currentText(), 9600)
                    ser.close()

                if not self.serial.open(QtCore.QIODevice.ReadWrite):
                    self.connect_button.setChecked(False)
            self.serial.clear()
            if self.saveanalog_button.isChecked():
                print(f'opening datafile {data_folder + self.filename + "_raw.dat"}')
                self.analog_file = open(data_folder + self.filename + "_raw.dat", "wb")
            self.stream_path.setEnabled(False)
            self.saveanalog_button.setEnabled(False)
        else:
            self.serial.close()
            self.stream_path.setEnabled(True)
            self.saveanalog_button.setEnabled(True)


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    app.setWindowIcon(QtGui.QIcon("assets/spikes.png"))
    w = Widget()
    w.resize(800, 400)
    w.show()

    sys.exit(app.exec())
