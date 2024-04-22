import sys

from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget, QPushButton, QSlider, QDial
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QPointF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QConicalGradient

import struct
import serial
import time
from enum import Enum


SENTINEL_1 = 0xAA
SENTINEL_2 = 0x55

class MessageType(Enum):
    SENSOR_DATA = 0
    LED_STATE = 1
    BUTTON_AND_STATUS = 2
    STRING_PACKET = 3

def pack_buffer(message_type, data):
    data_length = len(data)
    header = struct.pack('<BBHH', SENTINEL_1, SENTINEL_2, message_type, data_length)
    return header + data

def receive_and_unpack_buffer(ser):
    """
    data_length_map = {
        MessageType.SENSOR_DATA: 8 * 4,   # 6 * int32_t + 2 * uint32_t
        MessageType.LED_STATE: 1,         # 1 * uint8_t
        MessageType.BUTTON_AND_STATUS: 1, # 1 * uint8_t
    }
    """

    # Wait for the sentinel bytes
    while True:
        byte = ser.read(1)
        if byte == bytes([SENTINEL_1]):
            byte = ser.read(1)
            if byte == bytes([SENTINEL_2]):
                break

    # Read the remaining header bytes
    header_length = 4
    header_buffer = ser.read(header_length)
    header_buffer = bytes([SENTINEL_1, SENTINEL_2]) + header_buffer

    # Check the entire header was loaded
    if len(header_buffer) < 6:
        return None, None

    # unpack the header
    sentinel1, sentinel2, message_type_received, data_length = struct.unpack('<BBHH', header_buffer[:6])

    # double check the header
    if sentinel1 != SENTINEL_1 or sentinel2 != SENTINEL_2:
        return None, None

    # Read the data based on the data length specified in the header
    data_buffer = ser.read(data_length)
    return MessageType(message_type_received), data_buffer



class ColorfulDial(QDial):
    def __init__(self, parent=None):
        super().__init__(parent)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw the dial groove
        groove_rect = self.rect().adjusted(10, 10, -10, -10)
        groove_color = QColor(200, 200, 200)
        painter.setPen(QPen(groove_color, 4))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(groove_rect)

        # Draw the dial handle as an arrow
        value_angle = (self.value() - self.minimum()) / (self.maximum() - self.minimum()) * 270 - 135
        gradient = QConicalGradient(self.rect().center(), value_angle - 90)
        gradient.setColorAt(0, QColor(0, 255, 0))
        gradient.setColorAt(1, QColor(0, 0, 255))
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(gradient))

        handle_radius = (groove_rect.width() - 10) / 2
        arrow_size = handle_radius / 2
        painter.translate(self.rect().center())
        painter.rotate(value_angle)

        arrow_points = [
            QPointF(-arrow_size / 2, -handle_radius + arrow_size),
            QPointF(arrow_size / 2, -handle_radius + arrow_size),
            QPointF(0, -handle_radius),
        ]
        painter.drawPolygon(*arrow_points)

        painter.end()

class SerialReader(QThread):
    data_received = pyqtSignal(list)
    color_change = pyqtSignal(bool)

    def __init__(self, port, baud_rate=9600, timeout=1):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)

            # Toggle the state every 0.5 seconds so that the LED blinks
            toggle_value = False
            toggle_interval = 0.5  # Toggle interval in seconds
            next_toggle_time = time.time() + toggle_interval

            while True:
                message_type, data = receive_and_unpack_buffer(ser)

                if message_type == MessageType.SENSOR_DATA:
                    sensor_data = struct.unpack('<iiiiiiII', data)
                    self.data_received.emit(list(sensor_data))
                    #print('Sensor:', sensor_data)

                elif message_type == MessageType.BUTTON_AND_STATUS:
                    sensor_data = struct.unpack('<B', data)

                    if (sensor_data[0] & 0x01 == 1):
                        self.color_change.emit(True)
                    else:
                        self.color_change.emit(False)
                        

                current_time = time.time()

                if current_time >= next_toggle_time:
                    toggle_value = not toggle_value
                    next_toggle_time = current_time + toggle_interval

                if (toggle_value):
                    led_state = 0b01010101
                else:
                    led_state = 0b10101010
                
                # Example for sending LED state
                data = struct.pack('<B', led_state)
                buffer = pack_buffer(MessageType.LED_STATE.value, data)
                ser.write(buffer)


        except Exception as e:
            print(f"Error reading from serial port: {e}")



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.central_widget = QWidget()
        self.layout = QVBoxLayout(self.central_widget)

        self.button = QPushButton('Change Values', self.central_widget)
        self.button.clicked.connect(self.update_values)
        self.button.setMinimumHeight(50)  # Set the button's minimum height
        self.layout.addWidget(self.button)

        slider_style = '''
        QSlider::groove:horizontal {
            border: 1px solid #999999;
            height: 40px;
            background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4);
            margin: 2px 0;
        }
        QSlider::handle:horizontal {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #4444FF, stop:1 #6666FF);
            border: 1px solid #7777FF;
            width: 20px;
            margin: -2px 0;
            border-radius: 3px;
        }
        '''

        self.slider1 = QSlider(Qt.Horizontal, self.central_widget)
        self.slider1.setRange(0, 5000)
        self.slider1.setMinimumHeight(50)  # Set the slider's minimum height
        self.slider1.setStyleSheet(slider_style)  # Set the slider's style        
        self.layout.addWidget(self.slider1)

        self.slider2 = QSlider(Qt.Horizontal, self.central_widget)
        self.slider2.setRange(0, 5000)
        self.slider2.setMinimumHeight(50)  # Set the slider's minimum height
        self.slider2.setStyleSheet(slider_style)  # Set the slider's style        
        self.layout.addWidget(self.slider2)

        self.dials_layout = QHBoxLayout()
        self.layout.addLayout(self.dials_layout)

        self.dial1 = ColorfulDial(self.central_widget)
        self.dial1.setRange(-2000, 2000)
        self.dials_layout.addWidget(self.dial1)

        self.dial2 = ColorfulDial(self.central_widget)
        self.dial2.setRange(-2000, 2000)
        self.dials_layout.addWidget(self.dial2)

        self.dial3 = ColorfulDial(self.central_widget)
        self.dial3.setRange(-2000, 2000)
        self.dials_layout.addWidget(self.dial3)

        self.setWindowTitle("Dials and Sliders")
        self.setGeometry(100, 100, 800, 600)  # Change the window size on startup

        self.setCentralWidget(self.central_widget)

        self.start_serial_reading('/dev/ttyACM0')

    def update_values(self):
        self.slider1.setValue(1000)
        self.slider2.setValue(4000)
        self.dial1.setValue(-1000)
        self.dial2.setValue(500)
        self.dial3.setValue(1500)

    def start_serial_reading(self, port, baud_rate=115200, timeout=1):
        self.serial_reader = SerialReader(port, baud_rate, timeout)
        self.serial_reader.data_received.connect(self.handle_serial_data)
        self.serial_reader.color_change.connect(self.change_button_color)
        self.serial_reader.start()

    def change_button_color(self, change_color):
        if change_color:
            self.button.setStyleSheet('QPushButton { background-color: green; }')
        else:
            self.button.setStyleSheet('QPushButton { background-color: red; }')

    def handle_serial_data(self, data):
        #print(f"Data read from serial port: {data}")``
        self.dial1.setValue(data[3])
        self.dial2.setValue(data[4])
        self.dial3.setValue(data[5])
        self.slider1.setValue(data[6])
        self.slider2.setValue(data[7])


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec_())
