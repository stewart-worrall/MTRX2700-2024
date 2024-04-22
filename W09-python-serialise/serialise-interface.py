import serial
import struct
import time
from enum import Enum

from colorama import init, Fore, Style




# Initialize colorama
init()

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
        MessageType.SENSOR_DATA: 8 * 4,   # 8 * uint32_t
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


# Adjust the serial port and baud rate as needed
ser = serial.Serial('/dev/ttyACM0', 115200)

# Toggle the state every 0.5 seconds so that the LED blinks
toggle_value = False
toggle_interval = 0.5  # Toggle interval in seconds
next_toggle_time = time.time() + toggle_interval

# Loop for continuously receiving sensor data
while True:

    message_type, data = receive_and_unpack_buffer(ser)

    if message_type == MessageType.SENSOR_DATA:
        sensor_data = struct.unpack('<iiiiiiII', data)
        print('Sensor:', sensor_data)

    elif message_type == MessageType.BUTTON_AND_STATUS:
        sensor_data = struct.unpack('<B', data)

        output_color = Fore.RED
        
        if (sensor_data[0] & 0x01 == 1):
            output_color = Fore.GREEN
        # Print with the selected color
        #print(output_color + "This is some colored output." + Style.RESET_ALL)
        print(output_color + 'Button:' + str(sensor_data) + Style.RESET_ALL)

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

    # Sleep for a while to avoid flooding the output (adjust the sleep time as needed)
    #time.sleep(0.01)

# Close the serial port when done
ser.close()