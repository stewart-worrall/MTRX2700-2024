# Import required module 
from threading import *
import sys
import glob
from serial import Serial, SerialException
import time

# Function to list serial ports
def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = Serial(port)
            s.close()
            result.append(port)
        except (OSError, SerialException):
            pass
    return result
  
# Serial Reader Thread class 
class SerialReader(Thread): 

    def __init__(self, port, baud_rate=115200, timeout=1):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.lines_read = 0

    # Target function for thread 
    def run(self): 
        try:
            ser = Serial(self.port, self.baud_rate, timeout=self.timeout)

            while True:
                data = ser.readline()
                self.lines_read += 1
                print (self.port + " received data: " + str(data))

        except Exception as e:
            print(f"Error reading from serial port: {e}")
  

# Get list of serial ports
serial_ports = serial_ports()
print(f"Serial ports found: {serial_ports}")

# Make an array of the threads
port_threads = []

# Creating thread class object 
for serial_port in serial_ports:
    port_threads.append(SerialReader(port=serial_port))

# Start each thread
for port_thread in port_threads:
    port_thread.start()

# Executed by main thread 
# this is here to show that the main thread is still running despite the serial reading
for i in range(10): 
    print('Main Thread is still running')
    print(f"Lines read by threads: {[thread.lines_read for thread in port_threads]}")
    time.sleep(1)

# Wait until all threads are done
for port_thread in port_threads:
    port_thread.join()