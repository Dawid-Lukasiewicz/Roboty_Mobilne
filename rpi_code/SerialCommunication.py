# File for testin arduino - file script connection

import serial
import time

nucleo = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

def nucleoSendValue(x):
    nucleo.write(bytes(f"{x}\n", 'utf-8'))
    time.sleep(0.05)

