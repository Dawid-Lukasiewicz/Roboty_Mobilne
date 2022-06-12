# File for testin arduino - file script connection

import serial
import time
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)
def arduinoSendValue(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    print(data)

while True:
    num = input("Enter a number between 0-100: ")
    arduinoSendValue(num)
