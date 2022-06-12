import cv2
import serial
import time

wasQRDetected = False
camEnable = False
cam = None
detector = None

nucleo = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

def nucleoSendValue(x):
    nucleo.write(bytes(f"{x}\n", 'utf-8'))
    time.sleep(0.05)

def video_reader():
    _, img = cam.read()
    data, bbox, _ = detector.detectAndDecode(img)
    if data:#  and wasQRDetected == False:
        nucleo.write(bytes(f"{data}\n", 'utf-8'))
        wasQRDetected = True
    cam.release()

if __name__ == "__main__":
    cam = cv2.VideoCapture(-1, cv2.CAP_V4L)
    detector = cv2.QRCodeDetector()
    while True:
        switch = nucleo.readline()
        if switch == "0\n":
            camEnable = False
        if switch == "1\n" or camEnable == True:
            camEnable = True
            video_reader()

