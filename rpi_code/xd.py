import serial
import time
import cv2
import omxplayer
from pathlib import Path
import os

AUDIO_PATH = "/home/pi/Kamera/rpi_code/audio/"

class Nucleo:
    def __init__(self):
        self.com_port = serial.Serial(port='/dev/ttyS0', baudrate=115200, timeout=.1)
        self.msg_dict = {
            "1": "START",
            "2": "STOP",
            "3": "OBSTACLE",
            "4": "NFC",
            "5": "DONE",
            "6": "FREE"
        }
        self.inv_msg_dict = dict((v,k) for k,v in self.msg_dict.items())

    def dataAvailable(self):
        return self.com_port.inWaiting() > 0

    def sendMsg(self, msg):
        print(f"TX: {msg}")
        val = self.inv_msg_dict[msg]
        self.com_port.write(bytes(f"{val}\r\n", 'utf-8'))

    def readMsg(self):
        val = self.com_port.read(1).decode("utf-8")
        self.com_port.reset_input_buffer()
        msg = self.msg_dict[val]
        print(f"RX: {msg}")
        return msg



class AudioPlayer:
    def __init__(self):
        self.is_audio_on = 0
        self.songs = {
            "engine": Path( AUDIO_PATH + "engine.mp3"),
            "star": Path(AUDIO_PATH + "star.mp3"),
            "horn": Path(AUDIO_PATH + "horn.mp3"),
            "0": Path(AUDIO_PATH + "yolo.mp3"),
            "1": Path(AUDIO_PATH + "xd.mp3")
        }
        self.player = omxplayer.player.OMXPlayer(self.songs["engine"], pause=True)

    def is_playing(self):
        try:
            return self.player.is_playing()
        except Exception:
            return False



    def stop(self):
        try:
            if self.player.is_playing():
                self.player.pause()
        except Exception:
            pass

    def play(self, song):
        # self.stop()
        self.player.load(self.songs[song], pause=False)


class Reader:
    def __init__(self):
        self.cam = cv2.VideoCapture(-1, cv2.CAP_V4L)
        # self.detector = cv2.QRCodeDetector()

    def __del__(self):
        self.cam.release()

    def readQr(self,timeout):
        start = time.time()

        while time.time()-start < timeout:
            time.sleep(1)
            _, img = self.cam.read()
            detector = cv2.QRCodeDetector()
            data, bbox, _ = detector.detectAndDecode(img)
            print(f"DATA: {data}")
            if data == "0" or data == "1":
                return data
        return -1




if __name__ == "__main__":
    nucleo = Nucleo()
    audio = AudioPlayer()
    reader = Reader()

    while True:
        if nucleo.dataAvailable():
            msg = nucleo.readMsg()

            if msg == "START":
                audio.play("engine")
            
            elif msg == "STOP":
                audio.stop()

            elif msg == "OBSTACLE":
                audio.play("horn")

            elif msg == "FREE":
                audio.play("engine")
            
            elif msg == "NFC":
                audio.stop()
                qr = reader.readQr(10)
                if qr != -1:
                    print(f"QR DETECTED {qr}")
                    audio.play(qr)
                while audio.is_playing():
                    pass
                audio.play("engine")
                nucleo.sendMsg("DONE")
                
            
        
