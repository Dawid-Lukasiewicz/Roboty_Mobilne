import qrtools
from qrtools.qrtools import QR
import png

text = "Hello world!"

def qr_generate(text):
    qr = QR(text)
    qr.encode()
    print(qr.filename)

def qr_read(msg):
    pass

if __name__ == "__main__":
    qr_generate(text)