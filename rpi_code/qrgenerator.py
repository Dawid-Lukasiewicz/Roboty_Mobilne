import qrcode

def qrGenerate(msg, filename):
    code = qrcode.make(msg)
    code.save('pictures/'+filename +'.png')

if __name__ == '__main__':
    qrGenerate("Hello world!", 'qr1')
