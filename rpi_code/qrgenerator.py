import qrcode

def qrGenerate(msg, filename):
    code = qrcode.make(msg)
    code.save('pictures/'+filename +'.png')

if __name__ == '__main__':
    qrGenerate("0", 'off')
