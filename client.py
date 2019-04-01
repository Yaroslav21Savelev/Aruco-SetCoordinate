from picamera.array import PiRGBArray
from picamera import PiCamera
import socket
import io
import struct
import time
import serial
import sys

addr = sys.argv[1]
print(addr)
frame_size = [336, 256]
camera = PiCamera()
camera.resolution = (frame_size[0], frame_size[1])
camera.framerate = 15
rawCapture = PiRGBArray(camera, size=(frame_size[0], frame_size[1]))
port = serial.Serial("/dev/serial0", baudrate = 57600, timeout = 1.0)

while True:
    try:
        sock = socket.socket()
        sock.connect((addr, 9090))
        conn = sock.makefile('wb')
        time.sleep(10)                       # give 2 secs for camera to initilize
        stream = io.BytesIO()
        sock.settimeout(1)

        for frame_ in camera.capture_continuous(stream, 'jpeg', use_video_port = True):

            conn.write(struct.pack('<L', stream.tell()))
            #conn.flush()
            stream.seek(0)
            conn.write(stream.read())
            stream.seek(0)
            stream.truncate()
            try:
                data = sock.recv(20).decode().lstrip('0')
                print(data)
                port.write(data.encode())
            except:
                pass
            rawCapture.truncate(0)

    except:
        pass
