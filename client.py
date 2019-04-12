from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import sys
import cv2
import serial
import sys
import conLib

try:
    addr = sys.argv[1]
except:
    addr = "192.168.1.123"

frame_size = [336, 256]
camera = PiCamera()
camera.resolution = (frame_size[0], frame_size[1])
camera.framerate = 15
rawCapture = PiRGBArray(camera, size=(frame_size[0], frame_size[1]))
port = serial.Serial("/dev/serial0", baudrate = 57600, timeout = 1.0)

session = conLib.client(addr, 4219)
while True:
    if session.connect():
        break
print(addr)
time.sleep(1)

for i in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = rawCapture.array
    session.send(frame)
    data = session.read()
    print(data)
    if not(data is None):
        port.write(data.encode())
    else:
        port.write(",0,,0,\n")
    rawCapture.truncate(0)
camera.close()