import socket
import conLib
import time
import sys
import cv2

try:
    addr = sys.argv[1]
except:
    addr = "192.168.1.123"
    
frame_size = [336, 256]
cap = cv2.VideoCapture(0)

session = conLib.client(addr, 4219)
while True:
    if session.connect():
        break
print(addr)
time.sleep(1)

while True:
    ret, frame = cap.read()
    session.send(frame)
    data = session.read()
    print(data)
    if not(data is None):
        print(data.encode())
    else:
        print(",0,,0,\n")
    