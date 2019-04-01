import socket
import json
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math
import imutils
import time 
import sys
import pidcontroller

x, y = 130, 220

pid = pidcontroller.PID(0.2, 0.0, 0.05)

sock = socket.socket()
sock.bind(('', 4219))
sock.listen(1)
out = "-1,-1,-1,-1"


marker_size = 0.03
KOFF = 0.4

markers = np.array()

field = np.array([[[15, 15], [50, 15], [85, 15], [120, 15], [155, 15], [190, 15], [225, 15], [260, 15]],
                [[15, 50], [50, 50], [85, 50], [120, 50], [155, 50], [190, 50], [225, 50], [260, 50]],
                [[15, 85], [50, 85], [85, 85], [120, 85], [155, 85], [190, 85], [225, 85], [260, 85]],
                [[15, 120], [50, 120], [85, 120], [120, 120], [155, 120], [190, 120], [225, 120], [260, 120]],
                [[15, 155], [50, 155], [85, 155], [120, 155], [155, 155], [190, 155], [225, 155], [260, 155]],
                [[15, 190], [50, 190], [85, 190], [120, 190], [155, 190], [190, 190], [225, 190], [260, 190]],
                [[15, 225], [50, 225], [85, 225], [120, 225], [155, 225], [190, 225], [225, 225], [260, 225]],
                [[15, 260], [50, 260], [85, 260], [120, 260], [155, 260], [190, 260], [225, 260], [260, 260]],
                [[15, 295], [50, 295], [85, 295], [120, 295], [155, 295], [190, 295], [225, 295], [260, 295]],
                [[15, 330], [50, 330], [85, 330], [120, 330], [155, 330], [190, 330], [225, 330], [260, 330]],
                [[15, 365], [50, 365], [85, 365], [120, 365], [155, 365], [190, 365], [225, 365], [260, 365]],
                [[15, 400], [50, 400], [85, 400], [120, 400], [155, 400], [190, 400], [225, 400], [260, 400]],
                [[15, 435], [50, 435], [85, 435], [120, 435], [155, 435], [190, 435], [225, 435], [260, 435]]])

frame_size = [336, 256]
frame_center = [int(frame_size[0]/2), int(frame_size[1]/2)]

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

cv_file = cv.FileStorage("calib_settings.yaml", cv.FILE_STORAGE_READ)
mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()

def constrain(x, a, b):
    if x < a:
        return a
    elif b < x:
        return b
    else:
        return x
def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
def rotate(val):
    if val > 359:
        val -= 359
        return val
    if val < 0:
        val += 359
        return val
    return val
def getCoordinates(m_id, dots):
    a = [int(dots[m_id][0][0][0]), int(dots[m_id][0][0][1])] # Two points in opposite corners
    b = [int(dots[m_id][0][2][0]), int(dots[m_id][0][2][1])]
    c = [int(dots[m_id][0][1][0]), int(dots[m_id][0][1][1])]
    center = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] # x, y coordinates of marker`s center from frame`s corner
    pos = [center[0] - frame_center[0], center[1] - frame_center[1]] # x, y coordinates of marker`s center from frame`s center
    angle = np.arctan2((c[1] - a[1]), (c[0] - a[0])) # Count angle to field
    mtx_pos = [field[np.where(markers == markers_id[m_id])][0][0], field[np.where(markers == markers_id[m_id])][0][1]] # Count marker`s position to field
    try:
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(dots, marker_size, mtx, dist)
        coordinates = [int(mtx_pos[0] - pos[0] * np.cos(angle) * KOFF - pos[1] * np.sin(angle) * KOFF), int(mtx_pos[1] - pos[1] * np.cos(angle) * KOFF + pos[0] * np.sin(angle) * KOFF), int(tvec[m_id][0][2] * 1000)] # Count frame`s center position to field
    except:
        coordinates = [int(mtx_pos[0] - pos[0] * np.cos(angle) * KOFF - pos[1] * np.sin(angle) * KOFF), int(mtx_pos[1] - pos[1] * np.cos(angle) * KOFF + pos[0] * np.sin(angle) * KOFF), -1] # Count frame`s center position to field
    #coordinates[0], coordinates[1] = int(coordinates[0]/2)*2, int(coordinates[1]/2)*2
    return coordinates, int(np.degrees(angle)) + 179

def find_nearest(ids, dots):
    distance = []
    for i in range(ids.size):
        a = [int(dots[i][0][0][0]), int(dots[i][0][0][1])] # Two points in opposite corners
        b = [int(dots[i][0][2][0]), int(dots[i][0][2][1])]  
        center = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] # x, y coordinates of marker`s center from frame`s corner
        pos = [center[0] - frame_center[0], center[1] - frame_center[1]] # x, y coordinates of marker`s center from frame`s center
        distance.append((pos[0]**2 + pos[1]**2)**0.5)
    return distance.index(min(distance)) # Nearest marker to frame`s center

def draw_info(frame_, position, ang):
    frame = frame_.copy()
    cv.putText(frame, "X: " + str(position[0]), (5, 80), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
    cv.putText(frame, "Y: " + str(position[1]), (5, 130), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
    cv.putText(frame, "Z: " + str(position[2]), (5, 180), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
    #print(position[2])
    cv.putText(frame, "ang: " + str(ang), (5, 230), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
    aruco.drawDetectedMarkers(frame, corners, markers_id)
    try:
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, marker_size, mtx, dist)
        aruco.drawAxis(frame, mtx, dist, rvec[nearest], tvec[nearest], marker_size)
    except:
        pass
    return frame

def cerc(frame_):
    frame = frame_.copy()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    global markers_id, corners, nearest, coordinates, angle
    corners, markers_id, rejectedImgPoint = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    coordinates = [-1, -1, -1]
    angle = -1
    try:
        if not(markers_id is None) and not(corners is None):
            nearest = find_nearest(markers_id, corners)
            coordinates, angle = getCoordinates(nearest, corners)
            frame = draw_info(frame, coordinates, angle)
    except:
        pass
    
    cv.circle(frame, (frame_center[0], frame_center[1]), 3, (0, 0, 255), -1)
    return frame, coordinates, angle
'''
def setAngle(needAng, angle):
    a_l = rotate(angle) - 180
    print(a_l, " ", end = "")
    a_l = pid.Update(a_l)
    print(int(a_l))
    return -int(a_l), int(a_l)'''
def setAngle(needAngle, angle):
    angle = rotate(angle - needAngle - 180)
    #print(angle)
    d = 25
    if angle > 181:
        return -d, d
    elif angle < 179:
        return d, -d
    else:
        return 0, 0
def forward(needAngle, angle):
    angle = rotate(rotate(angle - needAngle) - 180)
    #print(angle, end = ", ")
    go = 15
    d = 25
    if angle >= 270 or angle <= 90:
        angle = rotate(angle + 180)
        #print(angle)
        if angle > 190:
            return -d, d
        elif angle < 170:
            return d, -d
        else:
            return -d - go, -d - go
    else:
        #print(angle)
        if angle > 190:
            return -d, d
        elif angle < 170:
            return d, -d
        else:
            return d + go, d + go
while True:
    sock.settimeout(99999)
    conn, addr = sock.accept()
    conn_f = conn.makefile('rb')
    print(addr)
    data = b' '
    sock.settimeout(2)
    while True:
        try:
            data += conn_f.read(1024)
            first = data.find(b'\xff\xd8')
            last = data.find(b'\xff\xd9')        
            if first != -1 and last != -1:
                jpg = data[first:last + 2]
                data = data[last + 2:]
                frame = cv.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv.IMREAD_COLOR)
                #cv.imshow('PiCamera', frame)
                frame, coordinates, angle = cerc(frame)
                #frame = imutils.resize(frame, 1000)
                cv.imshow('Aruco', frame)
                #lspeed, rspeed = 0, 0
                #setAngle(90, angle)
                #lspeed, rspeed = setAngle(90, angle)
                
                x_l, y_l = coordinates[0] - x, coordinates[1] - y
                #print(x_l, y_l)
                if abs(x_l) >= 10 or abs(y_l) >= 10:
                    needAngle = int(constrain(abs(math.acos(abs(x_l) / math.sqrt(y_l**2 + x_l**2)) * 180 / 3.141593), 0, 90))
                    if x_l < 0 and y_l >= 0:
                        needAngle += 90
                    elif x_l >= 0 and y_l >= 0:
                        needAngle = map(needAngle, 0, 90, 90, 0) + 180
                    elif x_l < 0 and y_l < 0:
                        needAngle = map(needAngle, 0, 90, 90, 0)
                    elif x_l >= 0 and y_l < 0:
                        needAngle += 270
                    #print(needAngle)
                    lspeed, rspeed = forward(needAngle, angle)
                else:
                    lspeed, rspeed = 0, 0
                    
                #lspeed, rspeed = 0, 0
                conn.send((',' + str(rspeed) + ',,' + str(lspeed) + ',\n').zfill(20).encode())
                time.sleep(0.01)
                cv.waitKey(1)
        except:
            cv.destroyAllWindows()
            break
        #conn.send(out.encode())
    cv.destroyAllWindows()
conn.close()
