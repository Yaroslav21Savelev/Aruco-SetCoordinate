import conLib as connect;
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math
import imutils
import time 
import sys
x, y = 130, 220
out = "-1,-1,-1,-1"
marker_size = 0.03
KOFF = 0.4

markers = np.array([[0, 1, 2, 3, 4, 5, 6, 7],
                    [8, 9, 10, 11, 12, 13, 14, 15],
                    [16, 17, 18, 19, 20, 21, 22, 23],
                    [24, 25, 26, 27, 28, 29, 30, 31],
                    [32, 33, 34, 35, 36, 37, 38, 39],
                    [40, 41, 42, 43, 44, 45, 46, 47],
                    [48, 49, 50, 51, 52, 53, 54, 55],
                    [56, 57, 58, 59, 60, 61, 62, 63],
                    [64, 65, 66, 67, 68, 69, 70, 71],
                    [72, 73, 74, 75, 76, 77, 78, 79],
                    [80, 81, 82, 83, 84, 85, 86, 87],
                    [88, 89, 90, 91, 92, 93, 94, 95],
                    [96, 97, 98, 99, 100, 101, 102, 103]])

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
class arucoField():
    def __init__(self, frame_size):
        self.frame_size = frame_size
        self.frame_center = [int(frame_size[0]/2), int(frame_size[1]/2)]
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()
        cv_file = cv.FileStorage("calib_settings.yaml", cv.FILE_STORAGE_READ)
        self.mtx = cv_file.getNode("camera_matrix").mat()
        self.dist = cv_file.getNode("dist_coeff").mat()

    def getCoordinates(self, markers_id, m_id, dots):
        a = [int(dots[m_id][0][0][0]), int(dots[m_id][0][0][1])] # Two points in opposite corners
        b = [int(dots[m_id][0][2][0]), int(dots[m_id][0][2][1])]
        c = [int(dots[m_id][0][1][0]), int(dots[m_id][0][1][1])]
        center = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] # x, y coordinates of marker`s center from frame`s corner
        pos = [center[0] - self.frame_center[0], center[1] - self.frame_center[1]] # x, y coordinates of marker`s center from frame`s center
        angle = np.arctan2((c[1] - a[1]), (c[0] - a[0])) # Count angle to field
        
        mtx_pos = [field[np.where(markers == markers_id[m_id])][0][0], field[np.where(markers == markers_id[m_id])][0][1]] # Count marker`s position to field
        
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(dots, marker_size, self.mtx, self.dist)
        coordinates = [int(mtx_pos[0] - pos[0] * np.cos(angle) * KOFF - pos[1] * np.sin(angle) * KOFF), int(mtx_pos[1] - pos[1] * np.cos(angle) * KOFF + pos[0] * np.sin(angle) * KOFF), int(tvec[m_id][0][2] * 1000)] # Count frame`s center position to field
        #coordinates[0], coordinates[1] = int(coordinates[0]/2)*2, int(coordinates[1]/2)*2
        return coordinates, int(np.degrees(angle)) + 179

    def find_nearest(self, ids, dots):
        distance = []
        for i in range(ids.size):
            a = [int(dots[i][0][0][0]), int(dots[i][0][0][1])] # Two points in opposite corners
            b = [int(dots[i][0][2][0]), int(dots[i][0][2][1])]  
            center = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] # x, y coordinates of marker`s center from frame`s corner
            pos = [center[0] - self.frame_center[0], center[1] - self.frame_center[1]] # x, y coordinates of marker`s center from frame`s center
            distance.append((pos[0]**2 + pos[1]**2)**0.5)
        return distance.index(min(distance)) # Nearest marker to frame`s center

    def draw_info(self, frame_, dots, markers_id, nearest, position, ang):
        frame = frame_.copy()
        
        cv.putText(frame, "X: " + str(position[0]), (5, 80), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
        cv.putText(frame, "Y: " + str(position[1]), (5, 130), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
        cv.putText(frame, "Z: " + str(position[2]), (5, 180), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
        #print(position[2])
        cv.putText(frame, "ang: " + str(ang), (5, 230), cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3, cv.LINE_AA)
        
        aruco.drawDetectedMarkers(frame, dots, markers_id)
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(dots, marker_size, self.mtx, self.dist)
        aruco.drawAxis(frame, self.mtx, self.dist, rvec[nearest], tvec[nearest], marker_size)
        return frame

    def main(self, frame_):
        frame = frame_.copy()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, markers_id, rejectedImgPoint = aruco.detectMarkers(gray, self.aruco_dict, parameters = self.parameters)
        coordinates = [-1, -1, -1]
        angle = -1
        if not(markers_id is None) and not(corners is None):
            nearest = self.find_nearest(markers_id, corners)
            coordinates, angle = self.getCoordinates(markers_id, nearest, corners)
            frame = self.draw_info(frame, corners, markers_id, nearest, coordinates, angle)
        cv.circle(frame, (self.frame_center[0], self.frame_center[1]), 3, (0, 0, 255), -1)
        return frame, coordinates, angle

def translate(coordinates, angle):
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
    return [lspeed, rspeed]
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
    go = 10
    d = 30
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
session = connect.host(4219)
session.accept()
handler = arucoField([336, 256])
while True:
    ret, frame = session.read()
    if ret:
        frame, coordinates, angle = handler.main(frame)
        cv.imshow('Aruco', frame)
        if(angle != -1):
            speed = translate(coordinates, angle)
        else:
            speed = [0, 0]
        session.write((',' + str(speed[0]) + ',,' + str(speed[1]) + ',\n').zfill(20).encode())
        
        if cv.waitKey(1) == ord('q'):
            break
cv.destroyAllWindows()
session.close()
