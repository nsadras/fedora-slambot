import numpy as np
import cv2
from time import sleep
from constants import constants

cap = cv2.VideoCapture(0)

def HSVFilter(img, h_lower, s_lower, v_lower, h_upper, s_upper, v_upper, sigma):
    blur = cv2.GaussianBlur(img,(sigma,sigma),0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower = np.array([max(0,h_lower),max(0,s_lower),max(0,v_lower)])
    upper = np.array([min(255,h_upper),min(255,s_upper),min(255,v_upper)])
    return cv2.inRange(hsv, lower, upper)

def HSVRange(img, hsvs, tolerances, blur_radius = 1):
    h_tolerance = tolerances[0]
    s_tolerance = tolerances[1]
    v_tolerance = tolerances[2]
    out = None
    for hsv in hsvs:
        h,s,v = hsv
        if out is None:
            out = HSVFilter(img, h - h_tolerance, s - s_tolerance, v - v_tolerance, h + h_tolerance, s + s_tolerance, v + v_tolerance, blur_radius)
        else:
            out = cv2.bitwise_or(out, HSVFilter(img, h - h_tolerance, s - s_tolerance, v - v_tolerance, h + h_tolerance, s + s_tolerance, v + v_tolerance, blur_radius))
    return out

def GetBlobs(img, hsvs, tolerances, blur_radius = 1, name = ''):
    hsvs = HSVRange(img, hsvs, tolerances, blur_radius = 1)
    if name != '':
        cv2.imshow(name, hsvs)
    contours,hierarchy = cv2.findContours(hsvs,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    best_cnt = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            best_cnt = cnt
    if not (best_cnt is None):
        M = cv2.moments(best_cnt)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        cv2.circle(frame,(cx,cy),5,255,-1)
    return hsvs




while(cap.isOpened()):
    ret, frame = cap.read()

    red = GetBlobs(frame, [[173, 102, 133], [-10, 104, 136]], [20,40,40], 5, 'Red')
    green = GetBlobs(frame, [[70, 46, 88]], [20,40,40], 5, 'Green')
    blue = GetBlobs(frame, [[120, 141, 105]], [20,50,50], 5, 'Blue')
    yellow = GetBlobs(frame, [[25, 104, 143]], [10,20,20], 5, 'Yellow')
    cv2.imshow('detected',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
