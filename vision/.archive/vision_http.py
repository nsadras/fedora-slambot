import cv2
import numpy as np
import urllib
from vision import CalibrationFrameIs, Vision, GetDistance
from threading import Thread

def run_vision():
    stream=urllib.urlopen('http://192.168.1.123:8080/?action=stream')
    data = ''
    while(True):
        data+=stream.read(1024)
        a = data.find('\xff\xd8')
        b = data.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = data[a:b+2]
            data= data[b+2:]
            frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            #frame = cv2.resize(frame, (640,480), interpolation=cv2.INTER_LINEAR)
            vision.ProcessFrame(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def callback(message):
    global frame
    frame = cv2.imdecode(np.fromstring(message.data,np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

if __name__ == '__main__':
    vision = Vision(['./markers/a.png','./markers/b.png','./markers/c.png','./markers/d.png','./markers/e.png'])
    vision.PreCalibrate({
                            'BLUE': {'hsvs': [[106, 215, 155]], 'thresh': [30.0, 50.0, 100.0]},
                            'BLACK': {'hsvs': [[43, 189, 56]], 'thresh': [1000.0, 30.0, 20.0]},
                            'GREEN': {'hsvs': [[52, 184, 160]], 'thresh': [8.0, 71.0, 100.0]},
                            'YELLOW': {'hsvs': [[28, 252, 157]], 'thresh': [50.0, 50.0, 50.0]},
                            'RED': {'hsvs': [[160, 195, 193]], 'thresh': [50.0, 60.0, 61.0]},
                            'coord_sys': {
                                'origin':(349, 355),
                                'x':(511, 350),
                                'y':(350, 267),
                                'x_dist':10,
                                'y_dist':10,
                                'origin_pos':(0,30),
                                }
                        },
                        np.matrix([[-3.08693110e-01,  1.02025689e-01,  1.16822030e+02],
                                   [ 4.16499924e-02,  7.91349856e-01, -1.41360074e+02],
                                   [ 9.45180079e-04,  1.04929105e-02,  1.00000000e+00]]))
    run_vision()
