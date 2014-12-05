import rospy
import cv2
import numpy as np
import sensor_msgs.msg as sensor_msgs
import json
from vision import CalibrationFrameIs, Vision, GetDistance
from threading import Thread
from time import sleep
from std_msgs.msg import String

frame = None

def vision_thread():
    while(True):
        if frame is not None:
            markers = vision.ProcessFrame(frame)
            message = json.dumps(markers)
            pub.publish(message)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("Done!") 
                break
        else:
            print "Waiting for image data."
            sleep(1)

def callback(message):
    global frame
    frame = cv2.imdecode(np.fromstring(message.data,np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

if __name__ == '__main__':

    rospy.init_node('vision')
    pub = rospy.Publisher('/markers', String, queue_size=1)

    vision = Vision()
    vision.PreCalibrate({
                            'BLUE': {'hsvs': [[106, 215, 155]], 'thresh': [30.0, 150.0, 150.0]},
                            'BLACK': {'hsvs': [[43, 189, 56]], 'thresh': [1000.0, 30.0, 20.0]},
                            'GREEN': {'hsvs': [[52, 184, 160]], 'thresh': [8.0, 71.0, 100.0]},
                            'YELLOW': {'hsvs': [[30, 235, 200]], 'thresh': [50.0, 40.0, 70.0]},
                            'RED': {'hsvs': [[160, 195, 193]], 'thresh': [60.0, 60.0, 60.0]},
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
    rospy.Subscriber('/throttled_camera_image', sensor_msgs.CompressedImage, callback)
    th = Thread(target=vision_thread)
    th.start()
    rospy.spin()
