import rospy
import cv2
import numpy as np
import sensor_msgs.msg as sensor_msgs
from vision import CalibrationFrameIs, Vision
from threading import Thread

frame = None

def vision_thread():
    while(True):
        if frame is not None:
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def PrintPt(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONUP:
        out = cv2.cvtColor(np.array([[frame[y,x]]]), cv2.COLOR_BGR2HSV)
        print "{0} {1} {2}".format(out[0,0,0],out[0,0,1],out[0,0,2])

cv2.namedWindow('frame')
cv2.setMouseCallback('frame',PrintPt)


def callback(message):
    global frame
    frame = cv2.imdecode(np.fromstring(message.data,np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

if __name__ == '__main__':
    rospy.init_node('image_listener')
    rospy.Subscriber('/throttled_camera_image', sensor_msgs.CompressedImage, callback)
    th = Thread(target=vision_thread)
    th.start()
    rospy.spin()
