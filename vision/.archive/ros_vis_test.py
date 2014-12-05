import rospy
import sensor_msgs.msg as sensor_msgs
import numpy as np
import cv2
from vision import *

def callback(message):
    frame = cv2.imdecode(np.fromstring(message.data,np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
    frame = RedFilter(frame)
    if not vision.isCalibrated:
        cv2.imshow('calibration',frame)
        if len(calibration_points) == 5:
            calibration_images.append(frame)
        if len(calibration_images) == Vision.NUM_CALIBRATION_IMAGES:
            vision.Calibrate(calibration_images,
                    calibration_points[0],
                    calibration_points[1],
                    calibration_points[2],
                    calibration_points[3],
                    calibration_points[4])
            cv2.destroyWindow('calibration')
    else:
        vision.ProcessFrame(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

vision = Vision(['./markers/a.png','./markers/b.png','./markers/c.png','./markers/d.png','./markers/e.png'])

rospy.init_node('image_listener')
rospy.Subscriber('rpi_camera/image/compressed', sensor_msgs.CompressedImage, callback)
rospy.spin()
