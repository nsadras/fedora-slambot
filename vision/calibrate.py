import rospy
import cv2
import numpy as np
import sensor_msgs.msg as sensor_msgs
from vision import CalibrationFrameIs, Vision, RedFilter
from threading import Thread

frame = None

def vision_thread():
    global state
    while(True):
        if frame is not None:
            cv2.imshow('frame',frame)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                PrintConfig(data)
                rospy.signal_shutdown("Done!") 
                break
            elif key & 0xFF == ord('r'):
                state = 0
                print 'Please click a {0} spot on a marker.'.format(words[state])
            elif key & 0xFF == ord('g'):
                state = 1
                print 'Please click a {0} spot on a marker.'.format(words[state])
            elif key & 0xFF == ord('b'):
                state = 2
                print 'Please click a {0} spot on a marker.'.format(words[state])
            elif key & 0xFF == ord('y'):
                state = 3
                print 'Please click a {0} spot on a marker.'.format(words[state])
            elif key & 0xFF == ord('k'):
                state = 4
                print 'Please click a {0} spot on a marker.'.format(words[state])


words = ['red', 'green', 'blue', 'yellow', 'black']
data = [{'h':[],'s':[],'v':[]}, {'h':[],'s':[],'v':[]}, {'h':[],'s':[],'v':[]}, {'h':[],'s':[],'v':[]}, {'h':[],'s':[],'v':[]}]
state = -1

def OnClick(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONUP:
        if state == -1:
            print 'Please press one of r g b y k.'
            return
        color = cv2.cvtColor(np.array([[frame[y,x]]]), cv2.COLOR_BGR2HSV)
        h,s,v = int(color[0,0,0]),int(color[0,0,1]),int(color[0,0,2])
        data[state]['h'].append(h)
        data[state]['s'].append(s)
        data[state]['v'].append(v)
        print (h,s,v)

cv2.namedWindow('frame')
cv2.setMouseCallback('frame',OnClick)


def callback(message):
    global frame
    frame = RedFilter(cv2.imdecode(np.fromstring(message.data,np.uint8), cv2.CV_LOAD_IMAGE_COLOR))

def GetListStats(s):
    if len(s) == 0:
        return None, None
    print int((min(s)+max(s))/2.0), ((min(s)+max(s))/2.0) - min(s)
    return int((min(s)+max(s))/2.0), ((min(s)+max(s))/2.0) - min(s)

def PrintConfig(data):
    r_h, r_h_thresh = GetListStats(data[0]['h'])
    r_s, r_s_thresh = GetListStats(data[0]['s'])
    r_v, r_v_thresh = GetListStats(data[0]['v'])
    g_h, g_h_thresh = GetListStats(data[1]['h'])
    g_s, g_s_thresh = GetListStats(data[1]['s'])
    g_v, g_v_thresh = GetListStats(data[1]['v'])
    b_h, b_h_thresh = GetListStats(data[2]['h'])
    b_s, b_s_thresh = GetListStats(data[2]['s'])
    b_v, b_v_thresh = GetListStats(data[2]['v'])
    y_h, y_h_thresh = GetListStats(data[3]['h'])
    y_s, y_s_thresh = GetListStats(data[3]['s'])
    y_v, y_v_thresh = GetListStats(data[3]['v'])
    k_h, k_h_thresh = GetListStats(data[4]['h'])
    k_s, k_s_thresh = GetListStats(data[4]['s'])
    k_v, k_v_thresh = GetListStats(data[4]['v'])

    print {
        'BLUE': {'hsvs': [[b_h, b_s, b_v]], 'thresh': [b_h_thresh, b_s_thresh, b_v_thresh]},
        'BLACK': {'hsvs': [[k_h, k_s, k_v]], 'thresh': [k_h_thresh, k_s_thresh, k_v_thresh]},
        'GREEN': {'hsvs': [[g_h, g_s, g_v]], 'thresh': [g_h_thresh, g_s_thresh, g_v_thresh]},
        'YELLOW': {'hsvs': [[y_h, y_s, y_v]], 'thresh': [y_h_thresh, y_s_thresh, y_v_thresh]},
        'RED': {'hsvs': [[r_h, r_s, r_v]], 'thresh': [r_h_thresh, r_s_thresh, r_v_thresh]},
    }


if __name__ == '__main__':
    print 'Please press one of r g b y k.'
    rospy.init_node('image_listener')
    rospy.Subscriber('/throttled_camera_image', sensor_msgs.CompressedImage, callback)
    th = Thread(target=vision_thread)
    th.start()
    rospy.spin()
