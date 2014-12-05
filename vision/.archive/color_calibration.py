import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# mouse callback function
def OnClick(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONUP:
        print cv2.cvtColor(np.array([[img[y,x]]]), cv2.COLOR_BGR2HSV)

img = cap.read()[1]
cv2.namedWindow('image')
cv2.setMouseCallback('image',OnClick)

while(1):
    img = cap.read()[1]
    cv2.imshow('image',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()

