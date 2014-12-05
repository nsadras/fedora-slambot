import numpy as np
import cv2
import IPython
from scipy.interpolate import RectBivariateSpline, interp2d
from scipy.signal import convolve2d

CALLIBRATION_AID = False

def Homography(im1_pts, im2_pts):
    x1,y1 = im1_pts[0]
    x2,y2 = im1_pts[1]
    x3,y3 = im1_pts[2]
    x4,y4 = im1_pts[3]

    u1,v1 = im2_pts[0]
    u2,v2 = im2_pts[1]
    u3,v3 = im2_pts[2]
    u4,v4 = im2_pts[3]

    A = np.matrix([
        [x1,y1,1,0,0,0,-u1*x1,-u1*y1],
        [0,0,0,x1,y1,1,-v1*x1,-v1*y1],
        [x2,y2,1,0,0,0,-u2*x2,-u2*y2],
        [0,0,0,x2,y2,1,-v2*x2,-v2*y2],
        [x3,y3,1,0,0,0,-u3*x3,-u3*y3],
        [0,0,0,x3,y3,1,-v3*x3,-v3*y3],
        [x4,y4,1,0,0,0,-u4*x4,-u4*y4],
        [0,0,0,x4,y4,1,-v4*x4,-v4*y4],
    ])
    b = np.matrix([u1,v1,u2,v2,u3,v3,u4,v4]).T
    h = (np.linalg.inv(A)*b).T
    H = np.matrix([
        [h[0,0],h[0,1],h[0,2]],
        [h[0,3],h[0,4],h[0,5]],
        [h[0,6],h[0,7],1],
    ])
    return H

def MakeInterp(img):
    height, width, _ = np.shape(img)
    fr=RectBivariateSpline(np.r_[:height], np.r_[:width], img[:,:,0])                                                                                                                        
    fg=RectBivariateSpline(np.r_[:height], np.r_[:width], img[:,:,1])                                                                                                                        
    fb=RectBivariateSpline(np.r_[:height], np.r_[:width], img[:,:,2])
    def f(x,y):
        r=fr.ev(y,x)
        g=fg.ev(y,x)
        b=fb.ev(y,x)
        r[x > width] = 1.0
        r[y > height] = 1.0
        r[x < 0] = 1.0
        r[y < 0] = 1.0
        g[x > width] = 1.0
        g[y > height] = 1.0
        g[x < 0] = 1.0
        g[y < 0] = 1.0
        b[x > width] = 1.0
        b[y > height] = 1.0
        b[x < 0] = 1.0
        b[y < 0] = 1.0
        out = np.clip(np.dstack((r,g,b)),0.0,1.0)
        return out
    return f

def HomogenizeCoords(v):
    return v / v[2,0]

def ToCoordMatrix(rx, ry):
    coords = np.empty((len(ry), len(rx), 3))                                                                                                                                                     
    coords[..., 0] = rx                                                                                                                                                 
    coords[..., 1] = ry[:,None]
    coords[..., 2] = 1
    return np.matrix(np.concatenate(coords)).T

def ToXYList(coord_matrix):
    tr = coord_matrix.T
    x = np.array(tr[:,0]).flatten()
    y = np.array(tr[:,1]).flatten()
    return x,y

def HomogenizeCoordMatrix(m):
    m[2][m[2]==0.0]=1.0
    return m/m[2,:]

def WarpImage(img, H, H_inv):
    height, width, _ = np.shape(img)
    ul_x, ul_y, _ = HomogenizeCoords(H_inv*np.matrix([0,0,1]).T).flatten().tolist()[0]
    ur_x, ur_y, _ = HomogenizeCoords(H_inv*np.matrix([width,0,1]).T).flatten().tolist()[0]
    ll_x, ll_y, _ = HomogenizeCoords(H_inv*np.matrix([0,height,1]).T).flatten().tolist()[0]
    lr_x, lr_y, _ = HomogenizeCoords(H_inv*np.matrix([width,height,1]).T).flatten().tolist()[0]
    x_min = int(min(ul_x, ur_x, ll_x, lr_x))
    x_max = int(max(ul_x, ur_x, ll_x, lr_x))
    y_min = int(min(ul_y, ur_y, ll_y, lr_y))
    y_max = int(max(ul_y, ur_y, ll_y, lr_y))
    interp = MakeInterp(img)
    if (x_max - x_min)*(y_max - y_min) > 100000:
        print (x_max - x_min)*(y_max - y_min)
        return None, (None,None,None,None)
    coords = ToCoordMatrix(np.r_[x_min:x_max],np.r_[y_min:y_max])
    x,y = ToXYList(coords)
    u,v = ToXYList(HomogenizeCoordMatrix(H*coords))
    pixels = interp(u,v)
    out = np.reshape(pixels,(y_max-y_min, x_max - x_min, 3))
    return out, (x_min, x_max, y_min, y_max)

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

def GetBlobs(mask, min_area = 50.0):
    contours,hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        M = cv2.moments(cnt)
        if M['m00'] == 0 or area < min_area:
            continue
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        out.append((cx,cy))
    return out

def BlobsToMatrix(blobs):
    return np.matrix(np.append(np.array(blobs),np.ones((len(blobs),1)), axis=1).T)

def BlobsFromMatrix(blobs):
    return (blobs / blobs[2,:]).T[:,:2].tolist()

def TransformBlobs(blobs, H):
    if len(blobs) == 0:
        return None
    return BlobsFromMatrix(H*BlobsToMatrix(blobs))

def RedFilter(frame, threshold = 20, red_value = 170):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv[hsv[:,:,0] < threshold,0] = red_value
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def GetDistance(p1, p2, H):
    x1,y1 = p1
    x2,y2 = p2
    p1_h = H*np.array([[x1,y1,1]]).T
    p2_h = H*np.array([[x2,y2,1]]).T
    p1_h = p1_h / p1_h[2,0]
    p2_h = p2_h / p2_h[2,0]
    dist = p2_h - p1_h
    return np.linalg.norm(dist)

def GetPointUnderH(p1, H):
    x1,y1 = p1
    p1_h = H*np.array([[x1,y1,1]]).T
    p1_h = p1_h / p1_h[2,0]
    return np.array([p1_h[0,0], p1_h[1,0]])


def GetAverageClusters(blob, H, suppression_radius = 8):
    blob_h = TransformBlobs(blob, H)
    if blob_h is None:
        return None
    valid = np.ones(len(blob))
    blob = np.array(blob)
    blob_h = np.array(blob_h)
    out = []
    for i in range(len(blob_h)):
        if valid[i]:
            selector = np.linalg.norm(blob_h - blob_h[i], axis=1) < suppression_radius
            avg = np.average(blob[selector],axis=0).astype(int)
            valid[selector] = 0
            out.append(avg)
    return out

def FindSquareCorrespondences(a, b, a_h, b_h, dist, tolerance):
    out = {}
    for i in range(len(a)):
        x,y = a[i]
        indices = np.nonzero(np.abs(np.linalg.norm(b_h - a_h[i], axis=1) - dist) < tolerance)
        if len(indices[0]) == 0:
            out[(x,y)] = None
        else:
            out[(x,y)] = tuple(b[indices[0][0]])
    return out

def DetectMarkers(r,g,b,y,H, tolerance = 6):
    if r is None or g is None or b is None or y is None:
        return []
    r_h = np.array(TransformBlobs(r,H))
    g_h = np.array(TransformBlobs(g,H))
    b_h = np.array(TransformBlobs(b,H))
    y_h = np.array(TransformBlobs(y,H))
    r = np.array(r)
    g = np.array(g)
    b = np.array(b)
    y = np.array(y)
    r_next = FindSquareCorrespondences(r,g,r_h,g_h,Vision.CM_WIDTH, tolerance)
    g_next = FindSquareCorrespondences(g,b,g_h,b_h,Vision.CM_HEIGHT, tolerance)
    b_next = FindSquareCorrespondences(b,y,b_h,y_h,Vision.CM_WIDTH, tolerance)
    y_next = FindSquareCorrespondences(y,r,y_h,r_h,Vision.CM_HEIGHT, tolerance)
    out = []
    for k in r_next:
        if r_next[k] and g_next[r_next[k]] and b_next[g_next[r_next[k]]] and y_next[b_next[g_next[r_next[k]]]] == k: 
            out.append((k, r_next[k], g_next[r_next[k]], b_next[g_next[r_next[k]]]))
    return out

def GetBounds(a,b,c,d):
    ax,ay = a
    bx,by = b
    cx,cy = c
    dx,dy = d
    return min(ax,bx,cx,dx),max(ax,bx,cx,dx),min(ay,by,cy,dy),max(ay,by,cy,dy)

def GetRGBBounds(img, vision):
    kernel = np.ones((5,5),np.uint8)
    black_pix = HSVRange(img,
            vision.params['BLACK']['hsvs'],
            vision.params['BLACK']['thresh'],
            blur_radius = 5)
    red_pix = cv2.bitwise_and(cv2.bitwise_not(black_pix),HSVRange(img,
            vision.params['RED']['hsvs'],
            vision.params['RED']['thresh'],
            blur_radius = 1))
    green_pix = cv2.bitwise_and(cv2.bitwise_not(black_pix),HSVRange(img,
            vision.params['GREEN']['hsvs'],
            vision.params['GREEN']['thresh'],
            blur_radius = 1))
    blue_pix = cv2.bitwise_and(cv2.bitwise_not(black_pix),HSVRange(img,
            vision.params['BLUE']['hsvs'],
            vision.params['BLUE']['thresh'],
            blur_radius = 1))
    yellow_pix = cv2.bitwise_and(cv2.bitwise_not(black_pix),HSVRange(img,
            vision.params['YELLOW']['hsvs'],
            vision.params['YELLOW']['thresh'],
            blur_radius = 1))
    #red_pix = cv2.morphologyEx(red_pix, cv2.MORPH_OPEN, kernel)
    green_pix = cv2.morphologyEx(green_pix, cv2.MORPH_OPEN, kernel)
    #blue_pix = cv2.morphologyEx(blue_pix, cv2.MORPH_OPEN, kernel)
    #yellow_pix = cv2.morphologyEx(yellow_pix, cv2.MORPH_OPEN, kernel)

    r_y,r_x = np.nonzero(red_pix)
    g_y,g_x = np.nonzero(green_pix)
    b_y,b_x = np.nonzero(blue_pix)
    y_y,y_x = np.nonzero(yellow_pix)

    if len(r_x) == 0 or len(r_y) == 0 or len(g_x) == 0 or len(g_y) == 0 or len(b_x) == 0 or len(b_y) == 0  or len(y_x) == 0 or len(y_y) == 0:
        return None

    p1 = (np.max(r_x), np.max(r_y))
    p2 = (np.min(g_x), np.max(g_y))
    p3 = (np.min(b_x), np.min(b_y))
    p4 = (np.max(y_x), np.min(y_y))
    return p1,p2,p3,p4

def PercentBlack(img, threshold):
    h,w,_ = np.shape(img)
    if h*w == 0:
        return None
    black_pix = np.average(img/255.0, axis=2) < threshold
    height, width = np.shape(black_pix)
    return np.count_nonzero(black_pix) / float(height*width)

OFFSET=24/2
MARKER_WIDTH_PX = 100
MARKER_HEIGHT_PX = 129
RECTIFICATION_DIMENSION = 100
def IdentifyMarker(marker,img, vision, black_threshold = 0.5, cell_threshold = 0.5):
    r, g, b, y = marker
    r_x, r_y = r
    g_x, g_y = g
    b_x, b_y = b
    y_x, y_y = y
    min_x,max_x,min_y,max_y = GetBounds(r, g, b, y)
    cropped = img[min_y:max_y,min_x:max_x]
    H = Homography(
            [(OFFSET,OFFSET),(MARKER_WIDTH_PX - OFFSET,OFFSET),(MARKER_WIDTH_PX - OFFSET, MARKER_HEIGHT_PX - OFFSET), (OFFSET, MARKER_WIDTH_PX - OFFSET)],
            [(r_x-min_x,r_y-min_y),(g_x-min_x,g_y-min_y),(b_x-min_x,b_y-min_y),(y_x-min_x,y_y-min_y)],
            )
    H_inv = np.linalg.inv(H)
    ul_x, ul_y, _ = HomogenizeCoords(H_inv*np.matrix([r_x-min_x,r_y-min_y,1]).T).flatten().tolist()[0]
    warp_float, _ = WarpImage(cropped/255.0,H,H_inv)
    if warp_float is None:
        print "Failed first warp image."
        return None
    warped = (255*warp_float).astype(np.uint8)

    bounds = GetRGBBounds(warped, vision)
    if not bounds:
        print "Could not get RGB Bounds."
        return None
    p1,p2,p3,p4 = bounds

    H = Homography(
            [(0,0),(RECTIFICATION_DIMENSION,0),(RECTIFICATION_DIMENSION, RECTIFICATION_DIMENSION), (0, RECTIFICATION_DIMENSION)],
            [p1,p2,p3,p4],
            )
    H_inv = np.linalg.inv(H)
    rectified, (x_min,x_max,y_min,y_max) = WarpImage(warped/255.0,H,H_inv)
    if rectified is None:
        print "Could not rectify."
        return None
    rectified = (255*rectified).astype(np.uint8)

    bounds = GetRGBBounds(rectified, vision)
    if not bounds:
        print "Could not get RGBBounds 2."
        return None
    p1,p2,p3,p4 = bounds

    rectified = rectified[max(p1[1],p2[1]):min(p3[1],p4[1]),max(p1[0],p4[0]):min(p2[0],p3[0]),:]
    cell1 = rectified[:RECTIFICATION_DIMENSION/2, :RECTIFICATION_DIMENSION/2]
    cell2 = rectified[:RECTIFICATION_DIMENSION/2, RECTIFICATION_DIMENSION/2:]
    cell3 = rectified[RECTIFICATION_DIMENSION/2:, RECTIFICATION_DIMENSION/2:]
    cell4 = rectified[RECTIFICATION_DIMENSION/2:, :RECTIFICATION_DIMENSION/2]


    v1 = PercentBlack(cell1, black_threshold)
    v2 = PercentBlack(cell2, black_threshold)
    v3 = PercentBlack(cell3, black_threshold)
    v4 = PercentBlack(cell4, black_threshold)
    if v1 is None or v2 is None or v3 is None or v4 is None:
        print "Could not compute percent black for one or more cells."
        return None
    identity = ((v1 >  cell_threshold) << 0) + ((v2 > cell_threshold) << 1) + ((v3 > cell_threshold) << 2) + ((v4 > cell_threshold) << 3)
    return identity

def PositionMarker(marker, frame, vision):
    p1,p2,p3,p4 = marker
    marker_center = (GetPointUnderH(p1,vision.H) + GetPointUnderH(p2,vision.H) + GetPointUnderH(p3,vision.H) + GetPointUnderH(p4,vision.H))/4.0

    o = np.linalg.inv(vision.H)*np.matrix([marker_center[0],marker_center[1],1]).T
    o /= o[2,0]
    xo=int(o[0,0])
    yo=int(o[1,0])
    cv2.circle(frame,(xo,yo),3,[255,255,255],-1)

    origin = GetPointUnderH(vision.params['coord_sys']['origin'], vision.H)
    x = GetPointUnderH(vision.params['coord_sys']['x'], vision.H)
    y = GetPointUnderH(vision.params['coord_sys']['y'], vision.H)
    T = np.linalg.inv(np.matrix([[x[0] - origin[0], y[0] - origin[0]], [x[1] - origin[1], y[1] - origin[1]]]))
    v = np.matrix([[marker_center[0] - origin[0]],[marker_center[1] - origin[1]]])
    out_x, out_y = (T*v).T.tolist()[0]
    out_x = out_x * vision.params['coord_sys']['x_dist'] + vision.params['coord_sys']['origin_pos'][0]
    out_y = out_y * vision.params['coord_sys']['y_dist'] + vision.params['coord_sys']['origin_pos'][1]
    return out_x, out_y

class Vision:
    NUM_CALIBRATION_IMAGES = 100
    CM_HEIGHT = 22
    CM_WIDTH = 16
    SQUARE_DIAG = 8
    def __init__(self):
        self.isCalibrated = False
    def PreCalibrate(self, params, H):
        self.params = params
        self.H = H
        self.isCalibrated = True

    def Calibrate(self, images, r_point, g_point, b_point, y_point, black_point, n_std_r=6.0, n_std_g=6.0, n_std_b=6.0, n_std_y=6.0, n_std_black=1.0, std_offset = 2.0):
        r_hsv = []
        g_hsv = []
        b_hsv = []
        y_hsv = []
        black_hsv = []
        for img in images:
            r_x, r_y = r_point
            g_x, g_y = g_point
            b_x, b_y = b_point
            y_x, y_y = y_point
            black_x, black_y = black_point
            r_hsv.append(cv2.cvtColor(np.array([[img[r_y,r_x]]]), cv2.COLOR_BGR2HSV))
            g_hsv.append(cv2.cvtColor(np.array([[img[g_y,g_x]]]), cv2.COLOR_BGR2HSV))
            b_hsv.append(cv2.cvtColor(np.array([[img[b_y,b_x]]]), cv2.COLOR_BGR2HSV))
            y_hsv.append(cv2.cvtColor(np.array([[img[y_y,y_x]]]), cv2.COLOR_BGR2HSV))
            black_hsv.append(cv2.cvtColor(np.array([[img[black_y,black_x]]]), cv2.COLOR_BGR2HSV))
        r_h, r_s, r_v = np.average(r_hsv, axis=0).flatten().astype(int)
        r_h_t, r_s_t, r_v_t = np.std(r_hsv, axis=0).flatten().astype(int) + std_offset
        g_h, g_s, g_v = np.average(g_hsv, axis=0).flatten().astype(int)
        g_h_t, g_s_t, g_v_t = np.std(g_hsv, axis=0).flatten().astype(int) + std_offset
        b_h, b_s, b_v = np.average(b_hsv, axis=0).flatten().astype(int)
        b_h_t, b_s_t, b_v_t = np.std(b_hsv, axis=0).flatten().astype(int) + std_offset
        y_h, y_s, y_v = np.average(y_hsv, axis=0).flatten().astype(int)
        y_h_t, y_s_t, y_v_t = np.std(y_hsv, axis=0).flatten().astype(int) + std_offset
        black_h, black_s, black_v = np.average(black_hsv, axis=0).flatten().astype(int)
        black_h_t, black_s_t, black_v_t = np.std(black_hsv, axis=0).flatten().astype(int)

        img_pts = [r_point, g_point, b_point, y_point]
        world_pts = [(0,Vision.CM_HEIGHT), (Vision.CM_WIDTH, Vision.CM_HEIGHT), (Vision.CM_WIDTH,0), (0,0)]
        self.H = Homography(img_pts, world_pts)

        self.params = {
                'RED': {'hsvs':[[r_h,r_s,r_v]], 'thresh':[r_h_t*n_std_r,r_s_t*n_std_r,r_v_t*n_std_r]},
                'GREEN': {'hsvs':[[g_h,g_s,g_v]], 'thresh':[g_h_t*n_std_g,g_s_t*n_std_g,g_s_t*n_std_g]},
                'BLUE': {'hsvs':[[b_h,b_s,b_v]], 'thresh':[b_h_t*n_std_b,b_s_t*n_std_b,b_v_t*n_std_b]},
                'YELLOW': {'hsvs':[[y_h,y_s,y_v]], 'thresh':[y_h_t*n_std_y,y_s_t*n_std_y,y_v_t*n_std_y]},
                'BLACK': {'hsvs':[[black_h,black_s,black_v]], 'thresh':[black_h_t*n_std_black,black_s_t*n_std_black,black_v_t*n_std_black]},
                }
        print self.params
        print self.H
        self.isCalibrated = True

    def ProcessFrame(self, frame):
        frame = RedFilter(frame)
        black_pix = HSVRange(frame,
                self.params['BLACK']['hsvs'],
                self.params['BLACK']['thresh'],
                blur_radius = 5)
        red_pix = HSVRange(frame,
                self.params['RED']['hsvs'],
                self.params['RED']['thresh'],
                blur_radius = 5)
        red_blobs = GetBlobs(cv2.bitwise_and(cv2.bitwise_not(black_pix),red_pix))
        green_pix = HSVRange(frame,
                self.params['GREEN']['hsvs'],
                self.params['GREEN']['thresh'],
                blur_radius = 5)
        green_blobs = GetBlobs(cv2.bitwise_and(cv2.bitwise_not(black_pix),green_pix))
        blue_pix = HSVRange(frame,
                self.params['BLUE']['hsvs'],
                self.params['BLUE']['thresh'],
                blur_radius = 5)
        blue_blobs = GetBlobs(cv2.bitwise_and(cv2.bitwise_not(black_pix),blue_pix))
        yellow_pix = HSVRange(frame,
                self.params['YELLOW']['hsvs'],
                self.params['YELLOW']['thresh'],
                blur_radius = 5)
        yellow_blobs = GetBlobs(cv2.bitwise_and(cv2.bitwise_not(black_pix),yellow_pix))

        red_centers =  GetAverageClusters(red_blobs, self.H)
        green_centers =  GetAverageClusters(green_blobs, self.H)
        blue_centers =  GetAverageClusters(blue_blobs, self.H)
        yellow_centers =  GetAverageClusters(yellow_blobs, self.H)
        markers = DetectMarkers(red_centers,green_centers,blue_centers,yellow_centers,self.H)

        output = []
        for marker in markers:
            identity = IdentifyMarker(marker,frame,self)
            if not identity:
                continue
            position = PositionMarker(marker,frame,self)
            if not position:
                continue
            output.append((identity, position))

        if red_centers:
            for center in red_centers:
                cv2.circle(frame,tuple(center),5,[0,0,255],-1)
        if green_centers:
            for center in green_centers:
                cv2.circle(frame,tuple(center),5,[0,255,0],-1)
        if blue_centers:
            for center in blue_centers:
                cv2.circle(frame,tuple(center),5,[255,0,0],-1)
        if yellow_centers:
            for center in yellow_centers:
                cv2.circle(frame,tuple(center),5,[0,255,255],-1)

        for marker in markers:
            p1,p2,p3,p4 = marker
            cv2.line(frame, p1, p2, [0,0,0])
            cv2.line(frame, p2, p3, [0,0,0])
            cv2.line(frame, p3, p4, [0,0,0])
            cv2.line(frame, p4, p1, [0,0,0])

        if CALLIBRATION_AID:
            asdf[0] = frame
        cv2.imshow('frame',frame)

        return output

if CALLIBRATION_AID:
    asdf = [None]
    def PrintColor(event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONUP:
            out = cv2.cvtColor(np.array([[asdf[0][y,x]]]), cv2.COLOR_BGR2HSV)
            print "{0} {1} {2}".format(out[0,0,0],out[0,0,1],out[0,0,2])
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',PrintColor)


def CalibrationFrameIs(vision, frame):
        frame = RedFilter(frame)
        cv2.imshow('calibration',frame)
        if len(calibration_points) == 5:
            calibration_images.append(frame)
        if len(calibration_images) == Vision.NUM_CALIBRATION_IMAGES:
            print 'done waiting for points'
            vision.Calibrate(calibration_images,
                    calibration_points[0],
                    calibration_points[1],
                    calibration_points[2],
                    calibration_points[3],
                    calibration_points[4])
            cv2.destroyWindow('calibration')


if __name__ == '__main__':
    vision = Vision(['./markers/a.png','./markers/b.png','./markers/c.png','./markers/d.png','./markers/e.png'])
    cap = cv2.VideoCapture(0)
    while(cap.isOpened()):
        ret, frame = cap.read()
        frame = RedFilter(frame)
        if not vision.isCalibrated:
            CalibrationFrameIs(vision,frame)
        else:
            vision.ProcessFrame(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
