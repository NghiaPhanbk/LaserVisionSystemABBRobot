import cv2 as cv
import numpy as np
import glob
from App_Lib.Vision_Lib import Vision
from App_Lib.App import savematrix, load_eye2hand,load_laserplane
from skimage.measure import ransac, LineModelND
from GlobalVariables import *
from ultralytics import  YOLO
import torchvision.transforms as T
from matplotlib import pyplot as plt
from pypylon import pylon
import threading
import tqdm
import sys,time
from PyQt5.QtCore import QObject, pyqtSignal
rows = 2560
cols = 2560

vis = Vision()
intrinsic = vis.intrinsic_ex
dist_coffs = vis.dist_coffs_ex
eye2hand = load_eye2hand((eye_to_hand_params))
# print("base to camera",eye2hand)
c = -1
a, b, d = load_laserplane((laserplane_params_eye_to_hand))
fx = intrinsic[0][0]
fy = intrinsic[1][1]
cx = intrinsic[0][2]
cy = intrinsic[1][2]

# natural sorting https://stackoverflow.com/questions/5967500/how-to-correctly-sort-a-string-with-a-number-inside/5967539#5967539
import re

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]

def WeldSeamCenter(img):
    # adjust w to find the center of weldseam
    w = 1
    weldseam_center = []
    for j in range(670, 750, 1):
        d = []
        for i in range(450, 650, 1):
            # if img[i][j] < 50:
            a = 2*(img[i-1,j]+img[i,j]+img[i+1,j])
            b = img[i+w,j]+img[i+w+1,j]+img[i+w+2,j]
            c = img[i-w,j]+img[i-w-1,j]+img[i-w-2,j]
            e = a - b - c
            d.append(e)
            del a
            del b
            del c
        min_val = min(d)
        center_point = [j, d.index(min_val)+450]
        # center_point = [j, d.index(min_val)]
        weldseam_center.append(center_point)
        del d
    return weldseam_center

def LaserCenter(img):
        center = np.zeros((rows,cols))
        # find the center point
        for x in range(cols):
            sum1 = 0.0
            sum2 = 0.0
            roi = np.where(img[:,x] == 255)
            if roi[0].size != 0:
                for y in roi[0]:
                    sum1 += y * img[y][x]
                    sum2 += img[y][x]
                center[int(sum1/sum2)][x] = 255
        return center

def Preprocessing(img):
    blur = cv.GaussianBlur(img,(7,7),0)                 # use when img is gray image
    _, thresh = cv.threshold(blur,100,255,cv.THRESH_BINARY)
    closing = thresh
    del blur
    for _ in range(5):
        closing = cv.morphologyEx(closing, cv.MORPH_CLOSE, np.ones((7,7),np.uint8))
    del thresh
    return closing

# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])
    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]
    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return int(x), int(y)

def calc_weldpoint2robot(point):
    Zc = -d / (a / fx * (point[0] - cx) + b / fy * (point[1] - cy) + c)
    # Zc=800
    weldpoint2camera = np.array([[Zc / fx * (point[0] - cx)], [Zc / fy * (point[1] - cy)], [Zc], [1]])
    print("weldpoint2camera:", weldpoint2camera)
    weldpoint2robot = eye2hand.dot(weldpoint2camera).flatten()[0:3]
    print("coordinates of check point:" ,weldpoint2robot)
    return weldpoint2robot
def LaserCenter(img,coords):
    xmax = np.max(coords[:, 0])-200
    xmin = np.min(coords[:, 0])+200
    ymax = np.max(coords[:, 1])-200
    ymin = np.min(coords[:, 1])+200
    center = np.zeros((img.shape[0], img.shape[1]))
    for x in range(xmin, xmax):
        for y in range(ymin, ymax):
            if img[y][x] == 255:
                center[int(y)][int(x)] = 255
    return center
def Preprocessing(img):
    newcameramtx, _ =cv.getOptimalNewCameraMatrix(intrinsic,dist_coffs,(rows,cols),1,(rows,cols))
    img = cv.undistort(img, intrinsic, dist_coffs, None, newcameramtx)
    # grayimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)     # use when img is color image
    # blur = cv.GaussianBlur(grayimg,(7,7),0)
    blur = cv.GaussianBlur(img,(7,7),0)                 # use when img is gray image
    # _, thresh = cv.threshold(blur,80,255,cv.THRESH_BINARY)
    _, thresh = cv.threshold(blur, 20, 255, cv.THRESH_BINARY)
    # thresh = cv.adaptiveThreshold(blur, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY, 199, 5)
    closing = thresh
    del blur
    for _ in range(7):
        closing = cv.morphologyEx(closing, cv.MORPH_CLOSE, np.ones((7,7),np.uint8))
    del thresh
    return closing
def convert_to_scan_point(start_point,end_point):
    start_point[0]=start_point[0]-150
    end_point[0] = end_point[0] - 130
    start_point[1]=start_point[1]+40
    end_point[1] = end_point[1]+40
    common_point = (start_point[2]+end_point[2])/2+50
    start_point[2] = common_point
    end_point[2] = common_point
    return start_point,end_point
# if __name__ == '__main__':
class find_trajectory(QObject):
    data_signal = pyqtSignal(str)
    def __init__(self):
        QObject.__init__(self)
    def camera_task(self,path):
        desired_model = "acA3800-10gm"
        _tlFactory = pylon.TlFactory.GetInstance()
        devices = _tlFactory.EnumerateDevices()
        for dev_info in devices:
            print("Device model:", dev_info.GetModelName())
            self.data_signal.emit("Device model: {}".format(dev_info.GetModelName()))
        device = None
        for dev_info in devices:
            if dev_info.GetModelName() == desired_model:
                device = dev_info
                break
        if device is not None:
            camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(device))
            camera.Open()
            camera.Width.SetValue(2560)
            camera.Height.SetValue(2560)
            camera.OffsetX.SetValue(520)
            camera.OffsetY.SetValue(188)
            camera.ExposureTimeAbs = 30000
            camera.StartGrabbing(pylon.GrabStrategy_LatestImages)
            global CurrImg
            if camera.IsGrabbing():
                try:
                    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                    if grabResult.GrabSucceeded():
                        CurrImg = grabResult.Array
                except:
                    print("Camera error: ", sys.exc_info())
            cv.imwrite(path, CurrImg)
            return 1
        else:
            print("Không tìm thấy thiết bị: ",desired_model)
            self.data_signal.emit("Không tìm thấy thiết bị: {}".format(desired_model))
            return 0
        # camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    def main_func(self):
        path = "E:/Thesis/App_Data/Test_eye_to_hand/test6.jpg"
        a = self.camera_task(path)
        """ Test finding scantrajectory"""
        if a == 1:
            model = YOLO("E:/Thesis/App_Data/best.pt")
            input_img = cv.imread(path)  # Replace with actual path to your image
            resize_img = cv.resize(input_img, (640, 640), interpolation=cv.INTER_AREA)
            results = model.predict(show=True, source=resize_img, conf = 0.6, iou = 0)
            # img = T.ToPILImage()(results[0][1].masks.masks)
            img = T.ToPILImage()(results[0].masks.masks)

            binary_image = np.array(img)
            binary_image = cv.resize(binary_image, (2560, 2560))
            contours, hierarchy = cv.findContours(binary_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            # Draw contours on a blank image to visualize them
            contour_image = np.zeros_like(binary_image)
            cv.drawContours(contour_image, contours, -1, (255, 255, 255), thickness=cv.FILLED)

            # Get minimum bounding rectangles for each contour
            rectangles = [cv.minAreaRect(cnt) for cnt in contours]

            # Draw minimum bounding rectangles on the original image
            original_image = cv.imread(path)  # Replace with actual path to your image
            # original_image = cv.resize(original_image,(1728,1216))
            for rect in rectangles:
                box = cv.boxPoints(rect)
                box = np.int0(box)
                cv.drawContours(original_image, [box], 0, (0, 0, 255), thickness=2)
            original_image1 = cv.cvtColor(original_image, cv.COLOR_BGR2GRAY)
            original_image1 = Preprocessing(original_image1)
            thinned = cv.ximgproc.thinning(original_image1)

            # 2 point of rectangle
            line = LaserCenter(thinned, box)
            cv.namedWindow('thin', cv.WINDOW_NORMAL)
            cv.resizeWindow('thin', 640, 640)
            cv.imshow("thin",original_image1)
            cv.waitKey(0)
            for i in range(line.shape[0]):
                for j in range(line.shape[1]):
                    if line[i][j] == 255:
                        cv.circle(original_image, (j,i), 5, [0,255,0], 5)
            points = np.where(line == 255)
            data = np.vstack((points[1], points[0])).T
            # print(data[-1])
            print("Line",line)
            sorted_box = sorted(box, key=lambda x: x[0])
            print(sorted_box)
            point1 = sorted_box[0]
            point2 = sorted_box[1]
            point3 = sorted_box[2]
            point4 = sorted_box[3]
            top_edge = (point1,point2)
            bottom_edge = (point3,point4)
            # laser center line
            # Laser center is the column 716 of the image
            laser_centerline = (data[1], data[-2])
            # find intersection
            feature_point1 = line_intersection(top_edge, laser_centerline)
            feature_point2 = line_intersection(bottom_edge, laser_centerline)
            result = cv.imread(path)
            cv.drawContours(result, [box], 0, (255, 0, 0), thickness=2)
            print("feature_point1",feature_point1)
            cv.circle(result, (int(feature_point1[0]), int(feature_point1[1])), 5, (255, 0, 0), 5)
            print("feature_point2",feature_point2)
            cv.circle(result, (int(feature_point2[0]), int(feature_point2[1])), 5, (255, 0, 0), 5)
            rpy = np.array([180, -25, 0])
            scan_pos = []
            scan_pos1 = calc_weldpoint2robot(feature_point1)
            scan_pos1 = np.concatenate((scan_pos1,rpy))
            scan_pos2 = calc_weldpoint2robot(feature_point2)
            scan_pos2 = np.concatenate((scan_pos2,rpy))
            scan_pos1,scan_pos2 = convert_to_scan_point(scan_pos1,scan_pos2)
            print(scan_pos1)
            scan_pos.append(scan_pos1)
            scan_pos.append(scan_pos2)
            scan_trajectory = np.array(scan_pos)
            savematrix(scan_trajectory_path,scan_trajectory)
            #Display the final image
            # cv.namedWindow('Original Image with Minimum Bounding Rectangles', cv.WINDOW_NORMAL)
            # cv.resizeWindow('Original Image with Minimum Bounding Rectangles', 640, 640)
            # cv.imshow("Original Image with Minimum Bounding Rectangles", result)
            # cv.waitKey(0)
            # cv.destroyAllWindows()
            scan_pos1 = ', '.join(str(i) for i in scan_pos1)
            scan_pos2 = ', '.join(str(i) for i in scan_pos2)
            self.data_signal.emit("Start point")
            self.data_signal.emit(scan_pos1)
            self.data_signal.emit("Stop point")
            self.data_signal.emit(scan_pos2)
            result_path = "E:/Thesis/App_Data/Test_eye_to_hand/result1.jpg"
            cv.imwrite(result_path, result)
            return 1
        else:
            return 0
if __name__ == '__main__':
    run = find_trajectory()
    run.main_func()