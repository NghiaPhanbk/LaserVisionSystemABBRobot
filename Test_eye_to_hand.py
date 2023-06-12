import cv2 as cv
import numpy as np
import glob
from App_Lib.Vision_Lib import Vision
from App_Lib.App import savematrix, load_eye2hand,load_laserplane
from skimage.measure import ransac, LineModelND
from GlobalVariables import *
from matplotlib import pyplot as plt
from pypylon import pylon
import threading
import tqdm
import sys,time
rows = 1200
cols = 1700

vis = Vision()
intrinsic = vis.intrinsic
dist_coffs = vis.dist_coffs
eye2hand = load_eye2hand((eye_to_hand_params))
print("base to camera",eye2hand)
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

def calc_weldpoint2robot(point, pos):
    Zc                 = -d / (a/fx*(point[0]-cx) + b/fy*(point[1] - cy) + c)
    weldpoint2camera   = np.array([[Zc/fx*(point[0]-cx)], [Zc/fy*(point[1]-cy)], [Zc] , [1]])
    tool2robot         = vis.homogeneous(pos)
    weldpoint2robot    = tool2robot.dot(eye2hand).dot(weldpoint2camera).flatten()[0:3]
    return weldpoint2robot




if __name__ == '__main__':
    # camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    # camera.Open()
    # camera.Width.SetValue(1700)
    # camera.Height.SetValue(1200)
    # camera.OffsetX.SetValue(8)
    # camera.OffsetY.SetValue(8)
    # camera.ExposureTimeAbs = 20000
    # camera.StartGrabbing(pylon.GrabStrategy_LatestImages)
    # global CurrImg
    # path = "E:/New_Code/App_Data/Test_eye_to_hand/test.jpg"
    # if camera.IsGrabbing():
    #     try:
    #         grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    #         if grabResult.GrabSucceeded():
    #             CurrImg = grabResult.Array
    #     except:
    #         print("Camera error: ",sys.exc_info())
    # cv.imwrite(path, CurrImg)
    #
    # """ Test checkerboard  """
    # path = "E:/New_Code/App_Data/Test_eye_to_hand/test.jpg"
    # width = 7
    # height = 10
    # square_size = 15
    # point = []  # 2d points in image plane.
    # newcameramtx, _ = cv.getOptimalNewCameraMatrix(intrinsic, dist_coffs, (rows, cols), 1, (rows, cols))
    # find_chessboard_flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FILTER_QUADS + cv.CALIB_CB_NORMALIZE_IMAGE
    # criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    # calibrate_criteria = (cv.TermCriteria_COUNT + cv.TermCriteria_EPS, 500, 0.0001)
    # img = cv.imread(path, cv.IMREAD_COLOR)
    # img = cv.undistort(img, intrinsic, dist_coffs, None, newcameramtx)
    # # cv.imshow('Image', img)
    # # cv.waitKey(0)
    # gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # # cv.namedWindow('gray', cv.WINDOW_NORMAL)
    # # cv.resizeWindow('gray', 1000,1000)
    # # cv.imshow('gray',gray)
    # # cv.waitKey(0)
    # # cv.destroyAllWindows()
    # # Find the chess board corners
    #
    # ret, corners = cv.findChessboardCorners(gray, (width, height), None, find_chessboard_flags)
    #
    # # # If found, add object points, image points (after refining them)
    # if ret == True:
    #     # print(corners)
    #     corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    #     point.append(corners2[19])
    #     print("pixel coordinates of check point:",point)
    #     # Draw and display the corners
    #     img = cv.drawChessboardCorners(img, (width, height), corners2, ret)
    #     cv.namedWindow('img', cv.WINDOW_NORMAL)
    #     cv.resizeWindow('img', 1000,1000)
    #     # cv.imshow('img', img)
    #     # cv.waitKey(0)
    #
    # else:
    #     print("CHECKERBOARD NOT DETECTED!\t---> IMAGE PAIR: ", path)
    #
    # point1 = point[0][0]
    # cv.circle(img, (int(point1[0]), int(point1[1])), 5, (255, 0, 0), 5)
    # cv.imshow('img', img)
    # cv.waitKey(0)
    # # Zc = 569
    # Zc = -d / (a / fx * (point1[0] - cx) + b / fy * (point1[1] - cy) + c)
    # weldpoint2camera = np.array([[Zc / fx * (point1[0] - cx)], [Zc / fy * (point1[1] - cy)], [Zc], [1]])
    # print("weldpoint2camera:", weldpoint2camera)
    # weldpoint2robot = eye2hand.dot(weldpoint2camera).flatten()[0:3]
    # print("coordinates of check point:" ,weldpoint2robot)




    """ Test welding point"""
WeldPoints = []
pos_no = 0
path1 = "E:/New_Code/App_Data/Test_eye_to_hand/"
Images = glob.glob(path1 + '*.jpg')

Images.sort(key=natural_keys)

print(Images)
with open(scan_pos_path, 'r') as f:
    positions = [[float(num) for num in line.split('\t')] for line in f]

for image in Images:
    img = cv.imread(image, cv.IMREAD_GRAYSCALE)
    weldseam_center = WeldSeamCenter(img=img)
    data = np.array(weldseam_center)
    # RANSAC line fitting
    model = LineModelND()
    model_robust, inliers = ransac(data, LineModelND , min_samples=2,
                                residual_threshold=1, max_trials=1000)
    outliers = inliers == False
    line_x = [670, 750]
    line_y_robust = model_robust.predict_y(line_x)
    # weldseam line
    weldseam_point_1 = [line_x[0], line_y_robust[0]]
    weldseam_point_2 = [line_x[1], line_y_robust[1]]
    weldseam = (weldseam_point_1, weldseam_point_2)
    # laser center line
    # Laser center is the column 910 of the image
    laser_center_1 = [716, 600]
    laser_center_2 = [716, 700]
    laser_centerline = (laser_center_1, laser_center_2)
    # find intersection
    feature_point = line_intersection(weldseam, laser_centerline)
    print(feature_point)
    pos = positions[pos_no - 1]
    point1 = feature_point
    Zc = -d / (a / fx * (point1[0] - cx) + b / fy * (point1[1] - cy) + c)
    weldpoint2camera = np.array([[Zc / fx * (point1[0] - cx)], [Zc / fy * (point1[1] - cy)], [Zc], [1]])
    print("weldpoint2camera:", weldpoint2camera)
    weldpoint2robot = eye2hand.dot(weldpoint2camera).flatten()[0:3]
    print("coordinates of check point:" ,weldpoint2robot)
    # weldpoint2robot = calc_weldpoint2robot(feature_point, pos)
    # weldpoint2robot[2] = -35
    print(weldpoint2robot)
    pos_no += 1
    WeldPoints.append(weldpoint2robot)
    # show img
    for p in weldseam_center:
        cv.circle(img, (int(p[0]),int(p[1])), 1, (255, 255, 255), 1)
    # img[:,670] = 255
    # img[:,750] = 255

    frame = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
    cv.circle(frame, (int(feature_point[0]),int(feature_point[1])), 5, (255, 0, 0), 5)
    frame = cv.resize(frame, (870,687), interpolation = cv.INTER_AREA)
    cv.imshow("result",frame)
    cv.waitKey(0)
    # img_name = 'E:/New_Code/App_Data/Scanning_img_after_processed/' + str(pos_no) + '.jpg'
    # cv.imwrite(img_name, frame)

# WeldPoints = np.array(WeldPoints)
# savematrix(welding_trajectory, WeldPoints)

# illustrate the solution
# X = WeldPoints[:,0]
# Y = WeldPoints[:,1]
# Z = WeldPoints[:,2]
#
# fig = plt.figure()
# ax  = fig.add_subplot(111, projection="3d")
# ax.scatter(X, Y, Z, c='r' , marker = '.')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# del X, Y, Z
#
# plt.show()