import numpy as np
import cv2 as cv
import glob
width = 4
height = 5
square_size = 30
# width = 10
# height = 7
# square_size = 15
# termination criteria
find_chessboard_flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FILTER_QUADS + cv.CALIB_CB_NORMALIZE_IMAGE
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 40, 0.001)
calibrate_criteria = (cv.TermCriteria_COUNT + cv.TermCriteria_EPS, 500, 0.0001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
objp = np.zeros((height * width, 3), np.float32)
objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
objp = objp * square_size  # Create real world coords. Use your metric.
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
# images = glob.glob(r'D:\Code\test\test6\*.bmp')
images = glob.glob(r'E:\New_Code\App_Data\Checkerboard_calib_eye_to_hand\*.jpg')
print(images)
# for fname in images:
for fname in images:
    # cv.namedWindow('Image', cv.WINDOW_NORMAL)
    # cv.resizeWindow('Image', 1000,1000)
    img = cv.imread(fname, cv.IMREAD_COLOR)
    # cv.imshow('Image', img)
    # cv.waitKey(0)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # cv.namedWindow('gray', cv.WINDOW_NORMAL)
    # cv.resizeWindow('gray', 1000,1000)
    # cv.imshow('gray',gray)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    # Find the chess board corners

    ret, corners = cv.findChessboardCorners(gray, (width, height), None, find_chessboard_flags)

    # # If found, add object points, image points (after refining them)
    if ret == True:
        # print(corners)
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        img = cv.drawChessboardCorners(img, (width, height), corners2, ret)
        cv.namedWindow('img', cv.WINDOW_NORMAL)
        cv.resizeWindow('img', 640,640)
        cv.imshow('img', img)
        cv.waitKey(0)

    else:
        print("CHECKERBOARD NOT DETECTED!\t---> IMAGE PAIR: ", fname)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, cv.CALIB_USE_INTRINSIC_GUESS, criteria=calibrate_criteria)
cv.destroyAllWindows()
fx = mtx[0][0]
fy = mtx[1][1]
cx = mtx[0][2]
cy = mtx[1][2]
# Tính toán Re-projection Error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i],imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print("Re-projection error: ", mean_error/len(objpoints))

# def calculate_distance(path1,path2,mtx,dist,rvecs,tvecs):
#     ## point1
#     img1 = cv.imread(path1)
#     gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
#     ret, corners1 = cv.findChessboardCorners(gray1, (width,height),None, find_chessboard_flags)
#     # Undistort pixel point
#     print("mtx",mtx)
#     print("dist",dist)
#     undistorted_corners1 = cv.undistortPoints(corners1[1],mtx,dist)
#     print("undis",undistorted_corners1)
#     print("dis", corners1[1])
#     # ## point2
#     # img2 = cv.imread(path2)
#     # gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
#     # ret, corners2 = cv.findChessboardCorners(gray2, (width, height), None, find_chessboard_flags)
#     # # Convert pixel to worth
#     # undistorted_corners2 = cv.undistortPoints(corners2, mtx, dist)
#     # print("point2/n", undistorted_corners2[1])
#     # print("distance x2,y2", undistorted_corners2[2] - undistorted_corners2[1])
#     # # print("distance y", dis_y)
# def cal(path1,path2,mtx,dist,rvecs,tvecs,fx,fy,cx,cy,height,width):
# ## point1
#     square_size = 30
#     objp1 = np.zeros((height * width, 3), np.float32)
#     objp1[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
#     objp1 = objp1 * square_size  # Create real world coords. Use your metric.
#     img1 = cv.imread(path1)
#     gray1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
#     ret, corners1 = cv.findChessboardCorners(gray1, (width,height),None, find_chessboard_flags)
#     image_point1 = cv.cornerSubPix(gray1,corners1, (11,11), (-1,-1), criteria)
#     point_00 = image_point1[0, 0, 0]  # lấy tọa độ x của điểm đầu tiên
#     point_01 = image_point1[0, 0, 1]  # lấy tọa độ y của điểm đầu tiên
#     undistorted_point1 = cv.undistortPoints(image_point1, mtx, dist)
#     normalized_point1 = cv.convertPointsToHomogeneous(undistorted_point1)
#     retval1, rvec1, tvec1 = cv.solvePnP(objp1,image_point1, mtx, dist)
#     rotation_matrix1, _ = cv.Rodrigues(rvec1)
#     Zc =800
#     Xc1 =Zc / fx * (point_00 - cx)
#     Yc1 =Zc / fy * (point_01 - cy)
#     # world_point1 = -np.dot(rotation_matrix.T, tvec)
#     # world_point2 = -np.dot(rotation_matrix.T, tvec + np.array([[1], [1], [1]]))
#     # distance = np.linalg.norm(world_point2 - world_point1)
#     C1 = np.array([[Xc1], [Yc1], [Zc]])
#     print("point1: ",C1)
#     # print("point2: ",world_point2)
#     # print('Distance between the two points in world coordinates: {:.2f}'.format(distance))
#
# # ## point2
#     objp2 = np.zeros((height * width, 3), np.float32)
#     objp2[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
#     objp2 = objp2 * square_size  # Create real world coords. Use your metric.
#     img2 = cv.imread(path2)
#     gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
#     ret, corners2 = cv.findChessboardCorners(gray2, (width, height), None, find_chessboard_flags)
#     image_point2 = cv.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
#     point_10 = image_point2[0, 0, 0]  # lấy tọa độ x của điểm đầu tiên
#     point_11 = image_point2[0, 0, 1]  # lấy tọa độ y của điểm đầu tiên
#     undistorted_point2 = cv.undistortPoints(image_point2, mtx, dist)
#     normalized_point2 = cv.convertPointsToHomogeneous(undistorted_point2)
#     retval2, rvec2, tvec2 = cv.solvePnP(objp,image_point2, mtx, dist)
#     rotation_matrix, _ = cv.Rodrigues(rvec2)
#     Zc=800
#     Xc2 =Zc / fx * (point_10 - cx)
#     Yc2 =Zc / fy * (point_11 - cy)
#     # world_point1 = -np.dot(rotation_matrix.T, tvec)
#     # world_point2 = -np.dot(rotation_matrix.T, tvec + np.array([[1], [1], [1]]))
#     # distance = np.linalg.norm(world_point2 - world_point1)
#     C2 = np.array([[Xc2], [Yc2], [Zc]])
#     print("point2: ",C2)
#     # print("point2: ",world_point2)
#     distance = C2-C1
#     print('Distance between the two points in world coordinates:',distance)
#     # cv.circle(img1, (int(cx),int(cy)), 10, (0, 0, 255), 1)
#     # cv.namedWindow('img', cv.WINDOW_NORMAL)
#     # cv.resizeWindow('img', 640,640)
#     # cv.imshow("img",img1)
#     # cv.waitKey(0)
# # path1 = r'D:\Code\test\test5\ref1_000.bmp'
# # path2 = r'D:\Code\test\test5\ref1_001.bmp'
#
# path1 = r'D:\Code\test\new_test6\delx1.bmp'
# path2 = r'D:\Code\test\new_test6\dely1.bmp'
# cal(path1,path2,mtx,dist,rvecs,tvecs,fx,fy,cx,cy,height,width)
# print("cx:",cx,"cy:",cy)
