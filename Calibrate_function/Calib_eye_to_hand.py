'''
Eye to hand calibration function
'''

import numpy as np
import cv2 as cv
import glob
from App_Lib.App import load_coefficients
# from numpy.lib.npyio import load
from GlobalVariables import *
from matplotlib import pyplot as plt
import time

# global variables
R_path = R_eye_to_hand
t_path = t_eye_to_hand

def coordinate(variable, position, rotation, coordinateNo):
    xvecx = position[0][0]
    yvecx = position[1][0]
    zvecx = position[2][0]
    uvecx = rotation[0][0]
    vvecx = rotation[1][0]
    wvecx = rotation[2][0]

    xvecy = position[0][0]
    yvecy = position[1][0]
    zvecy = position[2][0]
    uvecy = rotation[0][1]
    vvecy = rotation[1][1]
    wvecy = rotation[2][1]

    xvecz = position[0][0]
    yvecz = position[1][0]
    zvecz = position[2][0]
    uvecz = rotation[0][2]
    vvecz = rotation[1][2]
    wvecz = rotation[2][2]

    return variable.quiver(xvecx, yvecx, zvecx, uvecx, vvecx, wvecx, length=50, color='r', normalize=True),\
            variable.quiver(xvecy, yvecy, zvecy, uvecy, vvecy, wvecy, length=50, color='g', normalize=True),\
            variable.quiver(xvecz, yvecz, zvecz, uvecz, vvecz, wvecz, length=50, color='b', normalize=True),\
            variable.text(position[0][0] , position[1][0], position[2][0], coordinateNo, fontsize=10)

def split_homo(mat):
    r = mat[0:3, 0:3]
    rot =[]
    for i in range (3):
        rot.append(r[i])
    t = mat[0:3, 3]
    trans =[[t[0]], [t[1]], [t[2]]]
    return rot, trans

def show_coordinate(mat):
    rot, trans = split_homo(mat)
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    # ax.scatter(X, Y, Z, c='r' , marker = '.')
    ax.set_xlabel('X - axis')
    ax.set_ylabel('Y - axis')
    ax.set_zlabel('Z - axis')
    coordinate(ax, [[0], [0], [0]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "  Base Coordinate")
    coordinate(ax, trans, rot, " Camera Coordinate")
    plt.show()

def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()
    cv_file.release()
    return [camera_matrix, dist_matrix]

def create_objpoint(width, height, square_size):
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    objp = objp * square_size  # Create real world coords. Use your metric.
    return objp

def toHomo(R, t):
    c = np.array([[0, 0, 0, 1]])
    d = np.concatenate((R, t), axis = 1)
    d = np.concatenate((d, c))
    return d

def save_hand_eye_mat(mat):
    cv_file = cv.FileStorage((xml + 'Eye_to_hand_mat.yml'), cv.FILE_STORAGE_WRITE)
    cv_file.write("K1", mat)

def load_pos(path):
    m = []
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    a = cv_file.getNode("K1").mat()
    cv_file.release()
    for i in a:
        m.append(i)
    return m

def eyetohandCalibrate():
    check_path = Checkerboard_calib_eye_to_hand_path
    R_gripper2base = load_pos(R_path)
    t_gripper2base = load_pos(t_path)

    # n1 = int(input('Number start: '))
    # n2 = int(input('Number end: '))

    n1 = 1
    n2 = int(np.shape(R_gripper2base)[0])

    print("\n HAND-EYE CALIBRATING _____________________________________________")
    start = time.time()

    R_gripper2base = R_gripper2base[n1-1:n2]
    t_gripper2base = t_gripper2base[n1-1:n2]

    R_base2gripper = []
    for i in R_gripper2base:
        matrix = np.transpose(i)
        R_base2gripper.append(matrix)
    t_base2gripper = []
    k=0
    for j in t_gripper2base:
        vec = -np.dot(R_base2gripper[k], j)
        t_base2gripper.append(vec)
        k+=1

    R_target2cam = []
    t_target2cam = []

    path_to_handeye = check_path + '*.jpg'
    intrinsic, dist_coffs = load_coefficients((camera_external_params))

    width = 4
    height = 5
    square_size = 30
    objp = create_objpoint(width, height, square_size)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 3000, 0.00001)
    find_chessboard_flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FILTER_QUADS + cv.CALIB_CB_NORMALIZE_IMAGE

    a = 0
    images = glob.glob(path_to_handeye)
    print(np.shape(images))
    for fname in images:
        a += 1
        if a > n2:
            break
        if n1 <= a <= n2:
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, (width, height), None, find_chessboard_flags)
            if ret:
                corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                img = cv.drawChessboardCorners(img, (width, height), corners2, ret)
                # Return the Rotation and the Translation VECTORS that transform a
                # 3D point expressed in the object coordinate frame to the camera coordinate frame
                retval, rvec, tvec = cv.solvePnP(objp, corners2, intrinsic, dist_coffs, flags=cv.SOLVEPNP_ITERATIVE)
                rotation_matrix = np.zeros(shape=(3, 3))
                # Convert a rotation matrix to a rotation vector or vice versa
                cv.Rodrigues(rvec, rotation_matrix)
                R_target2cam.append(rotation_matrix)
                t_target2cam.append(tvec)
                # resized = cv.resize(img, (720, 480), interpolation = cv.INTER_AREA)
                # cv.imshow("pic: ", resized)
                # cv.waitKey(0)
            else:
                print("CHECKERBOARD NOT DETECTED!\t---> IMAGE PAIR: ", fname)
            pass

    print(R_target2cam)
    print(t_target2cam)
    print(t_base2gripper)
    # Use Daniilidis due to better accuracy than Tsai
    # Do not use Tsai due to poor accuracy
    # R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv.CALIB_HAND_EYE_DANIILIDIS)
    R_cam2base, t_cam2base = cv.calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam, t_target2cam,method = cv.CALIB_HAND_EYE_TSAI)
    # R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method = cv.CALIB_HAND_EYE_TSAI)
    end = time.time()
    
    print("\t:: TRANSFORMATION MATRIX OF CAMERA TO END-EFFECTOR:")
    print("\t:: Rotation R:\n", R_cam2base)
    print("\t:: Translation t:\n", t_cam2base)
    H = toHomo(R_cam2base,t_cam2base)
    save_hand_eye_mat(H)
    print(f'\t:: Homogenous transformation:\n{H}')
    print("\t:: Time consumed: %.2lf s\t\t" %(end-start))
    print("_______________________________________ HAND-EYE CALIBRATION DONE |")
    # show_coordinate(H)
    return H
if __name__ == '__main__':

    '''
    test convert to R base to target
    '''
 #    # T_gripper_base = np.array([
 #    #     [0.7071, -0.7071, 0.0000, 0.0000],
 #    #     [0.7071, 0.7071, 0.0000, 0.0000],
 #    #     [0.0000, 0.0000, 1.0000, 0.1000],
 #    #     [0.0000, 0.0000, 0.0000, 1.0000]])
 #    # T_base_gripper = np.linalg.inv(T_gripper_base)
 #    # print(T_base_gripper)
 #    R = np.array([[ 9.27188431e-01,  6.53792075e-07,  3.74595266e-01],
 # [ 0.00000000e+00, -1.00000000e+00 , 1.74532925e-06],
 # [ 3.74595266e-01, -1.61824909e-06 ,-9.27188431e-01]])
 #    #
 #    t = np.array([ 761.995,  -30.994, -152.507])
 #    #
 #    # # compute the transformation matrix from base to gripper
 #    R_inv = np.transpose(R)
 #    t_inv = -np.dot(R_inv, t)
 #    print("R_inv",R_inv)
 #    print(" t_inv",t_inv)
 #    T_base_gripper = np.vstack((np.hstack((R_inv, t_inv.reshape((3, 1)))), np.array([0, 0, 0, 1])))
 #
 #    print(T_base_gripper)
 #    # R_inv[[9.27188431e-01  0.00000000e+00  3.74595266e-01]
 #    # [6.53792075e-07 - 1.00000000e+00 - 1.61824909e-06]
 #    # [3.74595266e-01
 #    # 1.74532925e-06 - 9.27188431e-01]]
 #    # t_inv[-649.38454825 - 30.99474498 - 426.84239167]
 #
 #    R_gripper2base = load_pos(R_path)
 #    t_gripper2base = load_pos(t_path)
 #    print('R_gripper2base:',R_gripper2base[1])
 #    print('t_gripper2base:', t_gripper2base[1])
 #    R_base2gripper = []
 #    for i in R_gripper2base:
 #        matrix = np.transpose(i)
 #        R_base2gripper.append(matrix)
 #    t_base2gripper = []
 #    k=0
 #    for j in t_gripper2base:
 #        vec = -np.dot(R_base2gripper[k], j)
 #        t_base2gripper.append(vec)
 #        k+=1
 #    # t_base2gripper = -np.dot(R_base2gripper[1], t_gripper2base[1])
 #    print('R_base2gripper:',R_base2gripper[15])
 #    print('R_t_base2gripper:',t_base2gripper[15])

 #end test.
    eyetohandCalibrate()