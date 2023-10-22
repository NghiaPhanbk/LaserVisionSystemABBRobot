import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtGui
from Seamtracking import Ui_MainWindow
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from GlobalVariables import *
import ScanningProcessing
import matplotlib.pyplot as plt
import Finding_trajectory
import Main_Scanning
import WeldingTraject_filter
import Main_Weld_Aft_Scan
import Calib_process
import Eye_to_hand_calib_process
from Calibrate_function import Calib_camera
from Calibrate_function import Calib_handeye
from Calibrate_function import Calib_laser
from Calibrate_function import Calib_external_camera
from Calibrate_function import Calib_eye_to_hand
from Calibrate_function import Calib_laser_eye_to_hand
import numpy as np
import cv2 as cv
import time, sys
from skimage.measure import ransac, LineModelND
import glob
import threading
from App_Lib import syntec_Lib
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from ABB_interface import ABB_Lib
# Globel variable #
# Eye in hand
import os
check_path_in = checkerboard_calib_cam_path
laser_path_in = checkerboard_calib_laser_path
robot_path_in = calib_trajectory  # Open when calibrating
R_path_in = R_path
t_path_in = t_path

# Eye to hand
check_path_ex = Checkerboard_calib_eye_to_hand_path
laser_path_ex = checkerboard_calib_laser_eye_to_hand_path
robot_path_ex = calib_trajectory_eye_to_hand  # Open when calibrating
R_path_ex = R_eye_to_hand
t_path_ex = t_eye_to_hand
caption_picture_ex = 0
class scanning(QThread):
    image = pyqtSignal(QImage)
    trajectory = pyqtSignal(str)
    def __init__(self, parent = None):
        super(QThread,self).__init__(parent)
        self.stopped = False
        self.CurrentPos = []
        self.robot = ABB_Lib.ABB()
        # self.robot_path = robot_path
        # self.robot.StartRequest()
        self.flag_robot = self.robot.check_connection()
        if self.flag_robot == 0:
            print("no robot")
    def run(self):
        WeldPoints = []
        pos_no = 1

        Images = glob.glob(scan_weld_seam_img_path +'/weldseam' + '*.jpg')

        Images.sort(key=ScanningProcessing.natural_keys)
        # laser center line
        # laser_center_1 = [785, 720]
        # laser_center_2 = [785, 480]
        path = "E:\Thesis\App_Data\Scan_data3\weldseam_6.jpg"
        image_laser = cv.imread(path, cv.IMREAD_GRAYSCALE)
        # img = cv.imread(image, cv.IMREAD_GRAYSCALE)
        # laser line
        laser_center_1, laser_center_2 = ScanningProcessing.get_laser_line(image_laser)
        laser_centerline = (laser_center_1, laser_center_2)
        # print(Images)
        with open(scan_pos_path, 'r') as f:
            positions = [[float(num) for num in line.split('\t')] for line in f]
        with open(scan_pos_path, 'w') as f:
            pass
        for image in Images:
            img = cv.imread(image, cv.IMREAD_GRAYSCALE)
            # laser line
            # laser_center_1, laser_center_2 = ScanningProcessing.get_laser_line(img)
            # laser_centerline = (laser_center_1, laser_center_2)
            weldseam_center = ScanningProcessing.WeldSeamCenter(img=img)
            for point in weldseam_center:
                center = tuple(point)
                # cv.circle(img, center, 5, [255, 0, 0], 5)
            # print(weldseam_center)
            # cv.namedWindow('img', cv.WINDOW_NORMAL)
            # cv.resizeWindow('img', 640,640)
            # cv.imshow("img",img)
            # cv.waitKey(0)
            data = np.array(weldseam_center)
            # RANSAC line fitting
            model = LineModelND()
            model_robust, inliers = ransac(data, LineModelND, min_samples=2,
                                           residual_threshold=1, max_trials=1000)
            outliers = inliers == False
            line_x = [750, 800]
            line_y_robust = model_robust.predict_y(line_x)
            # weldseam line
            weldseam_point_1 = [line_x[0], line_y_robust[0]]
            weldseam_point_2 = [line_x[1], line_y_robust[1]]
            weldseam = (weldseam_point_1, weldseam_point_2)
                # find intersection
            feature_point = ScanningProcessing.line_intersection(weldseam, laser_centerline)
            pos = positions[pos_no - 1]
            weldpoint2robot = ScanningProcessing.calc_weldpoint2robot(feature_point, pos)
            # weldpoint2robot[2] = -35
            weldpoint2robot_convert = ', '.join(str(i) for i in weldpoint2robot)
            # self.data_signal.emit(weldpoint2robot_convert)
            print(weldpoint2robot)
            # self.uic.infor_process_in.addItem((str(weldpoint2robot)))
            # self.trajectory.emit(weldpoint2robot)
            self.trajectory.emit(weldpoint2robot_convert)
            # self.trajectory_signal.emit("hihi")
            pos_no += 1
            WeldPoints.append(weldpoint2robot)
            # show img
            for p in weldseam_center:
                cv.circle(img, (int(p[0]), int(p[1])), 1, (255, 255, 255), 1)
            # img[:, 700] = 255
            # img[:, 900] = 255
            frame = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
            cv.circle(frame, (int(feature_point[0]), int(feature_point[1])), 5, (255, 0, 0), 5)
            frame = cv.resize(frame, (870, 687), interpolation=cv.INTER_AREA)


            self.image_in = frame
            self.img = cv.resize(self.image_in, (491, 381), interpolation=cv.INTER_AREA)
            height, width, channel = self.img.shape
            bytesPerLine = 3*width
            # cv.imshow("hihi",self.image_ex)
            self.img = QtGui.QImage(self.img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            # self.uic.show_image_calib_in.setPixmap(QtGui.QPixmap.fromImage(self.img))
            self.image.emit(self.img)
            img_name = 'E:/Thesis/App_Data/Scanning_img_after_processed/' + str(pos_no) + '.jpg'
            cv.imwrite(img_name, frame)
            if (pos_no >= np.shape(positions)[0]):
                break

        WeldPoints = np.array(WeldPoints)
        WeldPoints[:, 2] = 158
        ScanningProcessing.savematrix(welding_trajectory, WeldPoints)
        # self.trajectory_signal.emit("hihi")
        # illustrate the solution
        # self.X = WeldPoints[:, 0]
        # self.Y = WeldPoints[:, 1]
        # self.Z = WeldPoints[:, 2]
        # traj = ScanningProcessing.generate_trajectory()
        # self.X, self.Y, self.Z = self.scanning_process.main_scanning()
        # class show_chart(FigureCanvasQTAgg):
        #     def __init__(self,X,Y,Z):
        #         self.fig = plt.figure()
        #         self.ax = self.fig.add_subplot(111, projection="3d")
        #         super().__init__(self.fig)
        #         # traj = ScanningProcessing.generate_trajectory()
        #         # self.X, self.Y, self.Z = traj.main_scanning()
        #         # self.ax.scatter(self.X, self.Y, self.Z, c='r', marker='.')
        #         self.ax.scatter(X, Y,Z, c='r', marker='.')
        #         self.ax.set_xlabel('x')
        #         self.ax.set_zlabel('z')
        #         # del X, Y, Z
        #         # plt.show()
        # self.uic.welding_trajectory.addWidget(show_chart(self.X,self.Y,self.Z))

    def stop(self):
        self.stopped = True

class MainWindow:
    image = pyqtSignal(QImage)
    trajectory = pyqtSignal(str)
    def __init__(self):
        self.main_win = QMainWindow()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self.main_win)
#### global variable

                                    # eye in hand
        choice = 1
        self.pos_no_in = 0
        self.R_end2base_in = []
        self.T_end2base_in = []
        # self.H_end2base_model_in = []
        self.check_count_in = 0
        self.check_laser_count_in = 0
        self.laser_count_in = 0
        self.caption_picture_in = 0
        # self.image_in = np.zeros((1700, 1200, 3), np.uint8)


        # self.VisionSystem_in = Calib_process.BaslerCam(choice)
        # self.Robot_in = Calib_process.RobotTask(choice)


        # self.filename = ""
        self.flag_calib_in = 0
        self.flag = 0
                        # main scanning
        self.flag_scan = 0
        self.scanning_running = False
                        # function button

        # function button
        self.uic.Star_calib_process_in.clicked.connect(self.start_calib_in)
        self.uic.Move_in.clicked.connect(self.move_robot_in)
        self.uic.Get_pos_in.clicked.connect(self.get_pos_in)
        # self.uic.Save_pos_in.clicked.connect(self.save_pos_in)
        self.uic.Checkerboard_in.clicked.connect(lambda: self.capture_image(self.uic.checkBox.isChecked()))
        # self.uic.Checker_laser_in.clicked.connect(self.capture_checker_in)
        # self.uic.Cap_laser_in.clicked.connect(self.capture_laser_in)
        self.uic.Calib_main_in.clicked.connect(self.run_calibrate)


        #check
        # self.uic.Calib_laser_in_3.clicked.connect(self.calib_laser_internal_cam)
        # self.uic.Continue_8.clicked.connect(self.show_screen7)
        # self.uic.Back_8.clicked.connect(self.show_screen3)
        # self.uic.Scanning_processing.clicked.connect(self.show_diagram)
        self.uic.Run_scan.clicked.connect(self.run_main_scanning)
        # self.uic.Run_scan.clicked.connect(self.test)
        #check


        # Define button screen 7
        # self.uic.start_weld.clicked.connect(self.scanning)
        self.uic.Stop.clicked.connect(self.run_start_weld)
        # self.uic.Stop.clicked.connect(self.test)

        #main scanning
        # self.run_scan = Main_Scanning.main_scan()
        #scanning processing
        self.scanning_process = ScanningProcessing.generate_trajectory()
    # def show_screen4(self,Screen_ind):
    #     self.uic.tabWidget.setCurrentWidget(self.uic.Calib_laser)
    # def show_screen5(self,Screen_ind):
    #     self.uic.tabWidget.setCurrentWidget(self.uic.Get_scan_trectory)
    # def show_screen6(self,Screen_ind):
    #     self.uic.tabWidget.setCurrentWidget(self.uic.main_scan)
    # def show_screen7(self,Screen_ind):
    #     self.uic.tabWidget.setCurrentWidget(self.uic.welding)
    #     WeldingTraject_filter.trajectory_filter()
    #     with open(welding_trajectory_filtered, 'r') as f:
    #         positions = [[round(float(num), 3) for num in line.split('\t')] for line in f]
    #
    #     positions = np.array(positions)
    #     self.X = positions[:, 0]
    #     self.Y = positions[:, 1]
    #     self.Z = positions[:, 2]
    #     class show_weld_chart(FigureCanvasQTAgg):
    #         def __init__(self,X,Y,Z):
    #             self.fig = plt.figure()
    #             self.ax = self.fig.add_subplot(111, projection="3d")
    #             super().__init__(self.fig)
    #             # traj = ScanningProcessing.generate_trajectory()
    #             # self.X, self.Y, self.Z = traj.main_scanning()
    #             # self.ax.scatter(self.X, self.Y, self.Z, c='r', marker='.')
    #             self.ax.scatter(X, Y,Z, c='r', marker='.')
    #             self.ax.set_xlabel('x')
    #             self.ax.set_zlabel('z')
    #             # del X, Y, Z
    #             # plt.show()
    #     self.uic.welding_trajectory.addWidget(show_weld_chart(self.X,self.Y,self.Z))

    # function for page 2 eye in hand#
    def start_calib_in(self):
        choice = 1
        self.VisionSystem_in = Calib_process.BaslerCam(choice)
        self.Robot_in = Calib_process.RobotTask(choice)
        self.pos_no_in = 1
        if (self.VisionSystem_in.flag_cam_in == 1 & self.Robot_in.flag_robot == 1):
            self.VisionSystem_in.start()
            self.Robot_in.start()
            self.flag_calib_in = 1
            self.uic.infor_process_in.addItem("There are 16 waypoints in robot trajectory")
        if self.VisionSystem_in.flag_cam_in == 0:
            self.uic.infor_process_in.addItem("Connection error with acA1920-40gm")
        if self.Robot_in.flag_robot == 0:
            self.uic.infor_process_in.addItem("Robot connection error")
        self.uic.infor_process_in.scrollToBottom()
    def move_robot_in(self):
        if self.flag_calib_in == 1:
            self.Robot_in.robot_move_to_pos()
            self.pos_no_in = self.pos_no_in + 1
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
    def save_pos_in(self):
        if self.flag_calib_in == 1:
            RR = np.array(self.R_end2base_in)
            TT = np.array(self.T_end2base_in)
            self.Robot_in.save(R_path_in, RR)
            self.Robot_in.save(t_path_in, TT)
            print('ALL WAYPOINT TRANSFORMATIONS SAVED!')
            self.uic.infor_process_in.addItem('ALL WAYPOINT TRANSFORMATIONS SAVED!')
        else:
            print("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
        # time.sleep(3)
    def get_pos_in(self):
        if self.flag_calib_in == 1:
            self.Robot_in.read_pos_from_robot()
            R, t = self.Robot_in.read()
            print(f'\nTRANSFORMATION AT WAYPOINT {self.pos_no_in - 1}:')
            self.uic.infor_process_in.addItem(f'\nTRANSFORMATION AT WAYPOINT {self.pos_no_in - 1}:')
            print('Rotation R:\n', R)
            self.uic.infor_process_in.addItem("Rotation R:")
            self.uic.infor_process_in.addItem(str(R))
            print('Translation T:\n', t)
            self.uic.infor_process_in.addItem("Translation T:")
            self.uic.infor_process_in.addItem(str(t))
            self.R_end2base_in.append(R)
            self.T_end2base_in.append(t)
            self.save_pos_in()
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
        time.sleep(0.5)
    def capture_checker_in(self):
        if self.flag_calib_in == 1:
            self.show_calib_img_in()
            self.check_laser_count_in += 1
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if self.check_laser_count_in < 10:
                self.filename = laser_path_in + "\checker_0" + str(self.check_laser_count_in) + '.jpg'
            elif 10 <= self.check_laser_count_in <= 16:
                self.filename = laser_path_in + "\checker_" + str(self.check_laser_count_in) + '.jpg'
            else:
                self.check_laser_count_in = 0
                print("out of range")
                self.uic.infor_process_in.addItem("out of range")
            cv.imwrite(self.filename, self.VisionSystem_in.image)
            print('Captured checker laser image\nPair %d ' % (self.check_laser_count_in))
            self.uic.infor_process_in.addItem('Captured checker laser image')
            self.uic.infor_process_in.addItem('Pair %d ' % (self.check_laser_count_in))
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
    def capture_image_in(self):
        if self.flag_calib_in == 1:
            self.show_calib_img_in()
            self.check_count_in += 1
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if self.check_count_in < 10:
                self.filename = check_path_in + "\checkerboard_0" + str(self.check_count_in) + '.jpg'
            elif 10 <= self.check_count_in <= 16:
                self.filename = check_path_in + "\checkerboard_" + str(self.check_count_in) + '.jpg'
            else:
                self.check_count_in =0
                print("out of range")
                self.uic.infor_process_in.addItem("out of range")
            cv.imwrite(self.filename, self.VisionSystem_in.image)
            print('Captured checkerboard image\nPair %d ' % (self.check_count_in))
            self.uic.infor_process_in.addItem('Captured checkerboard image')
            self.uic.infor_process_in.addItem('Pair %d'% (self.check_count_in))
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()

    def capture_laser_in(self):
        if self.flag_calib_in == 1:
            self.show_calib_img_in()
            self.laser_count_in += 1
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if self.laser_count_in < 10:
                self.filename = laser_path_in + "\laser_0" + str(self.laser_count_in) + '.jpg'
            elif 10 <= self.laser_count_in <=16:
                self.filename = laser_path_in + "\laser_" + str(self.laser_count_in) + '.jpg'
            else:
                self.laser_count_in =0
                print("out of range")
                self.uic.infor_process_in.addItem("out of range")
            cv.imwrite(self.filename, self.VisionSystem_in.image)
            print('Captured laser line image\nPair %d ' % (self.laser_count_in))
            self.uic.infor_process_in.addItem('Captured laser line image')
            self.uic.infor_process_in.addItem('Pair %d ' % (self.laser_count_in))
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
    def capture_image(self, check):
        if check:
            self.capture_laser_in()
        else:
            self.capture_image_in()
            self.capture_checker_in()
    def quit_in(self):
        self.VisionSystem_in.stop()
        self.Robot_in.stop()
        print("End Process")
        self.uic.infor_process_in.addItem("End Process")
        self.uic.infor_process_in.scrollToBottom()
    def show_calib_img_in(self):
        # result_path = r"E:/New_Code/App_Data/Test_eye_to_hand/result.jpg"
        # self.img = cv.imread(result_path)
        self.image_in = self.VisionSystem_in.image
        self.img = cv.resize(self.image_in, (451, 331), interpolation=cv.INTER_AREA)
        height, width = self.img.shape
        bytesPerLine = width
        # cv.imshow("hihi",self.image_ex)
        self.img = QtGui.QImage(self.img.data, width, height, bytesPerLine, QtGui.QImage.Format_Grayscale8)
        self.uic.show_image_calib_in.setPixmap(QtGui.QPixmap.fromImage(self.img))

    def calib_eye_in_hand(self):
        Calib_camera.monoCalibrate()
        tranform_in = Calib_handeye.handeyeCalibrate()
        self.uic.infor_process_in.addItem("Transformation matrix Robot-Camera: ")
        tranform_in_string = str(tranform_in)
        self.uic.infor_process_in.addItem(tranform_in_string)
        self.rot_in, self.trans_in = Calib_handeye.split_homo(tranform_in)

        class show_chart(FigureCanvasQTAgg):
            def __init__(self, rot, trans):
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection="3d")
                super().__init__(self.fig)
                # traj = ScanningProcessing.generate_trajectory()
                # self.X, self.Y, self.Z = traj.main_scanning()
                # self.ax.scatter(self.X, self.Y, self.Z, c='r', marker='.')
                self.ax.set_xlabel('X - axis')
                self.ax.set_ylabel('Y - axis')
                self.ax.set_zlabel('Z - axis')
                Calib_eye_to_hand.coordinate(self.ax, [[0], [0], [0]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                             " Camera Coordinate")
                Calib_eye_to_hand.coordinate(self.ax, trans, rot, " End Effector Coordinate")
                # del X, Y, Z
                # plt.show()

        self.uic.eyeinhand_graph.addWidget(show_chart(self.rot_in, self.trans_in))
    def calib_laser_internal_cam(self):
        self.point,self.plane = Calib_laser.laserCalibrate()
        self.uic.infor_process_in.addItem("Laser Plane: ")
        for row in self.point:
            row_str = " ".join(str(val) for val in row)
            # print(row_str)
        self.uic.infor_process_in.addItem("Z = " + str(self.plane[0][0]).strip('[]') + "X + " + str(self.plane[1][0]).strip('[]') + "Y + "+str(self.plane[2][0]).strip('[]'))
        class show_chart(FigureCanvasQTAgg):
            def __init__(self,pointinlaserplane,plane):
                X = np.array([0])
                Y = np.array([0])
                Z = np.array([0])
                for i in range(len(pointinlaserplane)):
                    X = np.insert(X, i, pointinlaserplane[i][0][0])
                    Y = np.insert(Y, i, pointinlaserplane[i][1][0])
                    Z = np.insert(Z, i, pointinlaserplane[i][2][0])
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection="3d")
                super().__init__(self.fig)
                # traj = ScanningProcessing.generate_trajectory()
                # self.X, self.Y, self.Z = traj.main_scanning()
                # self.ax.scatter(self.X, self.Y, self.Z, c='r', marker='.')
                self.ax.scatter(X,Y,Z, c='r', marker='.')
                self.ax.set_xlabel('x')
                self.ax.set_ylabel('y')
                self.ax.set_zlabel('z')
                del X, Y, Z
                x = np.linspace(-100, 100, 10)
                y = np.linspace(-100, 100, 10)
                X, Y = np.meshgrid(x, y)
                Z = plane[0][0] * X + plane[1][0] * Y + plane[2][0]
                surf = self.ax.plot_surface(X, Y, Z, rstride=5, cstride=5, color='r', alpha=0.3)
                x = np.array([0, 0, 0])
                y = np.array([0, 0, 0])
                z = np.array([0, 0, 0])
                u = np.array([1, 0, 0])
                v = np.array([0, 1, 0])
                w = np.array([0, 0, 1])

                self.ax.quiver(x, y, z, u, v, w, length=50, normalize=True)
                self.ax.text(50, 0, 0, 'X', fontsize=10)
                self.ax.text(0, 50, 0, 'Y', fontsize=10)
                self.ax.text(0, 0, 50, 'Z', fontsize=10)
                self.ax.set_xlim3d(-300, 300)
                self.ax.set_ylim3d(-300, 300)
                self.ax.set_zlim3d(200, 600)
                # plt.show()
        self.uic.laser_in_graph.addWidget(show_chart(self.point,self.plane))

    def run_calibrate(self):
        self.quit_in()
        self.calib_eye_in_hand()
        self.calib_laser_internal_cam()
        self.uic.infor_process_in.scrollToBottom()


    ##scanning processing
    def show_trajectory(self):
        with open(welding_trajectory, 'r') as f:
            positions = [[round(float(num), 3) for num in line.split('\t')] for line in f]

        positions = np.array(positions)
        self.X = positions[:, 0]
        self.Y = positions[:, 1]
        self.Z = positions[:, 2]
        class show_weld_chart(FigureCanvasQTAgg):
            def __init__(self,X,Y,Z):
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection="3d")
                super().__init__(self.fig)
                # traj = ScanningProcessing.generate_trajectory()
                # self.X, self.Y, self.Z = traj.main_scanning()
                # self.ax.scatter(self.X, self.Y, self.Z, c='r', marker='.')
                self.ax.scatter(X, Y,Z, c='r', marker='.')
                self.ax.set_xlabel('x')
                self.ax.set_zlabel('z')
                # del X, Y, Z
                # plt.show()
        self.uic.welding_trajectory.addWidget(show_weld_chart(self.X,self.Y,self.Z))
        self.delete_all_images(scan_weld_seam_img_path)
        self.flag = 1
    def show_diagram(self):
        WeldPoints = []
        pos_no = 1

        Images = glob.glob(scan_weld_seam_img_path +'/weldseam' + '*.jpg')

        Images.sort(key=ScanningProcessing.natural_keys)
        # laser center line
        # laser_center_1 = [785, 720]
        # laser_center_2 = [785, 480]
        path = "E:\Thesis\App_Data\Scan_data3\weldseam_6.jpg"
        image_laser = cv.imread(path, cv.IMREAD_GRAYSCALE)
        # img = cv.imread(image, cv.IMREAD_GRAYSCALE)
        # laser line
        laser_center_1, laser_center_2 = ScanningProcessing.get_laser_line(image_laser)
        laser_centerline = (laser_center_1, laser_center_2)
        print(Images)
        with open(scan_pos_path, 'r') as f:
            positions = [[float(num) for num in line.split('\t')] for line in f]
        for image in Images:
            img = cv.imread(image, cv.IMREAD_GRAYSCALE)
            # laser line
            # laser_center_1, laser_center_2 = ScanningProcessing.get_laser_line(img)
            # laser_centerline = (laser_center_1, laser_center_2)
            weldseam_center = ScanningProcessing.WeldSeamCenter(img=img)
            for point in weldseam_center:
                center = tuple(point)
                cv.circle(img, center, 5, [255, 0, 0], 5)
            # print(weldseam_center)
            # cv.namedWindow('img', cv.WINDOW_NORMAL)
            # cv.resizeWindow('img', 640,640)
            # cv.imshow("img",img)
            # cv.waitKey(0)
            data = np.array(weldseam_center)
            # RANSAC line fitting
            model = LineModelND()
            model_robust, inliers = ransac(data, LineModelND, min_samples=2,
                                           residual_threshold=1, max_trials=1000)
            outliers = inliers == False
            line_x = [750, 800]
            line_y_robust = model_robust.predict_y(line_x)
            # weldseam line
            weldseam_point_1 = [line_x[0], line_y_robust[0]]
            weldseam_point_2 = [line_x[1], line_y_robust[1]]
            weldseam = (weldseam_point_1, weldseam_point_2)
                # find intersection
            feature_point = ScanningProcessing.line_intersection(weldseam, laser_centerline)
            pos = positions[pos_no - 1]
            weldpoint2robot = ScanningProcessing.calc_weldpoint2robot(feature_point, pos)
            # weldpoint2robot[2] = -35
            weldpoint2robot_convert = ', '.join(str(i) for i in weldpoint2robot)
            # self.data_signal.emit(weldpoint2robot_convert)
            print(weldpoint2robot)
            self.uic.infor_process_in.addItem((str(weldpoint2robot)))
            # self.trajectory_signal.emit(weldpoint2robot_convert)
            # self.trajectory_signal.emit("hihi")
            pos_no += 1
            WeldPoints.append(weldpoint2robot)
            # show img
            for p in weldseam_center:
                cv.circle(img, (int(p[0]), int(p[1])), 1, (255, 255, 255), 1)
            img[:, 700] = 255
            img[:, 900] = 255
            frame = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
            cv.circle(frame, (int(feature_point[0]), int(feature_point[1])), 5, (255, 0, 0), 5)
            frame = cv.resize(frame, (870, 687), interpolation=cv.INTER_AREA)


            self.image_in = frame
            self.img = cv.resize(self.image_in, (451, 331), interpolation=cv.INTER_AREA)
            height, width, channel = self.img.shape
            bytesPerLine = 3*width
            # cv.imshow("hihi",self.image_ex)
            self.img = QtGui.QImage(self.img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.uic.show_image_calib_in.setPixmap(QtGui.QPixmap.fromImage(self.img))

            img_name = 'E:/Thesis/App_Data/Scanning_img_after_processed/' + str(pos_no) + '.jpg'
            cv.imwrite(img_name, frame)
            if (pos_no >= np.shape(positions)[0]):
                break

        WeldPoints = np.array(WeldPoints)
        WeldPoints[:, 1] += 5
        ScanningProcessing.savematrix(welding_trajectory, WeldPoints)
        # self.trajectory_signal.emit("hihi")
        # illustrate the solution
        self.X = WeldPoints[:, 0]
        self.Y = WeldPoints[:, 1]
        self.Z = WeldPoints[:, 2]
        # traj = ScanningProcessing.generate_trajectory()
        # self.X, self.Y, self.Z = self.scanning_process.main_scanning()
        class show_chart(FigureCanvasQTAgg):
            def __init__(self,X,Y,Z):
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection="3d")
                super().__init__(self.fig)
                # traj = ScanningProcessing.generate_trajectory()
                # self.X, self.Y, self.Z = traj.main_scanning()
                # self.ax.scatter(self.X, self.Y, self.Z, c='r', marker='.')
                self.ax.scatter(X, Y,Z, c='r', maker='-')
                self.ax.set_xlabel('x')
                self.ax.set_zlabel('z')
                # del X, Y, Z
                # plt.show()
        self.uic.welding_trajectory.addWidget(show_chart(self.X,self.Y,self.Z))


        ## Function for page 7 ##

    def run_start_weld(self):
        robot = Main_Weld_Aft_Scan.RobotTask()
        # cam = Main_Weld_Aft_Scan.CameraTask()
        # sv = robot.robot.Servo("1")
        # print(sv)
        # print("--- Homing process ---")
        # # self.uic.welding_process.addItem("--- Homing process ---")
        # # robot.homing()
        # time.sleep(5)
        # print("--- Homing process done ---")
        # self.uic.welding_process.addItem("--- Homing process done ---")
        print("--- Start welding ---")
        self.uic.infor_process_in.addItem("--- Start welding ---")
        # Start new Threads
        robot.start()
    def delete_all_images(self, folder_path):
        # Kiểm tra xem folder_path có tồn tại hay không
        # self.run_start_weld()
        if not os.path.isdir(folder_path):
            print("Thư mục không tồn tại.")
            return

        # Lặp qua tất cả các tệp trong thư mục
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)

            # Kiểm tra xem file_path có phải là tệp ảnh hay không (vd: .jpg, .png, ...)
            if os.path.isfile(file_path) and any(
                    file_path.lower().endswith(image_ext) for image_ext in ['.jpg', '.jpeg', '.png', '.gif']):
                try:
                    # Xóa tệp ảnh
                    os.remove(file_path)
                    print(f"Đã xóa: {file_path}")
                except OSError as e:
                    print(f"Lỗi khi xóa {file_path}: {e}")
    def remove_graph(self):
        # Lấy widget biểu đồ hiện tại từ QVBoxLayout
        widget_chart = self.uic.welding_trajectory.itemAt(0).widget()  # Thay đổi số 0 thành vị trí của biểu đồ trong QVBoxLayout nếu cần thiết

        # Kiểm tra xem widget_chart có tồn tại hay không
        if widget_chart is not None:
            # Xóa widget biểu đồ khỏi QVBoxLayout
            self.uic.welding_trajectory.removeWidget(widget_chart)
            # Xóa widget biểu đồ
            widget_chart.deleteLater()
    def scanning(self):
        # self.run_main_scanning()
        self.remove_graph()
        self.uic.show_image_calib_in.clear()
        self.scan = scanning()
        # time.sleep(2)
        self.scan.start()
        self.scan.image.connect(self.display_image)
        self.scan.trajectory.connect(self.display_trajectory)
        self.scan.finished.connect(self.show_trajectory)
    def display_image(self, image):
        show_img = image
        self.uic.show_image_calib_in.setPixmap(QtGui.QPixmap.fromImage(show_img))
    def display_trajectory(self, trajectory):
        weldpoint2robot = trajectory
        self.uic.infor_process_in.addItem((str(weldpoint2robot)))
        self.uic.infor_process_in.scrollToBottom()
    ## main_scanning

    def test(self):
        img_list = []

    def stop(self):
        self.scanning_running = False  # Đặt biến cờ thành False để kết thúc quá trình calibrate
        self.flag_scan = syntec_Lib.robot_syntec().run_scan_flag()
        self.camera = Main_Scanning.CameraTask()
        self.robot = Main_Scanning.RobotTask()

    # def run_main_scanning(self):
    #     # class main_scan(QObject):
    #     #     data_signal = pyqtSignal(str)
    #     #     def __init__(self):
    #     #         QObject.__init__(self)
    #     # def run_scan(self):
    #     flag =0
    #     if self.scanning_running:
    #         return  # Nếu quá trình calibrate đã đang chạy, không làm gì
    #
    #     self.scanning_running = True  # Đặt biến cờ thành True để bắt đầu quá trình calibrate
    #     self.camera = Main_Scanning.CameraTask()
    #     self.robot = Main_Scanning.RobotTask()
    #     self.flag_scan = syntec_Lib.robot_syntec().run_scan_flag()            #
    #     while self.scanning_running:
    #         flag2 = 0
    #         while True:
    #             self.flag_scan = syntec_Lib.robot_syntec().run_scan_flag()
    #             self.flag_scan = self.flag_scan[0]
    #             # print(self.flag_scan)
    #             # if (self.flag_scan == 0):
    #             #     self.camera.stop()
    #             #     self.robot.stop()
    #             #     self.flag_scan = 0
    #             if (self.camera.flag_cam == 1 & self.robot.flag_robot == 1 & self.flag_scan == 1 ):
    #                 print("--- Start Scanning ---")
    #                 self.uic.infor_process_in.addItem("--- Start Scanning ---")
    #                 # Start new Threads
    #                 self.camera.start()
    #                 self.robot.start()
    #                 self.uic.infor_process_in.addItem("--- Scanning done ---")
    #                 self.flag_scan = 0
    #                 break
    #             # else:
    #             #     print("Don't have connection with robot and camera")
    #             #     self.uic.infor_process_in.addItem("Done have connection with robot, camera or both")
    #         while True:
    #             flag2 = syntec_Lib.robot_syntec().stop_scanning()
    #             if flag2 == 1:
    #                 self.scanning()
    #                 if (self.flag == 1):
    #                     self.flag = 0
    #                     break
    #         if not self.scanning_running:
    #             break  # Thoát khỏi vòng lặp nếu biến cờ đã được đặt lại thành False

    def run_main_scanning(self):
        # class main_scan(QObject):
        #     data_signal = pyqtSignal(str)
        #     def __init__(self):
        #         QObject.__init__(self)
        # def run_scan(self):
        # if self.scanning_running:
        #     return  # Nếu quá trình calibrate đã đang chạy, không làm gì
        #
        # self.scanning_running = True  # Đặt biến cờ thành True để bắt đầu quá trình calibrate

        # while self.scanning_running:
        flag2 = 0
        # ABB_Lib.ABB().StartRequest()
        # while True:
        self.uic.infor_process_in.addItem("--- Start Scanning ---")
        self.camera = Main_Scanning.CameraTask()
        self.robot = Main_Scanning.RobotTask()
        self.flag_scan = self.robot.flag_robot
        while True:
            # self.flag_scan = syntec_Lib.robot_syntec().run_scan_flag()
            # self.flag_scan = self.flag_scan[0]
            # print(self.flag_scan)
            # if (self.flag_scan == 0):
            #     self.camera.stop()
            #     self.robot.stop()
            #     self.flag_scan = 0
            if (self.camera.flag_cam == 1 & self.robot.flag_robot == 1):
                print("--- Start Scanning ---")
                # Start new Threads
                self.camera.start()
                self.robot.start()
                # self.camera.join()
                self.robot.join()
                self.flag_scan = 0
                    # break/
            # else:
            #     print("Don't have connection with robot and camera")
            #     self.uic.infor_process_in.addItem("Done have connection with robot, camera or both")
            # while True:
            # #     flag2 = syntec_Lib.robot_syntec().stop_scanning()
                if (self.robot.flag_Stop == True):
                    print("done")
                    self.uic.infor_process_in.addItem("--- Scanning done ---")
                    self.scanning()
                    # self.camera.stop()
                    # self.robot.stop()
                    break
            # time.sleep(20)
    def test(self):
        while True:
            self.run_main_scanning()
            time.sleep(30)
## show main_win
    def show(self):
        # command to run
        self.main_win.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())