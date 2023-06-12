import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtGui
from QtGui import Ui_MainWindow
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
# Globel variable #
# Eye in hand

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

class MainWindow:
    def __init__(self):
        self.main_win = QMainWindow()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self.main_win)
        # self.uic.Button_Start.clicked.connect(self.show_screen)
        # self.uic.Button_Sop.clicked.connect(self.main_win.close)
#### global variable
                                    # eye to hand
        self.pos_no_ex = 0
        self.R_end2base_ex = []
        self.T_end2base_ex = []
        self.check_count_ex = 0
        self.check_laser_count_ex = 0
        self.laser_count_ex = 0
        self.caption_picture_ex = 0
        # self.H_end2base_model_ex = []
        # self.image_ex = np.zeros((2560, 2560, 3), np.uint8)
        choice = 1
        self.VisionSystem_ex = Eye_to_hand_calib_process.BaslerCam(choice)
        self.Robot_ex = Eye_to_hand_calib_process.RobotTask(choice)
        self.flag_calib_ex = 0
                                    # eye in hand
        self.pos_no_in = 0
        self.R_end2base_in = []
        self.T_end2base_in = []
        # self.H_end2base_model_in = []
        self.check_count_in = 0
        self.check_laser_count_in = 0
        self.laser_count_in = 0
        self.caption_picture_in = 0
        # self.image_in = np.zeros((1700, 1200, 3), np.uint8)
        self.VisionSystem_in = Calib_process.BaslerCam(choice)
        self.Robot_in = Calib_process.RobotTask(choice)
        # self.filename = ""
        self.flag_calib_in = 0
                        # main scanning
        self.flag_scan = 0
        # Define button screen 1
        self.uic.Continue.clicked.connect(self.show_screen2)
        self.uic.Close.clicked.connect(self.main_win.close)

            # function button
        self.uic.Star_calib_process_ex.clicked.connect(self.start_calib_ex)
        self.uic.Move_ex.clicked.connect(self.move_robot_ex)
        self.uic.Get_pos_ex.clicked.connect(self.get_pos_ex)
        self.uic.Save_pos_ex.clicked.connect(self.save_pos_ex)
        self.uic.Checkerboard_ex.clicked.connect(self.capture_image_ex)
        self.uic.Checker_laser_ex.clicked.connect(self.capture_checker_ex)
        self.uic.Cap_laser_ex.clicked.connect(self.capture_laser_ex)
        # Define button screen 2
        self.uic.Continue_2.clicked.connect(self.show_screen3)
        self.uic.Back_2.clicked.connect(self.show_screen1)

        # function button
        self.uic.Star_calib_process_in.clicked.connect(self.start_calib_in)
        self.uic.Move_in.clicked.connect(self.move_robot_in)
        self.uic.Get_pos_in.clicked.connect(self.get_pos_in)
        self.uic.Save_pos_in.clicked.connect(self.save_pos_in)
        self.uic.Checkerboard_in.clicked.connect(self.capture_image_in)
        self.uic.Checker_laser_in.clicked.connect(self.capture_checker_in)
        self.uic.Cap_laser_in.clicked.connect(self.capture_laser_in)

        # Define button screen 3
        self.uic.Continue_3.clicked.connect(self.show_screen4)
        self.uic.Back_3.clicked.connect(self.show_screen2)
        self.uic.Calib_main_ex.clicked.connect(self.calib_eye_to_hand)
        self.uic.Calib_main_in.clicked.connect(self.calib_eye_in_hand)
        # Define button screen 4
        self.uic.Continue_4.clicked.connect(self.show_screen5)
        self.uic.Back_4.clicked.connect(self.show_screen3)
        self.uic.Calib_laser_ex.clicked.connect(self.calib_laser_external_cam)
        self.uic.Calib_laser_in.clicked.connect(self.calib_laser_internal_cam)
        # Define button screen 5
        self.uic.Continue_7.clicked.connect(self.show_screen6)
        self.uic.Back_7.clicked.connect(self.show_screen4)
        self.uic.run_find_scan_trajectory.clicked.connect(self.run_finding_trajectory)
        # Define button screen 6
        self.uic.Continue_8.clicked.connect(self.show_screen7)
        self.uic.Back_8.clicked.connect(self.show_screen5)
        self.uic.Scanning_processing.clicked.connect(self.show_diagram)
        self.uic.Run_scan.clicked.connect(self.run_main_scanning)
        # Define button screen 7
        self.uic.Done.clicked.connect(self.main_win.close)
        self.uic.Back_9.clicked.connect(self.show_screen6)
        self.uic.start_weld.clicked.connect(self.run_start_weld)
        #Find trajevtory
        self.find_tra = Finding_trajectory.find_trajectory()
        #main scanning
        # self.run_scan = Main_Scanning.main_scan()
        #scanning processing
        self.scanning_process = ScanningProcessing.generate_trajectory()
    # show screen function
    def show_screen1(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.eye_to_hand)
    def show_screen2(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.Eye_in_hand)
        self.quit_ex()
    def show_screen3(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.Calib_main)
        self.quit_in()
    def show_screen4(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.Calib_laser)
    def show_screen5(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.Get_scan_trectory)
    def show_screen6(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.main_scan)
    def show_screen7(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.welding)
        WeldingTraject_filter.trajectory_filter()
        with open(welding_trajectory_filtered, 'r') as f:
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

    # function for page 1 eye to hand#
    def start_calib_ex(self):
        self.pos_no_ex = 1
        self.flag_calib_ex = 1
        if (self.VisionSystem_ex.flag_cam_ex == 1 & self.Robot_ex.flag_robot_ex == 1):
            self.VisionSystem_ex.start()
            self.Robot_ex.start()
            self.uic.infor_process_ex.addItem("There are 16 waypoints in robot trajectory")
        if self.VisionSystem_ex.flag_cam_ex == 0:
            self.uic.infor_process_ex.addItem("Connection error with acA3800-10gm")
        if self.Robot_ex.flag_robot_ex == 0:
            self.uic.infor_process_ex.addItem("Robot connection error")
        self.uic.infor_process_ex.scrollToBottom()
    def move_robot_ex(self):
        if self.flag_calib_ex == 1:
            trajectory = open(robot_path_ex, "r")
            waypoint = len(trajectory.readlines())
            print("move")
            if self.pos_no_ex <= waypoint:
                a = self.Robot_ex.read_pos_from_txt(self.pos_no_ex)
                print(f"ROBOT TO WAYPOINT {self.pos_no_ex}...")
                self.uic.infor_process_ex.addItem(f"ROBOT TO WAYPOINT {self.pos_no_ex}...")
                self.Robot_ex.robot_move_to_pos(a)
                print(a)
                time.sleep(2)
                self.pos_no_ex += 1
                if self.pos_no_ex == waypoint + 1:
                    print("----------ROBOT TRAJECTORY IS FINISHED----------\nInput 'r' to return...")
                    self.uic.infor_process_ex.addItem("----------ROBOT TRAJECTORY IS FINISHED----------")
            else:
                self.pos_no_ex = 1
                a = self.Robot_ex.read_pos_from_txt(self.pos_no_ex)
                print("\nReturning to first waypoint...")
                self.uic.infor_process_ex.addItem("Returning to first waypoint...")
                print(f"ROBOT TO WAYPOINT {self.pos_no_ex}...")
                self.uic.infor_process_ex.addItem(f"ROBOT TO WAYPOINT {self.pos_no_ex}...")
                self.Robot_ex.robot_move_to_pos(a)
                print(a)
                time.sleep(2)
                self.pos_no_ex += 1
        else:
            self.uic.infor_process_ex.addItem("Click Start please")
        self.uic.infor_process_ex.scrollToBottom()
    def get_pos_ex(self):
        if self.flag_calib_ex == 1:
            self.Robot_ex.read_pos_from_robot()
            R, t = self.Robot_ex.read()
            print(f'\nTRANSFORMATION AT WAYPOINT {self.pos_no_ex - 1}:')
            self.uic.infor_process_ex.addItem(f'TRANSFORMATION AT WAYPOINT {self.pos_no_ex - 1}:')
            print('Rotation R:\n', R)
            self.uic.infor_process_ex.addItem('Rotation R:')
            self.uic.infor_process_ex.addItem(str(R))
            print('Translation T:\n', t)
            self.uic.infor_process_ex.addItem('Translation T:')
            self.uic.infor_process_ex.addItem(str(t))
            self.R_end2base_ex.append(R)
            self.T_end2base_ex.append(t)
        else:
            self.uic.infor_process_ex.addItem("Click Start please")
        self.uic.infor_process_ex.scrollToBottom()
        time.sleep(0.5)
    def save_pos_ex(self):
        if self.flag_calib_ex == 1:
            RR = np.array(self.R_end2base_ex)
            TT = np.array(self.T_end2base_ex)
            self.Robot_ex.save(R_path_ex, RR)
            self.Robot_ex.save(t_path_ex, TT)
            print('ALL WAYPOINT TRANSFORMATIONS SAVED!')
            self.uic.infor_process_ex.addItem('ALL WAYPOINT TRANSFORMATIONS SAVED!')
        else:
            self.uic.infor_process_ex.addItem("Click Start please")
        self.uic.infor_process_ex.scrollToBottom()
        time.sleep(3)
    def capture_image_ex(self):
        if self.flag_calib_ex == 1:
            self.show_calib_img_ex()
            self.check_count_ex += 1
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if self.check_count_ex < 10:
                self.filename = check_path_ex + "\checkerboard_0" + str(self.check_count_ex) + '.jpg'
                # print("filename",self.filename)
            elif 10 <= self.check_count_ex <= 16:
                self.filename = check_path_ex + "\checkerboard_" + str(self.check_count_ex) + '.jpg'
            else:
                self.check_count_ex =0
                print("out of range")
                self.uic.infor_process_ex.addItem("out of range")
            cv.imwrite(self.filename, self.VisionSystem_ex.image)
            print('Captured checkerboard image\nPair %d ' % (self.check_count_ex))
            self.uic.infor_process_ex.addItem('Captured checkerboard image')
            self.uic.infor_process_ex.addItem("Pair %d" % (self.check_count_ex))
        else:
            self.uic.infor_process_ex.addItem("Click Start please")
        self.uic.infor_process_ex.scrollToBottom()
    def capture_checker_ex(self):
        if self.flag_calib_ex == 1:
            self.show_calib_img_ex()
            self.check_laser_count_ex += 1
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if self.check_laser_count_ex < 10:
                self.filename = laser_path_ex + "\checker_0" + str(self.check_laser_count_ex) + '.jpg'
            elif 10 <= self.check_laser_count_ex <= 16:
                self.filename = laser_path_ex + "\checker_" + str(self.check_laser_count_ex) + '.jpg'
            else:
                self.check_laser_count_ex = 0
                print("out of range")
                self.uic.infor_process_ex.addItem("out of range")
            cv.imwrite(self.filename, self.VisionSystem_ex.image)
            print('Captured checker laser image\nPair %d ' % (self.check_laser_count_ex))
            self.uic.infor_process_ex.addItem('Captured checker laser image')
            self.uic.infor_process_ex.addItem('Pair %d ' % (self.check_laser_count_ex))
        else:
            self.uic.infor_process_ex.addItem("Click Start please")
        self.uic.infor_process_ex.scrollToBottom()
    def capture_laser_ex(self):
        if self.flag_calib_ex == 1:
            self.show_calib_img_ex()
            self.laser_count_ex += 1
            # To start number with 0, e.g, 01 02 03 ... 09 10 11 12 ... 99
            if self.laser_count_ex < 10:
                self.filename = laser_path_ex + "\laser_0" + str(self.laser_count_ex) + '.jpg'
            elif 10 <= self.laser_count_ex <=16:
                self.filename = laser_path_ex + "\laser_" + str(self.laser_count_ex) + '.jpg'
            else:
                self.laser_count_ex =0
                print("out of range")
                self.uic.infor_process_ex.addItem("out of range")
            cv.imwrite(self.filename, self.VisionSystem_ex.image)
            print('Captured laser line image\nPair %d ' % (self.laser_count_ex))
            self.uic.infor_process_ex.addItem('Captured laser line image')
            self.uic.infor_process_ex.addItem('Pair %d ' % (self.laser_count_ex))
        else:
            self.uic.infor_process_ex.addItem("Click Start please")
        self.uic.infor_process_ex.scrollToBottom()
    def quit_ex(self):
        self.VisionSystem_ex.stop()
        self.Robot_ex.stop()
        print("End Process")
        self.uic.infor_process_ex.addItem("End Process")
        self.uic.infor_process_ex.scrollToBottom()
    def show_calib_img_ex(self):
        # result_path = r"E:/New_Code/App_Data/Test_eye_to_hand/result.jpg"
        # self.img = cv.imread(result_path)
        self.image_ex = self.VisionSystem_ex.image
        self.img = cv.resize(self.image_ex, (451, 331), interpolation=cv.INTER_AREA)
        height, width = self.img.shape
        bytesPerLine = width
        # cv.imshow("hihi",self.image_ex)
        self.img = QtGui.QImage(self.img.data, width, height, bytesPerLine, QtGui.QImage.Format_Grayscale8)
        self.uic.show_image_calib_ex.setPixmap(QtGui.QPixmap.fromImage(self.img))

    # function for page 2 eye in hand#
    def start_calib_in(self):
        self.pos_no_in = 1
        self.flag_calib_in = 1
        if (self.VisionSystem_in.flag_cam_in == 1 & self.Robot_in.flag_robot == 1):
            self.VisionSystem_in.start()
            self.Robot_in.start()
            self.uic.infor_process_in.addItem("There are 16 waypoints in robot trajectory")
        if self.VisionSystem_in.flag_cam_in == 0:
            self.uic.infor_process_in.addItem("Connection error with acA1920-40gm")
        if self.Robot_in.flag_robot == 0:
            self.uic.infor_process_in.addItem("Robot connection error")
        self.uic.infor_process_in.scrollToBottom()
    def move_robot_in(self):
        if self.flag_calib_in == 1:
            trajectory = open(robot_path_in, "r")
            waypoint = len(trajectory.readlines())
            print("move")
            if self.pos_no_in <= waypoint:
                a = self.Robot_in.read_pos_from_txt(self.pos_no_in)
                print(f"ROBOT TO WAYPOINT {self.pos_no_in}...")
                self.uic.infor_process_in.addItem(f"ROBOT TO WAYPOINT {self.pos_no_in}...")
                self.Robot_in.robot_move_to_pos(a)
                print(a)
                time.sleep(2)
                self.pos_no_in += 1
                if self.pos_no_in == waypoint + 1:
                    print("----------ROBOT TRAJECTORY IS FINISHED----------")
                    self.uic.infor_process_in.addItem("----------ROBOT TRAJECTORY IS FINISHED----------")
            else:
                self.pos_no_in = 1
                a = self.Robot_in.read_pos_from_txt(self.pos_no_in)
                print("\nReturning to first waypoint...")
                self.uic.infor_process_in.addItem("Returning to first waypoint...")
                print(f"ROBOT TO WAYPOINT {self.pos_no_in}...")
                self.uic.infor_process_in.addItem("fROBOT TO WAYPOINT {self.pos_no_in}...")
                self.Robot_in.robot_move_to_pos(a)
                print(a)
                time.sleep(2)
                self.pos_no_in += 1
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
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
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
        time.sleep(0.5)
    def save_pos_in(self):
        if self.flag_calib_in == 1:
            RR = np.array(self.R_end2base_in)
            TT = np.array(self.T_end2base_in)
            self.Robot_in.save(R_path_in, RR)
            self.Robot_in.save(t_path_in, TT)
            print('ALL WAYPOINT TRANSFORMATIONS SAVED!')
            self.uic.infor_process_in.addItem('ALL WAYPOINT TRANSFORMATIONS SAVED!')
        else:
            self.uic.infor_process_in.addItem("Click Start please")
        self.uic.infor_process_in.scrollToBottom()
        time.sleep(3)
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

    # function for page 3  ##
    def calib_eye_to_hand(self):
        Calib_external_camera.ex_monoCalibrate()
        tranform_ex = Calib_eye_to_hand.eyetohandCalibrate()
        tranform_ex_string = str(tranform_ex)
        self.uic.infor_eye_to_hand.addItem(tranform_ex_string)
        self.rot_ex, self.trans_ex = Calib_eye_to_hand.split_homo(tranform_ex)
        class show_chart(FigureCanvasQTAgg):
            def __init__(self,rot,trans):
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection="3d")
                super().__init__(self.fig)
                # traj = ScanningProcessing.generate_trajectory()
                # self.X, self.Y, self.Z = traj.main_scanning()
                # self.ax.scatter(self.X, self.Y, self.Z, c='r', marker='.')
                self.ax.set_xlabel('X - axis')
                self.ax.set_ylabel('Y - axis')
                self.ax.set_zlabel('Z - axis')
                Calib_eye_to_hand.coordinate(self.ax, [[0], [0], [0]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], " Base Coordinate")
                Calib_eye_to_hand.coordinate(self.ax, trans, rot, " Camera Coordinate")
                # del X, Y, Z
                # plt.show()
        self.uic.eye2hand_graph.addWidget(show_chart(self.rot_ex,self.trans_ex))
    def calib_eye_in_hand(self):
        Calib_camera.monoCalibrate()
        tranform_in = Calib_handeye.handeyeCalibrate()
        tranform_in_string = str(tranform_in)
        self.uic.infor_eye_in_hand.addItem(tranform_in_string)
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
    # function for page 4  ##
    def calib_laser_external_cam(self):
        self.point, self.plane = Calib_laser_eye_to_hand.ex_laserCalibrate()
        self.uic.infor_laser_ex.addItem("Laser Plane: ")
        for row in self.point:
            row_str = " ".join(str(val) for val in row)
            # print(row_str)
        self.uic.infor_laser_ex.addItem("Z = " + str(self.point[0][0]).strip('[]') + "X + " + str(self.point[1][0]).strip('[]') + "Y + "+str(self.point[2][0]).strip('[]'))
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
                self.ax.set_zlim3d(400, 600)
                # plt.show()
        self.uic.laser_ex_graph.addWidget(show_chart(self.point,self.plane))
    def calib_laser_internal_cam(self):
        self.point,self.plane = Calib_laser.laserCalibrate()
        self.uic.infor_laser_in.addItem("Laser Plane: ")
        for row in self.point:
            row_str = " ".join(str(val) for val in row)
            # print(row_str)
        self.uic.infor_laser_in.addItem("Z = " + str(self.plane[0][0]).strip('[]') + "X + " + str(self.plane[1][0]).strip('[]') + "Y + "+str(self.plane[2][0]).strip('[]'))
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

    # function for page 5  ##
    def run_finding_trajectory(self):
        # self.find_tra.data_signal.connect(self.show_trajectory)
        # if self.find_tra.main_func():
        #     self.show_result_eye_to_hand()
        self.show_result_eye_to_hand()
        self.show_trajectory()
    def show_result_eye_to_hand(self):
        result_path = r"E:/New_Code/App_Data/Test_eye_to_hand/result.jpg"
        time.sleep(10)
        self.img = cv.imread(result_path)
        self.img = cv.resize(self.img, (451, 331), interpolation=cv.INTER_AREA)
        height, width, channel = self.img.shape
        bytesPerLine = 3*width
        self.img = QtGui.QImage(self.img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888).rgbSwapped()
        self.uic.show_image_ex.setPixmap(QtGui.QPixmap.fromImage(self.img))
    # def show_trajectory(self, message):
    def show_trajectory(self):
        # self.uic.output_scan_trajectory.addItem(message)
        with open(scan_trajectory_path, 'r') as f:
            positions = [[float(num) for num in line.split('\t')] for line in f]
        Scan_point1 = positions[0]
        Scan_point2 = positions[1]
        self.uic.output_scan_trajectory.addItem("Start point")
        self.uic.output_scan_trajectory.addItem(str(Scan_point1))
        self.uic.output_scan_trajectory.addItem("Stop point")
        self.uic.output_scan_trajectory.addItem(str(Scan_point2))
    ## function for page 6  ##
    ## main_scanning
    def run_main_scanning(self):
        # class main_scan(QObject):
        #     data_signal = pyqtSignal(str)
        #     def __init__(self):
        #         QObject.__init__(self)
        # def run_scan(self):
        self.camera = Main_Scanning.CameraTask()
        self.robot = Main_Scanning.RobotTask()
        if (self.flag_scan == 1):
            self.camera.stop()
            self.robot.stop()
            self.flag_scan = 0
        if (self.camera.flag_cam == 1 & self.robot.flag_robot == 1):

            # software = SoftwareTask()

            print("--- Homing process ---")
            self.uic.Scanlist.addItem("--- Homing process ---")
            self.robot.homing()
            time.sleep(3)
            print("--- Homing process done ---")
            self.uic.Scanlist.addItem("--- Homing process done ---")
            print("--- Start Scanning ---")
            self.uic.Scanlist.addItem("--- Start Scanning ---")
            # Start new Threads
            self.camera.start()
            self.robot.start()
            # camera.stop()
            # camera.join()
            # robot.stop()
            # robot.join()
            self.uic.Scanlist.addItem("--- Scanning done ---")
            self.flag_scan = 1
        else:
            print("Don't have connection with robot and camera")
            self.uic.Scanlist.addItem("Done have connection with robot, camera or both")
    ##scanning processing
    def show_diagram(self):
        # self.scanning_process.trajectory_signal.connect(self.show_result_scan)
        WeldPoints = []
        pos_no = 0

        Images = glob.glob(r'E:/Thesis/App_Data/Scan_data4/weldseam' + '*.jpg')

        Images.sort(key=ScanningProcessing.natural_keys)
        # laser center line
        # laser_center_1 = [785, 720]
        # laser_center_2 = [785, 480]

        print(Images)
        with open(scan_pos_path, 'r') as f:
            positions = [[float(num) for num in line.split('\t')] for line in f]
        for image in Images:
            img = cv.imread(image, cv.IMREAD_GRAYSCALE)
            # laser line
            laser_center_1, laser_center_2 = ScanningProcessing.get_laser_line(img)
            laser_centerline = (laser_center_1, laser_center_2)
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
            self.uic.Cal_trajectory.addItem((str(weldpoint2robot)))
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
            img_name = 'E:/Thesis/App_Data/Scanning_img_after_processed/' + str(pos_no) + '.jpg'
            cv.imwrite(img_name, frame)
            if (pos_no >= np.shape(positions)[0]):
                break

        WeldPoints = np.array(WeldPoints)
        WeldPoints[:, 2] = -634
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
                self.ax.scatter(X, Y,Z, c='r', marker='.')
                self.ax.set_xlabel('x')
                self.ax.set_zlabel('z')
                # del X, Y, Z
                # plt.show()
        self.uic.trajectory_graph.addWidget(show_chart(self.X,self.Y,self.Z))
    def show_result_scan(self, message_tra):
        self.uic.Cal_trajectory.addItem(message_tra)
        print(message_tra)

    ## Function for page 7 ##
    def run_start_weld(self):
        robot = Main_Weld_Aft_Scan.RobotTask()
        # cam = Main_Weld_Aft_Scan.CameraTask()
        sv = robot.robot.Servo("1")
        print(sv)
        print("--- Homing process ---")
        self.uic.welding_process.addItem("--- Homing process ---")
        robot.homing()
        time.sleep(5)
        print("--- Homing process done ---")
        self.uic.welding_process.addItem("--- Homing process done ---")
        print("--- Start welding ---")
        self.uic.welding_process.addItem("--- Start welding ---")
        # Start new Threads
        robot.start()
## show main_win
    def show(self):
        # command to run
        self.main_win.show()



if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())