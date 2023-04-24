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
from Calibrate_function import Calib_camera
from Calibrate_function import Calib_handeye
from Calibrate_function import Calib_laser
from Calibrate_function import Calib_external_camera
from Calibrate_function import Calib_eye_to_hand
from Calibrate_function import Calib_laser_eye_to_hand
import numpy as np
import cv2 as cv
import time, sys
class MainWindow:
    def __init__(self):
        self.main_win = QMainWindow()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self.main_win)
        # self.uic.Button_Start.clicked.connect(self.show_screen)
        # self.uic.Button_Sop.clicked.connect(self.main_win.close)

        # Define button screen 1
        self.uic.Continue.clicked.connect(self.show_screen2)
        self.uic.Close.clicked.connect(self.main_win.close)
        # Define button screen 2
        self.uic.Continue_2.clicked.connect(self.show_screen3)
        self.uic.Back_2.clicked.connect(self.show_screen1)
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
    def show_screen3(self,Screen_ind):
        self.uic.tabWidget.setCurrentWidget(self.uic.Calib_main)
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
                # plt.show()
        self.uic.laser_ex_graph.addWidget(show_chart(self.point,self.plane))
    def calib_laser_internal_cam(self):
        self.point,self.plane = Calib_laser.laserCalibrate()
        self.uic.infor_laser_in.addItem("Laser Plane: ")
        for row in self.point:
            row_str = " ".join(str(val) for val in row)
            # print(row_str)
        self.uic.infor_laser_in.addItem("Z = " + str(self.point[0][0]).strip('[]') + "X + " + str(self.point[1][0]).strip('[]') + "Y + "+str(self.point[2][0]).strip('[]'))
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
                # plt.show()
        self.uic.laser_in_graph.addWidget(show_chart(self.point,self.plane))

    # function for page 5  ##
    def run_finding_trajectory(self):
        self.find_tra.data_signal.connect(self.show_trajectory)
        if self.find_tra.main_func():
            self.show_result_eye_to_hand()
    def show_result_eye_to_hand(self):
        result_path = r"E:/New_Code/App_Data/Test_eye_to_hand/result.jpg"
        self.img = cv.imread(result_path)
        self.img = cv.resize(self.img, (451, 331), interpolation=cv.INTER_AREA)
        height, width, channel = self.img.shape
        bytesPerLine = 3*width
        self.img = QtGui.QImage(self.img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888).rgbSwapped()
        self.uic.show_image_ex.setPixmap(QtGui.QPixmap.fromImage(self.img))
    def show_trajectory(self, message):
        self.uic.output_scan_trajectory.addItem(message)

    ## function for page 6  ##
    ## main_scanning
    def run_main_scanning(self):
        # class main_scan(QObject):
        #     data_signal = pyqtSignal(str)
        #     def __init__(self):
        #         QObject.__init__(self)
        # def run_scan(self):
        camera = Main_Scanning.CameraTask()
        robot = Main_Scanning.RobotTask()
        if (Main_Scanning.flag_cam == 1 & Main_Scanning.flag_robot == 1):

            # software = SoftwareTask()

            print("--- Homing process ---")
            self.uic.Scanlist.addItem("--- Homing process ---")
            robot.homing()
            time.sleep(3)
            print("--- Homing process done ---")
            self.uic.Scanlist.addItem("--- Homing process done ---")
            print("--- Start Scanning ---")
            self.uic.Scanlist.addItem("--- Start Scanning ---")
            # Start new Threads
            camera.start()
            robot.start()
            camera.stop()
            camera.join()
            robot.stop()
            robot.join()
        else:
            print("Done have connection with robot and camera")
            self.uic.Scanlist.addItem("Done have connection with robot, camera or both")
    ##scanning processing
    def show_diagram(self):
        self.scanning_process.trajectory_signal.connect(self.show_result_scan)
        # traj = ScanningProcessing.generate_trajectory()
        self.X, self.Y, self.Z = self.scanning_process.main_scanning()
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