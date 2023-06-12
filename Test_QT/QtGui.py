# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'TestQT.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1082, 895)
        MainWindow.setMinimumSize(QtCore.QSize(0, 0))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(40, 20, 1001, 801))
        self.tabWidget.setMinimumSize(QtCore.QSize(1000, 800))
        self.tabWidget.setObjectName("tabWidget")
        self.eye_to_hand = QtWidgets.QWidget()
        self.eye_to_hand.setObjectName("eye_to_hand")
        self.Move_ex = QtWidgets.QPushButton(self.eye_to_hand)
        self.Move_ex.setGeometry(QtCore.QRect(60, 120, 121, 51))
        self.Move_ex.setObjectName("Move_ex")
        self.Checkerboard_ex = QtWidgets.QPushButton(self.eye_to_hand)
        self.Checkerboard_ex.setGeometry(QtCore.QRect(60, 230, 121, 51))
        self.Checkerboard_ex.setObjectName("Checkerboard_ex")
        self.Get_pos_ex = QtWidgets.QPushButton(self.eye_to_hand)
        self.Get_pos_ex.setGeometry(QtCore.QRect(220, 120, 121, 51))
        self.Get_pos_ex.setObjectName("Get_pos_ex")
        self.Checker_laser_ex = QtWidgets.QPushButton(self.eye_to_hand)
        self.Checker_laser_ex.setGeometry(QtCore.QRect(220, 230, 121, 51))
        self.Checker_laser_ex.setObjectName("Checker_laser_ex")
        self.Save_pos_ex = QtWidgets.QPushButton(self.eye_to_hand)
        self.Save_pos_ex.setGeometry(QtCore.QRect(380, 120, 121, 51))
        self.Save_pos_ex.setObjectName("Save_pos_ex")
        self.Cap_laser_ex = QtWidgets.QPushButton(self.eye_to_hand)
        self.Cap_laser_ex.setGeometry(QtCore.QRect(380, 230, 121, 51))
        self.Cap_laser_ex.setObjectName("Cap_laser_ex")
        self.Continue = QtWidgets.QPushButton(self.eye_to_hand)
        self.Continue.setGeometry(QtCore.QRect(820, 710, 121, 41))
        self.Continue.setObjectName("Continue")
        self.Close = QtWidgets.QPushButton(self.eye_to_hand)
        self.Close.setGeometry(QtCore.QRect(670, 710, 121, 41))
        self.Close.setObjectName("Close")
        self.label = QtWidgets.QLabel(self.eye_to_hand)
        self.label.setGeometry(QtCore.QRect(60, 20, 481, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.eye_to_hand)
        self.label_2.setGeometry(QtCore.QRect(60, 80, 321, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.eye_to_hand)
        self.label_3.setGeometry(QtCore.QRect(60, 190, 321, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.infor_process_ex = QtWidgets.QListWidget(self.eye_to_hand)
        self.infor_process_ex.setGeometry(QtCore.QRect(30, 370, 931, 311))
        self.infor_process_ex.setObjectName("infor_process_ex")
        self.Star_calib_process_ex = QtWidgets.QPushButton(self.eye_to_hand)
        self.Star_calib_process_ex.setGeometry(QtCore.QRect(60, 300, 121, 51))
        self.Star_calib_process_ex.setObjectName("Star_calib_process_ex")
        self.show_image_calib_ex = QtWidgets.QLabel(self.eye_to_hand)
        self.show_image_calib_ex.setGeometry(QtCore.QRect(510, 20, 451, 331))
        self.show_image_calib_ex.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.show_image_calib_ex.setText("")
        self.show_image_calib_ex.setObjectName("show_image_calib_ex")
        self.tabWidget.addTab(self.eye_to_hand, "")
        self.Eye_in_hand = QtWidgets.QWidget()
        self.Eye_in_hand.setObjectName("Eye_in_hand")
        self.Checker_laser_in = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Checker_laser_in.setGeometry(QtCore.QRect(220, 230, 121, 51))
        self.Checker_laser_in.setObjectName("Checker_laser_in")
        self.Get_pos_in = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Get_pos_in.setGeometry(QtCore.QRect(220, 120, 121, 51))
        self.Get_pos_in.setObjectName("Get_pos_in")
        self.Checkerboard_in = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Checkerboard_in.setGeometry(QtCore.QRect(60, 230, 121, 51))
        self.Checkerboard_in.setObjectName("Checkerboard_in")
        self.Save_pos_in = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Save_pos_in.setGeometry(QtCore.QRect(380, 120, 121, 51))
        self.Save_pos_in.setObjectName("Save_pos_in")
        self.Move_in = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Move_in.setGeometry(QtCore.QRect(60, 120, 121, 51))
        self.Move_in.setObjectName("Move_in")
        self.Cap_laser_in = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Cap_laser_in.setGeometry(QtCore.QRect(380, 230, 121, 51))
        self.Cap_laser_in.setObjectName("Cap_laser_in")
        self.Continue_2 = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Continue_2.setGeometry(QtCore.QRect(820, 710, 121, 41))
        self.Continue_2.setObjectName("Continue_2")
        self.Back_2 = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Back_2.setGeometry(QtCore.QRect(670, 710, 121, 41))
        self.Back_2.setObjectName("Back_2")
        self.label_6 = QtWidgets.QLabel(self.Eye_in_hand)
        self.label_6.setGeometry(QtCore.QRect(60, 190, 321, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.label_5 = QtWidgets.QLabel(self.Eye_in_hand)
        self.label_5.setGeometry(QtCore.QRect(60, 20, 481, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.label_4 = QtWidgets.QLabel(self.Eye_in_hand)
        self.label_4.setGeometry(QtCore.QRect(60, 80, 321, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.infor_process_in = QtWidgets.QListWidget(self.Eye_in_hand)
        self.infor_process_in.setGeometry(QtCore.QRect(30, 370, 931, 311))
        self.infor_process_in.setObjectName("infor_process_in")
        self.Star_calib_process_in = QtWidgets.QPushButton(self.Eye_in_hand)
        self.Star_calib_process_in.setGeometry(QtCore.QRect(60, 300, 121, 51))
        self.Star_calib_process_in.setObjectName("Star_calib_process_in")
        self.show_image_calib_in = QtWidgets.QLabel(self.Eye_in_hand)
        self.show_image_calib_in.setGeometry(QtCore.QRect(510, 20, 451, 331))
        self.show_image_calib_in.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.show_image_calib_in.setText("")
        self.show_image_calib_in.setObjectName("show_image_calib_in")
        self.tabWidget.addTab(self.Eye_in_hand, "")
        self.Calib_main = QtWidgets.QWidget()
        self.Calib_main.setObjectName("Calib_main")
        self.Calib_main_ex = QtWidgets.QPushButton(self.Calib_main)
        self.Calib_main_ex.setGeometry(QtCore.QRect(40, 20, 111, 41))
        self.Calib_main_ex.setObjectName("Calib_main_ex")
        self.Calib_main_in = QtWidgets.QPushButton(self.Calib_main)
        self.Calib_main_in.setGeometry(QtCore.QRect(40, 360, 111, 41))
        self.Calib_main_in.setObjectName("Calib_main_in")
        self.Back_3 = QtWidgets.QPushButton(self.Calib_main)
        self.Back_3.setGeometry(QtCore.QRect(670, 710, 121, 41))
        self.Back_3.setObjectName("Back_3")
        self.Continue_3 = QtWidgets.QPushButton(self.Calib_main)
        self.Continue_3.setGeometry(QtCore.QRect(820, 710, 121, 41))
        self.Continue_3.setObjectName("Continue_3")
        self.infor_eye_to_hand = QtWidgets.QListWidget(self.Calib_main)
        self.infor_eye_to_hand.setGeometry(QtCore.QRect(40, 80, 501, 261))
        self.infor_eye_to_hand.setObjectName("infor_eye_to_hand")
        self.infor_eye_in_hand = QtWidgets.QListWidget(self.Calib_main)
        self.infor_eye_in_hand.setGeometry(QtCore.QRect(40, 420, 501, 261))
        self.infor_eye_in_hand.setObjectName("infor_eye_in_hand")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.Calib_main)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(580, 80, 351, 261))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.eye2hand_graph = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.eye2hand_graph.setContentsMargins(0, 0, 0, 0)
        self.eye2hand_graph.setObjectName("eye2hand_graph")
        self.widget = QtWidgets.QWidget(self.verticalLayoutWidget_2)
        self.widget.setAutoFillBackground(True)
        self.widget.setObjectName("widget")
        self.eye2hand_graph.addWidget(self.widget)
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.Calib_main)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(580, 420, 351, 261))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.eyeinhand_graph = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.eyeinhand_graph.setContentsMargins(0, 0, 0, 0)
        self.eyeinhand_graph.setObjectName("eyeinhand_graph")
        self.widget_2 = QtWidgets.QWidget(self.verticalLayoutWidget_3)
        self.widget_2.setAutoFillBackground(True)
        self.widget_2.setObjectName("widget_2")
        self.eyeinhand_graph.addWidget(self.widget_2)
        self.tabWidget.addTab(self.Calib_main, "")
        self.Calib_laser = QtWidgets.QWidget()
        self.Calib_laser.setObjectName("Calib_laser")
        self.verticalLayoutWidget_5 = QtWidgets.QWidget(self.Calib_laser)
        self.verticalLayoutWidget_5.setGeometry(QtCore.QRect(580, 420, 351, 261))
        self.verticalLayoutWidget_5.setObjectName("verticalLayoutWidget_5")
        self.laser_in_graph = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_5)
        self.laser_in_graph.setContentsMargins(0, 0, 0, 0)
        self.laser_in_graph.setObjectName("laser_in_graph")
        self.widget_4 = QtWidgets.QWidget(self.verticalLayoutWidget_5)
        self.widget_4.setAutoFillBackground(True)
        self.widget_4.setObjectName("widget_4")
        self.laser_in_graph.addWidget(self.widget_4)
        self.infor_laser_in = QtWidgets.QListWidget(self.Calib_laser)
        self.infor_laser_in.setGeometry(QtCore.QRect(40, 420, 501, 261))
        self.infor_laser_in.setObjectName("infor_laser_in")
        self.infor_laser_ex = QtWidgets.QListWidget(self.Calib_laser)
        self.infor_laser_ex.setGeometry(QtCore.QRect(40, 80, 501, 261))
        self.infor_laser_ex.setObjectName("infor_laser_ex")
        self.Calib_laser_in = QtWidgets.QPushButton(self.Calib_laser)
        self.Calib_laser_in.setGeometry(QtCore.QRect(40, 360, 191, 41))
        self.Calib_laser_in.setObjectName("Calib_laser_in")
        self.verticalLayoutWidget_6 = QtWidgets.QWidget(self.Calib_laser)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(580, 80, 351, 261))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.laser_ex_graph = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_6)
        self.laser_ex_graph.setContentsMargins(0, 0, 0, 0)
        self.laser_ex_graph.setObjectName("laser_ex_graph")
        self.widget_3 = QtWidgets.QWidget(self.verticalLayoutWidget_6)
        self.widget_3.setAutoFillBackground(True)
        self.widget_3.setObjectName("widget_3")
        self.laser_ex_graph.addWidget(self.widget_3)
        self.Calib_laser_ex = QtWidgets.QPushButton(self.Calib_laser)
        self.Calib_laser_ex.setGeometry(QtCore.QRect(40, 20, 201, 41))
        self.Calib_laser_ex.setObjectName("Calib_laser_ex")
        self.Continue_4 = QtWidgets.QPushButton(self.Calib_laser)
        self.Continue_4.setGeometry(QtCore.QRect(820, 710, 121, 41))
        self.Continue_4.setObjectName("Continue_4")
        self.Back_4 = QtWidgets.QPushButton(self.Calib_laser)
        self.Back_4.setGeometry(QtCore.QRect(670, 710, 121, 41))
        self.Back_4.setObjectName("Back_4")
        self.tabWidget.addTab(self.Calib_laser, "")
        self.Get_scan_trectory = QtWidgets.QWidget()
        self.Get_scan_trectory.setObjectName("Get_scan_trectory")
        self.run_find_scan_trajectory = QtWidgets.QPushButton(self.Get_scan_trectory)
        self.run_find_scan_trajectory.setGeometry(QtCore.QRect(50, 90, 131, 41))
        self.run_find_scan_trajectory.setObjectName("run_find_scan_trajectory")
        self.Continue_7 = QtWidgets.QPushButton(self.Get_scan_trectory)
        self.Continue_7.setGeometry(QtCore.QRect(820, 710, 121, 41))
        self.Continue_7.setObjectName("Continue_7")
        self.Back_7 = QtWidgets.QPushButton(self.Get_scan_trectory)
        self.Back_7.setGeometry(QtCore.QRect(670, 710, 121, 41))
        self.Back_7.setObjectName("Back_7")
        self.label_7 = QtWidgets.QLabel(self.Get_scan_trectory)
        self.label_7.setGeometry(QtCore.QRect(50, 40, 481, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.Get_scan_trectory)
        self.label_8.setGeometry(QtCore.QRect(30, 420, 481, 31))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.output_scan_trajectory = QtWidgets.QListWidget(self.Get_scan_trectory)
        self.output_scan_trajectory.setGeometry(QtCore.QRect(30, 470, 931, 211))
        self.output_scan_trajectory.setObjectName("output_scan_trajectory")
        self.show_image_ex = QtWidgets.QLabel(self.Get_scan_trectory)
        self.show_image_ex.setGeometry(QtCore.QRect(440, 50, 451, 331))
        self.show_image_ex.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.show_image_ex.setText("")
        self.show_image_ex.setObjectName("show_image_ex")
        self.tabWidget.addTab(self.Get_scan_trectory, "")
        self.main_scan = QtWidgets.QWidget()
        self.main_scan.setObjectName("main_scan")
        self.Run_scan = QtWidgets.QPushButton(self.main_scan)
        self.Run_scan.setGeometry(QtCore.QRect(30, 20, 131, 41))
        self.Run_scan.setObjectName("Run_scan")
        self.Continue_8 = QtWidgets.QPushButton(self.main_scan)
        self.Continue_8.setGeometry(QtCore.QRect(820, 710, 121, 41))
        self.Continue_8.setObjectName("Continue_8")
        self.Back_8 = QtWidgets.QPushButton(self.main_scan)
        self.Back_8.setGeometry(QtCore.QRect(670, 710, 121, 41))
        self.Back_8.setObjectName("Back_8")
        self.Scanning_processing = QtWidgets.QPushButton(self.main_scan)
        self.Scanning_processing.setGeometry(QtCore.QRect(30, 280, 131, 41))
        self.Scanning_processing.setObjectName("Scanning_processing")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.main_scan)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(30, 330, 471, 351))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.trajectory_graph = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.trajectory_graph.setContentsMargins(0, 0, 0, 0)
        self.trajectory_graph.setObjectName("trajectory_graph")
        self.widget_5 = QtWidgets.QWidget(self.verticalLayoutWidget)
        self.widget_5.setAutoFillBackground(True)
        self.widget_5.setObjectName("widget_5")
        self.trajectory_graph.addWidget(self.widget_5)
        self.Cal_trajectory = QtWidgets.QListWidget(self.main_scan)
        self.Cal_trajectory.setGeometry(QtCore.QRect(530, 330, 441, 351))
        font = QtGui.QFont()
        font.setPointSize(13)
        self.Cal_trajectory.setFont(font)
        self.Cal_trajectory.setObjectName("Cal_trajectory")
        self.Scanlist = QtWidgets.QListWidget(self.main_scan)
        self.Scanlist.setGeometry(QtCore.QRect(30, 81, 931, 181))
        self.Scanlist.setObjectName("Scanlist")
        self.tabWidget.addTab(self.main_scan, "")
        self.welding = QtWidgets.QWidget()
        self.welding.setObjectName("welding")
        self.start_weld = QtWidgets.QPushButton(self.welding)
        self.start_weld.setGeometry(QtCore.QRect(440, 510, 181, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.start_weld.setFont(font)
        self.start_weld.setObjectName("start_weld")
        self.Done = QtWidgets.QPushButton(self.welding)
        self.Done.setGeometry(QtCore.QRect(820, 710, 121, 41))
        self.Done.setObjectName("Done")
        self.Back_9 = QtWidgets.QPushButton(self.welding)
        self.Back_9.setGeometry(QtCore.QRect(670, 710, 121, 41))
        self.Back_9.setObjectName("Back_9")
        self.welding_process = QtWidgets.QListWidget(self.welding)
        self.welding_process.setGeometry(QtCore.QRect(570, 50, 381, 431))
        self.welding_process.setObjectName("welding_process")
        self.verticalLayoutWidget_4 = QtWidgets.QWidget(self.welding)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(40, 50, 471, 431))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.welding_trajectory = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_4)
        self.welding_trajectory.setContentsMargins(0, 0, 0, 0)
        self.welding_trajectory.setObjectName("welding_trajectory")
        self.widget_6 = QtWidgets.QWidget(self.verticalLayoutWidget_4)
        self.widget_6.setAutoFillBackground(True)
        self.widget_6.setObjectName("widget_6")
        self.welding_trajectory.addWidget(self.widget_6)
        self.tabWidget.addTab(self.welding, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1082, 26))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.Move_ex.setText(_translate("MainWindow", "Move"))
        self.Checkerboard_ex.setText(_translate("MainWindow", "Capture image"))
        self.Get_pos_ex.setText(_translate("MainWindow", "Get pos"))
        self.Checker_laser_ex.setText(_translate("MainWindow", "Capture checker"))
        self.Save_pos_ex.setText(_translate("MainWindow", "Save pos"))
        self.Cap_laser_ex.setText(_translate("MainWindow", "Capture laser"))
        self.Continue.setText(_translate("MainWindow", "Continue"))
        self.Close.setText(_translate("MainWindow", "Close"))
        self.label.setText(_translate("MainWindow", "EYE TO HAND CALIBRATION PROCESS"))
        self.label_2.setText(_translate("MainWindow", "ROBOT TASK"))
        self.label_3.setText(_translate("MainWindow", "CAMERA TASK"))
        self.Star_calib_process_ex.setText(_translate("MainWindow", "Start"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.eye_to_hand), _translate("MainWindow", "Eye to hand"))
        self.Checker_laser_in.setText(_translate("MainWindow", "Capture checker"))
        self.Get_pos_in.setText(_translate("MainWindow", "Get pos"))
        self.Checkerboard_in.setText(_translate("MainWindow", "Capture image"))
        self.Save_pos_in.setText(_translate("MainWindow", "Save pos"))
        self.Move_in.setText(_translate("MainWindow", "Move"))
        self.Cap_laser_in.setText(_translate("MainWindow", "Capture laser"))
        self.Continue_2.setText(_translate("MainWindow", "Continue"))
        self.Back_2.setText(_translate("MainWindow", "Back"))
        self.label_6.setText(_translate("MainWindow", "CAMERA TASK"))
        self.label_5.setText(_translate("MainWindow", "EYE IN HAND CALIBRATION PROCESS"))
        self.label_4.setText(_translate("MainWindow", "ROBOT TASK"))
        self.Star_calib_process_in.setText(_translate("MainWindow", "Start"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.Eye_in_hand), _translate("MainWindow", "Eye in hand"))
        self.Calib_main_ex.setText(_translate("MainWindow", "Calib eye to hand"))
        self.Calib_main_in.setText(_translate("MainWindow", "Calib eye in hand"))
        self.Back_3.setText(_translate("MainWindow", "Back"))
        self.Continue_3.setText(_translate("MainWindow", "Continue"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.Calib_main), _translate("MainWindow", "Calib hand eye"))
        self.Calib_laser_in.setText(_translate("MainWindow", "Calib laser internal camera"))
        self.Calib_laser_ex.setText(_translate("MainWindow", "Calib laser external camera"))
        self.Continue_4.setText(_translate("MainWindow", "Continue"))
        self.Back_4.setText(_translate("MainWindow", "Back"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.Calib_laser), _translate("MainWindow", "Calib_laser "))
        self.run_find_scan_trajectory.setText(_translate("MainWindow", "Capture workpiece"))
        self.Continue_7.setText(_translate("MainWindow", "Continue"))
        self.Back_7.setText(_translate("MainWindow", "Back"))
        self.label_7.setText(_translate("MainWindow", "FIND SCAN TRAJECTORY"))
        self.label_8.setText(_translate("MainWindow", "SCAN TRAJECTORY"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.Get_scan_trectory), _translate("MainWindow", "Scan trajectory"))
        self.Run_scan.setText(_translate("MainWindow", "Scan workpiece "))
        self.Continue_8.setText(_translate("MainWindow", "Continue"))
        self.Back_8.setText(_translate("MainWindow", "Back"))
        self.Scanning_processing.setText(_translate("MainWindow", "Scanning processing"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.main_scan), _translate("MainWindow", "Scan"))
        self.start_weld.setText(_translate("MainWindow", "Start weld"))
        self.Done.setText(_translate("MainWindow", "Done"))
        self.Back_9.setText(_translate("MainWindow", "Back"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.welding), _translate("MainWindow", "Weld"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
