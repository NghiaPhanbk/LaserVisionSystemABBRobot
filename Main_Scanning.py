import cv2 as cv
from pypylon import pylon
from App_Lib.Vision_Lib import Vision
from App_Lib.Yaskawa_Lib import Yaskawa
from App_Lib.App import savematrix
import numpy as np
import threading
import time, sys
from GlobalVariables import *
from PyQt5.QtCore import QObject, pyqtSignal
from App_Lib import syntec_Lib
from ABB_interface import ABB_Lib

# ------------------- Choose the Scan_HomePos --------------------------------
with open(scan_trajectory_path, 'r') as f:
    positions = [[float(num) for num in line.split('\t')] for line in f]

# positions = np.array(positions)
Scan_point1 = positions[0]
Scan_point2 = positions[1]
Scan_HomePos = Scan_point1  # butt welding spline
# Scan_HomePos =
Scan_StartPos = Scan_point1
Scan_EndPos = Scan_point2

ImgArr = []
StartScanning = True
StartWelding = False
WeldDone = False
NowPos = []
WeldPoints = []
CurrentPos = []
# flag_cam = 0
# flag_robot = 0


class RobotTask(threading.Thread):
    def __init__(self):
        global StopProcess
        threading.Thread.__init__(self)
        StopProcess = False
        self.image_list = []
        self.flag_Stop = False
        self.robot = ABB_Lib.ABB()
        # self.robot.StartRequest()
        self.flag_robot = self.robot.check_connection()
        if self.flag_robot == 0:
            print("no robot")

    def read_pos_from_txt(self, filename, pos_no):
        trajectory = open(filename, "r")
        for i, line in enumerate(trajectory):
            if i == pos_no - 1:
                command = line.rstrip()
        return command

    def pos2Movlcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "0, 10, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def pos2Movjcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "5, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def homing(self):
        if self.flag_robot == 1:
            command = self.pos2Movjcommand(Scan_HomePos)
            self.robot.MovJ(command)
        else:
            print("No robot")

    def stop(self):
        global StopProcess, StartWelding
        StopProcess = True
        StartWelding = False
        print("Scan done")
        self.robot.Stop()

    def ScanProcess(self):
        '''
        1. move robot to home position
        2. move robot to scan the workpiece
        '''
        if self.flag_robot == 1:
            global StartScanning
            # self.robot.MovJ(self.pos2Movjcommand(Scan_StartPos))
            self.robot.StartRequest()
            self.robot.StartScan()
            self.robot.RposC()
            time.sleep(0.02)
            StartScanning = True
            count = 1
            NowPos = []
            while True:
                a = self.robot.read()
                if len(a) < 2:
                    # time.sleep(3)
                    self.robot.client2.close()
                    self.robot.StartRequest()
                    self.robot.StopScan()
                    self.robot.RposC()
                    time.sleep(0.02)
                    print("start camera")
                    while True:
                        print("lena", len(a))
                        img_name = scan_weld_seam_img_path + "\weldseam_" + str(count) + '.jpg'
                        CurrentPos = self.robot.read()
                        cv.imwrite(img_name, CurrImg)
                        # self.image_list.append(CurrImg)
                        print('pos no: ', count)
                        # self.data_signal.emit("pos no: {}".format(str(count)))
                        NowPos.append(CurrentPos)
                        time.sleep(0.16)
                        count += 1
                        if len(CurrentPos) < 5:
                            savematrix(scan_pos_path, NowPos)
                            print(count)
                            break

                    break
                print('scanning done')
                # self.data_signal.emit('scanning done')
            print('ImgArr length:\n', len(ImgArr))
            # self.data_signal.emit("ImgArr length: {}".format(str(len(ImgArr))))
            print('NowPos length:\n', len(NowPos))
            # self.data_signal.emit("ImgArr length: {}".format(str(len(NowPos))))
            self.flag_Stop = True
        else:
            print("No robot")

    def run(self):
        global StartScanning, StartWelding, WeldPoints
        if StartScanning and not StartWelding:
            self.ScanProcess()
    # def __init__(self):
    #     global StopProcess
    #     threading.Thread.__init__(self)
    #     StopProcess = False
    #     # self.robot = syntec_Lib.robot_syntec()
    #     self.robot = ABB_Lib.ABB
    #     self.flag_robot = self.robot.check_connection()
    #     if self.flag_robot == 0:
    #         print("no robot")
    # def start_scan(self):
    #     self.robot.StartRequest()
    #     self.robot.StopScan()
    # # def read_pos_from_txt(self, filename, pos_no):
    # #     trajectory = open(filename, "r")
    # #     for i, line in enumerate(trajectory):
    # #         if i == pos_no - 1:
    # #             command = line.rstrip()
    # #     return command
    # #
    # # def pos2Movlcommand(self, pos):
    # #     position = ""
    # #     for i in pos:
    # #         position += str(i) + ", "
    # #     command = "0, 10, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
    # #     return command
    # #
    # # def pos2Movjcommand(self, pos):
    # #     position = ""
    # #     for i in pos:
    # #         position += str(i) + ", "
    # #     command = "5, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
    # #     return command
    # #
    # # def homing(self):
    # #     if self.flag_robot == 1:
    # #         command = self.pos2Movjcommand(Scan_HomePos)
    # #         self.robot.MovJ(command)
    # #     else:
    # #         print("No robot")
    # #
    # def stop(self):
    #     global StopProcess, StartWelding
    #     StopProcess = True
    #     StartWelding = False
    #     print("Scan done")
    #     self.robot.Stop()
    #
    # def ScanProcess(self):
    #     '''
    #     1. move robot to home position
    #     2. move robot to scan the workpiece
    #     '''
    #     if self.flag_robot == 1:
    #         global StartScanning
    #         StartScanning = True
    #         time.sleep(0.05)
    #         count = 1
    #         stop = 0
    #         self.robot.StartRequest()
    #         self.robot.StartScan()
    #         ## cần check
    #         while StartScanning:
    #             self.start_scan()
    #             time.sleep(0.5)
    #             while True:
    #                 img_name = scan_weld_seam_img_path + "\weldseam_" + str(count) + '.jpg'
    #                 CurrentPos = self.robot.RposC()
    #                 NowPos.append(CurrentPos)
    #                 cv.imwrite(img_name, CurrImg)
    #                 print('pos no: ', count)
    #                 time.sleep(0.02)
    #                 count += 1
    #                 stop = self.robot.stop_scanning()
    #                 ## cần check
    #
    #                 print("flag: ",stop)
    #                 if stop:
    #                     StartScanning = False
    #                     savematrix(scan_pos_path, NowPos)
    #                     # self.stop()
    #                     break
    #             print('scanning done')
    #             # self.data_signal.emit('scanning done')
    #         print('ImgArr length:\n', len(ImgArr))
    #         print('NowPos length:\n', len(NowPos))
    #     else:
    #         print("No robot")
    #
    # def run(self):
    #     global StartScanning, StartWelding, WeldPoints
    #     if StartScanning and not StartWelding:
    #         self.ScanProcess()


class CameraTask(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        # self.key = key
        self.stopped = False
        self.image = 1
        self.desired_model = "acA1920-40gm"
        self.started = True
        self._tlFactory = pylon.TlFactory.GetInstance()
        self.devices = self._tlFactory.EnumerateDevices()
        for dev_info in self.devices:
            print("Device model:", dev_info.GetModelName())
        self.device = None
        for dev_info in self.devices:
            if dev_info.GetModelName() == self.desired_model:
                self.device = dev_info
                break
        if self.device is not None:
            self.flag_cam = 1
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.device))
            try:
                self.camera.Open()
                self.camera.Width.SetValue(1700)
                self.camera.Height.SetValue(1200)
                self.camera.OffsetX.SetValue(8)
                self.camera.OffsetY.SetValue(8)
                self.camera.ExposureTimeAbs = 40000
                self.camera.StartGrabbing(pylon.GrabStrategy_LatestImages)
            except:
                print("oke")
        else:
            print("Không tìm thấy thiết bị:", self.desired_model)
            self.flag_cam = 0

    def run(self):
        global CurrImg, StopProcess
        while self.camera.IsGrabbing() and not self.stopped:
            try:
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    CurrImg = grabResult.Array
            except:
                print("Camera error: ", sys.exc_info())
        self.camera.StopGrabbing()
        self.camera.Close()

    def stop(self):
        self.stopped = True


if __name__ == '__main__':
    camera = CameraTask()
    robot = RobotTask()
    print(robot.flag_robot)
    if (camera.flag_cam == 1 & robot.flag_robot == 1):

        # software = SoftwareTask()

        # print("--- Homing process ---")

        # robot.homing()
        time.sleep(3)
        print("--- Homing process done ---")

        print("--- Start Scanning ---")

        # Start new Threads
        camera.start()
        robot.start()
    else:
        print("Done have connection with robot and camera")
