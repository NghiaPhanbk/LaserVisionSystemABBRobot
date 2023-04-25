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

# ------------------- Choose the Scan_HomePos --------------------------------
with open(scan_trajectory_path, 'r') as f:
    positions = [[float(num) for num in line.split('\t')] for line in f]

# positions = np.array(positions)
Scan_point1 = positions[0]
Scan_point2 = positions[1]
Scan_HomePos = Scan_point2  # butt welding spline
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
        self.robot = Yaskawa()
        self.flag_robot = self.robot.StartRequest()
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
            self.robot.MovJ(self.pos2Movjcommand(Scan_StartPos))
            StartScanning = True
            time.sleep(0.05)
            count = 1
            while StartScanning:
                self.robot.MovL(self.pos2Movlcommand(Scan_EndPos))
                time.sleep(0.05)
                while True:
                    img_name = scan_weld_seam_img_path + "\weldseam_" + str(count) + '.jpg'
                    CurrentPos = self.robot.RposC()
                    NowPos.append(CurrentPos)
                    cv.imwrite(img_name, CurrImg)
                    print('pos no: ', count)
                    # self.data_signal.emit("pos no: {}".format(str(count)))
                    time.sleep(0.05)
                    count += 1
                    if CurrentPos[0] >= Scan_point2[0] - 5:
                        StartScanning = False
                        savematrix(scan_pos_path, NowPos)
                        self.stop()
                        break
                print('scanning done')
                # self.data_signal.emit('scanning done')
            print('ImgArr length:\n', len(ImgArr))
            # self.data_signal.emit("ImgArr length: {}".format(str(len(ImgArr))))
            print('NowPos length:\n', len(NowPos))
            # self.data_signal.emit("ImgArr length: {}".format(str(len(NowPos))))
        else:
            print("No robot")

    def run(self):
        global StartScanning, StartWelding, WeldPoints
        if StartScanning and not StartWelding:
            self.ScanProcess()


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
            self.camera.Open()
            self.camera.Width.SetValue(1700)
            self.camera.Height.SetValue(1200)
            self.camera.OffsetX.SetValue(8)
            self.camera.OffsetY.SetValue(8)
            self.camera.ExposureTimeAbs = 15000
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImages)
        else:
            print("Không tìm thấy thiết bị:", self.desired_model)
            # self.data_signal.emit("Không tìm thấy thiết bị: {}".format(self.desired_model))
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


# class SoftwareTask(threading.Thread):
#    def __init__(self):
#       threading.Thread.__init__(self)
#       self.vis = Vision()
#       self.intrinsic = self.vis.intrinsic
#       self.dist_coffs = self.vis.dist_coffs
#       self.eye2hand = self.vis.eye2hand
#       [self.a, self.b, self.c, self.d] = self.vis.plane
#       self.fx = self.intrinsic[0][0]
#       self.fy = self.intrinsic[1][1]
#       self.cx = self.intrinsic[0][2]
#       self.cy = self.intrinsic[1][2]
#
#    def run(self):
#         pass
if __name__ == '__main__':
    # class main_scan(QObject):
    #     data_signal = pyqtSignal(str)
    #     def __init__(self):
    #         QObject.__init__(self)
    # def run_scan(self):
    camera = CameraTask()
    robot = RobotTask()
    print(robot.flag_robot)
    if (camera.flag_cam == 1 & robot.flag_robot == 1):

        # software = SoftwareTask()

        print("--- Homing process ---")
        # self.data_signal.emit("--- Homing process ---")
        robot.homing()
        time.sleep(3)
        print("--- Homing process done ---")
        # self.data_signal.emit("--- Homing process done ---")
        print("--- Start Scanning ---")
        # self.data_signal.emit("--- Start Scanning ---")
        # Start new Threads
        camera.start()
        robot.start()
    else:
        print("Done have connection with robot and camera")
