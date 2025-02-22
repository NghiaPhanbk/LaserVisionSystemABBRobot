from matplotlib import pyplot as plt
from pypylon import pylon
import cv2 as cv
from App_Lib.Yaskawa_Lib import Yaskawa
import numpy as np
import threading
import time, sys
from GlobalVariables import *
from ABB_interface import ABB_Lib
from App_Lib import syntec_Lib
from Finding_orientation import orientation
# --- Choose the Welding_HomePos ---
robot_flag =0
Welding_HomePos = [1332, 51, -579, -180, 0, 0]

StartWelding = True

class RobotTask(threading.Thread):
    def __init__(self):
        global CurrentPos, StopProcess
        threading.Thread.__init__(self)
        StopProcess = False
        # self.robot = Yaskawa()
        # self.robot = syntec_Lib.robot_syntec()
        self.robot = ABB_Lib.ABB()
        self.robot.StartRequest()
        CurrentPos = Welding_HomePos
    # def pos2Movlcommand_start(self, pos):
    #     position = ""
    #     for i in pos:
    #         position += str(i) + ", "
    #     command = "0, 10, 0, " + position + "-180, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    #     return command
    # def pos2Movlcommand(self, pos,ori):
    #     position = ""
    #     for i in pos:
    #         position += str(i) + ", "
    #     command = "0, 10, 0, " + position + ori + "0, 0, 0, 0, 0, 0, 0, 0"
    #     return command
    #
    # def pos2Movjcommand(self, pos):
    #     position = ""
    #     for i in pos:
    #         position += str(i) + ", "
    #     command = "5, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
    #     return command
    #
    # def homing(self):
    #     command = self.pos2Movjcommand(Welding_HomePos)
    #     self.robot.MovJ(command)

    def stop(self):
        global StopProcess, StartWelding
        self.robot.Stop()
        StopProcess = True
        StartWelding = False
        print("weld done")

    # def WeldProcess(self):
    #     '''
    #     1. move robot to home position
    #     2. move robot AND weld
    #     '''
    #     global StartWelding
    #     pos_no = 1
    #
    #     with open(welding_trajectory_filtered, 'r') as f:
    #         positions = [[round(float(num),3) for num in line.split('\t')] for line in f]
    #     waypoint = len(positions)
    #     positions = list(reversed(positions))
    #     print('waypoint: ', waypoint)
    #     # ori,plane = orientation()
    #     # print(ori)
    #     ori = "-180, 0, 0, "
    #     StartWelding = True
    #     while StartWelding:
    #         if pos_no == 1:
    #             pos = positions[pos_no-1]
    #             self.robot.MovL(self.pos2Movlcommand_start(pos))
    #             time.sleep(10)
    #             # self.robot.Stop()
    #             # self.robot.StartRequest()
    #             # on = self.robot.ArcOn()
    #             # print('pos_no: ',pos_no)
    #             # print(on)
    #             pos_no += 1
    #         elif pos_no <= waypoint:
    #             pos = positions[pos_no-1]
    #             self.robot.MovL(self.pos2Movlcommand(pos,ori))
    #             time.sleep(0.02)
    #             # time.sleep(0.2)
    #             print('pos_no: ',pos_no)
    #             pos_no += 1
    #             if pos_no == 3:
    #                 on = self.robot.WriteIO('25010,8,1') # on0
    #                 print(on)
    #             if pos_no == waypoint - 1:
    #                 off = self.robot.WriteIO('25010,8,2') # off
    #                 print(off)
    #             if pos_no == waypoint + 1:
    #                 print('finished')
    #                 endpos = positions[waypoint-1]
    #                 self.robot.MovL(self.pos2Movlcommand(endpos, ori))
    #                 time.sleep(1)
    #                 self.robot.Stop()
    #                 print('end pos')
    #         else:
    #             StartWelding = False
    #     print('Welding - Done')

    def WeldProcess(self):
        '''
        1. move robot to home position
        2. move robot AND weld
        '''
        global StartWelding
        pos_no = 1
        # self.robot.import_multiple_point(welding_trajectory)
        # self.robot.Start_welding()
        self.robot.Welding(welding_trajectory)
        print('Welding - Done')

    def run(self):
        global StartScanning, StartWelding, WeldPoints
        if StartWelding:
            self.WeldProcess()



if __name__ == "__main__":
    with open(welding_trajectory_filtered, 'r') as f:
        positions = [[round(float(num),3) for num in line.split('\t')] for line in f]

    positions = np.array(positions)
    X = positions[:,0]
    Y = positions[:,1]
    Z = positions[:,2]
    fig = plt.figure()
    ax  = fig.add_subplot(111, projection="3d")
    ax.scatter(X, Y, Z, c='r' , marker = '.')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

    robot = RobotTask()

    print("--- Homing process ---")
    robot.homing()
    time.sleep(5)
    robot.start()
    print("--- Homing process done ---")
    print("--- Start welding ---")
    
    # Start new Threads

    # cam.start()
    # while True:
    #     img = cam.image
    #     frame = cv.resize(img,(720,480))
    #     cv.imshow('Frame',frame)