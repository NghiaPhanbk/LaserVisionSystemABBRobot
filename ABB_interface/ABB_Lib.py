import numpy as np
import cv2 as cv
from GlobalVariables import *
import socket
import time

CR = "\r"
CRLF = "\r\n"
command_default = "[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]"
class ABB():
    def __init__(self):
        self.ip_address = '192.168.125.1'
        self.port = 1026
        self.port2 = 6521
    def utf8len(self, inputString):
        return len(inputString.encode('utf-8'))

    def command_data_length(self, command):
        if len(command) == 0:
            return 0
        else:
            return self.utf8len(command + CR)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
          :param roll: The roll (rotation around x-axis) angle in radians.
          :param pitch: The pitch (rotation around y-axis) angle in radians.
          :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
          :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]

    def pos2command(self, pos):
        command = ""
        for i in range(len(pos)):
            command += str(pos[i]) + ","
        # command = command[:-1]
        return command

    # def read_pos_from_txt(self, pos, robot_path):
    #     with open(self.robot_path, "r") as trajectory:
    #         for i, line in enumerate(trajectory):
    #             if i == pos - 1:
    #                 command = line.rstrip()
    #     # self.CurrentPos = np.fromstring(command, dtype = float, sep=',')
    #     return command
    #
    def read_pos_from_txt(self, filename, pos_no):
        with open(filename, "r") as trajectory:
            for i, line in enumerate(trajectory):
                if i == pos_no - 1:
                    command = line.rstrip()
        return command

    def StartRequest(self):
        # self.client.connect((self.ip_address, self.port))
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.settimeout(5)
        global  flag
        try:
            self.client.connect((self.ip_address, self.port))
            flag = 1
            # print(flag)
        except socket.timeout:
            print("Kết nối tới {}:{} thất bại. Thời gian chờ đã vượt quá giới hạn.".format(self.ip_address, self.port))
            flag =0
            return 0
        #START request
        if flag ==1:
            time.sleep(0.01)
            response = self.client.recv(4096)      #4096: buffer size
            startResponse = repr(response)
            print(startResponse)
            if 'Robot ready' not in startResponse:
                self.client.close()
                print('[E] Start request Command start request response to ABB is not successful!')
                return 0
            else:
                return 1
            # startRequest = "OKE"
            # self.client.send(startRequest.encode())
            # # while True:
            # response = self.client.recv(4096)      #4096: buffer size
            # startResponse = repr(response)
            # print(startResponse)
    # def Stop(self):
    #     self.client.close()
    def check_connection(self):
        self.StartRequest()
        commandRequest = "check"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)  # 4096: buffer size
        commandResponse = repr(response)
        if ("OK Connect" not in commandResponse):
            print('[E] OK Connect Command request response to ABB robot is not successful!')
            return 0
        else:
            self.client.close()
            return 1
    def MovJ(self, pos, ori):
        commandRequest = "MOVEJ"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)  # 4096: buffer size
        commandResponse = repr(response)
        if ("OK MOVEJ" not in commandResponse):
            print('[E] Command request response to ABB robot is not successful!')
            return
        else:
            # COMMAND DATA request
            # commandDataRequest = command + (CR if len(command) > 0 else '')
            # cmd_current_pos = np.fromstring(command[0:50], dtype=float, sep=',')
            commandDataRequest = pos
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
            print(commandDataResponse)
            self.client.send(ori.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
            print(commandDataResponse)
            self.client.close()
        return commandDataResponse

    def Rposition(self):
        commandRequest = "RPOSC"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)
        commandDataResponse = repr(response)
        # time.sleep(0.01)
        print(commandDataResponse)
        current_pos = np.fromstring(commandDataResponse[2:50], dtype=float, sep=',')
        self.client.close()
        return current_pos
    def RposC(self):
        global flag
        self.client2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client2.settimeout(5)
        try:
            self.client2.connect((self.ip_address, self.port2))
            # print(flag)
        except socket.timeout:
            print("Kết nối tới {}:{} thất bại. Thời gian chờ đã vượt quá giới hạn.".format(self.ip_address, self.port2))
            flag = 0
            return 0
    def read(self):
        # START request
        # time.sleep(0.01)
        # response = self.client2.recv(4096)  # 4096: buffer size
        # startResponse = repr(response)
        # print(startResponse)
        # if 'Robot ready' not in startResponse:
        #     self.client2.close()
        #     print('[E] Command start request response to ABB is not successful!')
        #     return 0
    #     startRequest = "OKE"
    #     self.client.send(startRequest.encode())
    #     # while True:
    #     response = self.client.recv(4096)  # 4096: buffer size
    #     startResponse = repr(response)
    #     print(startResponse)
    # #COMMAND request
    # # command = "1, 0"
    # # commandLength = self.command_data_length(command)
    # commandRequest ="RPOSC"
    # self.client.send(commandRequest.encode())
    # time.sleep(0.01)
    # response = self.client.recv(4096)      #4096: buffer size
    # commandResponse = repr(response)
    # if ("OK:RPOSC" not in commandResponse):
    #     print('[E] Command request response to ABB is not successful!')
    #     return
    # else:
    #     #COMMAND DATA request
    #     commandDataRequest = "Send_curpos"
    #     self.client.send(commandDataRequest.encode())
    #     time.sleep(0.01)
        startRequest = "OK"
        self.client2.send(startRequest.encode())
        time.sleep(0.01)
        response = self.client2.recv(4096)
        commandDataResponse = repr(response)
        # time.sleep(0.01)
        print(commandDataResponse)
        current_pos = np.fromstring(commandDataResponse[2:50], dtype=float, sep=',')
        # self.client2.close()
        return current_pos

    def MovL(self, pos, ori):
        # self.client.connect((self.ip_address, self.port))
        #COMMAND request
        # commandLength = self.command_data_length(command)
        commandRequest = "MOVEL"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ("OK MOVEL" not in commandResponse):
            print('[E] Command request response to ABB is not successful!')
            return
        else:
            #COMMAND DATA request
            # commandDataRequest = command + (CR if len(command) > 0 else '')
            # cmd_current_pos = np.fromstring(command[0:50], dtype=float, sep=',')
            commandDataRequest = pos
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
            print(commandDataResponse)
            self.client.send(ori.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
            print(commandDataResponse)
        return commandDataResponse
    def Move_calib(self, index):
        # self.client.connect((self.ip_address, self.port))
        #COMMAND request
        # commandLength = self.command_data_length(command)
        commandRequest = "Calib"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        index = str(index)
        commandRequest = "calib" + index
        self.client.send(commandRequest.encode())
        # time.sleep(0.01)
        # response = self.client.recv(4096)      #4096: buffer size
        # commandResponse = repr(response)
        # print(commandResponse)
        self.client.close()
        return 1
    def StartScan(self):
        # self.client.connect((self.ip_address, self.port))
        #COMMAND request
        # commandLength = self.command_data_length(command)
        commandRequest = "Scan"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        commandRequest = "StartScan"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        return 1
    def StopScan(self):
        # self.client.connect((self.ip_address, self.port))
        #COMMAND request
        # commandLength = self.command_data_length(command)
        commandRequest = "Scan"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        commandRequest = "StopScan"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        print(commandResponse)
        return 1
    def OFFSET_Z(self, lst):
        for item in lst:
            item[2] = 155
        return lst
    def Welding(self,trajectory_path):
        # self.client.connect((self.ip_address, self.port))
        #COMMAND request
        # commandLength = self.command_data_length(command)
        with open(trajectory_path, 'r') as f:
            positions = [[(float(num)) for num in line.split('\t')] for line in f]
            # print(positions)
        positions = self.OFFSET_Z(positions)
        # print(positions)
        # ori = [172, 4, 173]
        # quatornion = self.get_quaternion_from_euler(np.deg2rad(ori[0]), np.deg2rad(ori[1]),
                                                     # np.deg2rad(ori[2]))
        # quatornion = [round(item, 5) for item in quatornion]
        quatornion = [0.00381, -0.00111, -0.9999, -0.01336]
        quatornion = str(quatornion)
        print(quatornion)
        commandRequest = "welding"
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ("OK welding" not in commandResponse):
            print('[E] Command request response to ABB is not successful!')
            return
        else:
            #COMMAND DATA request
            # commandDataRequest = command + (CR if len(command) > 0 else '')
            # cmd_current_pos = np.fromstring(command[0:50], dtype=float, sep=',')
            num_point = str(len(positions))
        # print(num_point)
            commandDataRequest = num_point
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
            print(commandDataResponse)
            for i in positions:
                print(i)
                commandDataRequest = str(i)
                self.client.send(commandDataRequest.encode())
                time.sleep(0.01)
                response = self.client.recv(4096)
                commandDataResponse = repr(response)
                print(commandDataResponse)
                self.client.send(quatornion.encode())
                print(quatornion)
                time.sleep(0.01)
                response = self.client.recv(4096)
                commandDataResponse = repr(response)
                print(commandDataResponse)

        return commandDataResponse
    def ArcOn(self):
        #COMMAND request
        ArcStart = 'ARCSTART'
        commandLength = self.command_data_length(ArcStart)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "START" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "START" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = ArcStart + (CR if len(ArcStart) > 0 else '')
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        return commandDataResponse

    def ArcOff(self):
        #COMMAND request
        ArcStop = 'ARCSTOP'
        commandLength = self.command_data_length(ArcStop)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "START" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "START" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = ArcStop + (CR if len(ArcStop) > 0 else '')
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        return commandDataResponse
    def WriteIO(self, data):
        #COMMAND request
        command = data
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "IOWRITE" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "IOWRITE" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + (CR if len(command) > 0 else '')
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        return commandDataResponse
    def Servo(self, data):
        #COMMAND request
        command = data
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "SVON" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "SVON" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + (CR if len(command) > 0 else '')
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        return commandDataResponse
    def Read(self, data):
        #COMMAND request
        command = data
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "IOREAD" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "IOREAD" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + (CR if len(command) > 0 else '')
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        return commandDataResponse
if __name__ == "__main__":
    robot = ABB()
    robot.StartRequest()
    # # print(robot.RposC())
    # command_moveL = "354.09, -95.22, 650.42, 170.10, 13.27, 175.58"
    # command_moveL_test = "[354.09, -95.22, 650.42]"
    # command_moveJ_test = "[364.09, -90.22, 640.42]"
    # current_pos = np.fromstring(command_moveL[0:50], dtype=float, sep=',')
    # # print(current_pos[5])
    # quatornion = robot.get_quaternion_from_euler(np.deg2rad(current_pos[3]), np.deg2rad(current_pos[4]), np.deg2rad(current_pos[5]))
    # quatornion = [round(item, 5) for item in quatornion]
    # quatornion = str(quatornion[::-1])
    # movl = command_moveL_test + "," + quatornion
    # # print(movl)
    # # robot.MovL(command_moveL_test,quatornion)
    # # time.sleep(0.1)
    # # robot.StartRequest()
    # # robot.MovJ(command_moveJ_test, quatornion)
    # # # test = ABB.RposC()
    # # # print(test)
    # # robot.Stop()
    # move = "[731.75,113.55,703.99]"
    # qua = "[2.44365E-05,-0.66283,-0.74877,2.0064E-05]"
    # # robot.MovL(move, qua)
    # robot.Move_calib(1)
    # time.sleep(0.01)
    # print(robot.RposC())
    # robot.Welding(welding_trajectory)

    count =0
    robot.StartScan()
    robot.RposC()
    time.sleep(0.02)
    while True:
        a = robot.read()
        # print(a)
        if len(a)<2:
            print(a)
            robot.client2.close()
            robot.StartRequest()
            robot.StopScan()
            robot.RposC()
            time.sleep(0.02)
            while True:
                # time0 = time.time()
                b = robot.read()
                # time1 = time.time()
                # time2 = time1-time0
                # print(time2)
                count +=1
                if len(b)<5:
                    print(count)
                    robot.client2.close()
                    break
            break