import numpy as np
import cv2 as cv
from GlobalVariables import *
import socket
import time

CR = "\r"
CRLF = "\r\n"

class Yaskawa():
    def __init__(self):
        self.ip_address = '192.168.255.1'
        self.port = 80
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.settimeout(5)

    def utf8len(self, inputString):
        return len(inputString.encode('utf-8'))

    def command_data_length(self, command):
        if len(command) == 0:
            return 0
        else:
            return self.utf8len(command + CR)

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
        global  flag
        try:
            self.client.connect((self.ip_address, self.port))
            flag =1
        except socket.timeout:
            print("Kết nối tới {}:{} thất bại. Thời gian chờ đã vượt quá giới hạn.".format(self.ip_address, self.port))
            flag =0
            return 0
        #START request
        if flag ==1:
            startRequest = "CONNECT Robot_access Keep-Alive:-1" + CRLF
            self.client.send(startRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)      #4096: buffer size
            startResponse = repr(response)
            # print(startResponse)
            if 'OK: DX Information Server' not in startResponse:
                self.client.close()
                print('[E] Command start request response to DX100 is not successful!')
                return 0
            return 1
    def Stop(self):
        self.client.close()

    def MovJ(self, command):
        #COMMAND request
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVJ" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        # print(commandResponse)
        if ('OK: ' + "MOVJ" not in commandResponse):
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

    def RposC(self):
        #COMMAND request
        command = "1, 0"
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "RPOSC" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "RPOSC" not in commandResponse):
            print('[E] Command request response to DX100 is not successful!')
            return
        else:
            #COMMAND DATA request
            commandDataRequest = command + CR
            self.client.send(commandDataRequest.encode())
            time.sleep(0.01)
            response = self.client.recv(4096)
            commandDataResponse = repr(response)
        time.sleep(0.01)
        current_pos = np.fromstring(commandDataResponse[2:50], dtype=float, sep=',')
        return current_pos

    def MovL(self, command):
        #COMMAND request
        commandLength = self.command_data_length(command)
        commandRequest = "HOSTCTRL_REQUEST" + " " + "MOVL" + " " + str(commandLength) + CRLF
        self.client.send(commandRequest.encode())
        time.sleep(0.01)
        response = self.client.recv(4096)      #4096: buffer size
        commandResponse = repr(response)
        if ('OK: ' + "MOVL" not in commandResponse):
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