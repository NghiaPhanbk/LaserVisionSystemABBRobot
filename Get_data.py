'''''
Written by: Nghia Phan

Code for get train data
    1. Create trajectory file from 'generate trajectory'.
    2. Run Get_data file.
    3. Image will be saved in data_train_path
'''''
from App_Lib.Yaskawa_Lib import *
from GlobalVariables import *
from pypylon import pylon
import threading
import sys,time
Home = [10, 0, 840, 0, 80, 180.000, -30.00, 0.000, 0, 0, 0, 0, 0, 0, 0, 0]
class CameraTask(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()
        self.camera.Width.SetValue(1700)
        self.camera.Height.SetValue(1200)
        self.camera.OffsetX.SetValue(8)
        self.camera.OffsetY.SetValue(8)
        self.camera.ExposureTimeAbs = 20000
        self.camera.StartGrabbing()

    def run(self):
        global Curr_image, StopProcess
        while self.camera.IsGrabbing() and not StopProcess:
            try:
                grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                if grabResult.GrabSucceeded():
                    Curr_image = grabResult.Array
            except:
                print("Camera error: ", sys.exc_info())


class RobotTask(threading.Thread):
    def __init__(self):
        global StopProcess
        StopProcess = False
        threading.Thread.__init__(self)
        self.robot = Yaskawa()
        self.robot.StartRequest()
    def homing(self):
        command = self.robot.pos2command(Home)
        self.robot.MovJ(command)

    def pos2Movjcommand(self, pos):
        position = ""
        for i in pos:
            position += str(i) + ", "
        command = "10, 0, " + position + "0, 0, 0, 0, 0, 0, 0, 0"
        return command

    def stop(self):
        global StopProcess
        StopProcess = True
        print("Get_data_done")
        self.robot.Stop()

    def get_data_process(self):
        '''
        1. move to point on work space in txt file
        2. cap picture to get train data
        '''
        global StartGetdata
        time.sleep(1)
        pos_index = 1
        positions = []
        with open(Get_data_trajectory, 'r') as f:
            for line in f:
                line = line.strip()  # loại bỏ ký tự xuống dòng ở cuối dòng
                positions.append(line)
        waypoint = len(positions)
        print('waypoint: ', waypoint)
        StartGetdata = True
        global count
        count = 279
        while StartGetdata:
            if pos_index <= waypoint:
                pos = positions[pos_index - 1]
                print(pos)
                self.robot.MovJ(pos)
                time.sleep(1)
                img_name = data_train_path + "\data_" + str(count) + '.jpg'
                cv.imwrite(img_name, Curr_image)
                count +=1
                print("pos_index",pos_index)
                pos_index +=1
                if pos_index == waypoint+1:
                    endpos = positions[waypoint-1]
                    self.robot.MovJ(pos)
                    img_name = data_train_path + "\data_" + str(count) + '.jpg'
                    cv.imwrite(img_name, Curr_image)
                    time.sleep(1)
                    self.robot.Stop()
                    print("finish_get_data")
                    StartGetdata = False
            else:
                StartGetdata = False
    def run(self):
        global StartGetdata
        if StartGetdata:
            self.get_data_process()
if __name__ == '__main__':
    StartGetdata = True
    robot = RobotTask()
    cam = CameraTask()
    print("--- Homing process ---")
    robot.homing()
    time.sleep(5)
    print("--- Homing process done ---")
    print("--- Start getting data ---")

    # Start new Threads
    cam.start()
    robot.start()
