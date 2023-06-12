from App_Lib.Yaskawa_Lib import Yaskawa
import time
robot = Yaskawa()

robot.StartRequest()
# on = robot.ArcOn()
# print (on)
# time.sleep(0.05)
# off = robot.ArcOff()
# print (off)
# robot.Stop()
sv = robot.Servo("1")
print(sv)
# io = robot.Read('25010,8')# off
io = robot.WriteIO('25010,8,1')# on0
print(io)
time.sleep(1)

io = robot.WriteIO('25010,8,2')
print(io)