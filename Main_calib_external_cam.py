'''
Main code for all the calibration processses.
    1. Camera calibration
    2. Laser calibration
    3. Robot hand-eye calibration

'''

from Calibrate_function.Calib_external_camera import monoCalibrate
from Calibrate_function.Calib_eye_to_hand import eyetohandCalibrate
from Calibrate_function.Calib_laser_eye_to_hand import laserCalibrate
import time


process_start = time.time()

"""
Run calibration only ONCE right after setting up/modifying the system.
Calibration tasks need to be done everytime the laser vision system or welding torch 
is dissembled, changed in baseline distance, modified. 
"""

# Calibration for camera
monoCalibrate()

# Calibration for eye-to-hand robot
eyetohandCalibrate()

# Calibration for laser triangulation 
laserCalibrate()

process_end = time.time()
print("\n\t* * * FULL PROCESS COMPLETE * * *")
print("\t:: Total process time: %.3f s" %(process_end-process_start))