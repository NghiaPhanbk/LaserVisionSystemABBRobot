import numpy as np
import cv2 as cv
from GlobalVariables import *
import socket
import time

def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def savematrix(filename:str, matrix):
    with open(filename, 'w') as f:
        f.writelines('\t'.join(str(j) for j in i) + '\n' for i in matrix)

def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    # note we also have to specify the type to retrieve otherwise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()
    cv_file.release()
    return [camera_matrix, dist_matrix]

def load_eye2hand(path):
     # FILE_STORAGE_READ
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    eye2hand = cv_file.getNode("K1").mat()
    cv_file.release()
    return eye2hand

def load_laserplane(path):
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    a, b, d = cv_file.getNode("P").mat()
    cv_file.release()
    return float(a), float(b), float(d)