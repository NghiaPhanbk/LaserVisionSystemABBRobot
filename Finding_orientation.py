from GlobalVariables import *
import numpy as np
import math
def orientation():
    positions = []
    with open(welding_trajectory_filtered, 'r') as f:
        for line in f:
            # Tách các giá trị trong dòng và chuyển về kiểu float
            nums = [float(num) for num in line.split('\t')]
            # Lưu giá trị đã chuyển kiểu vào list
            positions.append(nums)
    print(positions)
    x_square = 0
    xy = 0
    x = 0
    y_square = 0
    y = 0
    N = 0
    xz = 0
    yz = 0
    z = 0

    for i in range(len(positions)):
        x_square += positions[i][0]**2
        xy       += positions[i][0] * positions[i][1]
        x        += positions[i][0]
        y_square += positions[i][1]**2
        y        += positions[i][1]
        N        += 1
        xz       += positions[i][0]*positions[i][2]
        yz       += positions[i][1]*positions[i][2]
        z        += positions[i][2]

    M = np.array([[x_square,  xy, x], [xy, y_square, y],[x, y, N]])
    plane = np.linalg.inv(M).dot(np.array([[xz], [yz], [z]]))
    print(plane)
    normal_vector = [plane[0][0],plane[1,0],1]
    norm_xy = math.sqrt(normal_vector[0] ** 2 + normal_vector[1] ** 2)
    Rx = math.degrees(math.atan2(normal_vector[1], normal_vector[2]))+180
    Ry = math.degrees(math.atan2(-normal_vector[0], norm_xy))+30
    Rz = 0
    print(Rx,Ry,Rz)
    ori = str(Rx) + ", " + str(Ry) + ", " + str(Rz) + ", "
    return ori, plane
if __name__ == "__main__":
    R, plane = orientation()
