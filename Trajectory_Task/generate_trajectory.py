#first and last scanning positions
x1 = 915
x2 = 1200
scan_trajectory_path = r"E:\New_Code\Trajectory_Task\Test_trajectory.txt"
with open(scan_trajectory_path, "w")  as f:
    for i in range(x1, x2, 5):
        f.write("0, 10, 0, " + str(i) + ", -50.000, -460.000, -180.000, 0.00, 0.000, 0, 0, 0, 0, 0, 0, 0, 0" + "\n")