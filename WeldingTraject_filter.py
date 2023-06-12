from matplotlib import pyplot as plt
from GlobalVariables import *
import numpy as np
from App_Lib.App import savematrix
from sklearn.cluster import AgglomerativeClustering
from scipy.stats import zscore
from scipy.stats import iqr
from Finding_orientation import orientation
def remove_outliers_x_iqr(data, threshold=3):
    q1 = np.percentile(data, 25, axis=0)
    q3 = np.percentile(data, 75, axis=0)
    iqr = q3 - q1
    upper_bound = q3 + threshold * iqr
    lower_bound = q1 - threshold * iqr
    is_inlier = np.all((data >= lower_bound) & (data <= upper_bound), axis=1)
    inliers = data[is_inlier]
    return inliers
def remove_outliers_y_iqr(data, threshold=3):
    q1 = np.percentile(data, 25, axis=0)
    q3 = np.percentile(data, 75, axis=0)
    iqr = q3 - q1
    upper_bound = q3 + threshold * iqr
    lower_bound = q1 - threshold * iqr
    is_inlier = np.all((data >= lower_bound) & (data <= upper_bound), axis=1)
    inliers = data[is_inlier]
    return inliers
def remove_outliers_z_iqr(data, threshold=3):
    q1 = np.percentile(data, 25, axis=0)
    q3 = np.percentile(data, 75, axis=0)
    iqr = q3 - q1
    upper_bound = q3 + threshold * iqr
    lower_bound = q1 - threshold * iqr
    is_inlier = np.all((data >= lower_bound) & (data <= upper_bound), axis=1)
    inliers = data[is_inlier]
    return inliers
def trajectory_filter():
    with open(welding_trajectory, 'r') as f:
        positions = [[float(num) for num in line.split('\t')] for line in f]

    positions = np.array(positions)
    # https://stackoverflow.com/questions/2828059/sorting-arrays-in-numpy-by-column
    positions[positions[:, 0].argsort()] #sort by column
    positions[:,2] = -638
    print(positions.shape)
    positions = np.array(positions)
    positions1 = positions
    # inliers_x = remove_outliers(positions[:, 0])
    # inliers_y = remove_outliers(inliers_x[:, 1])



## pp zscore
    # z_scoresz = zscore(positions[:,2])
    # thresholdz = 1.5
    # positions = positions[abs(z_scoresz) < thresholdz]
    # z_scoresx = zscore(positions[:,0])
    # thresholdx = 2
    # positions = positions[abs(z_scoresx) < thresholdx]
    # z_scoresy = zscore(positions[:,1])
    # thresholdy = 2
    # positions = positions[abs(z_scoresy) < thresholdy]


## PP IQR cho toàn tập
    inliers_z = remove_outliers_z_iqr(positions)
    inliers_x = remove_outliers_x_iqr(inliers_z)
    inliers_y = remove_outliers_y_iqr(inliers_x)
    # print(inliers_y)
    # print(inliers_y.shape)
    # diff = np.diff(inliers_y)
    # outliers = np.concatenate((np.array([False]), np.ravel(np.abs(diff)) > 5, np.array([False])))
    # inliers_y = inliers_y[~outliers]
    positions = inliers_y

## pp dùng LocalOutlierFactor
    from sklearn.neighbors import LocalOutlierFactor
    data = positions

    # set LOF parameters
    n_neighbors = 20
    contamination = 0.08

    # fit LOF model and calculate scores
    lof = LocalOutlierFactor(n_neighbors=n_neighbors, contamination=contamination)
    lof_scores = lof.fit_predict(data)

    # create mask for inlier points
    inlier_mask = lof_scores > 0

    # select inlier points
    inlier_data = data[inlier_mask]

    # remove nan values
    positions = inlier_data
    outliers = []
    for i in range(data.shape[1]):  # lặp qua mỗi cột
        outliers_col = positions1[~np.isin(positions1[:, i], inlier_data[:, i]), i]
        outliers.append(outliers_col)

    outliers = np.array(outliers).T
    # positions = outliers
    print(positions.shape)
    # Tọa độ điểm bắt đầu
    x, y, z = positions[int((positions.shape[0])/2)][0], positions[int((positions.shape[0])/2)][1], positions[int((positions.shape[0])/2)][2]
    R, plane = orientation()
    # Vector pháp tuyến
    Nx, Ny, Nz = plane[0][0], plane[1][0], plane[2][0]
    origin = np.array([x, y, z])
    normal_vector = np.array([Nx, Ny, Nz])
    end_point = origin + normal_vector
    print("origin",normal_vector)
    # Vẽ vector pháp tuyến lên đồ thị
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    i = np.linspace(900,1100,10)
    j = np.linspace(100,150,10)
    X,Y = np.meshgrid(i,j)
    Z = plane[0]*X + plane[1]*Y + plane[2]
    surf = ax.plot_surface(X, Y, Z, rstride=5, cstride=5, color='r', alpha=0.3)
    ax.quiver(origin[0], origin[1], origin[2], end_point[0],  end_point[1], end_point[2], length=0.03, color='red')
    # ax.plot([origin[0], end_point[0]], [origin[1], end_point[1]], [origin[2], end_point[2]], length=0.1)
    X = positions[:, 0]
    Y = positions[:, 1]
    Z = positions[:, 2]

    ax.scatter(X, Y, Z, c='r', marker='.')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # plt.subplot(2, 1, 1)
    # plt.show()
    # X1 = positions1[:, 0]
    # Y1= positions1[:, 1]
    # Z1 = positions1[:, 2]
    # fig1 = plt.figure()
    # ax = fig1.add_subplot(111, projection="3d")
    # ax.scatter(X1, Y1, Z1, c='r', marker='.')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # plt.subplot(2, 1, 2)
    plt.show()
    print(positions)
    savematrix(welding_trajectory_filtered, positions)
if __name__ == "__main__":
    trajectory_filter()