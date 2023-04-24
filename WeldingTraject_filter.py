from matplotlib import pyplot as plt
from GlobalVariables import *
import numpy as np
from App_Lib.App import savematrix
from sklearn.cluster import AgglomerativeClustering
from scipy.stats import zscore
def trajectory_filter():
    with open(welding_trajectory, 'r') as f:
        positions = [[float(num) for num in line.split('\t')] for line in f]

    positions = np.array(positions)

    # https://stackoverflow.com/questions/2828059/sorting-arrays-in-numpy-by-column
    positions[positions[:, 0].argsort()] #sort by column
    z_scoresx = zscore(positions[:,0])
    thresholdx = 1.5
    positions = positions[abs(z_scoresx) < thresholdx]
    z_scoresy = zscore(positions[:,1])
    thresholdy = 3
    positions = positions[abs(z_scoresy) < thresholdy]
    z_scoresz = zscore(positions[:,2])
    thresholdz = 3
    positions = positions[abs(z_scoresz) < thresholdz]
    print(positions.shape)
    # positions = positions[0:152]
    # positions.sort(key=natural_keys)
    # Create an AgglomerativeClustering object using the Distance method
    # Set n_clusters to the desired number of clusters
    # clustering = AgglomerativeClustering(n_clusters=2, linkage='ward')
    #
    # # Fit the clustering object to the data
    # clustering.fit(positions[:],positions[:,2])
    #
    # # Get the cluster labels for each data point
    # labels = clustering.labels_
    #
    # cluster_0_indices = np.where(labels == 0)[0]
    # cluster_1_indices = np.where(labels == 1)[0]
    # cluster_0_points = positions[cluster_0_indices]
    # cluster_1_points = positions[cluster_1_indices]
    # print(cluster_0_points.shape)
    # print(cluster_1_points.shape)
    # positions = cluster_0_points
    X = positions[:, 0]
    Y = positions[:, 1]
    Z = positions[:, 2]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(X, Y, Z, c='r', marker='.')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
    print(positions)
    savematrix(welding_trajectory_filtered, positions)