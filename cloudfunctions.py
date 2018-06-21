import cv2,time,zmq,pickle
import numpy as np
import time

#Import PCL and pcl_visualization
import pcl

# Import pyrealsense2
import pyrealsense2 as rs

def remove_ground(pointcloud, set_distance_threshold):
    # Calculate the surface normals for each point by fitting a plane to the nearest
    # 50 neighbours to the candidate point.
    seg = pointcloud.make_segmenter_normals(ksearch=50)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  # Fit a plane to the points.
    seg.set_optimize_coefficients(True)  # Do a little bit more optimisation once the plane has been fitted.
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)  # Use RANSAC for the sample consensus algorithm.
    seg.set_max_iterations(100)  # Number of iterations for the RANSAC algorithm.
    seg.set_distance_threshold(set_distance_threshold)  # The max distance from the fitted model a point can be for it to be an inlier.
    inliers, model = seg.segment()  # Returns all the points that fit the model, and the parameters of the model.
    # Save all the  outliers as a point cloud. This forms the non ground plane
    cloud_objects = pointcloud.extract(inliers, negative=True)
    return cloud_objects

def cloud_filter(pointcloud, axis, limit1, limit2):
    #Filters pointcloud along x/y/z axis between limits
    fil = pointcloud.make_passthrough_filter()
    fil.set_filter_field_name(axis)
    fil.set_filter_limits(limit1, limit2)
    return fil.filter()

def voxel_filter(pointcloud, leaf_x, leaf_y, leaf_z):
    #Applies a voxel filter to a point cloud
    fil = pointcloud.make_voxel_grid_filter()
    fil.set_leaf_size(leaf_x, leaf_y, leaf_z)
    return fil.filter()

def cluster_extraction(pointcloud, cluster_tolerance, min_size, max_size):
    #Exctacts pont cloud clusters from a point-cloud
    clusters = []
    tree = pointcloud.make_kdtree()
    ec = pointcloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(cluster_tolerance)
    ec.set_MinClusterSize(min_size)
    ec.set_MaxClusterSize(max_size)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    print("Objects_detected =", len(cluster_indices))

    for j, indices in enumerate(cluster_indices):
        cloud_cluster = pcl.PointCloud()
        points = np.zeros((len(indices), 3), dtype=np.float32)
        for i, indice in enumerate(indices):
            # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
            # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
            points[i][0] = pointcloud[indice][0]
            points[i][1] = pointcloud[indice][1]
            points[i][2] = pointcloud[indice][2]

        cloud_cluster.from_array(points)
        clusters.append(cloud_cluster)
    return clusters

def find_closest(objects):
    #Finds the cloud object with the smallest Z value
    closest_z = 10000
    for i, object in enumerate(objects):
        ob_array = object.to_array()
        min = np.min(ob_array, axis=0)
        min_z = min[2]
        if i != 0:
            closest_index = i
            closest_z = min_z
        elif min_z < closest_z:
            closest_index = i
            closest_z = min_z
    return closest_index



def getObjects(pointcloud):

    # Removes points outside of the range 0.1 to 1.5 in the Z axis.
    cloud_filtered = cloud_filter(pointcloud, "z", 0.1, 3)

    # print("Original Point Cloud Size:", cloud_filtered.size, "points")


    # pcl.save(cloud_filtered, './new_clouds/z_filter.pcd')
    voxel_size = 0.05
    #Applies a voxel grid to the cloud filter
    voxel_cloud = voxel_filter(cloud_filtered, voxel_size, voxel_size, voxel_size)

    # pcl.save(voxel_cloud, './new_clouds/voxel_filter.pcd')

    # print("Voxel filter Cloud Size:", voxel_cloud.size, "points")

    # Remove the ground plane inrfont of the robot
    ground_removed = remove_ground(voxel_cloud, 0.2)

    # pcl.save(ground_removed, './new_clouds/ground_removed.pcd')

    objects = cluster_extraction(ground_removed, 0.5, 10, 10000)

    return objects
