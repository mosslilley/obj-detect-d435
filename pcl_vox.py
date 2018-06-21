# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import cv2,time,zmq,pickle
import numpy as np
import time

#Import PCL and pcl_visualization
import pcl
import pcl.pcl_visualization

# First import the library
import pyrealsense2 as rs

import cloudfunctions

def main():

    visual = pcl.pcl_visualization.PCLVisualizering('3D Viewer')  # Initialize the visualizer.
    # visual.SetBackgroundColor(255, 255, 255)

    cloud = pcl.load("./new_clouds/multi_objects_2.pcd")  # Load the point cloud.

    time_start = time.time()

    objects = cloudfunctions.getObjects(cloud)

    visual.RemoveAllPointClouds(0)
    # template = "Points left {},Points infront {}, points right {}"
    # print(template.format(left_objects.size,path_objects.size, right_objects.size))
    #
    # path_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(path_objects, 255, 0, 0)
    # left_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(left_objects, 0, 0, 255)
    # right_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(right_objects, 0, 255, 0)
    #
    # visual.AddPointCloud_ColorHandler(left_objects, left_color, b'outliers')
    # visual.AddPointCloud_ColorHandler(right_objects, right_color, b'inliers')
    # visual.AddPointCloud_ColorHandler(path_objects, path_color, b'original')

    # print(command)

    # # Provide a colour for the point cloud.
    # cloud_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(ground_removed, 255, 255, 255)
    #
    # # Display the point cloud.
    # visual.AddPointCloud_ColorHandler(ground_removed, cloud_color, b'ground_removed')

    # for obstacle in enumerate(objects):
    #     ob_num = obstacle[0]
    #     object_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(obstacle[1], 255 - ob_num*50, 0, 0)
    #     visual.AddPointCloud_ColorHandler(obstacle[1], object_color, b'outlier' + bytes(ob_num))

    object_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(objects[0], 255, 0, 0)
    visual.AddPointCloud_ColorHandler(objects[0], object_color, b'object1')

    object_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(objects[1], 0, 255, 0)
    visual.AddPointCloud_ColorHandler(objects[1], object_color, b'object2')

    object_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(objects[2], 0, 0, 255)
    visual.AddPointCloud_ColorHandler(objects[2], object_color, b'object3')
    #
    # object_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(objects[0], 255, 255, 0)
    # visual.AddPointCloud_ColorHandler(objects[0], object_color, b'object4')


    #Display Result
    visual.Spin()

    # import pdb; pdb.set_trace()

main()
