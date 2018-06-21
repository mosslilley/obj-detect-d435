## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
#Import PCL and pcl_visualization
import pcl
import pcl.pcl_visualization

import time

import cloudfunctions

# Create a pipeline
pipeline = rs.pipeline()

#Create PointCloud
pc = rs.pointcloud()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


 # Initialize the PCL visualizer.
visual = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

no_frames = 0

time_start = time.time()

save_name = './new_clouds/multi_objects_2'

try:
    while True:
        time_start = time.time()
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        color_image = np.asanyarray(color_frame.get_data())
        cv2.imwrite(save_name + '.png',color_image)
        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        #Calculate point cloud
        points = pc.calculate(aligned_depth_frame)
        pc.map_to(color_frame)

        vtx = np.asanyarray(points.get_vertices())


        # import pdb; pdb.set_trace()
        cloud = pcl.PointCloud()
        cloud.from_list(vtx)
        pcl.save(cloud, save_name + '.pcd')
        print("Original Point Cloud Size:", cloud.size, "points")



        objects = cloudfunctions.getObjects(cloud)

        visual.RemoveAllPointClouds(0)

        template = "Points left {},Points infront {}, points right {}"
        print(template.format(left_objects.size,path_objects.size, right_objects.size))

        path_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(path_objects, 0, 0, 255)
        left_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(left_objects, 255, 0, 0)
        right_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(right_objects, 0, 255, 0)

        visual.AddPointCloud_ColorHandler(left_objects, left_color, b'outliers')
        visual.AddPointCloud_ColorHandler(right_objects, right_color, b'inliers')
        visual.AddPointCloud_ColorHandler(path_objects, path_color, b'original')

        print(command)

        # Provide a colour for the point cloud.
        # cloud_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_filtered, 255, 255, 255)
        # # Display the point cloud.
        # visual.AddPointCloud_ColorHandler(cloud_filtered, cloud_color, b'ground_removed')
        # cv2.imshow('Test ', color_image)

        visual.Spin()  # Update the screen.

        no_frames += 1
        print("FPS", (time.time() - time_start)/no_frames)


finally:
    pipeline.stop()
