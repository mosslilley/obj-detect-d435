# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np

import time

import cv2

#Import PCL and pcl_visualization
import pcl
import pcl.pcl_visualization

def remove_ground(pointcloud):
    # Calculate the surface normals for each point by fitting a plane to the nearest
    # 50 neighbours to the candidate point.
    seg = pointcloud.make_segmenter_normals(ksearch=50)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  # Fit a plane to the points.
    seg.set_optimize_coefficients(True)  # Do a little bit more optimisation once the plane has been fitted.
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)  # Use RANSAC for the sample consensus algorithm.
    seg.set_max_iterations(50)  # Number of iterations for the RANSAC algorithm.
    seg.set_distance_threshold(0.03)  # The max distance from the fitted model a point can be for it to be an inlier.
    inliers, model = seg.segment()  # Returns all the points that fit the model, and the parameters of the model.
    # Save all the  outliers as a point cloud. This forms the non ground plane
    cloud_objects = pointcloud.extract(inliers, negative=True)
    return cloud_objects

def cloud_filter(pointcloud, axis, limit1, limit2):
    fil = pointcloud.make_passthrough_filter()
    fil.set_filter_field_name(axis)
    fil.set_filter_limits(limit1, limit2)
    return fil.filter()

def voxel_filter(pointcloud, leaf_x, leaf_y, leaf_z):
    fil = pointcloud.make_voxel_grid_filter()
    fil.set_leaf_size(leaf_x, leaf_y, leaf_z)
    return fil.filter()




path_to_bag = "20180329_161957.bag"

#Create PointCloud
pc = rs.pointcloud()

config = rs.config()
config.enable_device_from_file(path_to_bag)

# Create a pipeline
pipeline = rs.pipeline()

# Start streaming
profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

 # Initialize the PCL visualizer.
visual = pcl.pcl_visualization.PCLVisualizering('3D Viewer')

no_frames = 0

time_start = time.time()

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


        print("Original Point Cloud Size:", cloud.size, "points")



        # Removes points outside of the range 0.1 to 1.5 in the Z axis.
        cloud_filtered = cloud_filter(cloud, "z", 0.1, 5)

        #Applies a voxel grid to the cloud filter
        voxel_cloud = voxel_filter(cloud_filtered, 0.04, 0.04, 0.04)

        print("Voxel filter Cloud Size:", voxel_cloud.size, "points")

        # Remove the ground plane inrfont of the robot
        ground_removed = remove_ground(voxel_cloud)

        # objects = cluster_extraction(ground_removed, 0.05, 20, 1000)
        #
        # closest_object_index = find_closest(objects)
        #
        # closest_object = objects[closest_object_index]

        # Create a passthrough filter shows points infront of the robot
        path_objects = cloud_filter(ground_removed, "x", -0.3, 0.3)

        # Create a passthrough filter shows points left of the robot
        left_objects = cloud_filter(ground_removed, "x", -2, -0.3)

        # Create a passthrough filter shows points right of the robot
        right_objects = cloud_filter(ground_removed, "x", 0.3, 2)



        if path_objects.size > 10:
            if left_objects.size > right_objects.size:
                command = "right"
            else:
                command = "left"
        else:
            command = "forward"

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

        visual.SpinOnce()  # Update the screen.
        cv2.imshow('Test ', color_image)

        no_frames += 1
        print("FPS", (time.time() - time_start)/no_frames)

finally:
    pipeline.stop()
