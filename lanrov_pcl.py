# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import cv2,time,zmq,pickle
import numpy as np
import time

#Import PCL and pcl_visualization
import pcl
import pcl.pcl_visualization

# First import the library
import pyrealsense2 as rs




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

def main():
    ############ connecting to landrov ##################

    context = zmq.Context()
    # sensor_socket = context.socket(zmq.SUB)
    # sensor_socket.setsockopt(zmq.CONFLATE, 1)
    # sensor_socket.connect("tcp://192.168.8.106:5557")
    # sensor_socket.setsockopt(zmq.SUBSCRIBE,b'pointcloud')
    # sensor_socket.setsockopt(zmq.SUBSCRIBE,b'rgbimage')

    control_socket = context.socket(zmq.PUB)
    control_socket.connect("tcp://192.168.8.106:5556")
    print('connected to landrov server')

    ############      main loop        ##################


    #initialize Kernal
    kernel = np.ones((5,5), np.uint8)

    # Initialize the PCL visualizer.
    # visual_2 = pcl.pcl_visualization.CloudViewing()

    visual = pcl.pcl_visualization.PCLVisualizering('3D Viewer')  # Initialize the visualizer.

    depth_scale = 0.0010000000474974513 + 0.03
    # depth_scale = 0.03

    #Create PointCloud
    pc = rs.pointcloud()

    clipping_distance_in_meters = 2
    clipping_distance = clipping_distance_in_meters / depth_scale

    found_cloud = False
    updated_cloud = False

    cnt = 0
    left_turns = 0
    right_turns = 0


    while 1:


        # k=cv2.waitKey(20)
        #     #if sensor_socket.poll(0.001):
        #
        # if k!=-1:
        #     if k  == 27 or k == ord('q'):
        #         break
        #     # import pdb; pdb.set_trace()
        #         # cv2.imshow('img', color)

        if not found_cloud:
            time.sleep(0.5)
            sensor_socket = context.socket(zmq.SUB)
            # sensor_socket.setsockopt(zmq.CONFLATE, 1)
            sensor_socket.connect("tcp://192.168.8.106:5557")
            sensor_socket.setsockopt(zmq.SUBSCRIBE,b'pointcloud')

            while not found_cloud:
                if  len(zmq.select([sensor_socket],[],[],0)[0]):
                    topic,buf = sensor_socket.recv_multipart()
                    # print('got topic',topic)
                    if topic == b'pointcloud':
                        cloud_vtx = np.fromstring(buf, dtype=[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')])
                        found_cloud = True
            # print("Recieved PC")
            sensor_socket.close()

        else:

            time_start = time.time()
            print("Original Point Cloud Size:", cloud.size, "points")
            # import pdb; pdb.set_trace()
            cloud = pcl.PointCloud()
            cloud.from_list(cloud_vtx)


            # Removes points outside of the range 0.1 to 1.5 in the Z axis.
            cloud_filtered = cloud_filter(cloud, "z", 0.1, 1.2)

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

            # # Provide a colour for the point cloud.
            # cloud_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(closest_object, 255, 255, 255)
            # # Display the point cloud.
            # visual.AddPointCloud_ColorHandler(closest_object, cloud_color, b'ground_removed')


            print("Time taken", time.time() - time_start)


            cnt += 1
            updated_cloud = True
            # import pdb; pdb.set_trace()


        # import pdb; pdb.set_trace()
        # Navigate Robot
        if updated_cloud:

            fact = 0.6

            if command == "forward":
                cmd = (fact,fact) # straight
                travel_time = 3.0
                print("Going Forward")
            elif command == "right":
                cmd = (fact,-fact) # turn right
                travel_time = 0.5
                print("Turn Right")
            elif command == "left":
                cmd = (-fact,fact) # turn left
                travel_time = 0.5
                print("Turn Left")
            else:
                print("No Command")
            send_command(cmd, travel_time)
            found_cloud=False
            updated_cloud = False

main()
