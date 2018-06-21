# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

path_to_bag = "20180329_161957.bag"

config = rs.config()
config.enable_device_from_file(path_to_bag)

# Create a pipeline
pipeline = rs.pipeline()

# Start streaming
profile = pipeline.start(config)

#Initilise Kernal
kernel = np.ones((5,5), np.uint8)


try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # Get aligned frames
        depth_frame = frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = frames.get_color_frame()


        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # d_scaled = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
        # # d_scaled = ((d_scaled*18)/(256)).astype('uint8')
        # d_scaled = ((d_scaled*18)/(256)).astype('uint8')
        # # # import pdb; pdb.set_trace()
        # #
        # # cv2.imshow('Colour ',d_scaled)
        # #
        # # cv2.rectangle(depth_array,(0,0),(640,480),(40,100,0),2)
        # # # # dst = (depth_array/20)*2
        # closed_img = cv2.morphologyEx(d_scaled, cv2.MORPH_CLOSE, kernel)
        # ret, thresh = cv2.threshold(closed_img, 20, 255, cv2.THRESH_BINARY)
        # # import pdb; pdb.set_trace()
        # edges = cv2.Canny(np.uint8(thresh), 50, 120)
        # img2, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #
        # cv2.drawContours(closed_img, contours, -1, (0, 0, 255), -1)



        cv2.imshow('Test ', depth_image)
        cv2.waitKey(1)

finally:
    pipeline.stop()
