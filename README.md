# Object Detection with Intel RealSense D435

The Department of Computer Science (COSC) and Software Engineering at the University of Canterbury recent has recently acquired a land rover and mounted
an Intel RealSense D435 3D camera on it. The intention is that it will be able to complete a Mini-DARPA challenge travelling across campus. This project
works towards that goal by implimenting object dection which can be used to naviagte the rover

The methods used are detailed in the paper "MossLilley-COSC428.pdf"

## Getting Started

Libraries Required for Object Detection

 - pyrealsense2
 - python-pcl
 - Numpy

Libraries for Landrov Interaction
 - pickle
 - zmq

cloudfunction.py contains the functions used to manipulate the PointClouds

The other scripts used cloudfunctions.py in different contexts

* pcl_d435.py is used when streaming data directly from an IntelRealSense D435 camera
* pcl_vox.py is used on pre recorded point clouds
* bag_collison.py is used to apply the method to pre-recorded bag files
* landrov_server.py is adapted landrov_server code to send point cloud data



## Built With

* [Landrov](https://github.com/uc-csse/landrov) - UC CSSE Landrover resourse
* [RealSense](https://github.com/IntelRealSense/librealsense) - Intel RealSense library
* [PCL](https://github.com/PointCloudLibrary/pcl) - Point Cloud Library
* [python-pcl](https://github.com/strawlab/python-pcl) - PCL Python Wrappers

## Authors

* **Moss Lilley** - *Initial work* - [Wimazar](https://github.com/Wimazar)
