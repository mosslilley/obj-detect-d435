# Object Detection with Intel RealSense D435

The Department of Computer Science (COSC) and Software Engineering at the University of Canterbury recent has recently acquired a land rover and mounted
an Intel RealSense D435 3D camera on it. The intention is that it will be able to complete a Mini-DARPA challenge travelling across campus. This project
works towards that goal by implimenting object dection which can be used to naviagte the rover

## Getting Started

Libraries Required for Object Detection

 - pyrealsense2
 - python-pcl
 - Numpy

Libraries for Landrov Interaction
 - pickle
 - zmq

Currently cloudfunction.py contains the functions used to manipulate the PointClouds

The other scripts used cloudfunctions.py in different contexts

pcl_d435.py


## Built With

* [Landrov](https://github.com/uc-csse/landrov) - UC CSSE Landrover resourse
* [RealSense](https://github.com/IntelRealSense/librealsense) - Intel RealSense library
* [PCL](https://github.com/PointCloudLibrary/pcl) - Point Cloud Library
* [python-pcl](https://github.com/strawlab/python-pcl) - PCL Python Wrappers

## Authors

* **Moss Lilley** - *Initial work* - [Wimazar](https://github.com/Wimazar)
