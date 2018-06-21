# pcl_visualise_table.py

import numpy as np
import pcl
import pcl.pcl_visualization

table = pcl.load("./new_clouds/voxel_filter.pcd")  # Load the point cloud.

visual = pcl.pcl_visualization.PCLVisualizering('3D Viewer')  # Initialize the visualizer.

# Provide a colour for the point cloud.
table_color = pcl.pcl_visualization.PointCloudColorHandleringCustom(table, 255, 255, 255)
# Display the point cloud.
visual.AddPointCloud_ColorHandler(table, table_color, b'table')

while not visual.WasStopped():  # Quit if "q" is pressed.
    visual.SpinOnce()  # Update the screen.
