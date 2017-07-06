# Obstacles Detector for Self-Driving Cars

This repo contains a ROS node to detect cars and pedestrian in a self-driving car with LiDAR/radar/camera output.

## lidar_node
<img src="heightmap.png" alt="height map" width="400px"></img>

The node takes the PointCloud from the LIDAR, and does a series of operations to help the detection:
- ground plane segmentation and removal
- removing points above and below certain heights (z-value), and those corresponding to the capture vehicle
- performing clustering via a Breadth-First Search, and computing the centroid of each cluster found

All of the real code currently resides in `/src/lidar/src/lidar_node.cpp`.

To use this node, first build using `catkin-make`. 

## camera

The detection of the camera frames is done using Deep Learning, specifically the Faster-RCNN library. 

## Fusion

Using the output from the camera, we can use the radar and LIDAR to get a more precise detection. Also, a Kalman filter is used to get more accurate results.


