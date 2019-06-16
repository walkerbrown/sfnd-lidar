# Lidar and The Point Cloud Library (PCL)

This project is the first in a series for Udacity's Sensor Fusion Nanodegree. The project covers the following key concepts:

- Lidar point cloud processing under real-time constraints
- RANSAC for ground plane segmentation
- K-D tree data structure for Euclidean distance clustering

Some other topics covered include:
- Point cloud simulation by raycasting
- Voxels 

## Visualization of the results

## Build and run on macOS
```
brew install pcl
mkdir build
cd build
cmake ..
make
./environment
```

## Overview of the point cloud processing pipeline
1.
1.
1.
1.

## Comments
While the results are visually impressive, this implementation relies on hand selected hyperparameters. These include the extent of the region of interest and upper/lower bounds on the number of points in a cluster, among others. This hand tuned approach seems way too brittle for general use.

The quantity of data produced by a lidar system is enormous. For the Velodyne HDL 64, it's 2,800,000 `XYZI` points per second!
```
64 horizontal layers
0.08 degrees angular resolution (360 / 0.08 = 4500 rays)
10 Hz refresh rate
```
The concepts covered in this project, such as downsampling and k-d trees, show how to make the problem of clustering tractable.
