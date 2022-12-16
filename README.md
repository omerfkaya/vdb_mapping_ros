<div align="center">
  <h1>VDB Mapping ROS2</h1>
  <h3>Implementation of vdb_mapping_ros for ROS2.0 </h3>
</div>

Port of the ROS1 [VDB Mapping ](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros) for ROS2.0 


The VDB Mapping ROS Package is a ROS wrapper around [VDB Mapping](https://github.com/omerfkaya/vdb_mapping)

## Getting Started

### Requirements
This library requires [OpenVDB](https://www.openvdb.org/) as it is build around it. This library was initially developed using Version 6.2

Either use the apt package which will be automatically installed via rosdep or compile the package from source using the provided [build instructions](https://github.com/AcademySoftwareFoundation/openvdb)

### Build instructions


``` bash
# source global ros
source /opt/ros/<your_ros_version>/setup.bash

# create a colcon workspace
mkdir -p ~/colcon_ws/src && cd colcon_ws

# clone packages
git clone https://github.com/omerfkaya/vdb_mapping.git
git clone https://github.com/omerfkaya/vdb_mapping_msgs.git
git clone https://github.com/omerfkaya/vdb_mapping_ros2.git

# install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace.  
colcon build --symlink-install

# source the workspace
source install/setup.bash
```

## ROS API
### Advertised ROS Topics
``` 
~/vdb_map_visualization (type: visualization_msgs/Marker)
```
Publishes the resulting map as voxel marker
``` 
~/vdb_map_pointcloud (type: sensor_msgs/PointCloud2)
```
Publishes the resulting map as pointcloud 

### Subscribed ROS Topics
```
~/Parameter:aligned_points (type: sensor_msgs/PointCloud2)
```
Subscriber for pointclouds which are already aligned to a specific frame
```
~/Parameter:raw_points (type: sensor_msgs/PointCloud2)
```
Subscriber for pointclouds in sensor coordinates

### ROS Parameters
All parameters can be passed as commandline arguments to the launch file.
| Parameter Name     | Type    | Default              | Information
| ------------------ | ------- | -------------------- | -----------
| aligned_points     | string  | scan_matched_points2 | Pointclouds which are already aligned to a specific frame (e.g. /map)
| raw_points         | string  | raw_points           | Pointclouds in sensor frame
| sensor_frame       | string  | velodyne             | Sensor frame for raycasting aligned pointclouds
| map_frame          | string  | map                  | Coordinate frame of the map
| max_range          | double  | 10.0                 | Maximum raycasting range
| resolution         | double  | 0.07                 | Map resolution
| prob_hit           | double  | 0.8                  | Probability update if a beam hits a voxel
| prob_miss          | double  | 0.1                  | Probability update if a beam misses a voxel
| prob_thres_min     | double  | 0.12                 | Lower occupancy threshold of a voxel
| prob_thres_max     | double  | 0.8                  | Upper occupancy threshold of a voxel
| publish_pointcloud | boolean | true                 | Specify whether the map should be published as pointcloud
| publish_vis_marker | boolean | true                 | Specify whether the map should be published as visual marker 


