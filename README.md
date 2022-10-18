# tabletop_segment
Simple ROS package to segment point clouds from tabletop surfaces. Uses a simple RANSAC routine to identify planes in a point cloud and removes them. Depends on PCL. Tested on ROS Melodic + PCL 1.8/1.9.

## Usage

Launch the node with

```shell
roslaunch tabletop_segment tabletop_segment.launch <optional parameters>
```

Optional parameters include whether you wish to also launch a RealSense node or not, and which node should be used as a source of point clouds (default will be `tabletop_segment/input_cloud`.
Parameters can be set at startup using the [config file](https://github.com/hsp-panda/tabletop_segment/blob/main/tabletop_segment/cfg/tabletop_segment.yaml), or at runtime using `rosparam`. Subscribe or remap `tabletop_segment/objects_cloud` to use the segmented output. 

The node crops the input point cloud before detecting planes. This happens according to the workspace parameters applied around the **origin** of the point cloud. If your scene features more planes, the largest plane in the workspace will get removed, the others will stay in the output point cloud. If your scene includes more than one object, the output point cloud will include all of them. The output point cloud preserves the input header reference frame. 

## Topics and parameters

### Subscribes to

| Topic | Type | Description | 
| --- | --- | --- |
| `tabletop_segment/input_cloud`                          | `sensor_msgs/PointCloud2` | Point cloud to be segmented | 

### Publishes

| Topic | Type | Description | 
| --- | --- | --- | 
| `tabletop_segment/objects_cloud`                          | `sensor_msgs/PointCloud2` | Segmented point cloud including everything that is not a planar surface |
| `tabletop_segment/plane_cloud`                            | `sensor_msgs/PointCloud2` | Segmented point cloud including the points belonging to the planar surface | 
| `tabletop_segment/plane_coefficients`                     | `pcl_msgs/ModelCoefficients` | Coefficients of the detected plane | 

### Rosparam

| Parameters | Type | Description | 
| --- | --- | --- |
|`tabletop_segment/crop_max_x`                              | double | Workspace X upper bound |
|`tabletop_segment/crop_max_y`                              | double | Workspace Y upper bound |
|`tabletop_segment/crop_max_z`                              | double | Workspace Z upper bound |
|`tabletop_segment/crop_min_x`                              | double | Workspace X lower bound |
|`tabletop_segment/crop_min_y`                              | double | Workspace Y lower bound |
|`tabletop_segment/crop_min_z`                              | double | Workspace Z lower bound |
|`tabletop_segment/plane_distance_threshold`                | double | Maximum distance from the plane of a point belonging to the plane itself |
|`tabletop_segment/remove_outliers`                         | double | Whether to remove outliers or not (not implemented yet) | 


## TODO
- Implement rejection of small outlier clusters after segmentation
- Implement basic object-wise segmentation (e.g. euclidean)
