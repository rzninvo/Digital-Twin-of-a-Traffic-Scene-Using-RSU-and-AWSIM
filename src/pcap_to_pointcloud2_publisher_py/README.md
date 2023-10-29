# ouster_pcap_to_pointcloud2_publisher
A ROS2 Package that publishes Ouster .pcap Lidar recordings to a PointCloud2 topic.  
![humble](https://github.com/rsasaki0109/lidarslam_ros2/workflows/humble/badge.svg)  

![DEMO](./resource/pcap_to_pointcloud2_Demo.gif)
![DEMO_Intesity](./resource/PointCloud_Conversion.gif)

## Distribution
This package is distributed as a **ROS2 Humble** Package. It is not intended to be a standalone package, but rather a component of a larger system.

## Requirements 
* open3d = 0.17.0
* ouster-sdk = 0.9.0

## Installation
1. Clone this repository into your ROS2 workspace
2. Install the dependencies listed above:  
   * open3d:
    ```console
    pip3 install open3d==0.17.0
    ```
   * ouster-sdk: 
    ```console
    pip3 install ouster-sdk==0.9.0
    ```
3. Build your ROS2 workspace:  
```console
colcon build --packages-select ouster_pcap_to_pointcloud2_publisher
```
4. Source your ROS2 workspace:  
```console
source /path/to/your/ros2_ws/install/setup.bash
```
5. Run the package:  
```console
ros2 launch ouster_pcap_to_pointcloud2_publisher pcap_to_ros_launch.xml pcap_file_path:=/path/to/your/pcap/file.pcap pcap_metadata_path:=/path/to/your/pcap/metadata.json topic:=/output/pointcloud2 frame_id:=map
```
6. Open rviz2 and add a PointCloud2 display with the topic `/output/pointcloud2`. You should see the pointcloud from your .pcap file.


## Acknowledgements
This package is based on two sources:
* [ouster pcap to pcd example](https://static.ouster.dev/sdk-docs/_modules/ouster/sdk/examples/pcap.html#pcap_to_pcd)   
* the [edited code for open3d to ros pointcloud2 package](https://github.com/felixchenfy/open3d_ros_pointcloud_conversion/issues/6) by [youliangtan](https://github.com/youliangtan/) and the original repository owner [felixchenfy](https://github.com/felixchenfy/).
