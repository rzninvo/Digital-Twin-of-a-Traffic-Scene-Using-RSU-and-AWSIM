from typing import List
import open3d as o3d
from ouster import client, pcap
import numpy as np
from contextlib import closing

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

class PcapToRosPublisher(Node):
    def __init__(self):
        super().__init__('pcap_to_ros_publisher')
        self.declare_parameter(name='pcap_file_path', value='test.pcap')
        self.declare_parameter(name='pcap_metadata_path', value='test.json')
        self.declare_parameter(name='/output/pointcloud2', value='/output/pointcloud2')
        self.declare_parameter(name='frame_id', value='map')
        topic = self.get_parameter('/output/pointcloud2').get_parameter_value().string_value
        self.publisher = self.create_publisher(PointCloud2, topic, 1)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        source, metadata = self.read_pcap()
        with closing(source):
            self.pcap_to_ros(source, metadata)

    def read_pcap(self):
        pcap_file_path = self.get_parameter('pcap_file_path').get_parameter_value().string_value
        pcap_metadata_path = self.get_parameter('pcap_metadata_path').get_parameter_value().string_value
        with open(pcap_metadata_path, 'r') as f:
            metadata = client.SensorInfo(f.read())
        source = pcap.Pcap(pcap_file_path, metadata)
        return [source, metadata]

    def pcap_to_ros(self, source : client.PacketSource, metadata : client.SensorInfo) -> o3d.geometry.PointCloud:

        if (metadata.format.udp_profile_lidar == client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL):
            self.get_logger().info("Note: You've selected to convert a dual returns pcap. Second "
                                   "returns are ignored in this conversion by this example "
                                   "for clarity reasons.  You can modify the code as needed by "
                                   "accessing it through github or the SDK documentation.")
            
        # precompute xyzlut to save computation in a loop
        xyzlut = client.XYZLut(metadata)

        # create an iterator of LidarScans from pcap and bound it if num is specified
        scans = iter(client.Scans(source))

        # get the frame_id
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        for idx, scan in enumerate(scans):

            xyz = xyzlut(scan.field(client.ChanField.RANGE))
            pcd = o3d.geometry.PointCloud()  # type: ignore
            pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))  # type: ignore
            ros_cloud = self.open3d_to_ros(pcd, frame_id)

            self.publisher.publish(ros_cloud)

    def open3d_to_ros(self, open3d_cloud, frame_id="map") -> PointCloud2:
        # Set "header"
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        # Set "fields" and "cloud_data"
        points=np.asarray(open3d_cloud.points)
        if not open3d_cloud.colors: # XYZ only
            fields=FIELDS_XYZ
            cloud_data=points
        else: # XYZ + RGB
            fields=FIELDS_XYZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.
            colors = np.floor(np.asarray(open3d_cloud.colors)*255)
            colors = colors.astype(np.uint32)
            colors = colors[:,2] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,1]  
            colors = colors.view(np.float32)
            cloud_data = [tuple((*p, c)) for p, c in zip(points, colors)]
        # create ros_cloud
        return pc2.create_cloud(header, fields, cloud_data)
    
def main(args=None):
    rclpy.init(args=args)
    pcap_to_ros_publisher = PcapToRosPublisher()
    rclpy.spin(pcap_to_ros_publisher)
    pcap_to_ros_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()