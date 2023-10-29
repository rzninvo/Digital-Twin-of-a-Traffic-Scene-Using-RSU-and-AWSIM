from typing import List
import open3d as o3d
from ouster import client, pcap
import numpy as np
from scipy.spatial.transform import Rotation
from contextlib import closing
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

FIELDS_XYZI = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.UINT16, count=1)
]

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8

class PcapToRosPublisher(Node):
    def __init__(self):
        super().__init__('pcap_to_ros_publisher')
        self.declare_parameter(name='pcap_file_path', value='test.pcap')
        self.declare_parameter(name='pcap_metadata_path', value='test.json')
        self.declare_parameter(name='topic', value='/output/pointcloud2')
        self.declare_parameter(name='frame_id', value='map')
        self.declare_parameter(name='aut_dt', value=False)
        self.declare_parameter(name='publish_intensity', value=True)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.aut_dt = self.get_parameter('aut_dt').get_parameter_value().bool_value
        self.publish_intensity = self.get_parameter('publish_intensity').get_parameter_value().bool_value
        self.source, self.metadata = self.read_pcap()
        self.publisher = self.create_publisher(PointCloud2, topic, 5)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.pcap_to_ros(self.metadata)

    def read_pcap(self):
        pcap_file_path = self.get_parameter('pcap_file_path').get_parameter_value().string_value
        pcap_metadata_path = self.get_parameter('pcap_metadata_path').get_parameter_value().string_value
        with open(pcap_metadata_path, 'r') as f:
            metadata = client.SensorInfo(f.read())
        source = pcap.Pcap(pcap_file_path, metadata)
        self.xyzlut = client.XYZLut(metadata)
        self.scans = iter(client.Scans(source))
        return [source, metadata]

    def pcap_to_ros(self, metadata : client.SensorInfo) -> o3d.geometry.PointCloud:

        if (metadata.format.udp_profile_lidar == client.UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL):
            self.get_logger().info("Note: You've selected to convert a dual returns pcap. Second "
                                   "returns are ignored in this conversion by this example "
                                   "for clarity reasons.  You can modify the code as needed by "
                                   "accessing it through github or the SDK documentation.")

        # get the frame_id
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        try:
            scan = next(self.scans)
        except StopIteration:
            self.get_logger().info("ًReached end of pcap file.")
            sys.exit()
            
        xyz = self.xyzlut(scan.field(client.ChanField.RANGE))
        intensity = scan.field(client.ChanField.REFLECTIVITY)
        pcd = o3d.geometry.PointCloud()  # type: ignore
        points = xyz.reshape(-1, 3)
        intensity = intensity.reshape(-1)
        if self.aut_dt == True:
            # Create a rotation object for the y-axis
            rotation_pitch = Rotation.from_euler('y', -0.55)
            # Create a rotation object for the z-axis
            rotation_yaw = Rotation.from_euler('z', np.pi)
            # Apply the rotations
            points = rotation_pitch.apply(points)
            points = rotation_yaw.apply(points)

            z_transformed = points[:, 2] + 6  # Add 6 from all Z coordinates

            # Create a new array with transformed Z-coordinates
            points = np.column_stack((points[:, 0], points[:, 1], z_transformed))

        pcd.points = o3d.utility.Vector3dVector(points)  # type: ignore
        ros_cloud = self.open3d_to_ros(pcd, intensity, frame_id)

        self.publisher.publish(ros_cloud)

    def open3d_to_ros(self, open3d_cloud, intensity, frame_id="map") -> PointCloud2:
        # Set "header"
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        # Set "fields" and "cloud_data"
        points=np.asarray(open3d_cloud.points)

        if self.publish_intensity == False:
            fields=FIELDS_XYZ
            return pc2.create_cloud(header, fields, points)

        # if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZI
        cloud_data=[tuple((*p, r)) for p, r in zip(points, intensity)]
        return pc2.create_cloud(header, fields, cloud_data)
    
def main(args=None):
    rclpy.init(args=args)
    pcap_to_ros_publisher = PcapToRosPublisher()
    rclpy.spin(pcap_to_ros_publisher)
    pcap_to_ros_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()