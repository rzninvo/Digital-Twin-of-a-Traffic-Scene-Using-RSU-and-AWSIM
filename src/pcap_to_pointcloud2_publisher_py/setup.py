import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pcap_to_pointcloud2_publisher_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohamzn',
    maintainer_email='rzninvo@gmail.com',
    description='A python package to publish Ouster Lidar PCAP data to ROS2 PointCloud2 Topic',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcap_to_ros_publisher = pcap_to_pointcloud2_publisher_py.pcap_to_ros_publisher:main',
        ],
    },
)
