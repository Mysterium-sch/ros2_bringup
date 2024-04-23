from glob import glob
from setuptools import setup

package_name = 'ros2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lilian lamb',
    maintainer_email='lamblily90@gmail.com',
    description='ROS 2 Launch for StereoRig',
    license='MIT',
    tests_require=['pytest'],
)
