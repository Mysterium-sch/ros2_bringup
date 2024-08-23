#!/usr/bin/python3 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, CompressedImage
from apriltag_msgs.msg import AprilTagDetectionArray
from imagenex831l_ros2.msg import RawRange, ProcessedRange
from std_msgs.msg import Float32
from microstrain_inertial_msgs.msg import HumanReadableStatus
import datetime
import rosbag2_py
from rclpy.serialization import serialize_message

class Rosbag(Node):

    def __init__(self):

        super().__init__('rosbag')

        self.namespace = self.get_namespace()

        ct = datetime.datetime.now()
        ct_str = ct.strftime("%Y-%m-%d-%H_%M_%S")
        name = "/ws/data/"+ct_str
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri= name,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        self.set_topics()

        #self.april_tag_sub = self.create_subscription(AprilTagDetectionArray, f'{self.namespace}/detections', self.april_tag_callback, 10)

        self.image_sub = self.create_subscription(CompressedImage, f'{self.namespace}/flir_camera/image_raw/compressed', self.image_callback, 10)

        self.depth_sub = self.create_subscription(Float32, f'{self.namespace}/bar30/depth', self.depth_callback, 10)

        self.pressure_sub = self.create_subscription(Float32, f'{self.namespace}/bar30/pressure', self.pressure_callback, 10)
        
        self.temp_sub = self.create_subscription(Float32, f'{self.namespace}/bar30/temperature', self.temp_callback, 10)

        self.sonar_sub = self.create_subscription(ProcessedRange, f'{self.namespace}/imagenex831l/range', self.sonar_callback, 10)
        
        self.imu_sub = self.create_subscription(Imu, f'{self.namespace}/imu/data', self.imu_callback, 10)

        self.ekf_sub = self.create_subscription(HumanReadableStatus, f'{self.namespace}/ekf/status', self.ekf_callback, 10)

        self.sonar_raw_sub = self.create_subscription(RawRange, f'{self.namespace}/imagenex831l/range_raw', self.sonar_raw_callback, 10)

    def set_topics(self):

        topic_info_image = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/flir_camera/image_raw/compressed',
            type='sensor_msgs/msg/CompressedImage',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_image)

        topic_info_depth = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/bar30/depth',
            type='std_msgs/msg/Float32',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_depth)

        topic_info_pressure = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/bar30/pressure',
            type='std_msgs/msg/Float32',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_pressure)
        
        topic_info_temp = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/bar30/temperature',
            type='std_msgs/msg/Float32',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_temp)

        topic_info_sonar = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/imagenex831l/range',
            type='imagenex831l_ros2/msg/ProcessedRange',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_sonar)
        
        topic_info_imu = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/imu/data',
            type='sensor_msgs/msg/Imu',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_imu)

        topic_info_ekf = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/ekf/status',
            type='microstrain_inertial_msgs/msg/HumanReadableStatus',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_ekf)

        topic_info_sonar_raw = rosbag2_py._storage.TopicMetadata(
            name=f'{self.namespace}/imagenex831l/range_raw',
            type='imagenex831l_ros2/msg/RawRange',
            serialization_format='cdr')
        self.writer.create_topic(topic_info_sonar_raw)

    def image_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/flir_camera/image_raw/compressed',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def depth_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/bar30/depth',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def pressure_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/bar30/pressure',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def temp_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/bar30/temperature',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def sonar_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/imagenex831l/range',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def imu_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/imu/data',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def ekf_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/ekf/status',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def sonar_raw_callback(self, msg):
        self.writer.write(
            f'{self.namespace}/imagenex831l/range_raw',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
    
    def april_tag_callback(self, msg):
        if msg.family == "36h11":
            self.writer.close()
        if msg.family == "16h5":
            ct = datetime.datetime.now()
            ct_str = ct.strftime("%Y-%m-%d-%H_%M_%S")
            name = "/ws/data/"+ct_str
            self.writer = rosbag2_py.SequentialWriter()
            storage_options = rosbag2_py._storage.StorageOptions(
                uri= name,
                storage_id='sqlite3')
            converter_options = rosbag2_py._storage.ConverterOptions('', '')
            self.writer.open(storage_options, converter_options)
            self.set_topics()


def main(args=None):
    rclpy.init(args=args)
    sbr = Rosbag()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
