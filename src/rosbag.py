#!/usr/bin/python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image, CompressedImage
import datetime

import rosbag2_py

class rosbag(Node):

    def __init__(self):

        super().__init__('rosbag')

        ct = datetime.datetime.now()
        ct_str = ct.strftime("%Y-%m-%d-%H_%M_%S")
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri= ct_str,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        self.set_topics()

        self.image_sub = self.create_subscription(ObjectDetection2DArray, '/flir_camera/image_raw/compressed', self.image_callback, 10)
        self.image_sub

        self.depth_sub = self.create_subscription(LocalizationArray, '/bar30/depth', self.depth_callback, 10)
        self.depth_sub

        self.pressure_sub = self.create_subscription(Imu, '/bar30/pressure', self.pressure_callback, 10)
        self.pressure_sub
        
        self.temp_sub = self.create_subscription(SbgEkfEuler, '/bar30/temperature', self.temp_callback, 10)
        self.temp_sub

        self.sonar_sub = self.create_subscription(SbgEkfQuat, '/imagenex831l/range', self.sonar_callback, 10)
        self.sonar_sub
        
        self.imu_sub = self.create_subscription(Mode, '/imu/data', self.imu_callback, 10)
        self.imu_sub

        self.ekf_sub = self.create_subscription(String, '/ekf/status', self.ekf_callback, 10)
        self.ekf_sub

        self.sonar_raw_sub = self.create_subscription(String, '/imagenex831l/range_raw', self.sonar_raw_callback, 10)
        self.sonar_raw_sub

    def set_topics(self, index):

            topic_info_image = rosbag2_py._storage.TopicMetadata(
                name='/flir_camera/image_raw/compressed',
                type='sensor_msgs/msg/CompressedImage',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_image)

            topic_info_depth = rosbag2_py._storage.TopicMetadata(
                name='/bar30/depth',
                type='',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_depth)

            topic_info_pressure = rosbag2_py._storage.TopicMetadata(
                name='/bar30/pressure',
                type='',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_pressure)
            
            topic_info_temp = rosbag2_py._storage.TopicMetadata(
                name='/bar30/temperature',
                type='',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_temp)

            topic_info_sonar = rosbag2_py._storage.TopicMetadata(
                name='/imagenex831l/range',
                type='',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_sonar)
            
            topic_info_imu = rosbag2_py._storage.TopicMetadata(
                name='/imu/data',
                type='',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_imu)

            topic_info_ekf = rosbag2_py._storage.TopicMetadata(
                name='/ekf/status',
                type='',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_ekf)

            topic_info_sonar_raw = rosbag2_py._storage.TopicMetadata(
                name='/imagenex831l/range_raw',
                type='',
                serialization_format='cdr')
            self.writer.create_topic(topic_info_sonar_raw)

             
    def image_callback(self, msg):
        self.writer.write(
            '/flir_camera/image_raw/compressed',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def depth_callback(self, msg):
        self.writer.write(
            '/bar30/depth',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def pressure_callback(self, msg):
        self.writer.write(
            '/bar30/pressure',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def temp_callback(self, msg):
        self.writer.write(
            '/bar30/temperature',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def sonar_callback(self, msg):
        self.writer.write(
            '/imagenex831l/range',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def imu_callback(self, msg):
        self.writer.write(
            '/imu/data',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def ekf_callback(self, msg):
        self.writer.write(
            '/ekf/status',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def sonar_raw_callback(self, msg):
        self.writer.write(
            '/imagenex831l/range_raw',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
    

def main(args=None):
    rclpy.init(args=args)
    sbr = RosbagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

