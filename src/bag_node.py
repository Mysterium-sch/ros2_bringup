import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32, String
from imagenex831l_ros2.msg import ProcessedRange, RawRange

import rosbag2_py

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')

        # Declare parameters
        self.declare_parameter('cam_topic', 'image/compressed')
        self.declare_parameter('depth_topic_d', 'bar30/depth')
        self.declare_parameter('depth_topic_p', 'bar30/pressure')
        self.declare_parameter('depth_topic_t', 'bar30/temperature')
        self.declare_parameter('sonar_topic_pr', 'imagenex831l/range')
        self.declare_parameter('sonar_topic_rr', 'imagenex831l/raw_range')
        self.declare_parameter('imu_topic', 'imu/data')

        # Get parameter values
        cam_topic = self.get_parameter('cam_topic').get_parameter_value().string_value
        depth_topic_d = self.get_parameter('depth_topic_d').get_parameter_value().string_value
        depth_topic_p = self.get_parameter('depth_topic_p').get_parameter_value().string_value
        depth_topic_t = self.get_parameter('depth_topic_t').get_parameter_value().string_value
        sonar_topic_pr = self.get_parameter('sonar_topic_pr').get_parameter_value().string_value
        sonar_topic_rr = self.get_parameter('sonar_topic_rr').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='my_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Create topics metadata
        topics = [
            (cam_topic, 'sensor_msgs/msg/Image'),
            (depth_topic_d, 'std_msgs/msg/Float32'),
            (depth_topic_p, 'std_msgs/msg/Float32'),
            (depth_topic_t, 'std_msgs/msg/Float32'),
            (sonar_topic_pr, 'imagenex831l_ros2/msg/ProcessedRange'),
            (sonar_topic_rr, 'imagenex831l_ros2/msg/RawRange'),
            (imu_topic, 'sensor_msgs/msg/Imu')
        ]

        for topic_name, topic_type in topics:
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            self.writer.create_topic(topic_info)

    #Image
        self.cam_sub_ = self.create_subscription(
            Image,
            cam_topic,
            lambda msg: self.topic_callback(cam_topic, msg),
            10)

    #depth
        self.depth_sub_d_ = self.create_subscription(
            Float32,
            depth_topic_d,
            lambda msg: self.topic_callback(depth_topic_d, msg),
            10)
        self.depth_sub_p_ = self.create_subscription(
            Float32,
            depth_topic_p,
            lambda msg: self.topic_callback(depth_topic_p, msg),
            10)
        self.depth_sub_t_ = self.create_subscription(
            Float32,
            depth_topic_t,
            lambda msg: self.topic_callback(depth_topic_t, msg),
            10)
    #sonar
        self.sonar_sub_ = self.create_subscription(
            ProcessedRange,
            sonar_topic_pr,
            lambda msg: self.topic_callback(sonar_topic, msg),
            10)
        self.sonar_sub_ = self.create_subscription(
            RawRange,
            sonar_topic_rr,
            lambda msg: self.topic_callback(sonar_topic, msg),
            10)
    #imu
        self.imu_sub_ = self.create_subscription(
            Imu,
            imu_topic,
            lambda msg: self.topic_callback(imu_topic, msg),
            10)

    def topic_callback(self, topic_name, msg):
        self.writer.write(
            topic_name,
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
