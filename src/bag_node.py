import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32, String
import rosbag2_py

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')

        # Declare parameters
        self.declare_parameter('cam_topic', 'debayer/image_raw/rgb')
        self.declare_parameter('depth_topic', 'bar30/depth')
        self.declare_parameter('sonar_topic', 'imagenex831l/sonar_health')
        self.declare_parameter('imu_topic', 'imu/data')

        # Get parameter values
        cam_topic = self.get_parameter('cam_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        sonar_topic = self.get_parameter('sonar_topic').get_parameter_value().string_value
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
            (depth_topic, 'std_msgs/msg/Float32'),
            (sonar_topic, 'std_msgs/msg/String'),
            (imu_topic, 'sensor_msgs/msg/Imu')
        ]

        for topic_name, topic_type in topics:
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            self.writer.create_topic(topic_info)

        # Create subscriptions
        self.cam_sub_ = self.create_subscription(
            Image,
            cam_topic,
            lambda msg: self.topic_callback(cam_topic, msg),
            10)

        self.depth_sub_ = self.create_subscription(
            Float32,
            depth_topic,
            lambda msg: self.topic_callback(depth_topic, msg),
            10)

        self.sonar_sub_ = self.create_subscription(
            String,
            sonar_topic,
            lambda msg: self.topic_callback(sonar_topic, msg),
            10)

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
