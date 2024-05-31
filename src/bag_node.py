import rclpy
from rclpy.node import Node
import subprocess

class Ros2BagRecorder(Node):
    def __init__(self):
        super().__init__('ros2_bag_recorder')

        # Declare and read the namespace parameter
        self.declare_parameter('namespace', '')
        namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # Define the topics based on the namespace
        topics = [
            '/image/compressed',
            '/bar30/depth',
            '/bar30/pressure',
            '/bar30/temperature',
            '/imagenex831l/range',
            '/imu/data',
            '/ekf/status',
            '/imagenex831l/range_raw'
        ]

        # Construct the ros2 bag command
        ros2_bag_command = ['ros2', 'bag', 'record'] + topics

        # Log the command to be executed
        self.get_logger().info('Executing command: %s' % ' '.join(ros2_bag_command))

        # Execute the ros2 bag command
        try:
            subprocess.Popen(ros2_bag_command)
        except Exception as e:
            self.get_logger().error('Failed to execute ros2 bag command: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = Ros2BagRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
