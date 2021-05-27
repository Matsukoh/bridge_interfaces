import math

import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from bridge_interfaces.msg import Bridge, BridgeArray
from bridge_interfaces_ros2.msg import BridgeROS2, BridgeArrayROS2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TestSubNode(Node):
    def __init__(self):
        super().__init__('test_sub_node')

        group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(
            BridgeArray, 
            "bridge", 
            self.sub_callback, 
            10, 
            callback_group=group
            )

        self.sub_ros = self.create_subscription(
            BridgeArrayROS2, 
            "bridge_ros", 
            self.sub_callback_ros, 
            10, 
            callback_group=group
            )
        
        group = ReentrantCallbackGroup()

    def sub_callback(self, data):
       self.get_logger().info("Bridge callback : subsclibing message from {} clients.".format(len(data.bridge_array)))
    
    def sub_callback_ros(self, data):
       self.get_logger().info("BridgeROS callback : subsclibing message from {} clients.".format(len(data.bridge_array_ros2)))
    
def main():
    rclpy.init()
    print('Hi from test_sub node.')
    test_sub_node = TestSubNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(test_sub_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        test_sub_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
