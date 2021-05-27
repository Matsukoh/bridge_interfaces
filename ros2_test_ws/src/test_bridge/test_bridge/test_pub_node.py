import math

import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from bridge_interfaces.msg import Bridge, BridgeArray
from bridge_interfaces_ros2.msg import BridgeROS2, BridgeArrayROS2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TestPubNode(Node):
    def __init__(self):
        super().__init__('test_pub_node')

        timer_period_pub = 0.1

        self.publisher = self.create_publisher(
            BridgeArray, 
            "bridge", 
            10)

        self.publisher_ros = self.create_publisher(
            BridgeArrayROS2, 
            "bridge_ros", 
            10)
        
        group = ReentrantCallbackGroup()

        self.timer_pub = self.create_timer(timer_period_pub, self.pub_callback, callback_group=group)
        self.timer_pub_ros = self.create_timer(timer_period_pub, self.pub_callback_ros, callback_group=group)

    def pub_callback(self):
        bridge_array_msg = BridgeArray()
        for i in range(5):
            bridge_msg = Bridge()
            bridge_msg.client_id = str(i)
            bridge_msg.message = "This is the test message No.{}".format(i)
            bridge_array_msg.bridge_array.append(bridge_msg)

        self.publisher.publish(bridge_array_msg)
    
    def pub_callback_ros(self):
        bridge_array_ros2_msg = BridgeArrayROS2()
        for i in range(5):
            bridge_ros2_msg = BridgeROS2()
            bridge_ros2_msg.client_id_ros2 = str(i)
            bridge_ros2_msg.message_ros2 = "This is the test message No.{}".format(i)
            bridge_array_ros2_msg.bridge_array_ros2.append(bridge_ros2_msg)

        self.publisher_ros.publish(bridge_array_ros2_msg)

def main():
    rclpy.init()
    print('Hi from test_sub node.')
    test_pub_node = TestPubNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(test_pub_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        test_pub_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
