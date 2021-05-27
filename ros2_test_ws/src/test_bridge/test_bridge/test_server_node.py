import rclpy

from rclpy.node import Node
from bridge_interfaces.srv import BridgeSrv
from bridge_interfaces_ros2.srv import BridgeROS2Srv
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TestServerNode(Node):
    def __init__(self):
        super().__init__('test_server_node')

        group = ReentrantCallbackGroup()
        self.bridge_srv = self.create_service(BridgeSrv, 'bridge_srv', self.bridge_srv_callback, callback_group=group)
        self.bridge_ros_srv = self.create_service(BridgeROS2Srv, 'bridge_ros_srv', self.bridge_srv_ros_callback, callback_group=group)

    def bridge_srv_callback(self, request, response):
        response.success = True
        response.message = "The server received message like {} from client {}.".format(request.message, request.client_id)
        return response
    
    def bridge_srv_ros_callback(self, request, response):
        response.success_ros2 = True
        response.message_ros2 = "The server received message like {} from client {}.".format(request.message_ros2, request.client_id_ros2)
        return response

def main():
    rclpy.init()
    print('Hi from test_server node.')
    test_server_node = TestServerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(test_server_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        test_server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
