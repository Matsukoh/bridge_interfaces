import rclpy

from rclpy.node import Node
from bridge_interfaces.srv import BridgeSrv
from bridge_interfaces_ros2.srv import BridgeROS2Srv
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class TestClientNode(Node):
    def __init__(self):
        super().__init__('test_client_node')

        group = ReentrantCallbackGroup()
        timer_period_pub = 0.1
        
        self.timer_srv = self.create_timer(
            timer_period_pub, 
            self.srv_callback, 
            callback_group=group
            )
        
        self.timer_srv_ros = self.create_timer(
            timer_period_pub, 
            self.srv_ros_callback, 
            callback_group=group
            )

        self.cli = self.create_client(BridgeSrv, 'bridge_srv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('bridge_srv service not available, waiting again...')
        self.req = BridgeSrv.Request()

        self.cli_ros = self.create_client(BridgeROS2Srv, 'bridge_ros_srv')
        while not self.cli_ros.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('bridge_ros_srv service not available, waiting again...')
        self.req_ros = BridgeROS2Srv.Request()

    async def srv_callback(self):
        self.req.client_id = "test"
        self.req.message = "This is the test message."
        future = self.cli.call_async(self.req)
        try:
            response = await future
        except Exception as e:
            self.get_logger().info(
                'Bridge service call failed %r' % (e,))
        else:
            self.get_logger().info(response.message)
    
    async def srv_ros_callback(self):
        self.req_ros.client_id_ros2 = "test_ros"
        self.req_ros.message_ros2 = "This is the test message."
        future = self.cli_ros.call_async(self.req_ros)
        try:
            response = await future
        except Exception as e:
            self.get_logger().info(
                'BridgeROS service call failed %r' % (e,))
        else:
            self.get_logger().info(response.message_ros2)

def main():
    rclpy.init()
    print('Hi from test_client node.')
    test_client_node = TestClientNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(test_client_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        test_client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
