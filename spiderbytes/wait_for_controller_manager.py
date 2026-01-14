# spiderbytes/wait_for_controller_manager.py
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

class WaitForControllerManager(Node):
    def __init__(self):
        super().__init__('wait_for_controller_manager')
        self.cli = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        self.get_logger().info('Waiting for controller_manager...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            pass

        self.get_logger().info('controller_manager ready')
        rclpy.shutdown()

def main():
    rclpy.init()
    WaitForControllerManager()
