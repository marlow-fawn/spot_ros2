from typing import Optional

import rclpy
from rclpy.node import Node
from .simple_spot_commander import SimpleSpotCommander
from spot_diarc_msgs.srv import DiarcCommand
from spot_diarc_msgs.srv import DiarcDock
from spot_msgs.srv import Dock
from std_srvs.srv import Trigger

from spot_msgs.action import RobotCommand  # type: ignore
from bdai_ros2_wrappers.action_client import ActionClientWrapper

TRIGGER_SERVICES = [
    "claim",
    "release",
    "stop",
    "self_right",
    "sit",
    "stand",
    "power_on",
    "power_off",
    "estop/hard",
    "estop/gentle",
    "estop/release",
    "undock",
]


class Client:
    def __init__(self, node, name, srv=Trigger):
        self.client = node.create_client(srv, name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"service {name} not available, waiting again...")
        node.get_logger().info(f"Done with {name}")


class DiarcWrapper(Node):

    def __init__(self):
        super().__init__('diarc_wrapper')
        self.service_map = {}

        self.sub_node = rclpy.create_node('sub_node')
        for service in TRIGGER_SERVICES:
            self.service_map[service] = Client(self.sub_node, service)

        self.service_map["dock"] = Client(self.sub_node, "dock", Dock)

        self.create_service(DiarcCommand, 'diarc_command', self.diarc_command_callback)
        self.create_service(DiarcDock, 'diarc_dock', self.diarc_dock_callback)

    async def diarc_dock_callback(self, request, response):
        self.get_logger().info(f"Calling dock")
        client = self.service_map["dock"].client
        spot_request = Dock.Request()
        spot_request.dock_id = int(request.dockid)
        future = client.call_async(spot_request)
        rclpy.spin_until_future_complete(self.sub_node, future)

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            response.success = False
            response.message = str(e)
        else:
            response.success = res.success
            response.message = res.message
            self.get_logger().info(f"Done with dock")
        return response

    async def diarc_command_callback(self, request, response):
        self.get_logger().info(f"Calling {request.command}")
        client = self.service_map[request.command].client
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.sub_node, future)

        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            response.success = False
            response.message = str(e)
        else:
            response.success = res.success
            response.message = res.message
            self.get_logger().info(f"Done with {request.command}")
        return response

def main():
    rclpy.init()
    diarc_wrapper = DiarcWrapper()
    rclpy.spin(diarc_wrapper)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
