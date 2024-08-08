from typing import Optional

import rclpy
from rclpy.node import Node
from spot_diarc_msgs.srv import DiarcCommand
from spot_diarc_msgs.srv import DiarcDock
from std_srvs.srv import Trigger

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
]


class TriggerClient:
    def __init__(self, node, name):
        self.client = node.create_client(Trigger, name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')
        node.get_logger().info(f"Done with {name}")


class DiarcWrapper(Node):

    def __init__(self):
        super().__init__('diarc_wrapper')
        self.service_map = {}

        for service in TRIGGER_SERVICES:
            self.service_map[service] = TriggerClient(self, service)

        self.create_service(DiarcCommand, 'diarc_command', self.diarc_command_callback)
        self.create_service(DiarcDock, 'diarc_dock', self.diarc_dock_callback)

    def diarc_dock_callback(self, request, response):
        res = self.service_map[request.command].client.call_async(Trigger.Request())
        response.success = res.success
        response.message = res.message
        return response

    def diarc_command_callback(self, request, response):
        self.get_logger().info(f"Calling {request.command}")
        client = self.service_map[request.command].client
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(client, future)
        res = future.result()
        response.success = res.success
        response.message = res.message
        return response


def main():
    rclpy.init()
    diarc_wrapper = DiarcWrapper()
    rclpy.spin(diarc_wrapper)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
