from typing import Optional

from bosdyn.api import geometry_pb2, manipulation_api_pb2, robot_command_pb2, full_body_command_pb2, \
    network_compute_bridge_pb2, image_pb2, image_service_pb2

from bosdyn.api.robot_state_pb2 import RobotState
from bosdyn.client import frame_helpers, manipulation_api_client
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bdai_ros2_wrappers.action_client import ActionClientWrapper

import rclpy
from bosdyn_api_msgs.msg import GetImageRequest
from spot_msgs.action import RobotCommand  # type: ignore
from rclpy.node import Node
from spot_diarc_msgs.srv import DiarcCommand
from spot_diarc_msgs.srv import DiarcDock
from spot_diarc_msgs.srv import DiarcPickup
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from bosdyn_msgs.conversions import convert

TRIGGER_SERVICES = [
    "claim",
    "release",
    "stop",
    "self_right",
    "sit",
    "stand",
    "power_on",
    "power_off",
    "arm_stow",
    "arm_carry",
    "arm_unstow",
    "close_gripper",
    "open_gripper",
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
        self.client_node = rclpy.create_node('client_node')  # Create a node for calling clients
        self._robot_command_client = ActionClientWrapper(RobotCommand, "robot_command",
                                                         self)  # Add the subscribers this node consumes

        # self.sub_node = rclpy.create_node('sub_node')  # Create a node for calling clients
        # self.image_sub = self.sub_node.create_subscription(Image, 'camera/hand/image', self.image_sub_callback, 1)
        # self.image = None
        # self.camera_info_sub = self.sub_node.create_subscription(CameraInfo, 'camera/hand/camera_info',
        #                                                             self.camera_info_sub_callback, 1)
        # self.camera_info = None

        # Add the clients that this node consumes
        self.service_map = {}
        for service in TRIGGER_SERVICES:  # Add all of the simple trigger services to the client node
            self.service_map[service] = TriggerClient(self.client_node, service)

        # Add the services that this node provides
        self.create_service(DiarcPickup, 'diarc_pickup', self.diarc_pickup_callback)
        self.create_service(DiarcCommand, 'diarc_command', self.diarc_command_callback)
        self.create_service(DiarcDock, 'diarc_dock', self.diarc_dock_callback)

        # Stuff below here is SDK specific - try to figu
        self._robot_state = RobotState()
        print("Done init")

    def image_sub_callback(self, msg):
        self.image = msg

    def camera_info_sub_callback(self, msg):
        self.camera_info = msg

    def diarc_pickup_callback(self, request, response):
        return None
        # # Request Pick Up on that pixel.
        # # Todo: This can be converted from tf i think
        # transforms_snapshot = self._robot_state.kinematic_state.transforms_snapshot
        #
        # rclpy.spin_once(self.sub_node)
        # model = convert(image_pb2.ImageSource.PinholeModel(), self.camera_info)
        # pick_vec = geometry_pb2.Vec2(x=request.x, y=request.y)
        # grasp = manipulation_api_pb2.PickObjectInImage(
        #     pixel_xy=pick_vec,
        #     transforms_snapshot_for_camera=transforms_snapshot,
        #     frame_name_image_sensor=VISION_FRAME_NAME,
        #     camera_model=model)
        #
        # # We can specify where in the gripper we want to grasp. About halfway is generally good for
        # # small objects like this. For a bigger object like a shoe, 0 is better (use the entire
        # # gripper)
        # grasp.grasp_params.grasp_palm_to_fingertip = 0.6
        #
        # # Tell the grasping system that we want a top-down grasp.
        #
        # # Add a constraint that requests that the x-axis of the gripper is pointing in the
        # # negative-z direction in the vision frame.
        #
        # # The axis on the gripper is the x-axis.
        # axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)
        #
        # # The axis in the vision frame is the negative z-axis
        # axis_to_align_with_ewrt_vision = geometry_pb2.Vec3(x=0, y=0, z=-1)
        #
        # # Add the vector constraint to our proto.
        # constraint = grasp.grasp_params.allowable_orientation.add()
        # constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
        #     axis_on_gripper_ewrt_gripper)
        # constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
        #     axis_to_align_with_ewrt_vision)
        #
        # # We'll take anything within about 15 degrees for top-down or horizontal grasps.
        # constraint.vector_alignment_with_tolerance.threshold_radians = 0.25
        #
        # # Specify the frame we're using.
        # grasp.grasp_params.grasp_params_frame_name = frame_helpers.VISION_FRAME_NAME
        #
        # # Build the proto
        # grasp_request = manipulation_api_pb2.ManipulationApiRequest(
        #     pick_object_in_image=grasp)
        #
        # action_goal = RobotCommand.Goal()
        # convert(grasp_request, action_goal.command)
        # # Send the request
        # print('Sending grasp request...')
        # return self._robot_command_client.send_goal_and_wait("pick_up_object_in_image", action_goal)

    def diarc_dock_callback(self, request, response):
        res = self.service_map[request.command].client.call_async(Trigger.Request())
        response.success = res.success
        response.message = res.message
        return response

    async def diarc_command_callback(self, request, response):
        print("aaa")
        self.get_logger().info(f"Calling {request.command}")
        client = self.service_map[request.command].client
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.client_node, future)

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
