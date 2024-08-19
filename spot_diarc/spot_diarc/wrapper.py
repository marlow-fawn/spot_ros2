from typing import List

from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bosdyn.api import geometry_pb2, manipulation_api_pb2
from bosdyn.client import frame_helpers
from bosdyn.client.math_helpers import Vec3

import geometry_msgs
import rclpy
from bosdyn_msgs.conversions import convert
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from spot_diarc_msgs.srv import DiarcCommand, DiarcDock, DiarcPickUp, DiarcMoveTo
from spot_msgs.action import RobotCommand, Manipulation  # type: ignore
from spot_msgs.srv import GetInverseKinematicSolutions  # type: ignore
from std_srvs.srv import Trigger
from .ik import IKWrapper

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
        self._transforms: List[geometry_msgs.msg.TransformStamped] = []
        self._client_node = rclpy.create_node('client_node')  # Create a node for calling clients
        self._robot_command_client = ActionClientWrapper(Manipulation, "manipulation", self)

        # Add the clients that this node consumes
        self._service_map = {}
        for service in TRIGGER_SERVICES:  # Add all of the simple trigger services to the client node
            self._service_map[service] = TriggerClient(self._client_node, service)

        # Add the services that this node provides
        self.create_service(DiarcPickUp, 'diarc_pick_up', self._diarc_pick_up_callback)
        self.create_service(DiarcCommand, 'diarc_command', self._diarc_command_callback)
        self.create_service(DiarcDock, 'diarc_dock', self._diarc_dock_callback)
        self.create_service(DiarcMoveTo, 'diarc_move_to', self._diarc_move_to_callback)

        self._ik_wrapper = IKWrapper(self, self._client_node)

        print("Done init")

    def startup(self):
        self._service_map["claim"].client.call_async(Trigger.Request())
        self._service_map["power_on"].client.call_async(Trigger.Request())
        self._service_map["stand"].client.call_async(Trigger.Request())

    def shutdown(self):
        self._service_map["sit"].client.call_async(Trigger.Request())
        self._service_map["power_off"].client.call_async(Trigger.Request())

    def _diarc_move_to_callback(self, request, response):
        # Todo: parse response
        res = self._ik_wrapper.move_to(request.x, request.y, request.z)
        response.success = True
        return response

    def _diarc_go_to_location_callback(self, request, response):
        pass

    def _diarc_pick_up_callback(self, request, response):

        print("In pickup")

        grasp = manipulation_api_pb2.PickObjectRayInWorld(
            ray_start_rt_frame=Vec3(request.sx, request.sy, request.sz).to_proto(),
            ray_end_rt_frame=Vec3(request.ex, request.ey, request.ez).to_proto(),
            frame_name=request.frame_name)

        # We can specify where in the gripper we want to grasp. About halfway is generally good for
        # small objects like this. For a bigger object like a shoe, 0 is better (use the entire
        # gripper)
        grasp.grasp_params.grasp_palm_to_fingertip = 0.6

        # The axis on the gripper is the x-axis.
        axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

        # The axis in the vision frame is the negative z-axis
        axis_to_align_with_ewrt_vision = geometry_pb2.Vec3(x=0, y=0, z=-1)

        # Add the vector constraint to our proto.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
            axis_on_gripper_ewrt_gripper)
        constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
            axis_to_align_with_ewrt_vision)

        # We'll take anything within about 15 degrees for top-down or horizontal grasps.
        constraint.vector_alignment_with_tolerance.threshold_radians = 0.25

        # Specify the frame we're using.
        grasp.grasp_params.grasp_params_frame_name = frame_helpers.VISION_FRAME_NAME

        # Build the proto
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(
            pick_object_ray_in_world=grasp)

        action_goal = Manipulation.Goal()
        convert(grasp_request, action_goal.command)
        # Send the request
        print('Sending grasp request...')
        return self._robot_command_client.send_goal_and_wait("pick_object_ray_in_world", action_goal)

    def _diarc_dock_callback(self, request, response):
        """

        Args:
            request:
            response:

        Returns:

        """
        res = self._service_map[request.command].client.call_async(Trigger.Request())
        response.success = res.success
        response.message = res.message
        return response

    def _diarc_command_callback(self, request, response):
        print(f"Calling {request.command}")
        client = self._service_map[request.command].client
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._client_node, future)

        try:
            res = future.result()
        except Exception as e:
            print(f"Service call failed: {e}")
            response.success = False
            response.message = str(e)
        else:
            response.success = res.success
            response.message = res.message
            print(f"Done with {request.command}")
        return response


def main():
    rclpy.init()
    diarc_wrapper = DiarcWrapper()
    rclpy.spin(diarc_wrapper)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
