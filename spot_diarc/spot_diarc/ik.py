from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bosdyn.api.spot import inverse_kinematics_pb2, robot_command_pb2
from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder

import rclpy
from bosdyn_msgs.conversions import convert
from tf2_ros import TransformStamped

from spot_msgs.action import RobotCommand  # type: ignore
from spot_msgs.srv import GetInverseKinematicSolutions  # type: ignore


def to_se3(ros_transform: TransformStamped) -> SE3Pose:
    """Convert from ROS TransformStamped to Bosdyn SE3Pose"""
    return SE3Pose(
        ros_transform.transform.translation.x,
        ros_transform.transform.translation.y,
        ros_transform.transform.translation.z,
        Quat(
            ros_transform.transform.rotation.w,
            ros_transform.transform.rotation.x,
            ros_transform.transform.rotation.y,
            ros_transform.transform.rotation.z,
        ),
    )


class IKWrapper:
    def __init__(self, node, client_node):
        self._node = node
        self._client_node = client_node
        self._tf_listener = TFListenerWrapper(node)
        self._robot_command_client = ActionClientWrapper(RobotCommand, "robot_command",
                                                         client_node)
        self._ik_client = client_node.create_client(GetInverseKinematicSolutions,
                                                    "get_inverse_kinematic_solutions")
        self.root = BODY_FRAME_NAME

    def move_to(self, x, y, z):
        """
       Send one or more IK requests, evaluate them and move Spot accordingly.
       Returns:
           True the process runs without errors, False otherwise.
       """
        # Wait for the robot to publish the TF state.
        self._tf_listener.wait_for_a_tform_b(self.root, self.root)

        # Set the "tool frame" to be the jaw
        wr1_T_tool: SE3Pose = SE3Pose(0.0, 0.0, 0.0, Quat())

        # Set the goal pose
        task_T_desired_tool = SE3Pose(x, y, z, Quat())

        # Define a stand command that we'll send if the IK service does not find a solution.
        body_control = robot_command_pb2.BodyControlParams(
            body_assist_for_manipulation=robot_command_pb2.BodyControlParams.BodyAssistForManipulation(
                enable_hip_height_assist=True, enable_body_yaw_assist=True
            )
        )
        body_assist_enabled_stand_command = RobotCommandBuilder.synchro_stand_command(
            params=robot_command_pb2.MobilityParams(body_control=body_control)
        )

        # Send IK requests.
        ik_response = self._send_ik_request(wr1_T_tool=wr1_T_tool, task_T_desired_tool=task_T_desired_tool)

        # Attempt to move to each of the desired tool pose to check the IK results.
        if ik_response.status == inverse_kinematics_pb2.InverseKinematicsResponse.Status.STATUS_OK:
            print("Solution found")
            stand_command = RobotCommandBuilder.synchro_stand_command()
        elif ik_response.status == inverse_kinematics_pb2.InverseKinematicsResponse.Status.STATUS_NO_SOLUTION_FOUND:
            print("No solution found")
            stand_command = body_assist_enabled_stand_command
        else:
            print("Status unknown")
            stand_command = body_assist_enabled_stand_command

        # Move the arm tool to the requested position.
        arm_command = RobotCommandBuilder.arm_pose_command_from_pose(
            hand_pose=task_T_desired_tool.to_proto(),
            frame_name=self.root,
            seconds=3,
            build_on_command=stand_command,
        )
        arm_command.synchronized_command.arm_command.arm_cartesian_command.wrist_tform_tool.CopyFrom(
            wr1_T_tool.to_proto()
        )
        arm_command_goal = RobotCommand.Goal()
        convert(arm_command, arm_command_goal.command)
        print("calling action...")
        result = self._robot_command_client.send_goal_and_wait(
            action_name="arm_move_one", goal=arm_command_goal, timeout_sec=10
        )
        # Todo: Return result
        return True

    def _send_ik_request(
            self, wr1_T_tool: SE3Pose, task_T_desired_tool: SE3Pose
    ) -> GetInverseKinematicSolutions.Request:
        """
        Send an IK request for a given task frame, tool and tool desired pose.
        Returns:
            An IK solution, if any.
        """
        ik_request = inverse_kinematics_pb2.InverseKinematicsRequest(
            root_frame_name=self.root,
            wrist_mounted_tool=inverse_kinematics_pb2.InverseKinematicsRequest.WristMountedTool(
                wrist_tform_tool=wr1_T_tool.to_proto()
            ),
            tool_pose_task=inverse_kinematics_pb2.InverseKinematicsRequest.ToolPoseTask(
                task_tform_desired_tool=task_T_desired_tool.to_proto()
            ),
        )
        request = GetInverseKinematicSolutions.Request()
        convert(ik_request, request.request)

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self._client_node, future)

        ik_response = future.result()
        proto = inverse_kinematics_pb2.InverseKinematicsResponse()
        convert(ik_response.response, proto)
        return proto
