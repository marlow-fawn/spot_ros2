from typing import List
import geometry_msgs.msg
import numpy as np
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api.spot import inverse_kinematics_pb2, robot_command_pb2
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    GROUND_PLANE_FRAME_NAME,
    ODOM_FRAME_NAME,
    get_a_tform_b,
)
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from tf2_ros import TransformBroadcaster, TransformStamped

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
    def __init__(self, node):
        self.node = node
        self._tf_listener = TFListenerWrapper(node)
        self._robot_command_client = ActionClientWrapper(RobotCommand, "robot_command",
                                                         node)
        # Part of the tf hack:
        self._ik_client = node.create_client(GetInverseKinematicSolutions,
                                             "get_inverse_kinematic_solutions")
        self._tf_broadcaster = TransformBroadcaster(node)
        self._transforms: List[geometry_msgs.msg.TransformStamped] = []

    def ik(self):
        """
               Send one or more IK requests, evaluate them and move Spot accordingly.
               Returns:
                   True the process runs without errors, False otherwise.
               """

        # Frame names.
        odom_frame_name = ODOM_FRAME_NAME
        flat_body_frame_name = GRAV_ALIGNED_BODY_FRAME_NAME
        ground_plane_frame_name = GROUND_PLANE_FRAME_NAME
        task_frame_name = "task_frame"
        arm_link_wr1_frame_name = "arm_link_wr1"
        jaw_frame_name = "jaw_frame"

        # Wait for the robot to publish the TF state.
        self._tf_listener.wait_for_a_tform_b(odom_frame_name, flat_body_frame_name)

        # Look for known transforms published by the robot.
        odom_T_flat_body: SE3Pose = to_se3(self._tf_listener.lookup_a_tform_b(odom_frame_name, flat_body_frame_name))
        odom_T_gpe: SE3Pose = to_se3(self._tf_listener.lookup_a_tform_b(odom_frame_name, ground_plane_frame_name))

        # Construct the frame on the ground right underneath the center of the body.
        odom_T_ground_body: SE3Pose = odom_T_flat_body
        odom_T_ground_body.z = odom_T_gpe.z

        # Now, construct a task frame slightly above the ground, in front of the robot.
        odom_T_task: SE3Pose = odom_T_ground_body * SE3Pose(x=0.4, y=0.0, z=0.05, rot=Quat(w=1.0, x=0.0, y=0.0, z=0.0))
        self._publish_transform(odom_frame_name, task_frame_name, odom_T_task)

        # Now, let's set our tool frame to be the tip of the robot's bottom jaw. Flip the
        # orientation so that when the hand is pointed downwards, the tool's z-axis is
        # pointed upward.
        wr1_T_tool: SE3Pose = SE3Pose(0.23589, 0.0, -0.03943, Quat.from_pitch(-np.pi / 2))
        self._publish_transform(arm_link_wr1_frame_name, jaw_frame_name, wr1_T_tool)

        # Generate several random poses in front of the task frame where we want the tool to move to.
        # The desired tool poses are defined relative to thr task frame in front of the robot and slightly
        # above the ground. The task frame is aligned with the "gravity aligned body frame", such that
        # the positive-x direction is to the front of the robot, the positive-y direction is to the left
        # of the robot, and the positive-z direction is opposite to gravity.
        task_T_desired_tool = SE3Pose(0.2, 0.2, 0.0, Quat())

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
        ik_response = self._send_ik_request(
            odom_T_task=odom_T_task, wr1_T_tool=wr1_T_tool, task_T_desired_tool=task_T_desired_tool
        )

        # Attempt to move to each of the desired tool pose to check the IK results.
        if ik_response.status == inverse_kinematics_pb2.InverseKinematicsResponse.Status.STATUS_OK:
            print("Solution found")
            self._publish_transform(task_frame_name, "T" + "-YES", task_T_desired_tool)

            odom_T_desired_body = get_a_tform_b(
                ik_response.robot_configuration.transforms_snapshot,
                ODOM_FRAME_NAME,
                BODY_FRAME_NAME,
            )
            mobility_params = robot_command_pb2.MobilityParams(
                body_control=robot_command_pb2.BodyControlParams(
                    body_pose=RobotCommandBuilder.body_pose(ODOM_FRAME_NAME, odom_T_desired_body.to_proto())
                )
            )
            stand_command = RobotCommandBuilder.synchro_stand_command(params=mobility_params)
        elif ik_response.status == inverse_kinematics_pb2.InverseKinematicsResponse.Status.STATUS_NO_SOLUTION_FOUND:
            print("No solution found")
            self._publish_transform(task_frame_name, "T" + "-NO", task_T_desired_tool)
            stand_command = body_assist_enabled_stand_command
        else:
            print("Status unknown")
            self._publish_transform(task_frame_name, "T" + "-NO", task_T_desired_tool)
            stand_command = body_assist_enabled_stand_command

        # Move the arm tool to the requested position.
        arm_command = RobotCommandBuilder.arm_pose_command_from_pose(
            hand_pose=(odom_T_task * task_T_desired_tool).to_proto(),
            frame_name=ODOM_FRAME_NAME,
            seconds=1,
            build_on_command=stand_command,
        )
        arm_command.synchronized_command.arm_command.arm_cartesian_command.wrist_tform_tool.CopyFrom(
            wr1_T_tool.to_proto()
        )
        arm_command_goal = RobotCommand.Goal()
        convert(arm_command, arm_command_goal.command)
        result = self._robot_command_client.send_goal_and_wait(
            action_name="arm_move_one", goal=arm_command_goal, timeout_sec=5
        )

        return True

    def _timer_callback(self) -> None:
        """
        Publish all cached tranformations at 10Hz so we can view
        """
        for tf in self._transforms:
            tf.header.stamp = self.node.get_clock().now().to_msg()
            self._tf_broadcaster.sendTransform(tf)

    def _send_ik_request(
            self, odom_T_task: SE3Pose, wr1_T_tool: SE3Pose, task_T_desired_tool: SE3Pose
    ) -> GetInverseKinematicSolutions.Request:
        """
        Send an IK request for a given task frame, tool and tool desired pose.
        Returns:
            An IK solution, if any.
        """
        ik_request = inverse_kinematics_pb2.InverseKinematicsRequest(
            root_frame_name=ODOM_FRAME_NAME,
            scene_tform_task=odom_T_task.to_proto(),
            wrist_mounted_tool=inverse_kinematics_pb2.InverseKinematicsRequest.WristMountedTool(
                wrist_tform_tool=wr1_T_tool.to_proto()
            ),
            tool_pose_task=inverse_kinematics_pb2.InverseKinematicsRequest.ToolPoseTask(
                task_tform_desired_tool=task_T_desired_tool.to_proto()
            ),
        )
        request = GetInverseKinematicSolutions.Request()
        convert(ik_request, request.request)
        ik_reponse = self._ik_client.call(request)

        proto = inverse_kinematics_pb2.InverseKinematicsResponse()
        convert(ik_reponse.response, proto)
        return proto

    def _publish_transform(self, parent_frame_name: str, child_frame_name: str, pose: SE3Pose) -> None:
        """
        Create a transform and add it to a list so we can display it later in RViz.
        """
        tf = geometry_msgs.msg.TransformStamped()
        tf.header.frame_id = parent_frame_name
        tf.child_frame_id = child_frame_name
        tf.transform.translation.x = float(pose.x)
        tf.transform.translation.y = float(pose.y)
        tf.transform.translation.z = float(pose.z)
        tf.transform.rotation.x = float(pose.rot.x)
        tf.transform.rotation.y = float(pose.rot.y)
        tf.transform.rotation.z = float(pose.rot.z)
        tf.transform.rotation.w = float(pose.rot.w)
        self._transforms.append(tf)
