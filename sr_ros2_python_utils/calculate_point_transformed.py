import sys
import math
import numpy
from numpy.typing import ArrayLike

import rclpy
from rclpy.node import Node

import tf2_geometry_msgs
from geometry_msgs.msg import Pose, Transform, TransformStamped
from sr_ros2_python_utils.transforms import _quaternion_from_euler, _euler_from_quaternion
import argparse
# defined command line options
# this also generates --help and error handling
CLI=argparse.ArgumentParser()
CLI.add_argument(
  "--position",  # name on the CLI - drop the `--` for positional/required parameters
  nargs="*",  # 0 or more values expected => creates a list
  type=float,
  default=[0, 0, 0],  # default if nothing is provided
)
CLI.add_argument(
  "--orientation",  # name on the CLI - drop the `--` for positional/required parameters
  nargs="*",  # 0 or more values expected => creates a list
  type=float,
  default=[0, 0, 0],  # default if nothing is provided
)
CLI.add_argument(
  "--translation",  # name on the CLI - drop the `--` for positional/required parameters
  nargs="*",  # 0 or more values expected => creates a list
  type=float,
  default=[0.0, 0.0, 0.0],  # default if nothing is provided
)
CLI.add_argument(
  "--rotation",  # name on the CLI - drop the `--` for positional/required parameters
  nargs="*",  # 0 or more values expected => creates a list
  type=float,
  default=[0.0, 0.0, 0.0],  # default if nothing is provided
)

# parse the command line
args = CLI.parse_args()
def return_transform_base(pos_from: list[float], rot_from: list[float], translation: list[float], rotation: list[float]):
    pose_from = Pose()
    pose_from.position.x = pos_from[0]
    pose_from.position.y = pos_from[1]
    pose_from.position.z = pos_from[2]
    quat_np = _quaternion_from_euler(math.radians(rot_from[0]), math.radians(rot_from[1]), math.radians(rot_from[2]))
    pose_from.orientation.x = quat_np[0]
    pose_from.orientation.y = quat_np[1]
    pose_from.orientation.z = quat_np[2]
    pose_from.orientation.w = quat_np[3]
    
    transform_stamped = TransformStamped()
    transform = Transform()
    transform.translation.x = translation[0]
    transform.translation.y = translation[1]
    transform.translation.z = translation[2]
    quat_rotation_np = _quaternion_from_euler(rotation[0], rotation[1], rotation[2])
    transform.rotation.x = quat_rotation_np[0]
    transform.rotation.y = quat_rotation_np[1]
    transform.rotation.z = quat_rotation_np[2]
    transform.rotation.w = quat_rotation_np[3]

    transform_stamped.transform = transform
    new_pose = tf2_geometry_msgs.do_transform_pose(pose_from, transform_stamped)
    new_orientation = _euler_from_quaternion(new_pose.orientation)
    print(new_pose.orientation)
    print(f"Transformed position:\n\n{new_pose.position.x}, {new_pose.position.y}, {new_pose.position.z}")
    print(f"Transformed rotation:\n\n{math.degrees(new_orientation[0])}, {math.degrees(new_orientation[1])}, {math.degrees(new_orientation[2])}")


return_transform_base(args.position, args.orientation, args.translation, args.rotation)

