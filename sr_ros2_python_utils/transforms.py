# Copyright (c) 2023, Stogl Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import numpy
from numpy.typing import ArrayLike

import rclpy
from rclpy.node import Node

import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Vector3Stamped
from numpy import arctan2, arcsin


def _apply_local_offset(pose:PoseStamped, x:float=0.0, y:float=0.0, z:float=0.0) -> PoseStamped:
    """Applies linear offset to the given pose in the local coordinate system"""
    result_pose = pose
    result_pose.pose.position.x += x
    result_pose.pose.position.y += y
    result_pose.pose.position.z += z
    
    return result_pose

def _euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def _quaternion_from_euler(ai:float, aj:float, ak:float):
    # quaternion order is [qx, qy, qz, qw]
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = numpy.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def _quaternion_multiply(q0:ArrayLike, q1:ArrayLike) -> ArrayLike:
    """
    Multiplies two quaternions. Convention used [qx, qy, qz, qw]

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    x0 = q0[0]
    y0 = q0[1]
    z0 = q0[2]
    w0 = q0[3]

    # Extract the values from q1
    x1 = q1[0]
    y1 = q1[1]
    z1 = q1[2]
    w1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = numpy.array([q0q1_x, q0q1_y, q0q1_z, q0q1_w])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


class TCPTransforms:
    def __init__(self, node:Node, tcp_link_name:str='tcp_link', tool_link_name:str='tcp_gripper') -> None:
        """
        TCP transformation helper class

        :param node: parent node to handle tf listener
        :param tcp_link_name: frame of the tcp (planning frame)
        :param: tool_link_name: working frame that should coincide with goal pose after motion
        """
        self.node = node

        # initialize TF2 listener and broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # store the tool transform
        self.tcp_frame = tcp_link_name
        self.tool_frame = tool_link_name

    def get_transform(self, target_frame: str, source_frame: str):
        transform = None
        rate = self.node.create_rate(1)
        for t in range(1, 6):
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except TransformException as e:
                self.node.get_logger().warn(f"Could not transform '{source_frame}' to '{target_frame}': {e}")
                if t == 5:
                    self.node.get_logger().error(f"Transform between {source_frame} and {target_frame} does not exist or the data are too old!")
                    return None
                self.node.get_logger().info(f"Trying {5-t} more times ...")
                rate.sleep()

        return transform

    def to_target_frame(self, pose_stamped: PoseStamped, target_frame: str):
        if not self.get_transform(target_frame, pose_stamped.header.frame_id):
            return None
        return self.tf_buffer.transform(pose_stamped, target_frame)

    def to_from_tcp_pose_conversion(self, pose_source_frame: Pose, source_frame: str, target_frame: str, apply_tool_offset: bool=True) -> Pose:
        """apply_tool_tf is used when pose source should be first transformed locally with a tool offset"""

        # frame transforms
        # P_tcp|tgt = tgt_T_src * P_tcp|src
        # tgt_T_src = lookup(target_frame, source_frame) = src_tgt_transform
        # P_tcp|src = src_T_tool * P_tcp|tool = tcp_pose_source_frame
        # src_T_tool = tool_src_transform (from input pose)
        # P_tcp|tool = tcp_pose_tool_frame (from tcp_tool_transform = lookup(tool, tcp))

        src_tgt_transform = self.get_transform(target_frame, source_frame)
        if not src_tgt_transform:
            return None
        if apply_tool_offset:
            # transformation from tcp to tool
            tcp_tool_transform = self.get_transform(self.tool_frame, self.tcp_frame)
            if not tcp_tool_transform:
                return None
            # create a tool pose in tcp_frame to which the transform from source (=tcp) frame can be applied
            tcp_pose_tool_frame = tf2_geometry_msgs.Pose()
            #tcp_pose_tool_frame.header.frame_id = self.tool_frame
            tcp_pose_tool_frame.position.x = tcp_tool_transform.transform.translation.x
            tcp_pose_tool_frame.position.y = tcp_tool_transform.transform.translation.y
            tcp_pose_tool_frame.position.z = tcp_tool_transform.transform.translation.z
            tcp_pose_tool_frame.orientation = tcp_tool_transform.transform.rotation
            # create a transform from source tool to source frame
            tool_src_transform = tf2_geometry_msgs.TransformStamped()
            tool_src_transform.header.frame_id = source_frame
            tool_src_transform.child_frame_id = self.tool_frame
            tool_src_transform.transform.translation.x = pose_source_frame.position.x
            tool_src_transform.transform.translation.y = pose_source_frame.position.y
            tool_src_transform.transform.translation.z = pose_source_frame.position.z
            tool_src_transform.transform.rotation = pose_source_frame.orientation
        else:
            tcp_pose_tool_frame = None
            tool_src_transform = None

        if tcp_pose_tool_frame is not None:
            # get the tcp pose in source frame by applying tool to source frame transform to the tcp in tool frame pose 
            tcp_pose_source_frame = tf2_geometry_msgs.do_transform_pose(tcp_pose_tool_frame, tool_src_transform)
        else:
            # the tcp pose is the source pose
            tcp_pose_source_frame = pose_source_frame
        # apply the target frame to source frame transformation
        pose_target_frame = tf2_geometry_msgs.do_transform_pose(tcp_pose_source_frame, src_tgt_transform)

        return pose_target_frame


    def to_from_tcp_vec3_conversion(self, vec3_source_frame: Vector3Stamped, source_frame: str, target_frame: str) -> Vector3:
        """Apply tf transformation to a vector3"""
        src_tgt_transform = self.get_transform(target_frame, source_frame)
        if not src_tgt_transform:
            return None

        vec3_target_frame = tf2_geometry_msgs.do_transform_vector3(vec3_source_frame, src_tgt_transform)

        return vec3_target_frame.vector
