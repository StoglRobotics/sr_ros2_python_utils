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

import rclpy

import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def _quaternion_from_euler(ai, aj, ak):
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


def _quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = numpy.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


class TCPTransforms:
    def __init__(self, node):
        self.node = node

        # initialize TF2 listener and broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)


    def to_from_tcp_pose_conversion(self, pose_source_frame, source_frame, target_frame, rotation=True):
        """Rotation is used when pose is intended for TCP to be in it"""

        rate = self.node.create_rate(1)
        for t in range(1, 6):
          try:
              transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
              break
          except TransformException as e:
              self.node.get_logger().warn(f"Could not transform '{source_frame}' to '{target_frame}': {e}")
              if t == 5:
                  return None
              self.node.get_logger().info(f"Trying {5-t} more times ...")
              rate.sleep()

        pose_target_frame = tf2_geometry_msgs.do_transform_pose(pose_source_frame, transform)

        if rotation:
            # Rotate for 180° about Y
            q_rot = _quaternion_from_euler(0, 0, math.pi)
            q_in = numpy.empty((4, ))
            q_in[0] = pose_target_frame.orientation.x
            q_in[1] = pose_target_frame.orientation.y
            q_in[2] = pose_target_frame.orientation.z
            q_in[3] = pose_target_frame.orientation.w

            q_in = _quaternion_multiply(q_rot, q_in)

            # Made the gripper look down
            pose_target_frame.orientation.x = q_in[0]
            pose_target_frame.orientation.y = q_in[1]
            pose_target_frame.orientation.z = q_in[2]
            pose_target_frame.orientation.w = q_in[3]

        return pose_target_frame
