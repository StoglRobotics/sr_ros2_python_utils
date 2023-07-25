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

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class VisualizatonPublisher:

    def __init__(self, node:Node) -> None:
        """
        Create and prepare class by providing a node.

        :param node: Node used for access to the rest of the ROS 2 system.
        """
        self.node = node
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.tf_static_broadcasters = {}
        self.tf_static_broadcaster = StaticTransformBroadcaster(self.node)


    def publish_pose_as_transform(self, pose:Pose, frame_id:str, child_frame_id:str, is_static:bool=False) -> bool:
        """
        Publish a Pose as a Transform to visualize with Rviz2 using "Axis" display.

        :param pose: Pose to visualize.
        :param frame_id: Frame in which pose is given.
        :param child_frame_id: Name of the frame that Pose defines.
        """

        trafo = TransformStamped()
        trafo.header.stamp = self.node.get_clock().now().to_msg()
        trafo.header.frame_id = frame_id
        trafo.child_frame_id = child_frame_id

        trafo.transform.translation.x = pose.position.x
        trafo.transform.translation.y = pose.position.y
        trafo.transform.translation.z = pose.position.z
        trafo.transform.rotation.x = pose.orientation.x
        trafo.transform.rotation.y = pose.orientation.y
        trafo.transform.rotation.z = pose.orientation.z
        trafo.transform.rotation.w = pose.orientation.w

        if is_static:
            self.tf_static_broadcaster.sendTransform(trafo)
        else:
            self.tf_broadcaster.sendTransform(trafo)


        return True


    def publish_pose_stamped_as_transform(self, pose:PoseStamped, child_frame_id:str, is_static:bool=False) -> bool:
        """
        Publish a Stamped Pose as a Transform to visualize with Rviz2 using "Axis" display.

        :param pose: PoseStamped to visualize. `frame_id` in the header defines its frame.
        :param child_frame_id: Name of the frame that Pose defines.
        :returns: False if input data are not valid.
        """

        if (not pose.header.frame_id):
            self.node.get_logger().error("To publish pose stamped as transform `frame_id` in the header has to be set!")
            return False

        return self.publish_pose_as_transform(pose.pose, pose.header.frame_id, child_frame_id, is_static)
