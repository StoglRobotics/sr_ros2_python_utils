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

from typing import Tuple

from builtin_interfaces.msg import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Pose, TransformStamped, Vector3Stamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker


class VisualizatonPublisher:

    def __init__(
        self, node: Node, marker_publisher_name: str = "visualization_marker_publisher"
    ) -> None:
        """
        Create and prepare class by providing a node.

        :param node: Node used for access to the rest of the ROS 2 system.
        """
        self.node = node
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.tf_static_broadcasters = {}
        self.marker_publisher_name = marker_publisher_name
        self.marker_publisher = node.create_publisher(Marker, self.marker_publisher_name, 10)

    def publish_vector(
        self,
        vec: Vector3Stamped,
        start_point: Point = Point(),
        lifetime: Duration = None,
        id: int = None,
        namespace: str = "",
        marker_color: Tuple[float, float, float, float] = (0.4, 0.18, 0.56862, 1.0),
        scale: Tuple[float, float, float] = (0.1, 0.2, 0.3),
    ):
        if not vec.header.frame_id:
            self.node.get_logger().error(f"Publishing of vector failed. No frame id given.")
            return

        marker = Marker()
        marker.header.frame_id = vec.header.frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = namespace if namespace else self.node.get_namespace()
        used_id = id
        # create a unique id based on time if none is given
        if not used_id:
            seconds, nanoseconds = self.node.get_clock().now().seconds_nanoseconds()
            ms = seconds * 1000 + nanoseconds // 1_000_000
            used_id = int(ms) & 0x7FFFFFF
            print(f"seconds= {seconds}")
            print(f"nanoseconds={nanoseconds}")
            print(f"ms={ms}")
            print(f"ms={used_id}")
        marker.id = used_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        end_point = Point()
        end_point.x = vec.vector.x  # Length of the vector
        end_point.y = vec.vector.y
        end_point.z = vec.vector.z

        marker.points.append(start_point)
        marker.points.append(end_point)

        # Set the color (RGBA)
        marker.color.r = marker_color[0]
        marker.color.g = marker_color[1]
        marker.color.b = marker_color[2]
        marker.color.a = marker_color[3]

        # Set the scale of the vector (shaft diameter, head diameter, head length)
        marker.scale.x = scale[0]  # Shaft diameter
        marker.scale.y = scale[1]  # Head diameter
        marker.scale.z = scale[2]  # Head length

        # Set the lifetime of the marker
        if lifetime:
            marker.lifetime = lifetime

        # Publish the marker
        self.marker_publisher.publish(marker)

    def publish_pose_as_transform(
        self, pose: Pose, frame_id: str, child_frame_id: str, is_static: bool = False
    ) -> bool:
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
            # check if the transform already exists
            pair = (frame_id, child_frame_id)
            tf_exists = False
            for key in self.tf_static_broadcasters.keys():
                if pair == key:
                    tf_exists = True
                    break
            if not tf_exists:
                # create a new static broadcaster
                self.tf_static_broadcasters[pair] = StaticTransformBroadcaster(self.node)
            self.tf_static_broadcasters[pair].sendTransform(trafo)
        else:
            self.tf_broadcaster.sendTransform(trafo)
        return True

    def publish_pose_stamped_as_transform(
        self, pose: PoseStamped, child_frame_id: str, is_static: bool = False
    ) -> bool:
        """
        Publish a Stamped Pose as a Transform to visualize with Rviz2 using "Axis" display.

        :param pose: PoseStamped to visualize. `frame_id` in the header defines its frame.
        :param child_frame_id: Name of the frame that Pose defines.
        :returns: False if input data are not valid.
        """

        if not pose.header.frame_id:
            self.node.get_logger().error(
                "To publish pose stamped as transform `frame_id` in the header has to be set!"
            )
            return False

        return self.publish_pose_as_transform(
            pose.pose, pose.header.frame_id, child_frame_id, is_static
        )
