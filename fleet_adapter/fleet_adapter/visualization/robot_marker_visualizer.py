"""Custom RViz-only fleet and schedule markers."""
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rmf_fleet_msgs.msg import FleetState
from visualization_msgs.msg import Marker, MarkerArray
from .colors import (NOSE_BLUE, R3_BLUE,
                      R3_SCHEDULE_BLUE, R5_BLUE,
                      R5_SCHEDULE_BLUE)


class RobotMarkerVisualizer(Node):
    def __init__(self):
        super().__init__('robot_marker_visualizer')
        self.declare_parameter('fleet_states_topic', '/fleet_states')
        self.declare_parameter('schedule_topic', '/schedule_markers')
        self.declare_parameter('map_markers_topic', '/map_markers')
        self.fleet_pub = self.create_publisher(MarkerArray, '/custom_fleet_markers', 1)
        self.schedule_pub = self.create_publisher(MarkerArray, '/custom_schedule_markers', 1)
        self.schedule_path_pub = self.create_publisher(MarkerArray, '/custom_schedule_paths', 1)
        self.create_subscription(FleetState, self.get_parameter('fleet_states_topic').value,
                                 self._fleet_callback, 10)
        # The RMF schedule visualizer publishes live updates. Subscribe with
        # volatile durability so this remains compatible with that publisher.
        schedule_qos = QoSProfile(depth=10)
        schedule_qos.reliability = ReliabilityPolicy.RELIABLE
        schedule_qos.durability = DurabilityPolicy.VOLATILE
        map_output_qos = QoSProfile(depth=10)
        map_output_qos.reliability = ReliabilityPolicy.RELIABLE
        map_output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_markers_pub = self.create_publisher(
            MarkerArray, '/custom_map_markers', map_output_qos)
        self.create_subscription(MarkerArray, self.get_parameter('schedule_topic').value,
                                 self._schedule_callback, schedule_qos)
        map_input_qos = QoSProfile(depth=10)
        map_input_qos.reliability = ReliabilityPolicy.RELIABLE
        map_input_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(MarkerArray, self.get_parameter('map_markers_topic').value,
                                 self._map_markers_callback, map_input_qos)

    @staticmethod
    def _marker(*, ns, marker_id, robot, shape, color, scale, z):
        m = Marker()
        m.header.frame_id = robot.location.level_name
        m.ns, m.id, m.type, m.action = ns, marker_id, shape, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = robot.location.x, robot.location.y, z
        m.pose.orientation.z = math.sin(robot.location.yaw / 2.0)
        m.pose.orientation.w = math.cos(robot.location.yaw / 2.0)
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = (*color, 1.0)
        return m

    def _fleet_callback(self, fleet):
        result = MarkerArray()
        for index, robot in enumerate(fleet.robots):
            is_r5 = fleet.name.lower().startswith('r5')
            if is_r5:
                body_shape = Marker.CYLINDER
                body_color = R5_BLUE
                body_width, body_depth, body_height = 0.6, 0.6, 0.9
                nose_color = NOSE_BLUE
                nose_width, nose_depth, nose_height = 0.25, 0.25, 0.25
            else:
                body_shape = Marker.CUBE
                body_color = R3_BLUE
                body_width, body_depth, body_height = 0.4, 0.4, 0.6
                nose_color = NOSE_BLUE
                nose_width, nose_depth, nose_height = 0.2, 0.2, 0.2
            nose_overlap = nose_width * 0.75
            body = self._marker(
                ns=fleet.name, marker_id=index, robot=robot, shape=body_shape,
                color=body_color, scale=(body_width, body_depth, body_height),
                z=body_height / 2.0)
            result.markers.append(body)
            nose = self._marker(
                ns=f'{fleet.name}_nose', marker_id=index, robot=robot,
                shape=Marker.CUBE, color=nose_color,
                scale=(nose_width, nose_depth, nose_height),
                z=body_height * 0.75)
            nose_forward_distance = body_width / 2.0 + nose_width / 2.0 - nose_overlap
            nose.pose.position.x += math.cos(robot.location.yaw) * nose_forward_distance
            nose.pose.position.y += math.sin(robot.location.yaw) * nose_forward_distance
            result.markers.append(nose)
        self.fleet_pub.publish(result)

    def _schedule_callback(self, incoming):
        scheduled_discs = MarkerArray()
        scheduled_paths = MarkerArray()
        seen_disks = set()
        scheduled_disc_diameter = 0.5
        scheduled_disc_thickness = 0.01
        scheduled_path_width_factor = 0.4
        for source_marker in incoming.markers:
            source_marker.header.frame_id = source_marker.header.frame_id or 'map'
            # Keep schedule paths/disks above map lanes and waypoints.
            if source_marker.type in (Marker.SPHERE, Marker.CYLINDER):
                disk_key = (source_marker.ns, source_marker.id)
                if disk_key in seen_disks:
                    continue
                seen_disks.add(disk_key)
                # RMF may publish the predicted position as a layered sphere.
                # Replace it with one thin floor disk.
                scheduled_disc = source_marker
                scheduled_disc.type = Marker.CYLINDER
                scheduled_disc.pose.position.z = 0.03
                scheduled_disc.scale.x = scheduled_disc_diameter
                scheduled_disc.scale.y = scheduled_disc_diameter
                scheduled_disc.scale.z = scheduled_disc_thickness
                scheduled_disc.color.r, scheduled_disc.color.g, scheduled_disc.color.b, scheduled_disc.color.a = (
                    (*R5_SCHEDULE_BLUE, 0.5)
                    if 'r5' in scheduled_disc.ns.lower()
                    else (*R3_SCHEDULE_BLUE, 0.5))
                scheduled_discs.markers.append(scheduled_disc)
            else:
                scheduled_path = source_marker
                scheduled_path.scale.x *= scheduled_path_width_factor
                scheduled_path.pose.position.z = 0.015
                for point in scheduled_path.points:
                    point.z = 0.0
                scheduled_paths.markers.append(scheduled_path)
        self.schedule_pub.publish(scheduled_discs)
        self.schedule_path_pub.publish(scheduled_paths)

    def _map_markers_callback(self, incoming):
        result = MarkerArray()
        lane_width = 0.2
        waypoint_diameter = lane_width * 1.5
        waypoint_thickness = 0.0005
        for marker in incoming.markers:
            namespace = marker.ns.lower()
            if 'lanes/' in namespace:
                marker.scale.x = lane_width
                marker.scale.y = lane_width
                marker.pose.position.z = 0.0
                for point in marker.points:
                    point.z = 0.0
                lane_color = R5_BLUE if namespace.startswith('r5/') else R3_BLUE
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*lane_color, 0.15)
            elif 'waypoints/' in namespace:
                waypoint_color = R5_BLUE if namespace.startswith('r5/') else R3_BLUE
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*waypoint_color, 0.25)
                if marker.type in (Marker.SPHERE_LIST, Marker.POINTS):
                    for point_index, point in enumerate(marker.points):
                        disk = Marker()
                        disk.header = marker.header
                        disk.ns = marker.ns
                        disk.id = marker.id * 10000 + point_index
                        disk.type = Marker.CYLINDER
                        disk.action = Marker.ADD
                        disk.pose = copy.deepcopy(marker.pose)
                        disk.pose.position.x += point.x
                        disk.pose.position.y += point.y
                        disk.pose.position.z += point.z + 0.01
                        disk.scale.x = waypoint_diameter
                        disk.scale.y = waypoint_diameter
                        disk.scale.z = waypoint_thickness
                        disk.color = marker.color
                        result.markers.append(disk)
                    continue
                else:
                    marker.type = Marker.CYLINDER
                    marker.pose.position.z = 0.01
                    marker.scale.x, marker.scale.y, marker.scale.z = waypoint_diameter, waypoint_diameter, waypoint_thickness
            elif 'labels/' in namespace:
                marker.scale.z *= 0.65
                if namespace.startswith('r5/') or namespace.startswith('r5_'):
                    marker.color.r, marker.color.g, marker.color.b = R5_BLUE
                elif namespace.startswith('r3/') or namespace.startswith('r3_'):
                    marker.color.r, marker.color.g, marker.color.b = R3_BLUE
            result.markers.append(marker)
        self.map_markers_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMarkerVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
