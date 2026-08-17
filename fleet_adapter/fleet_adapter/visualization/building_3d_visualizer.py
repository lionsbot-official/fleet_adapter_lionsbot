"""Publish RMF building walls as RViz 3D markers."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rmf_building_map_msgs.msg import BuildingMap
from visualization_msgs.msg import Marker, MarkerArray
from .colors import DOOR_CYAN, WALL_GRAY


class Building3DVisualizer(Node):
    def __init__(self):
        super().__init__('building_3d_visualizer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('floor_name', 'L8')
        self.declare_parameter('wall_height', 2.5)
        self.declare_parameter('wall_width', 0.12)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, '/building_3d_markers', marker_qos)
        self.subscription = self.create_subscription(
            BuildingMap, self.get_parameter('map_topic').value, self._map_callback, map_qos)

    @staticmethod
    def _param(params, name, default):
        for param in params:
            if param.name == name:
                if param.type == param.TYPE_DOUBLE:
                    return param.value_float
                if param.type == param.TYPE_INT:
                    return param.value_int
        return default

    def _map_callback(self, building_map):
        level_name = self.get_parameter('floor_name').value
        level = next((item for item in building_map.levels if item.name == level_name), None)
        if level is None:
            self.get_logger().warning(f'Level {level_name!r} was not found in /map')
            return

        markers = MarkerArray()
        for index, edge in enumerate(level.wall_graph.edges):
            if edge.v1_idx >= len(level.wall_graph.vertices) or edge.v2_idx >= len(level.wall_graph.vertices):
                continue
            start = level.wall_graph.vertices[edge.v1_idx]
            end = level.wall_graph.vertices[edge.v2_idx]
            dx, dy = end.x - start.x, end.y - start.y
            length = math.hypot(dx, dy)
            if length < 1e-4:
                continue
            height = self._param(edge.params, 'texture_height', self.get_parameter('wall_height').value)
            marker = Marker()
            marker.header.frame_id = level_name
            marker.ns = f'building_walls/{level_name}'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (start.x + end.x) / 2.0
            marker.pose.position.y = (start.y + end.y) / 2.0
            marker.pose.position.z = level.elevation + height / 2.0
            marker.pose.orientation.z = math.sin(math.atan2(dy, dx) / 2.0)
            marker.pose.orientation.w = math.cos(math.atan2(dy, dx) / 2.0)
            marker.scale.x = length
            marker.scale.y = self.get_parameter('wall_width').value
            marker.scale.z = height
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*WALL_GRAY, 0.85)
            marker.lifetime.sec = 0
            markers.markers.append(marker)

        # Render door leaves as thin, translucent panels. Door state markers from
        # rmf_visualization_building_systems can still be displayed separately.
        door_height = self.get_parameter('wall_height').value
        for index, door in enumerate(level.doors):
            dx, dy = door.v2_x - door.v1_x, door.v2_y - door.v1_y
            length = math.hypot(dx, dy)
            if length < 1e-4:
                continue
            marker = Marker()
            marker.header.frame_id = level_name
            marker.ns = f'building_doors/{level_name}'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (door.v1_x + door.v2_x) / 2.0
            marker.pose.position.y = (door.v1_y + door.v2_y) / 2.0
            marker.pose.position.z = level.elevation + door_height / 2.0
            marker.pose.orientation.z = math.sin(math.atan2(dy, dx) / 2.0)
            marker.pose.orientation.w = math.cos(math.atan2(dy, dx) / 2.0)
            marker.scale.x = length
            marker.scale.y = 0.06
            marker.scale.z = door_height
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*DOOR_CYAN, 0.65)
            marker.lifetime.sec = 0
            markers.markers.append(marker)
        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Building3DVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
