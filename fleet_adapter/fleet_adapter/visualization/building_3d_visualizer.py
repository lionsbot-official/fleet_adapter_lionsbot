"""
Publish the whole RMF building as RViz 3D markers in the map frame.

/building_3d_markers has every level at its elevation: the floor plan image,
the walls, the doors, the lanes, the level name and the lift shafts.
/building_3d_live_markers has the lift cars and the robots.
The node also publishes one tf frame for each level, at the level elevation.
"""

from collections import deque
import math
import time

from geometry_msgs.msg import Point, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rmf_building_map_msgs.msg import BuildingMap
from rmf_fleet_msgs.msg import FleetState
from rmf_lift_msgs.msg import LiftState
from std_msgs.msg import ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .building_3d import decode_gray, DOOR_COLOURS, ease_toward, FREE_ABOVE
from .building_3d import image_points, lift_car_z, mode_style, OCCUPIED_BELOW
from .colors import DOOR_CYAN, R3_BLUE, WALL_GRAY

PERIOD = 0.1          # seconds between two live marker messages
STALE_SECONDS = 5.0   # a robot with no newer fleet state is not drawn
MAP_BLOCK_HEIGHT = 1.0
CAR_HEIGHT = 2.2
FLOOR_COLOUR = (0.85, 0.87, 0.9)
SHAFT_COLOUR = (1.0, 0.75, 0.1)


def make_marker(ns, marker_id, shape, xyz, yaw=0.0, scale=(1.0, 1.0, 1.0),
                rgb=(1.0, 1.0, 1.0), alpha=1.0):
    m = Marker()
    m.header.frame_id = 'map'
    m.ns, m.id, m.type = ns, marker_id, shape
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
    m.pose.orientation.z = math.sin(yaw / 2.0)
    m.pose.orientation.w = math.cos(yaw / 2.0)
    m.scale.x, m.scale.y, m.scale.z = map(float, scale)
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = alpha
    return m


def edge_marker(ns, marker_id, start, end, z, width, height, rgb, alpha):
    """Make an upright panel from one point to another, or None."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-4:
        return None
    centre = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0,
              z + height / 2.0)
    return make_marker(ns, marker_id, Marker.CUBE, centre, math.atan2(dy, dx),
                       (length, width, height), rgb, alpha)


class Building3DVisualizer(Node):

    def __init__(self):
        super().__init__('building_3d_visualizer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('wall_height', 2.5)
        self.declare_parameter('wall_width', 0.12)
        self.declare_parameter('cell_size', 0.1)
        self.declare_parameter('max_cubes', 20000)
        self.declare_parameter('trail_length', 150)
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.elevations = {}
        self.lifts = {}
        self.lift_states = {}
        self.robots = {}   # (fleet, robot) -> (time, robot state)
        self.heights = {}  # drawn z of each lift car and robot
        self.trails = {}
        self.tf = StaticTransformBroadcaster(self)
        self.publisher = self.create_publisher(
            MarkerArray, '/building_3d_markers', latched)
        self.live_publisher = self.create_publisher(
            MarkerArray, '/building_3d_live_markers', 10)
        self.create_subscription(
            BuildingMap, self.get_parameter('map_topic').value,
            self._map_callback, latched)
        self.create_subscription(
            FleetState, '/fleet_states', self._fleet_callback, 10)
        self.create_subscription(
            LiftState, '/lift_states', self._lift_callback, 10)
        self.create_timer(PERIOD, self._publish_live)

    @staticmethod
    def _param(params, name, default):
        for param in params:
            if param.name == name:
                if param.type == param.TYPE_DOUBLE:
                    return param.value_float
                if param.type == param.TYPE_INT:
                    return param.value_int
        return default

    def _image_markers(self, level, image):
        """Make the floor tiles and the wall blocks of one floor plan."""
        try:
            gray = decode_gray(image.data)
        except Exception as error:
            self.get_logger().warning(
                f'Cannot read the image of level {level.name}: {error}')
            return []
        cell = self.get_parameter('cell_size').value
        limit = self.get_parameter('max_cubes').value
        result = []
        for name, mask, cell, min_fill, height, rgb, alpha in (
                ('floor', gray > FREE_ABOVE, 2 * cell, 0.5, 0.02,
                 FLOOR_COLOUR, 0.5),
                ('map', gray < OCCUPIED_BELOW, cell, 0.0, MAP_BLOCK_HEIGHT,
                 WALL_GRAY, 0.9)):
            points, size = image_points(
                mask, image.scale, cell, limit, min_fill)
            if not len(points):
                continue
            marker = make_marker(
                f'{name}/{level.name}', 0, Marker.CUBE_LIST,
                (image.x_offset, image.y_offset, level.elevation), image.yaw,
                (size, size, height), rgb, alpha)
            marker.points = [
                Point(x=x, y=y, z=height / 2.0) for x, y in points.tolist()]
            result.append(marker)
        return result

    def _level_markers(self, level):
        z = level.elevation
        wall_height = self.get_parameter('wall_height').value
        wall_width = self.get_parameter('wall_width').value
        result = []
        for image in level.images:
            result += self._image_markers(level, image)
        vertices = level.wall_graph.vertices
        for index, edge in enumerate(level.wall_graph.edges):
            if max(edge.v1_idx, edge.v2_idx) >= len(vertices):
                continue
            start, end = vertices[edge.v1_idx], vertices[edge.v2_idx]
            result.append(edge_marker(
                f'walls/{level.name}', index, (start.x, start.y),
                (end.x, end.y), z, wall_width,
                self._param(edge.params, 'texture_height', wall_height),
                WALL_GRAY, 0.85))
        for index, door in enumerate(level.doors):
            result.append(edge_marker(
                f'doors/{level.name}', index, (door.v1_x, door.v1_y),
                (door.v2_x, door.v2_y), z, 0.06, wall_height, DOOR_CYAN, 0.65))
        label_at = (0.0, 0.0)
        for index, graph in enumerate(level.nav_graphs):
            lanes = make_marker(
                f'lanes/{level.name}', index, Marker.LINE_LIST, (0, 0, z),
                scale=(0.08, 0, 0), rgb=R3_BLUE, alpha=0.8)
            for edge in graph.edges:
                for vertex_index in (edge.v1_idx, edge.v2_idx):
                    vertex = graph.vertices[vertex_index]
                    lanes.points.append(Point(x=vertex.x, y=vertex.y, z=0.05))
            if lanes.points:
                label_at = (lanes.points[0].x, lanes.points[0].y)
                result.append(lanes)
        if level.images:
            label_at = (level.images[0].x_offset, level.images[0].y_offset)
        label = make_marker(
            f'labels/{level.name}', 0, Marker.TEXT_VIEW_FACING,
            (*label_at, z + wall_height + 0.5), scale=(0, 0, 1.2))
        label.text = level.name
        result.append(label)
        return [marker for marker in result if marker is not None]

    def _lift_span(self, lift):
        """Give the lowest and the highest elevation that a lift serves."""
        served = [self.elevations[name] for name in lift.levels
                  if name in self.elevations] or [0.0]
        return min(served), max(served)

    def _map_callback(self, building_map):
        self.elevations = {
            level.name: level.elevation for level in building_map.levels}
        self.lifts = {lift.name: lift for lift in building_map.lifts}
        transforms = []
        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))
        for level in building_map.levels:
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = level.name
            transform.transform.translation.z = float(level.elevation)
            transforms.append(transform)
            markers.markers += self._level_markers(level)
        wall_height = self.get_parameter('wall_height').value
        for index, lift in enumerate(building_map.lifts):
            bottom, top = self._lift_span(lift)
            height = top - bottom + wall_height
            markers.markers.append(make_marker(
                'lift_shafts', index, Marker.CUBE,
                (lift.ref_x, lift.ref_y, bottom + height / 2.0), lift.ref_yaw,
                (lift.width, lift.depth, height), SHAFT_COLOUR, 0.15))
        self.tf.sendTransform(transforms)
        self.publisher.publish(markers)
        self.get_logger().info(
            f'3D view: {len(building_map.levels)} levels, '
            f'{len(building_map.lifts)} lifts, {len(markers.markers)} '
            f'markers, {sum(len(m.points) for m in markers.markers)} points')

    def _fleet_callback(self, fleet):
        for robot in fleet.robots:
            self.robots[(fleet.name, robot.name)] = (time.monotonic(), robot)

    def _lift_callback(self, state):
        self.lift_states[state.lift_name] = state

    def _eased(self, key, target):
        """Move the drawn height of a lift car or a robot toward its level."""
        z = ease_toward(self.heights.get(key, target), target)
        self.heights[key] = z
        return z

    def _robot_markers(self, fleet_name, robot, z):
        """Make the body, the heading arrow, the label and the trail."""
        ns = f'robots/{fleet_name}/{robot.name}'
        mode, rgb = mode_style(robot.mode.mode, robot.task_id)
        is_r5 = fleet_name.lower().startswith('r5')
        shape = Marker.CYLINDER if is_r5 else Marker.CUBE
        width, height = (0.6, 0.9) if is_r5 else (0.4, 0.6)
        x, y, yaw = robot.location.x, robot.location.y, robot.location.yaw
        label = make_marker(
            ns, 2, Marker.TEXT_VIEW_FACING, (x, y, z + height + 0.7),
            scale=(0, 0, 0.45))
        label.text = f'{robot.name}\n{mode}'
        result = [
            make_marker(ns, 0, shape, (x, y, z + height / 2.0), yaw,
                        (width, width, height), rgb),
            make_marker(ns, 1, Marker.ARROW, (x, y, z + height + 0.1), yaw,
                        (1.0, 0.12, 0.12), rgb),
            label]
        trail = self.trails.setdefault(ns, deque(
            maxlen=self.get_parameter('trail_length').value))
        here = (x, y, z + 0.1)
        if not trail or math.dist(here, trail[-1]) > 0.1:
            trail.append(here)
        if len(trail) > 1:
            line = make_marker(
                ns, 3, Marker.LINE_STRIP, (0, 0, 0), scale=(0.1, 0, 0))
            for index, (px, py, pz) in enumerate(trail):
                line.points.append(Point(x=px, y=py, z=pz))
                line.colors.append(ColorRGBA(
                    r=rgb[0], g=rgb[1], b=rgb[2], a=(index + 1) / len(trail)))
            result.append(line)
        return result

    def _publish_live(self):
        markers = []
        for index, (name, lift) in enumerate(sorted(self.lifts.items())):
            state = self.lift_states.get(name, LiftState())
            z = self._eased(('lift', name), lift_car_z(
                state.current_floor, self.elevations,
                self._lift_span(lift)[0]))
            markers.append(make_marker(
                'lift_cars', index, Marker.CUBE,
                (lift.ref_x, lift.ref_y, z + CAR_HEIGHT / 2.0), lift.ref_yaw,
                (lift.width * 0.9, lift.depth * 0.9, CAR_HEIGHT),
                DOOR_COLOURS.get(state.door_state, WALL_GRAY), 0.6))
        now = time.monotonic()
        for key, (stamp, robot) in self.robots.items():
            level = robot.location.level_name
            if now - stamp > STALE_SECONDS or level not in self.elevations:
                continue
            z = self._eased(key, self.elevations[level])
            markers += self._robot_markers(key[0], robot, z)
        for marker in markers:
            marker.lifetime.sec = 1
        self.live_publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = Building3DVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
