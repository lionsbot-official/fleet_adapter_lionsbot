"""Geometry and colours of the 3D building view. This file imports no ROS."""

import io

import numpy as np

OCCUPIED_BELOW = 100  # grey value of a wall pixel in a robot map
FREE_ABOVE = 230      # grey value of a free pixel. 205 is unknown space

# rmf_fleet_msgs/RobotMode value: (name on the label, colour of the robot)
MODE_STYLES = {
    0: ('idle', (0.55, 0.60, 0.65)),
    1: ('charging', (1.0, 0.85, 0.1)),
    2: ('moving', (0.2, 0.6, 1.0)),
    3: ('paused', (1.0, 0.55, 0.1)),
    4: ('waiting', (1.0, 0.55, 0.1)),
    5: ('emergency', (1.0, 0.1, 0.1)),
    6: ('going home', (0.2, 0.6, 1.0)),
    7: ('docking', (0.7, 0.4, 1.0)),
    8: ('error', (1.0, 0.1, 0.1)),
    9: ('cleaning', (0.1, 0.85, 0.4)),
}
DOOR_COLOURS = {  # rmf_lift_msgs/LiftState door_state
    0: (0.9, 0.2, 0.2),   # closed
    1: (1.0, 0.75, 0.1),  # moving
    2: (0.2, 0.9, 0.3),   # open
}


def mode_style(mode, task_id=''):
    """Give the label and the colour of a robot mode."""
    if mode == 0 and task_id:
        # RMF can report the idle mode for a robot that does a task.
        return ('on task', MODE_STYLES[2][1])
    return MODE_STYLES.get(mode, ('unknown', (1.0, 1.0, 1.0)))


def decode_gray(data):
    """Decode the image bytes of an AffineImage to a 2D grey array."""
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(bytes(data))).convert('L'))


def image_points(mask, scale, cell, max_points, min_fill=0.0):
    """
    Give the centres of the image cells where the mask is set.

    The image is cut into square cells of about `cell` metres. The cells grow
    until there are at most `max_points`. A cell counts when more than
    `min_fill` of its pixels are set. The points are in the image frame as RMF
    defines it: x to the right and y up, in metres, from the top left pixel.
    A marker at (x_offset, y_offset) with the image yaw puts them in the map.
    It returns the points as an (N, 2) array and the cell size in metres.
    """
    factor = max(1, round(cell / scale))
    while True:
        rows, cols = (-(-size // factor) for size in mask.shape)
        padded = np.zeros((rows * factor, cols * factor), dtype=bool)
        padded[:mask.shape[0], :mask.shape[1]] = mask
        fill = padded.reshape(rows, factor, cols, factor).mean(axis=(1, 3))
        row, col = np.nonzero(fill > min_fill)
        if len(row) <= max_points:
            break
        factor += 1
    points = np.column_stack((col + 0.5, -(row + 0.5))) * factor * scale
    return points, factor * scale


def ease_toward(value, target, fraction=0.2):
    """Move a value a fraction of the way to its target. Stop when close."""
    if abs(target - value) < 0.01:
        return target
    return value + (target - value) * fraction


def lift_car_z(current_floor, elevations, default):
    """Give the elevation of the floor that a lift state reports."""
    return elevations.get(current_floor, default)
