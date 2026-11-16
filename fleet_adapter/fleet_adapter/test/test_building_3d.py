# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Unit tests for the geometry and the colours of the 3D building view."""

import io

import numpy as np
import pytest

from ..visualization import building_3d as view


def test_image_points_follow_the_rmf_image_frame():
    # RMF puts pixel (column, row) at (column, -row) * scale from the offset.
    mask = np.zeros((4, 6), dtype=bool)
    mask[0, 0] = mask[3, 5] = True
    points, size = view.image_points(mask, 0.5, cell=0.5, max_points=100)
    assert size == 0.5
    assert points.tolist() == [[0.25, -0.25], [2.75, -1.75]]


def test_image_points_join_pixels_into_cells():
    mask = np.zeros((4, 4), dtype=bool)
    mask[0, 0] = True
    mask[2:, 2:] = True
    points, size = view.image_points(mask, 0.05, cell=0.1, max_points=100)
    assert size == pytest.approx(0.1)
    assert points.ravel().tolist() == pytest.approx([0.05, -0.05, 0.15, -0.15])
    # One pixel in four does not make a floor tile. A full cell does.
    points, _ = view.image_points(mask, 0.05, 0.1, 100, min_fill=0.5)
    assert points.ravel().tolist() == pytest.approx([0.15, -0.15])


def test_image_points_grow_the_cells_to_keep_the_limit():
    mask = np.ones((10, 10), dtype=bool)
    points, size = view.image_points(mask, 1.0, cell=1.0, max_points=10)
    assert len(points) == 9
    assert size == 4.0
    # The image size is not a multiple of the cell. No pixel is lost.
    assert points[-1].tolist() == [10.0, -10.0]


def test_image_points_of_an_empty_mask():
    points, _ = view.image_points(np.zeros((3, 3), dtype=bool), 1.0, 1.0, 10)
    assert points.shape == (0, 2)


def test_decode_gray_reads_a_png_of_a_robot_map():
    image = pytest.importorskip('PIL.Image')
    pixels = np.array([[0, 205], [254, 254], [205, 0]], dtype=np.uint8)
    data = io.BytesIO()
    image.fromarray(pixels).save(data, format='PNG')
    gray = view.decode_gray(list(data.getvalue()))
    assert gray.tolist() == pixels.tolist()
    assert (gray < view.OCCUPIED_BELOW).sum() == 2
    assert (gray > view.FREE_ABOVE).sum() == 2


def test_lift_car_is_at_the_elevation_of_its_current_floor():
    elevations = {'L7': -5.0, 'L8': 0.0}
    assert view.lift_car_z('L7', elevations, 0.0) == -5.0
    assert view.lift_car_z('L8', elevations, -5.0) == 0.0
    # No lift state yet, or a floor that the building does not have.
    assert view.lift_car_z('', elevations, -5.0) == -5.0


def test_ease_toward_moves_part_of_the_way_and_stops_at_the_target():
    assert view.ease_toward(-5.0, 0.0, 0.2) == pytest.approx(-4.0)
    assert view.ease_toward(0.0, -5.0, 0.2) == pytest.approx(-1.0)
    assert view.ease_toward(-0.005, 0.0) == 0.0
    assert view.ease_toward(3.0, 3.0) == 3.0
    # A lift trip of 10 m is drawn in about 3 seconds at 10 steps a second.
    z = 0.0
    for _ in range(35):
        z = view.ease_toward(z, 10.0)
    assert z == 10.0


@pytest.mark.parametrize('mode, name', [
    (0, 'idle'), (1, 'charging'), (2, 'moving'), (7, 'docking'),
    (8, 'error'), (9, 'cleaning'), (99, 'unknown')])
def test_mode_style_names_the_robot_mode(mode, name):
    assert view.mode_style(mode)[0] == name


def test_an_idle_robot_with_a_task_is_shown_as_on_task():
    assert view.mode_style(0, 'patrol.dispatch-1') == (
        'on task', view.mode_style(2)[1])
    assert view.mode_style(9, 'clean.dispatch-2')[0] == 'cleaning'


def test_the_main_modes_have_their_own_colour():
    colours = {view.mode_style(mode)[1] for mode in (0, 1, 2, 7, 8, 9)}
    assert len(colours) == 6
