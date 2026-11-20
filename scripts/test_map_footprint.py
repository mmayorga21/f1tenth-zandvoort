"""Check full-body contacts, rotated corners, and pixel/world orientation."""
import math

import numpy as np
from PIL import Image
import pytest
import yaml

from map_footprint import MapFootprint, rectangles_overlap


@pytest.fixture
def footprint(tmp_path):
    pixels = np.full((100, 100), 255, dtype=np.uint8)
    pixels[49, 50] = 0  # World square [5, 5.1] x [5, 5.1].
    Image.fromarray(pixels).save(tmp_path/'map.png')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(
        image='map.png', resolution=.1, origin=[0., 0., 0.],
        negate=0, occupied_thresh=.45)))
    return MapFootprint(tmp_path/'map', .58, .31)


def test_front_contact_with_center_outside_obstacle(footprint):
    assert footprint.collides(4.8, 5.05, 0.)
    assert not footprint.collides(4.7, 5.05, 0.)


def test_rotated_body_and_image_row_direction(footprint):
    assert footprint.collides(5.05, 4.8, math.pi/2)
    assert not footprint.collides(5.05, 4.7, math.pi/2)


def test_bounding_box_overlap_is_not_always_body_contact(footprint):
    assert not footprint.collides(4.75, 4.75, math.pi/4)
    assert footprint.collides(4.85, 4.85, math.pi/4)


def test_touching_an_occupied_pixel_boundary_is_contact(footprint):
    assert footprint.collides(5.39, 5.05, 0.)


def test_invalid_pose_and_map_boundary_are_rejected(footprint):
    assert footprint.collides(float('nan'), 5., 0.)
    assert footprint.collides(.1, .1, 0.)


@pytest.mark.parametrize('other,expected', [
    ([1., 0., 0.], False), ([.58, 0., 0.], True),
    ([.3, .1, math.pi/2], True), ([.5, .5, math.pi/4], False)])
def test_two_vehicle_body_contact(other, expected):
    assert rectangles_overlap([0., 0., 0.], other, .58, .31) is expected
