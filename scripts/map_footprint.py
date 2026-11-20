"""Independent rectangle-to-occupied-pixel check, used only for validation."""
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def rectangles_overlap(pose_a, pose_b, length, width):
    """Separating-axis test for two vehicle bodies at the same physics time."""
    xa, ya, aa = pose_a
    xb, yb, ab = pose_b
    axes_a = [(math.cos(aa), math.sin(aa)), (-math.sin(aa), math.cos(aa))]
    axes_b = [(math.cos(ab), math.sin(ab)), (-math.sin(ab), math.cos(ab))]
    for ax, ay in axes_a + axes_b:
        ra = length/2*abs(ax*axes_a[0][0]+ay*axes_a[0][1]) + width/2*abs(ax*axes_a[1][0]+ay*axes_a[1][1])
        rb = length/2*abs(ax*axes_b[0][0]+ay*axes_b[0][1]) + width/2*abs(ax*axes_b[1][0]+ay*axes_b[1][1])
        if abs((xb-xa)*ax+(yb-ya)*ay) > ra+rb:
            return False
    return True


class MapFootprint:
    def __init__(self, map_path, length, width):
        path = Path(map_path).with_suffix('.yaml')
        metadata = yaml.safe_load(path.read_text())
        if metadata['origin'][2] != 0 or metadata['negate'] != 0:
            raise ValueError('Footprint validation requires an unrotated, non-inverted map')
        image = np.asarray(Image.open(path.parent / metadata['image']).convert('L'))
        self.occupied = np.flipud((255. - image) / 255. > metadata['occupied_thresh'])
        self.resolution = float(metadata['resolution'])
        self.ox, self.oy = metadata['origin'][:2]
        self.half_length, self.half_width = length / 2, width / 2
        self.samples = 0

    def collides(self, x, y, yaw):
        self.samples += 1
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return True
        c, s = math.cos(yaw), math.sin(yaw)
        ac, ass = abs(c), abs(s)
        ex = self.half_length * ac + self.half_width * ass
        ey = self.half_length * ass + self.half_width * ac
        r = self.resolution
        height, width = self.occupied.shape
        if x-ex < self.ox or y-ey < self.oy or x+ex > self.ox+width*r or y+ey > self.oy+height*r:
            return True
        left = max(0, math.floor((x-ex-self.ox)/r)-1)
        right = min(width-1, math.floor((x+ex-self.ox)/r)+1)
        bottom = max(0, math.floor((y-ey-self.oy)/r)-1)
        top = min(height-1, math.floor((y+ey-self.oy)/r)+1)
        rows, columns = np.nonzero(self.occupied[bottom:top+1, left:right+1])
        if len(rows) == 0:
            return False
        dx = self.ox + (columns + left + .5) * r - x
        dy = self.oy + (rows + bottom + .5) * r - y
        # Separating-axis theorem for the oriented car and each square pixel.
        along = np.abs(dx*c + dy*s) <= self.half_length + r/2*(ac+ass)
        across = np.abs(-dx*s + dy*c) <= self.half_width + r/2*(ac+ass)
        horizontal = np.abs(dx) <= ex + r/2
        vertical = np.abs(dy) <= ey + r/2
        return bool(np.any(along & across & horizontal & vertical))
