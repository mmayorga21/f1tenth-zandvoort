#!/usr/bin/env python3
"""Check map changes, width clearance, and native LiDAR/collision responses.

The clearance test checks a connected route for the vehicle width plus margin.
It does not certify steering feasibility or ten collision-free laps.
"""
import argparse
import heapq
import json
import math
from pathlib import Path

import gym
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

ROOT = Path(__file__).resolve().parents[1]


def check(condition, message):
    if not condition:
        raise ValueError(message)


def path_length(free, start, goal, resolution):
    """Eight-connected A*, disallowing cuts through occupied corners."""
    frontier = [(0., 0., start)]
    best = {start: 0.}
    moves = [(dy, dx, math.hypot(dy, dx)) for dy in (-1, 0, 1)
             for dx in (-1, 0, 1) if dy or dx]
    height, width = free.shape
    while frontier:
        _, cost, (y, x) = heapq.heappop(frontier)
        if cost > best[(y, x)]:
            continue
        if (y, x) == goal:
            return cost * resolution
        for dy, dx, step in moves:
            ny, nx = y + dy, x + dx
            if not (0 <= ny < height and 0 <= nx < width and free[ny, nx]):
                continue
            if dy and dx and not (free[y, nx] and free[ny, x]):
                continue
            distance = cost + step
            if distance < best.get((ny, nx), math.inf):
                best[(ny, nx)] = distance
                heuristic = math.hypot(goal[0] - ny, goal[1] - nx)
                heapq.heappush(frontier, (distance + heuristic, distance, (ny, nx)))
    raise ValueError('No connected route through the checkpoint sequence')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    maps = ROOT / 'src/f1tenth_gym_ros/maps'
    original_path, obstacle_path = maps / 'Zandvoort_map', maps / 'Zandvoort_obstacles'
    metadata = yaml.safe_load(original_path.with_suffix('.yaml').read_text())
    selected = yaml.safe_load(obstacle_path.with_suffix('.yaml').read_text())
    for key in ('resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'):
        check(metadata[key] == selected[key], 'Map metadata changed: ' + key)
    check(selected['image'] == obstacle_path.name + '.png', 'Wrong PNG in obstacle YAML')
    original_image = Image.open(original_path.with_suffix('.png'))
    obstacle_image = Image.open(obstacle_path.with_suffix('.png'))
    check(original_image.size == obstacle_image.size, 'Image dimensions changed')
    check(original_image.mode == obstacle_image.mode == 'L', 'Expected grayscale images')
    original, modified = np.asarray(original_image), np.asarray(obstacle_image)
    resolution = float(metadata['resolution'])
    ox, oy, yaw = metadata['origin']
    check(yaw == 0., 'This validator expects an unrotated map')
    height = original.shape[0]

    def pixel(x, y):
        return height - 1 - math.floor((y - oy) / resolution), math.floor((x - ox) / resolution)

    def world(row, column):
        return ox + (column + .5) * resolution, oy + (height - row - .5) * resolution

    original_free, modified_free = original > 128, modified > 128
    check(not np.any(~original_free & modified_free), 'Original occupied pixels were removed')
    added = original_free & ~modified_free
    labels, count = ndimage.label(added)
    check(count == 5, f'Expected five added obstacles, found {count}')
    sim = yaml.safe_load((ROOT / 'src/f1tenth_gym_ros/config/sim.yaml').read_text())['bridge']['ros__parameters']
    start = [sim[k] for k in ('sx', 'sy', 'stheta')]
    components, _ = ndimage.label(original_free)
    track = components == components[pixel(*start[:2])]
    clearance = ndimage.distance_transform_edt(modified_free) * resolution
    original_clearance = ndimage.distance_transform_edt(original_free) * resolution
    objects, probe_poses = [], []
    # A circumscribed body radius makes each stationary probe start clear of walls.
    body_radius = math.hypot(.58, .31) / 2
    for index, region in enumerate(ndimage.find_objects(labels), 1):
        rows, columns = np.where(labels[region] == index)
        rows, columns = rows + region[0].start, columns + region[1].start
        center = np.array([rows.mean(), columns.mean()])
        nearest = np.argmin((rows - center[0])**2 + (columns - center[1])**2)
        row, column = int(rows[nearest]), int(columns[nearest])
        tx, ty = world(row, column)
        pose = None
        for distance in (1., 1.2, .8, 1.5, 2.):
            for angle in np.linspace(0., 2 * math.pi, 48, endpoint=False):
                x, y = tx + distance * math.cos(angle), ty + distance * math.sin(angle)
                v, u = pixel(x, y)
                if not (0 <= v < height and 0 <= u < original.shape[1]):
                    continue
                if not track[v, u] or clearance[v, u] < body_radius + .08:
                    continue
                rv = np.linspace(v, row, 100).astype(int)
                ru = np.linspace(u, column, 100).astype(int)
                if np.all(track[rv, ru]):
                    pose = [x, y, math.atan2(ty - y, tx - x)]
                    break
            if pose:
                break
        check(pose is not None, f'No clear LiDAR probe pose for obstacle {index}')
        probe_poses.append(pose)
        objects.append({'id': index, 'geometry': 'occupied_map_pixels',
                        'pixel_bbox': [int(columns.min()), int(rows.min()),
                                       int(columns.max()) + 1, int(rows.max()) + 1],
                        'added_occupied_pixels': len(rows),
                        'probe_target_world_m': [tx, ty],
                        'bbox_size_m': [float((columns.max() - columns.min() + 1) * resolution),
                                        float((rows.max() - rows.min() + 1) * resolution)]})

    env = gym.make('f110_gym:f110-v0', map=str(original_path), map_ext='.png', num_agents=1, seed=12345)
    try:
        width = float(env.unwrapped.params['width'])
        margin = .10
        # Account for the extent of a pixel, not just its center.
        radius = width / 2 + margin + resolution / math.sqrt(2)
        base_route = track & (original_clearance >= radius)
        modified_route = track & (clearance >= radius)
        flat = yaml.safe_load((ROOT / 'src/f1tenth_controller/config/lap_tracker.yaml').read_text())['lap_tracker']['ros__parameters']['checkpoint_regions']
        anchors = [start[:2]] + [flat[i:i+2] for i in range(0, len(flat), 3)] + [start[:2]]
        projected = []
        for x, y in anchors:
            v, u = pixel(x, y)
            candidates = np.argwhere(base_route[max(0, v-100):v+101, max(0, u-100):u+101])
            candidates += [max(0, v-100), max(0, u-100)]
            check(len(candidates) > 0, 'Checkpoint has no nearby drivable corridor')
            order = np.argsort(np.sum((candidates - [v, u])**2, axis=1))
            candidate = next((tuple(map(int, c)) for c in candidates[order] if modified_route[tuple(c)]), None)
            check(candidate is not None, 'Checkpoint corridor is blocked')
            projected.append(candidate)
        segments = []
        for a, b in zip(projected, projected[1:]):
            old = path_length(base_route, a, b, resolution)
            new = path_length(modified_route, a, b, resolution)
            check(new <= old * 1.5 + 4., 'Obstacle forces a detour around the circuit')
            segments.append({'original_m': old, 'obstacles_m': new})
        dt = env.unwrapped.timestep
        for item, pose in zip(objects, probe_poses):
            samples = []
            for map_path in (original_path, obstacle_path):
                env.unwrapped.update_map(str(map_path.with_suffix('.yaml')), '.png')
                obs, _, _, _ = env.reset(np.array([pose]))
                scan = obs['scans'][0]
                front = float(scan[len(scan)//2])
                collision_step = None
                # Same straight motion for both maps; the new obstacle must
                # produce the earlier collision, not an existing track wall.
                for step in range(round(5. / dt)):
                    obs, _, _, _ = env.step(np.array([[0., .5]]))
                    if bool(obs['collisions'][0]):
                        collision_step = step
                        break
                samples.append({'front_range_m': front, 'collision_step': collision_step})
            baseline, obstacle = samples
            check(obstacle['front_range_m'] < baseline['front_range_m'] - .10,
                  f'Obstacle {item["id"]} did not shorten the forward LiDAR ray')
            check(obstacle['collision_step'] is not None and
                  (baseline['collision_step'] is None or obstacle['collision_step'] < baseline['collision_step']),
                  f'Obstacle {item["id"]} did not produce an earlier native collision')
            item['probe'] = {'pose': pose, 'original': baseline, 'obstacles': obstacle}
    finally:
        env.close()
    report = {'result': 'passed', 'obstacles': objects,
              'route_segments': segments, 'vehicle_width_m': width,
              'lateral_margin_per_side_m': margin,
              'limitations': 'Width-clearance route and obstacle sensor/collision checks; steering feasibility and full laps are not certified.'}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
