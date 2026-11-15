"""Temporary lap verification for experiments, never an input to the controller."""
import math


class LapValidation:
    # Broad ordered regions distinguish the circuit's separate branches.
    # Coordinates are only for independent validation, not for navigation.
    checkpoints = [(15., 29., 5.), (73., -4., 6.), (57., -29., 5.),
                   (48., -21., 3.), (60., -12., 5.), (1., -42., 7.)]

    def __init__(self, start=(0., 0., 1.1954730407943466)):
        self.start = start
        self.previous = start[:2]
        self.previous_signed = 0.0
        self.distance = 0.0
        self.next_checkpoint = 0
        self.complete = False
        self.checkpoint_times = []

    def update(self, x, y, sim_time):
        self.distance += math.dist(self.previous, (x, y))
        if self.next_checkpoint < len(self.checkpoints):
            cx, cy, radius = self.checkpoints[self.next_checkpoint]
            if math.hypot(x-cx, y-cy) < radius:
                self.checkpoint_times.append(sim_time)
                self.next_checkpoint += 1
        dx, dy = x-self.start[0], y-self.start[1]
        signed = dx*math.cos(self.start[2])+dy*math.sin(self.start[2])
        lateral = -dx*math.sin(self.start[2])+dy*math.cos(self.start[2])
        if (self.next_checkpoint == len(self.checkpoints) and self.distance > 200.
                and self.previous_signed < 0 <= signed and abs(lateral)<1.3):
            self.complete = True
        self.previous_signed = signed
        self.previous = (x, y)
        return self.complete

    def summary(self):
        return {'complete': self.complete, 'distance_m': self.distance,
                'checkpoints_passed': self.next_checkpoint,
                'checkpoint_times_sim_s': self.checkpoint_times,
                'checkpoint_regions': self.checkpoints,
                'gate': {'pose': self.start, 'half_width_m': 1.3,
                         'minimum_distance_m': 200., 'direction': 'forward'}}
