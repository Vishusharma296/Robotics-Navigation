import math
import random
from quadtree import Rect, QuadTree
from map_generator import (
    generate_static_obstacles,
    generate_moving_obstacles,
)
from astar import astar


class RobotNavigationSimulation:
    def __init__(self, config, seed=None):
        self.c = config
        self.tick = 0
        self.done = False
        self.arrived = False
        self.replan_count = 0

        if seed is None:
            seed = random.randint(0, 999999)
        self.seed = seed
        random.seed(self.seed)

        M_L = config.M_L
        M_B = config.M_B
        RL = config.RL
        RB = config.RB

        self.static_obstacles = None
        self.quadtree = None
        self.moving_obstacles = None
        self.path = None

        for attempt in range(50):
            self.static_obstacles = generate_static_obstacles(
                M_L, M_B, config.OBSTACLE_COVERAGE,
                config.RS_X, config.RS_Y,
                config.RD_X, config.RD_Y,
                RL, RB
            )
            self.quadtree = QuadTree(Rect(0, 0, M_L, M_B), capacity=4)
            for obs in self.static_obstacles:
                self.quadtree.insert(obs)

            self.moving_obstacles = generate_moving_obstacles(
                config.NUM_MOVING, self.static_obstacles,
                config.OL, config.OB, config.MOVING_SPEED,
                M_L, M_B,
                config.RS_X, config.RS_Y,
                config.RD_X, config.RD_Y,
                RL, RB
            )

            self.path = astar(
                (float(config.RS_X), float(config.RS_Y)),
                (float(config.RD_X), float(config.RD_Y)),
                self.quadtree, self.moving_obstacles,
                M_L, M_B, config.INFLATION
            )
            if self.path is not None:
                break

        if self.path is None:
            raise RuntimeError("Could not generate a pathable map after 50 attempts")

        self.robot_pos = [float(config.RS_X), float(config.RS_Y)]
        self.robot_dest = (float(config.RD_X), float(config.RD_Y))
        self.path_index = 0
        self.wait_counter = 0

    def robot_rect(self):
        return Rect(
            self.robot_pos[0] - self.c.RL / 2,
            self.robot_pos[1] - self.c.RB / 2,
            self.c.RL, self.c.RB
        )

    def find_path(self):
        return astar(
            tuple(self.robot_pos), self.robot_dest,
            self.quadtree, self.moving_obstacles,
            self.c.M_L, self.c.M_B, self.c.INFLATION
        )

    def is_path_blocked(self):
        if not self.path or self.path_index >= len(self.path):
            return False
        remaining = self.path[self.path_index:]
        points = [tuple(self.robot_pos)] + remaining
        sm = self.c.SAFETY_MARGIN + self.c.INFLATION
        for mo in self.moving_obstacles:
            mc = (mo.rect.x + mo.rect.w / 2, mo.rect.y + mo.rect.h / 2)
            for i in range(len(points) - 1):
                d = _point_seg_dist(mc, points[i], points[i + 1])
                if d < sm:
                    return True
        return False

    def step(self):
        if self.done:
            return True
        self.tick += 1

        for mo in self.moving_obstacles:
            mo.update()

        blocked = self.is_path_blocked()

        if blocked:
            self.wait_counter += 1
            if self.wait_counter >= self.c.WAIT_TICKS:
                new_path = self.find_path()
                if new_path is not None:
                    self.path = new_path
                self.path_index = 0
                self.wait_counter = 0
                self.replan_count += 1
        else:
            self.wait_counter = 0
            if self.path and self.path_index < len(self.path):
                tgt = self.path[self.path_index]
                dx = tgt[0] - self.robot_pos[0]
                dy = tgt[1] - self.robot_pos[1]
                d = math.hypot(dx, dy)
                if d < 1.0:
                    self.path_index += 1
                else:
                    step = min(self.c.ROBOT_SPEED, d)
                    self.robot_pos[0] += dx / d * step
                    self.robot_pos[1] += dy / d * step

        dx = self.robot_dest[0] - self.robot_pos[0]
        dy = self.robot_dest[1] - self.robot_pos[1]
        if math.hypot(dx, dy) < 2.0:
            self.arrived = True
            self.done = True

        if self.tick >= self.c.MAX_STEPS:
            self.done = True

        return self.done

    def get_state(self):
        elapsed = self.tick * self.c.TIME_PER_TICK
        return {
            'tick': self.tick,
            'time_elapsed': elapsed,
            'robot_pos': tuple(self.robot_pos),
            'robot_dest': self.robot_dest,
            'robot_rect': self.robot_rect(),
            'static_obstacles': self.static_obstacles,
            'moving_obstacles': self.moving_obstacles,
            'path': self.path,
            'path_index': self.path_index,
            'arrived': self.arrived,
            'done': self.done,
            'seed': self.seed,
            'replan_count': self.replan_count,
        }


def _point_seg_dist(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
