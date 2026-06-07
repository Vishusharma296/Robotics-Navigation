import random
import math
from quadtree import Rect


def _inflated(rect, amount):
    return Rect(rect.x - amount, rect.y - amount,
                rect.w + 2 * amount, rect.h + 2 * amount)


def _rect_free(rect, obstacles, m_l, m_b, inflation=0.5):
    ir = _inflated(rect, inflation)
    if ir.x < 0 or ir.y < 0:
        return False
    if ir.x + ir.w > m_l or ir.y + ir.h > m_b:
        return False
    for o in obstacles:
        if ir.intersects(o):
            return False
    return True


def generate_static_obstacles(m_l, m_b, coverage, rs_x, rs_y, rd_x, rd_y, rl, rb):
    inflation = 0.5
    min_gap = max(rl, rb) * 2
    start_zone = _inflated(Rect(rs_x - min_gap, rs_y - min_gap, min_gap * 2, min_gap * 2), inflation)
    dest_zone = _inflated(Rect(rd_x - min_gap, rd_y - min_gap, min_gap * 2, min_gap * 2), inflation)

    target = m_l * m_b * coverage
    obstacles = []
    placed = 0.0

    for _ in range(2000):
        if placed >= target:
            break
        w = random.uniform(4, 12)
        h = random.uniform(4, 12)
        x = random.uniform(1, m_l - w - 1)
        y = random.uniform(1, m_b - h - 1)
        new_rect = Rect(x, y, w, h)
        new_inflated = _inflated(new_rect, inflation)

        if new_inflated.intersects(start_zone) or new_inflated.intersects(dest_zone):
            continue

        collision = False
        for obs in obstacles:
            if new_inflated.intersects(_inflated(obs, inflation)):
                collision = True
                break
        if collision:
            continue

        obstacles.append(new_rect)
        placed += w * h

    return obstacles


def find_free_position(static_obstacles, m_l, m_b, rect_w, rect_h, attempts=200):
    margin = max(rect_w, rect_h)
    for _ in range(attempts):
        x = random.uniform(margin, m_l - margin)
        y = random.uniform(margin, m_b - margin)
        test = Rect(x - rect_w / 2, y - rect_h / 2, rect_w, rect_h)
        if _rect_free(test, static_obstacles, m_l, m_b):
            return (x, y)
    return None


class MovingObstacle:
    def __init__(self, rect, start, end, speed, static_obstacles, m_l, m_b):
        self.rect = rect
        self.speed = speed
        self.static_obstacles = static_obstacles
        self.m_l = m_l
        self.m_b = m_b
        self.set_path(start, end)

    def set_path(self, start, end):
        self.start = list(start)
        self.end = list(end)
        self.pos = list(start)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dist = math.hypot(dx, dy)
        if dist > 0:
            self.dx = dx / dist * self.speed
            self.dy = dy / dist * self.speed
        else:
            self.dx = 0
            self.dy = 0

    def update(self):
        new_x = self.pos[0] + self.dx
        new_y = self.pos[1] + self.dy
        new_rect = Rect(new_x - self.rect.w / 2, new_y - self.rect.h / 2,
                        self.rect.w, self.rect.h)
        if _rect_free(new_rect, self.static_obstacles, self.m_l, self.m_b):
            self.pos[0] = new_x
            self.pos[1] = new_y
            self.rect = new_rect
        else:
            self.pick_new_destination()
            return

        dx = self.end[0] - self.pos[0]
        dy = self.end[1] - self.pos[1]
        if math.hypot(dx, dy) < 1.0:
            self.pick_new_destination()

    def pick_new_destination(self):
        end = find_free_position(self.static_obstacles, self.m_l, self.m_b,
                                 self.rect.w, self.rect.h)
        if end:
            self.set_path(self.pos, end)
        else:
            self.dx = 0
            self.dy = 0


def generate_moving_obstacles(num, static_obstacles, ol, ob, speed,
                              m_l, m_b, rs_x, rs_y, rd_x, rd_y, rl, rb):
    moving = []
    for _ in range(num):
        start = find_free_position(static_obstacles, m_l, m_b, ol, ob)
        if start is None:
            continue
        end = find_free_position(static_obstacles, m_l, m_b, ol, ob)
        if end is None:
            continue
        s_rect = Rect(start[0] - ol / 2, start[1] - ob / 2, ol, ob)
        mo = MovingObstacle(s_rect, start, end, speed, static_obstacles, m_l, m_b)
        moving.append(mo)
    return moving
