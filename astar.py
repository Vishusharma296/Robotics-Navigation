import math
import heapq
from quadtree import Rect


def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def cell_is_blocked(cx, cy, cell_size, quadtree, moving_obstacles, m_l, m_b, inflation=0.5):
    cell_rect = Rect(cx * cell_size, cy * cell_size, cell_size, cell_size)
    qr = Rect(cell_rect.x - inflation, cell_rect.y - inflation,
              cell_rect.w + 2 * inflation, cell_rect.h + 2 * inflation)
    if quadtree.query(qr):
        return True
    for mo in moving_obstacles:
        if qr.intersects(mo.rect):
            return True
    return False


def astar(start, goal, quadtree, moving_obstacles, m_l, m_b, inflation=0.5):
    cell_size = 2.0
    nx = max(1, int(math.ceil(m_l / cell_size)))
    ny = max(1, int(math.ceil(m_b / cell_size)))

    sx = max(0, min(nx - 1, int(start[0] / cell_size)))
    sy = max(0, min(ny - 1, int(start[1] / cell_size)))
    gx = max(0, min(nx - 1, int(goal[0] / cell_size)))
    gy = max(0, min(ny - 1, int(goal[1] / cell_size)))

    if cell_is_blocked(sx, sy, cell_size, quadtree, moving_obstacles, m_l, m_b, inflation):
        return None
    if cell_is_blocked(gx, gy, cell_size, quadtree, moving_obstacles, m_l, m_b, inflation):
        return None

    start_node = (sx, sy)
    goal_node = (gx, gy)

    open_set = [(0.0, start_node)]
    g_score = {start_node: 0.0}
    came_from = {}

    directions = [(1, 0), (-1, 0), (0, 1), (0, -1),
                  (1, 1), (-1, 1), (1, -1), (-1, -1)]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal_node:
            path_cells = []
            node = current
            while node in came_from:
                path_cells.append(node)
                node = came_from[node]
            path_cells.append(start_node)
            path_cells.reverse()

            path = []
            for cx, cy in path_cells:
                px = cx * cell_size + cell_size / 2
                py = cy * cell_size + cell_size / 2
                path.append((px, py))
            if path:
                path[0] = start
                path[-1] = goal
            return path

        cx, cy = current
        for dx, dy in directions:
            nx_c, ny_c = cx + dx, cy + dy
            if nx_c < 0 or nx_c >= nx or ny_c < 0 or ny_c >= ny:
                continue
            nkey = (nx_c, ny_c)
            if nkey in came_from and nkey != start_node:
                continue

            if cell_is_blocked(nx_c, ny_c, cell_size, quadtree, moving_obstacles, m_l, m_b, inflation):
                continue

            step_cost = cell_size if dx == 0 or dy == 0 else cell_size * math.sqrt(2)
            tentative = g_score[current] + step_cost

            if tentative < g_score.get(nkey, float('inf')):
                came_from[nkey] = current
                g_score[nkey] = tentative
                ncx = nx_c * cell_size + cell_size / 2
                ncy = ny_c * cell_size + cell_size / 2
                f = tentative + heuristic((ncx, ncy), goal)
                heapq.heappush(open_set, (f, nkey))

    return None
