import math


class Rect:
    def __init__(self, x, y, w, h):
        self.x = float(x)
        self.y = float(y)
        self.w = float(w)
        self.h = float(h)

    def intersects(self, other):
        return not (self.x + self.w <= other.x or other.x + other.w <= self.x or
                    self.y + self.h <= other.y or other.y + other.h <= self.y)

    def contains_point(self, p):
        x, y = p
        return self.x <= x <= self.x + self.w and self.y <= y <= self.y + self.h

    def center(self):
        return (self.x + self.w / 2, self.y + self.h / 2)

    def shares_edge(self, other):
        eps = 0.01
        if abs(self.x + self.w - other.x) < eps:
            if max(self.y, other.y) < min(self.y + self.h, other.y + other.h) - eps:
                return True
        if abs(other.x + other.w - self.x) < eps:
            if max(self.y, other.y) < min(self.y + self.h, other.y + other.h) - eps:
                return True
        if abs(self.y + self.h - other.y) < eps:
            if max(self.x, other.x) < min(self.x + self.w, other.x + other.w) - eps:
                return True
        if abs(other.y + other.h - self.y) < eps:
            if max(self.x, other.x) < min(self.x + self.w, other.x + other.w) - eps:
                return True
        return False

    def __repr__(self):
        return f"Rect({self.x:.1f}, {self.y:.1f}, {self.w:.1f}, {self.h:.1f})"


class QuadTree:
    MIN_LEAF_SIZE = 1.0

    def __init__(self, bounds, capacity=4):
        self.bounds = bounds
        self.capacity = capacity
        self.objects = []
        self.divided = False
        self.children = None

    def subdivide(self):
        if self.bounds.w < self.MIN_LEAF_SIZE * 2 or self.bounds.h < self.MIN_LEAF_SIZE * 2:
            return
        x, y, w, h = self.bounds.x, self.bounds.y, self.bounds.w, self.bounds.h
        hw, hh = w / 2, h / 2
        self.children = [
            QuadTree(Rect(x, y + hh, hw, hh), self.capacity),
            QuadTree(Rect(x + hw, y + hh, hw, hh), self.capacity),
            QuadTree(Rect(x, y, hw, hh), self.capacity),
            QuadTree(Rect(x + hw, y, hw, hh), self.capacity),
        ]
        self.divided = True
        for obj in self.objects:
            for child in self.children:
                if child.bounds.intersects(obj):
                    child.insert(obj)
        self.objects = []

    def insert(self, rect):
        if not self.bounds.intersects(rect):
            return
        if not self.divided:
            self.objects.append(rect)
            if len(self.objects) > self.capacity:
                self.subdivide()
        else:
            for child in self.children:
                if child.bounds.intersects(rect):
                    child.insert(rect)

    def query(self, rect):
        if not self.bounds.intersects(rect):
            return []
        if not self.divided:
            return [o for o in self.objects if o.intersects(rect)]
        results = []
        for child in self.children:
            results.extend(child.query(rect))
        return results

    def get_leaves(self):
        if not self.divided:
            return [(self.bounds, len(self.objects) > 0)]
        leaves = []
        for child in self.children:
            leaves.extend(child.get_leaves())
        return leaves

    def all_obstacles(self):
        if not self.divided:
            return self.objects[:]
        objs = []
        for child in self.children:
            objs.extend(child.all_obstacles())
        return objs
