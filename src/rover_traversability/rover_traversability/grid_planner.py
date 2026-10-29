"""
grid_planner.py
===============
A* over terrain_mapper's cost grid, plain numpy + heapq (no ROS), so it can be tested
offline. Same grid layout as terrain_grid.py / nav_msgs/OccupancyGrid: row i is y,
column j is x, cell (i, j) centred at (origin + (j + 0.5) * res, origin + (i + 0.5) * res).

Costs in: -1 unknown, 0..99 traversable (higher = steeper/rougher), 100 no-go.

1. No-go cells are inflated by the rover's half-width (robot_radius), so a path through the
   remaining cells keeps the whole rover clear, not just its centre. Cells a little further
   out (up to inflation_radius) get extra cost, so paths don't hug obstacles.
2. Unknown cells are passable at unknown_cost: the planner can head into ground it hasn't
   seen yet (the goal is usually beyond the map) and replans as the mapper fills it in.
3. Each 8-connected step costs  distance * (1 + cost_weight * cost / 100),  so a detour is
   taken when it is cheaper than crossing steep or rough ground. The heuristic is plain
   distance, which never overestimates (every step costs at least its length), so A* finds
   the cheapest path.
4. A goal outside the map is replaced by the map cell nearest to it; a goal on a no-go cell
   (an ArUco marker maps as an obstacle) by the nearest free cell within goal_tolerance.
   A start inside blocked ground (inflated or no-go: the rover is already there) is
   allowed, and the blocked region around it is drivable at maximum cost, so the rover
   can always plan its way out.
"""

import heapq
import math

import numpy as np

NO_GO = 100
UNKNOWN = -1


def _disc(radius_cells):
    r = int(math.ceil(radius_cells))
    di, dj = np.mgrid[-r:r + 1, -r:r + 1]
    keep = di * di + dj * dj <= radius_cells * radius_cells
    return list(zip(di[keep], dj[keep]))


class GridPlanner:
    def __init__(self, robot_radius=0.45, inflation_radius=1.0, inflation_cost=40.0,
                 unknown_cost=40.0, cost_weight=1.0, goal_tolerance=1.0):
        self.robot_radius = robot_radius
        self.inflation_radius = inflation_radius
        self.inflation_cost = inflation_cost
        self.unknown_cost = unknown_cost
        self.cost_weight = cost_weight
        self.goal_tolerance = goal_tolerance

    # ------------------------------------------------------------------
    def planning_cost(self, cost, res):
        """Per-cell traversal cost for A*: inf where the rover can't go (no-go, inflated by
        the rover's half-width), extra cost in a band around that, unknown_cost where unseen."""
        c = np.where(cost == UNKNOWN, self.unknown_cost, cost).astype(float)
        lethal = cost == NO_GO

        # Distance (in cells) to the nearest no-go cell, by repeated dilation over a disc
        n_i, n_j = cost.shape
        r_infl = self.inflation_radius / res
        dist = np.full(cost.shape, np.inf)
        dist[lethal] = 0.0
        if lethal.any():
            li, lj = np.nonzero(lethal)
            for di, dj in _disc(r_infl):
                d = math.hypot(di, dj)
                ii, jj = li + di, lj + dj
                ok = (ii >= 0) & (ii < n_i) & (jj >= 0) & (jj < n_j)
                np.minimum.at(dist, (ii[ok], jj[ok]), d)
        dist_m = dist * res

        band = (dist_m > self.robot_radius) & (dist_m <= self.inflation_radius)
        fall = (self.inflation_radius - dist_m[band]) / (self.inflation_radius - self.robot_radius)
        c[band] = np.maximum(c[band], self.inflation_cost * fall)
        c[dist_m <= self.robot_radius] = np.inf
        return np.where(np.isinf(c), np.inf, np.minimum(c, NO_GO - 1))

    # ------------------------------------------------------------------
    def plan(self, cost, origin_x, origin_y, res, start_xy, goal_xy):
        """Returns (path as [(x, y), ...] from start to goal, info dict). path is [] when no
        path exists. info has 'reason', 'goal_used' (x, y), 'cost', 'expanded'."""
        n_i, n_j = cost.shape
        pc = self.planning_cost(cost, res)

        def to_cell(x, y):
            return int(math.floor((y - origin_y) / res)), int(math.floor((x - origin_x) / res))

        def to_xy(i, j):
            return origin_x + (j + 0.5) * res, origin_y + (i + 0.5) * res

        si, sj = to_cell(*start_xy)
        if not (0 <= si < n_i and 0 <= sj < n_j):
            return [], {'reason': 'start outside the map'}

        # Goal: clamp into the map, then the nearest passable cell within goal_tolerance
        gi, gj = to_cell(*goal_xy)
        gi, gj = min(max(gi, 0), n_i - 1), min(max(gj, 0), n_j - 1)
        if np.isinf(pc[gi, gj]):
            best = None
            for di, dj in sorted(_disc(self.goal_tolerance / res), key=lambda o: o[0] ** 2 + o[1] ** 2):
                ii, jj = gi + di, gj + dj
                if 0 <= ii < n_i and 0 <= jj < n_j and not np.isinf(pc[ii, jj]):
                    best = (ii, jj)
                    break
            if best is None:
                return [], {'reason': 'goal is no-go and no free cell within goal_tolerance'}
            gi, gj = best

        if np.isinf(pc[si, sj]):
            # The rover is standing in blocked ground: squeezed near a rock (inflation), or on a
            # patch it judges too steep while climbing (no-go). It's already there, so make the
            # blocked region connected to it drivable at maximum cost and plan out of it.
            # (Freeing only the inflated cells left a rover on a no-go cell with "no path".)
            blocked = np.isinf(pc)
            region = np.zeros_like(blocked)
            stack = [(si, sj)]
            while stack:
                i, j = stack.pop()
                if 0 <= i < n_i and 0 <= j < n_j and blocked[i, j] and not region[i, j]:
                    region[i, j] = True
                    stack += [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]
            pc[region] = NO_GO - 1

        # A*
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        g = np.full(cost.shape, np.inf)
        parent = np.full(cost.shape + (2,), -1, dtype=np.int32)
        closed = np.zeros(cost.shape, dtype=bool)
        g[si, sj] = 0.0
        h0 = math.hypot(gi - si, gj - sj)
        heap = [(h0, 0.0, si, sj)]
        expanded = 0
        k = self.cost_weight / 100.0
        while heap:
            _, gc, i, j = heapq.heappop(heap)
            if closed[i, j]:
                continue
            closed[i, j] = True
            expanded += 1
            if (i, j) == (gi, gj):
                break
            for di, dj, d in steps:
                ii, jj = i + di, j + dj
                if not (0 <= ii < n_i and 0 <= jj < n_j) or closed[ii, jj]:
                    continue
                c = pc[ii, jj]
                if c == np.inf:
                    continue
                ng = gc + d * (1.0 + k * c)
                if ng < g[ii, jj]:
                    g[ii, jj] = ng
                    parent[ii, jj] = (i, j)
                    heapq.heappush(heap, (ng + math.hypot(gi - ii, gj - jj), ng, ii, jj))

        if not closed[gi, gj]:
            return [], {'reason': 'no path', 'expanded': expanded}

        cells = [(gi, gj)]
        while cells[-1] != (si, sj):
            pi, pj = parent[cells[-1]]
            cells.append((int(pi), int(pj)))
        cells.reverse()
        path = [to_xy(i, j) for i, j in cells]
        path[0] = tuple(start_xy)
        return path, {'reason': 'ok', 'goal_used': to_xy(gi, gj), 'cost': float(g[gi, gj] * res),
                      'expanded': expanded}
