"""
terrain_grid.py
===============
Rover-centred 2.5D height grid and traversability layers, plain numpy (no ROS),
so it can be tested offline against a known terrain.

Grid layout: row i is y, column j is x, cell (i, j) covers
  x in [origin_x + j*res, origin_x + (j+1)*res), y likewise with origin_y and i.
That is the nav_msgs/OccupancyGrid layout, so cost.ravel() is the message data.

Layers, all from a least-squares plane fitted to the heights in a
(2*window+1)^2-cell window around each cell:
  slope   angle of that plane from horizontal (deg)
  step    highest minus lowest height *relative to the plane* in the window (m).
          Measuring against the plane (not raw max - min) keeps a smooth slope
          from reading as a step: raw max - min over 0.75 m on a 20 deg slope is
          already 0.27 m.
  rough   RMS distance of the heights from the plane (m)
"""

import numpy as np

UNKNOWN = -1
LETHAL = 100


def shifted(a, di, dj, fill=np.nan):
    """out[i, j] = a[i + di, j + dj], with `fill` where that falls outside a."""
    out = np.full_like(a, fill)
    n_i, n_j = a.shape
    src_i = slice(max(di, 0), n_i + min(di, 0))
    src_j = slice(max(dj, 0), n_j + min(dj, 0))
    dst_i = slice(max(-di, 0), n_i + min(-di, 0))
    dst_j = slice(max(-dj, 0), n_j + min(-dj, 0))
    out[dst_i, dst_j] = a[src_i, src_j]
    return out


class TerrainGrid:
    def __init__(self, resolution=0.15, size_m=20.0, max_weight=10.0):
        self.res = resolution
        self.n = int(round(size_m / resolution))
        self.size_m = self.n * resolution
        self.max_weight = max_weight
        self.origin_x = -self.size_m / 2
        self.origin_y = -self.size_m / 2
        self.height = np.full((self.n, self.n), np.nan)   # fused ground height (m), NaN = never seen
        self.weight = np.zeros((self.n, self.n))          # frames fused into each cell (capped)

    # ------------------------------------------------------------------
    # Map maintenance
    # ------------------------------------------------------------------
    def recenter(self, x, y):
        """Moves the grid so (x, y) is near its centre. Cells that scroll off are
        dropped and new ones start unknown. Origin stays on a multiple of res so
        cells never resample."""
        new_ox = np.floor((x - self.size_m / 2) / self.res) * self.res
        new_oy = np.floor((y - self.size_m / 2) / self.res) * self.res
        dj = int(round((new_ox - self.origin_x) / self.res))
        di = int(round((new_oy - self.origin_y) / self.res))
        if di == 0 and dj == 0:
            return
        self.height = shifted(self.height, di, dj, np.nan)
        self.weight = shifted(self.weight, di, dj, 0.0)
        self.origin_x, self.origin_y = new_ox, new_oy

    def fuse(self, points):
        """Adds one frame of ground points (N x 3, map frame). Each cell's height
        is the mean of this frame's points in it, averaged with the cell's history
        weighted by how many frames it has seen (capped at max_weight, so the map
        still adapts if something moves)."""
        j = np.floor((points[:, 0] - self.origin_x) / self.res).astype(np.int64)
        i = np.floor((points[:, 1] - self.origin_y) / self.res).astype(np.int64)
        inside = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        flat = i[inside] * self.n + j[inside]
        count = np.bincount(flat, minlength=self.n * self.n).reshape(self.n, self.n)
        total = np.bincount(flat, weights=points[inside, 2],
                            minlength=self.n * self.n).reshape(self.n, self.n)
        seen = count > 0
        frame_z = np.zeros_like(self.height)
        frame_z[seen] = total[seen] / count[seen]

        new = seen & (self.weight == 0)
        old = seen & (self.weight > 0)
        self.height[new] = frame_z[new]
        w = self.weight[old]
        self.height[old] = (self.height[old] * w + frame_z[old]) / (w + 1)
        self.weight[seen] = np.minimum(self.weight[seen] + 1, self.max_weight)
        return int(seen.sum())

    def cell_centres(self):
        xs = self.origin_x + (np.arange(self.n) + 0.5) * self.res
        ys = self.origin_y + (np.arange(self.n) + 0.5) * self.res
        return np.meshgrid(xs, ys)

    # ------------------------------------------------------------------
    # Layers
    # ------------------------------------------------------------------
    def layers(self, window=2, min_points=6):
        """Returns (slope_deg, step_m, rough_m, valid). valid is False where the
        cell itself is unknown or its window has fewer than min_points known cells."""
        h = self.height
        known = ~np.isnan(h)
        offsets = [(di, dj) for di in range(-window, window + 1) for dj in range(-window, window + 1)]

        # Plane z = a + b*dx + c*dy around each cell, heights taken relative to
        # the centre cell for conditioning. Normal equations accumulated per offset.
        s = {k: np.zeros_like(h) for k in ('m', 'x', 'y', 'xx', 'yy', 'xy', 'z', 'xz', 'yz')}
        for di, dj in offsets:
            z = shifted(h, di, dj) - h
            m = ~np.isnan(z)
            z = np.where(m, z, 0.0)
            dx, dy = dj * self.res, di * self.res
            s['m'] += m
            s['x'] += m * dx
            s['y'] += m * dy
            s['xx'] += m * dx * dx
            s['yy'] += m * dy * dy
            s['xy'] += m * dx * dy
            s['z'] += z
            s['xz'] += z * dx
            s['yz'] += z * dy

        valid = known & (s['m'] >= min_points)
        A = np.stack([np.stack([s['m'], s['x'], s['y']], -1),
                      np.stack([s['x'], s['xx'], s['xy']], -1),
                      np.stack([s['y'], s['xy'], s['yy']], -1)], -2)[valid]
        rhs = np.stack([s['z'], s['xz'], s['yz']], -1)[valid]
        coef = np.linalg.solve(A + 1e-9 * np.eye(3), rhs[..., None])[..., 0]
        a = np.zeros_like(h)
        b = np.zeros_like(h)
        c = np.zeros_like(h)
        a[valid], b[valid], c[valid] = coef[:, 0], coef[:, 1], coef[:, 2]

        slope = np.degrees(np.arctan(np.hypot(b, c)))

        r_max = np.full_like(h, -np.inf)
        r_min = np.full_like(h, np.inf)
        r_sq = np.zeros_like(h)
        for di, dj in offsets:
            z = shifted(h, di, dj) - h
            m = ~np.isnan(z)
            r = np.where(m, z - (a + b * dj * self.res + c * di * self.res), np.nan)
            r_max = np.fmax(r_max, r)
            r_min = np.fmin(r_min, r)
            r_sq += np.where(m, r * r, 0.0)
        step = np.where(valid, r_max - r_min, np.nan)
        rough = np.where(valid, np.sqrt(r_sq / np.maximum(s['m'], 1)), np.nan)
        slope = np.where(valid, slope, np.nan)
        return slope, step, rough, valid

    @staticmethod
    def cost(slope, step, valid, max_slope_deg=20.0, max_step_m=0.12):
        """0..100 per cell (OccupancyGrid convention), -1 unknown. 100 (lethal) past
        either limit; otherwise rises linearly with whichever limit is closer."""
        frac = np.fmax(slope / max_slope_deg, step / max_step_m)
        cost = np.full(slope.shape, UNKNOWN, dtype=np.int8)
        c = np.clip(1 + np.round(frac * 98), 1, 99)
        cost[valid] = c[valid].astype(np.int8)
        cost[valid & (frac >= 1.0)] = LETHAL
        return cost
