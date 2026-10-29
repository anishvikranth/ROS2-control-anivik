"""
terrain_grid.py
===============
Rover-centred 2.5D height grid and traversability layers, plain numpy (no ROS),
so it can be tested offline against a known terrain.

Grid layout: row i is y, column j is x, cell (i, j) covers
  x in [origin_x + j*res, origin_x + (j+1)*res), y likewise with origin_y and i.
That is the nav_msgs/OccupancyGrid layout, so cost.ravel() is the message data.

Each cell holds a height and its variance. Every point comes with its own variance
(terrain_mapper derives it from the point's range: stereo depth error grows with
range^2), and cells are fused like a 1-D Kalman filter, so a reading's influence is
1/variance: a close, precise reading replaces a far, noisy one almost completely, and
far readings can't wash out good close ones. A little process noise is added on every
update so a cell can still follow real change (and a moved rock gets re-learned).

Layers, all from a plane fitted (weighted by 1/variance) to the cells in a
(2*window+1)^2-cell window around each cell:
  slope   angle of that plane from horizontal (deg)
  step    highest minus lowest height *relative to the plane* in the window (m),
          each cell's residual shrunk by 2 sigma first: the step that is certain to be
          there, so noisy far cells don't invent rocks. Measuring against the plane
          (not raw max - min) keeps a smooth slope from reading as a step.
  rough   RMS distance of the heights from the plane (m), weighted like the fit
  sigma   the cell's height standard deviation (m)
Cells with sigma above max_sigma are left out of the layers (treated as unknown).
"""

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

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
    def __init__(self, resolution=0.15, size_m=20.0, process_noise=0.01, min_sigma=0.005):
        """process_noise (m): added (as variance) to a cell each time it is updated, so it
        never becomes so certain that it stops responding. min_sigma (m): floor on a
        cell's uncertainty."""
        self.res = resolution
        self.n = int(round(size_m / resolution))
        self.size_m = self.n * resolution
        self.process_var = process_noise ** 2
        self.min_var = min_sigma ** 2
        self.origin_x = -self.size_m / 2
        self.origin_y = -self.size_m / 2
        self.height = np.full((self.n, self.n), np.nan)     # fused ground height (m), NaN = never seen
        self.variance = np.full((self.n, self.n), np.inf)   # its variance (m^2), inf = never seen

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
        self.variance = shifted(self.variance, di, dj, np.inf)
        self.origin_x, self.origin_y = new_ox, new_oy

    def fuse(self, points, variances):
        """Adds one frame of ground points (N x 3, map frame) with per-point height
        variances (N, m^2).

        Per cell, this frame's estimate is the inverse-variance weighted mean of its
        points, with the harmonic mean of their variances as its variance (not divided
        by the point count: points in one cell aren't independent measurements of one
        height). It is then fused into the cell's history with a Kalman update."""
        j = np.floor((points[:, 0] - self.origin_x) / self.res).astype(np.int64)
        i = np.floor((points[:, 1] - self.origin_y) / self.res).astype(np.int64)
        inside = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        flat = i[inside] * self.n + j[inside]
        inv_var = 1.0 / variances[inside]
        size = self.n * self.n
        count = np.bincount(flat, minlength=size).reshape(self.n, self.n)
        sum_w = np.bincount(flat, weights=inv_var, minlength=size).reshape(self.n, self.n)
        sum_wz = np.bincount(flat, weights=inv_var * points[inside, 2], minlength=size).reshape(self.n, self.n)
        seen = count > 0
        frame_z = sum_wz[seen] / sum_w[seen]
        frame_var = count[seen] / sum_w[seen]

        prior_var = self.variance[seen] + self.process_var   # inf for never-seen cells
        new = ~np.isfinite(prior_var)
        with np.errstate(invalid='ignore'):                  # inf arithmetic in the discarded branch
            gain = np.where(new, 1.0, prior_var / (prior_var + frame_var))
            post_var = np.where(new, frame_var, (1.0 - gain) * prior_var)
        old_h = np.where(new, 0.0, self.height[seen])
        self.height[seen] = old_h + gain * (frame_z - old_h)
        self.variance[seen] = np.maximum(post_var, self.min_var)
        return int(seen.sum())

    def cell_centres(self):
        xs = self.origin_x + (np.arange(self.n) + 0.5) * self.res
        ys = self.origin_y + (np.arange(self.n) + 0.5) * self.res
        return np.meshgrid(xs, ys)

    # ------------------------------------------------------------------
    # Layers
    # ------------------------------------------------------------------
    def layers(self, window=2, min_points=6, max_sigma=0.2):
        """Returns (slope_deg, step_m, rough_m, valid, sigma_m). valid is False where the
        cell is unknown or too uncertain (sigma > max_sigma), or its window has fewer than
        min_points usable cells.

        Every cell's window is a view into the padded grid (sliding_window_view, no copy), so
        the whole computation is a few dozen large numpy operations. A per-offset Python loop
        made ~1000 small calls, and inside the multi-threaded ROS node each one waited for the
        GIL: an update took 2-6 s there instead of milliseconds."""
        k = 2 * window + 1
        usable = np.isfinite(self.variance) & (self.variance <= max_sigma ** 2)
        h = np.where(usable, self.height, np.nan)
        w_cell = np.where(usable, 1.0 / np.where(usable, self.variance, 1.0), 0.0)
        sigma = np.sqrt(self.variance)

        def windows(a, fill):
            return sliding_window_view(np.pad(a, window, constant_values=fill), (k, k))   # (n, n, k, k)

        # Neighbour offsets within a window, in metres (rows are y, columns x)
        DY, DX = np.meshgrid(np.arange(-window, window + 1) * self.res,
                             np.arange(-window, window + 1) * self.res, indexing='ij')

        # Weighted plane z = a + b*dx + c*dy around each cell, heights relative to the
        # centre cell for conditioning.
        dz = windows(h, np.nan) - h[..., None, None]
        m = ~np.isnan(dz)
        W = np.where(m, windows(w_cell, 0.0), 0.0)
        Z = np.where(m, dz, 0.0)
        WZ = W * Z
        s = {
            'n': m.sum(axis=(-2, -1)),
            'w': W.sum(axis=(-2, -1)),
            'x': (W * DX).sum(axis=(-2, -1)),
            'y': (W * DY).sum(axis=(-2, -1)),
            'xx': (W * DX * DX).sum(axis=(-2, -1)),
            'yy': (W * DY * DY).sum(axis=(-2, -1)),
            'xy': (W * DX * DY).sum(axis=(-2, -1)),
            'z': WZ.sum(axis=(-2, -1)),
            'xz': (WZ * DX).sum(axis=(-2, -1)),
            'yz': (WZ * DY).sum(axis=(-2, -1)),
        }
        valid = usable & (s['n'] >= min_points)
        a, b, c = self._solve_planes(s, valid)
        slope = np.degrees(np.arctan(np.hypot(b, c)))

        # Step: residuals shrunk towards the plane by 2 sigma, so only height difference
        # that is certain to be there counts.
        with np.errstate(invalid='ignore'):
            R = dz - (a[..., None, None] + b[..., None, None] * DX + c[..., None, None] * DY)
            S2 = 2.0 * windows(sigma, np.inf)
            up = np.max(np.where(m, R - S2, -np.inf), axis=(-2, -1))
            down = np.min(np.where(m, R + S2, np.inf), axis=(-2, -1))
            r_sq = (W * np.where(m, R, 0.0) ** 2).sum(axis=(-2, -1))
            step = np.where(valid, np.maximum(up - down, 0.0), np.nan)
        rough = np.where(valid, np.sqrt(r_sq / np.where(s['w'] > 0, s['w'], 1.0)), np.nan)
        slope = np.where(valid, slope, np.nan)
        return slope, step, rough, valid, np.where(np.isfinite(sigma), sigma, np.nan)

    @staticmethod
    def _solve_planes(s, valid):
        """Solves every cell's 3x3 normal equations
            [w  x  y ] [a]   [z ]
            [x  xx xy] [b] = [xz]
            [y  xy yy] [c]   [yz]
        in closed form (Cramer's rule), elementwise over the grid. np.linalg.solve on a
        stack of 3x3s goes through LAPACK/BLAS threads, which inside the ROS node took
        1-2 s per update instead of milliseconds."""
        w = np.where(valid, s['w'], 1.0)            # normalise so the 1e-9 nudge is relative
        A11, A12, A13 = np.ones_like(w), s['x'] / w, s['y'] / w
        A22, A23, A33 = s['xx'] / w + 1e-9, s['xy'] / w, s['yy'] / w + 1e-9
        r1, r2, r3 = s['z'] / w, s['xz'] / w, s['yz'] / w
        c11 = A22 * A33 - A23 * A23                 # cofactors of the symmetric matrix
        c12 = A13 * A23 - A12 * A33
        c13 = A12 * A23 - A13 * A22
        c22 = A11 * A33 - A13 * A13
        c23 = A12 * A13 - A11 * A23
        c33 = A11 * A22 - A12 * A12
        det = A11 * c11 + A12 * c12 + A13 * c13
        det = np.where(valid & (np.abs(det) > 1e-18), det, np.inf)
        a = (c11 * r1 + c12 * r2 + c13 * r3) / det
        b = (c12 * r1 + c22 * r2 + c23 * r3) / det
        c = (c13 * r1 + c23 * r2 + c33 * r3) / det
        return a, b, c

    def copy(self):
        """Independent copy of the grid (for computing layers while new clouds arrive)."""
        g = TerrainGrid.__new__(TerrainGrid)
        g.__dict__.update(self.__dict__)
        g.height, g.variance = self.height.copy(), self.variance.copy()
        return g

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
