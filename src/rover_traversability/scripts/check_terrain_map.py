#!/usr/bin/env python3
"""
Checks a terrain_mapper dump (dump_path:=/some/file.npz) against the exact terrain
of the generated sim world (rover_description/scripts/generate_terrain_world.py).

  python3 check_terrain_map.py /tmp/terrain.npz

Reports height and slope error on the open ground (rocks excluded: the generator's
height() is the ground without rocks), and how each terrain feature was costed.
"""

import argparse
import importlib.util
import os
import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_GENERATOR = os.path.join(HERE, '..', '..', 'rover_description', 'scripts',
                                 'generate_terrain_world.py')



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('--generator', default=DEFAULT_GENERATOR)
    args = ap.parse_args()

    spec = importlib.util.spec_from_file_location('gen', args.generator)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    rocks = gen.make_rocks()  # same seed as the world, so the same rocks
    features = gen.FEATURES   # (name, x, y, radius, expected cost)

    d = np.load(args.dump)
    height, slope, cost, valid = d['height'], d['slope'], d['cost'], d['valid']
    ox, oy = d['origin']
    res = float(d['res'])
    n = height.shape[0]
    X, Y = np.meshgrid(ox + (np.arange(n) + 0.5) * res, oy + (np.arange(n) + 0.5) * res)

    near_rock = np.zeros_like(valid)
    for x, y, _, size, _ in rocks:
        near_rock |= np.hypot(X - x, Y - y) < size / 2 + 2.5 * res
    ground = valid & ~near_rock

    true_h = gen.height(X, Y)
    e = 1e-3
    gx = (gen.height(X + e, Y) - gen.height(X - e, Y)) / (2 * e)
    gy = (gen.height(X, Y + e) - gen.height(X, Y - e)) / (2 * e)
    true_slope = np.degrees(np.arctan(np.hypot(gx, gy)))

    dh = np.abs(height - true_h)[ground]
    ds = np.abs(slope - true_slope)[ground]
    print(f'Map: {n}x{n} cells of {res} m, x[{ox:.1f},{ox + n * res:.1f}] y[{oy:.1f},{oy + n * res:.1f}], '
          f'{valid.sum()} cells known')
    print(f'Height error on open ground: median {np.median(dh) * 100:.1f} cm, '
          f'95% {np.percentile(dh, 95) * 100:.1f} cm')
    print(f'Slope  error on open ground: median {np.median(ds):.1f} deg, '
          f'95% {np.percentile(ds, 95):.1f} deg')
    print()
    print(f'{"feature":19s} {"seen":>5s} {"no-go":>6s} {"mean cost":>9s}  expected')
    for name, cx, cy, r, expect in features:
        m = np.hypot(X - cx, Y - cy) < r
        seen = valid & m
        if not m.any() or seen.sum() == 0:
            print(f'{name:19s} {"0%":>5s}  (not seen)')
            continue
        c = cost[seen]
        ok = c[c < 100]
        print(f'{name:19s} {seen.sum() / m.sum() * 100:4.0f}% {np.mean(c == 100) * 100:5.0f}% '
              f'{ok.mean() if ok.size else float("nan"):9.0f}  {expect}')


if __name__ == '__main__':
    main()
