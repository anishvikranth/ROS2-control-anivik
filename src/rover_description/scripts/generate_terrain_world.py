#!/usr/bin/env python3
"""
Generates the terrain test world for terrain_aruco.launch.py:
  models/rover_terrain/        static ground mesh (OBJ) used for both visual and collision
  worlds/terrain_world.sdf     terrain + rock garden + boulders + 3 ArUco markers
  config/terrain_markers.yaml  the markers' x/y/yaw for goal_generator's fallback goals

Course, start at (0,0) facing +x:
  start -> marker 0 (6.5,0)      small hill, ~14 deg
  marker 0 -> marker 1 (15,-1)   trench (0.3 m deep, ~18 deg walls, crossable), then rock garden
  marker 1 -> marker 2 (24,2)    big hill, 1 m tall, ~19 deg; hides marker 2 from marker 1 on
                                 purpose, so the rover drives on the fallback coordinates and
                                 picks the marker up near the top
  off the direct path            crater (~33 deg walls), steep mound (~46 deg), boulders, a dune

Re-run after changing anything below:
  python3 src/rover_description/scripts/generate_terrain_world.py
"""

import math
import os
import numpy as np

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
rng = np.random.default_rng(7)  # fixed seed so the world is reproducible

# ---------------------------------------------------------------------------
# Terrain height field
# ---------------------------------------------------------------------------

X_MIN, X_MAX, Y_MIN, Y_MAX = -6.0, 32.0, -12.0, 12.0
RES = 0.2  # mesh grid spacing (m)

# (cx, cy, height, spread): z += height * exp(-r^2 / spread)
# max slope of such a bump = atan(|height| * sqrt(2/spread) * e^-0.5)
BUMPS = [
    (3.0, 0.0, 0.4, 2.0),     # small hill on the way to marker 0       (~14 deg)
    (19.5, 0.5, 1.0, 6.0),    # big hill between marker 1 and 2         (~19 deg)
    (8.0, -7.0, -1.2, 2.5),   # crater, off path                        (~33 deg walls)
    (18.0, -7.5, 1.5, 1.5),   # steep mound, off path                   (~46 deg)
    (26.0, 7.5, 0.8, 8.0),    # broad dune, off path                    (~14 deg)
]

# Trenches: (x, y_start, y_end, depth, bottom_width, wall_slope_deg). A straight ditch
# along y at x with rounded ends, flat bottom and straight walls. Walls are kept under
# terrain_mapper's 20 deg no-go limit so the trench reads (and drives) as crossable.
TRENCHES = [
    (9.0, -3.0, 3.0, 0.3, 0.6, 18.0),  # across the leg from marker 0 to marker 1
]

ROCK_GARDEN = (11.0, 13.5, -1.6, 0.8)  # x_min, x_max, y_min, y_max

MARKERS = [  # (model, name, x, y, yaw) -- yaws as in aruco_world.sdf
    ('marker_0', 'target_marker_0', 6.5, 0.0, 0.0),
    ('marker_1', 'target_marker_1', 15.0, -1.0, -0.95),
    ('marker_2', 'marker2', 24.0, 2.0, 1.12),
]
MARKER_HEIGHT = 0.25  # marker centre above ground, as in aruco_world.sdf
CAMERA_HEIGHT = 0.78  # ZED above ground on rover_sim.urdf (chassis ~0.48 + mount 0.30)

# Garden rock size (m). Square-edged rocks taller than the wheel radius (0.12 m) stall the
# wheels; a 0.14 m box wedged the rover in testing. Raise the upper bound to make it harder.
ROCK_SIZE_RANGE = (0.05, 0.11)
BOULDERS = [(4.0, 4.0, 1.2), (12.0, 4.5, 1.0), (17.0, -4.0, 1.4), (25.0, -4.0, 0.9)]  # x, y, diameter

# Regions check_terrain_map.py reports on: (name, x, y, radius, what the cost should say)
FEATURES = [
    ('flat start pad', 0.0, 0.0, 1.0, 'cheap'),
    ('small hill ~14 deg', 3.0, 0.0, 2.5, 'passable'),
    ('trench', 9.0, 0.0, 1.2, 'passable (walls ~18 deg, under the 20 deg limit)'),
    ('rock garden', 12.2, -0.4, 1.0, 'costlier than flat; rocks <= 11 cm, under the 12 cm limit'),
    ('big hill ~19 deg', 19.5, 0.5, 3.5, 'passable, steepest ring near the limit'),
    ('crater ~33 deg', 8.0, -7.0, 3.0, 'walls no-go'),
    ('mound ~46 deg', 18.0, -7.5, 2.5, 'sides no-go'),
    ('boulder (4, 4)', 4.0, 4.0, 0.7, 'no-go'),
    ('boulder (12, 4.5)', 12.0, 4.5, 0.6, 'no-go'),
]


def trench_depth(x, y, tx, y0, y1, depth, bottom, wall_deg):
    """How far the trench lowers the ground at (x, y): distance to the trench's centre
    segment, flat bottom out to bottom/2, then a straight wall up to ground level."""
    yc = np.clip(y, y0, y1)
    r = np.hypot(x - tx, y - yc)
    run = depth / math.tan(math.radians(wall_deg))
    frac = np.clip((r - bottom / 2) / run, 0.0, 1.0)
    return depth * (1.0 - frac)


def height(x, y):
    z = 0.03 * np.sin(0.7 * x) * np.cos(0.5 * y)
    z = z + 0.02 * np.sin(1.9 * x + 1.0) * np.sin(1.3 * y)
    for cx, cy, h, s in BUMPS:
        z = z + h * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / s)
    for t in TRENCHES:
        z = z - trench_depth(x, y, *t)
    return z


def sight_clearance(x0, y0, x1, y1):
    """Lowest gap (m) between the ground and the line from the ZED at (x0, y0) to a
    marker centre at (x1, y1). Negative means the terrain hides the marker."""
    t = np.linspace(0.05, 0.95, 200)
    z0 = float(height(x0, y0)) + CAMERA_HEIGHT
    z1 = float(height(x1, y1)) + MARKER_HEIGHT
    xs, ys = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
    return float(np.min(z0 + t * (z1 - z0) - height(xs, ys)))


def max_slope_deg(h, s):
    return math.degrees(math.atan(abs(h) * math.sqrt(2.0 / s) * math.exp(-0.5)))


# ---------------------------------------------------------------------------
# Rocks: (x, y, shape, size, yaw). Small rocks in the garden, big boulders off path.
# ---------------------------------------------------------------------------

def make_rocks():
    rocks = []
    gx0, gx1, gy0, gy1 = ROCK_GARDEN
    while len(rocks) < 45:
        x, y = rng.uniform(gx0, gx1), rng.uniform(gy0, gy1)
        if any(math.hypot(x - mx, y - my) < 0.8 for _, _, mx, my, _ in MARKERS):
            continue
        if any(math.hypot(x - r[0], y - r[1]) < 0.25 for r in rocks):
            continue
        shape = 'sphere' if rng.random() < 0.5 else 'box'
        rocks.append((x, y, shape, rng.uniform(*ROCK_SIZE_RANGE), rng.uniform(0, math.pi)))
    for x, y, d in BOULDERS:
        rocks.append((x, y, 'sphere', d, 0.0))
    return rocks


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------

def write_mesh(path):
    xs = np.arange(X_MIN, X_MAX + 1e-9, RES)
    ys = np.arange(Y_MIN, Y_MAX + 1e-9, RES)
    gx, gy = np.meshgrid(xs, ys)
    gz = height(gx, gy)
    dzdy, dzdx = np.gradient(gz, RES)
    normals = np.stack([-dzdx, -dzdy, np.ones_like(gz)], axis=-1)
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)

    ny, nx = gz.shape
    with open(path, 'w') as f:
        f.write('# generated by generate_terrain_world.py\nmtllib terrain.mtl\no terrain\n')
        for x, y, z in zip(gx.ravel(), gy.ravel(), gz.ravel()):
            f.write(f'v {x:.3f} {y:.3f} {z:.4f}\n')
        for n in normals.reshape(-1, 3):
            f.write(f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n')
        f.write('usemtl regolith\n')
        for j in range(ny - 1):
            for i in range(nx - 1):
                a = j * nx + i + 1  # OBJ indices are 1-based
                b, c, d = a + 1, a + nx, a + nx + 1
                f.write(f'f {a}//{a} {b}//{b} {d}//{d}\nf {a}//{a} {d}//{d} {c}//{c}\n')
    with open(os.path.join(os.path.dirname(path), 'terrain.mtl'), 'w') as f:
        f.write('newmtl regolith\nKa 0.45 0.30 0.20\nKd 0.62 0.42 0.28\nKs 0.05 0.05 0.05\nNs 5\n')
    return nx * ny, 2 * (nx - 1) * (ny - 1)


def write_terrain_model(model_dir):
    os.makedirs(os.path.join(model_dir, 'meshes'), exist_ok=True)
    n_v, n_f = write_mesh(os.path.join(model_dir, 'meshes', 'terrain.obj'))
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write("""<?xml version="1.0"?>
<model>
  <name>rover_terrain</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Generated by scripts/generate_terrain_world.py</description>
</model>
""")
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write("""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="rover_terrain">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><mesh><uri>model://rover_terrain/meshes/terrain.obj</uri></mesh></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://rover_terrain/meshes/terrain.obj</uri></mesh></geometry>
        <material>
          <ambient>0.45 0.30 0.20 1</ambient>
          <diffuse>0.62 0.42 0.28 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")
    return n_v, n_f


def rock_sdf(i, x, y, shape, size, yaw):
    z0 = float(height(x, y))
    if shape == 'sphere':
        r = size / 2
        geom = f'<sphere><radius>{r:.3f}</radius></sphere>'
        pose = f'{x:.3f} {y:.3f} {z0 - 0.35 * r:.3f} 0 0 0'  # partly buried
    else:
        sx, sy, sz = size * rng.uniform(0.8, 1.4), size * rng.uniform(0.8, 1.4), size
        geom = f'<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'
        roll, pitch = rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3)
        pose = f'{x:.3f} {y:.3f} {z0 + 0.2 * sz:.3f} {roll:.2f} {pitch:.2f} {yaw:.2f}'
    return f"""      <collision name="rock_{i}_collision">
        <pose>{pose}</pose>
        <geometry>{geom}</geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
      <visual name="rock_{i}_visual">
        <pose>{pose}</pose>
        <geometry>{geom}</geometry>
        <material><ambient>0.35 0.33 0.30 1</ambient><diffuse>0.45 0.42 0.38 1</diffuse></material>
      </visual>
"""


def write_world(path, rocks):
    markers = ''
    for model, name, x, y, yaw in MARKERS:
        z = float(height(x, y)) + MARKER_HEIGHT
        markers += f"""    <include>
      <uri>model://{model}</uri>
      <name>{name}</name>
      <pose>{x} {y} {z:.3f} 0 0 {yaw}</pose>
    </include>
"""
    rock_xml = ''.join(rock_sdf(i, *r) for i, r in enumerate(rocks))
    with open(path, 'w') as f:
        f.write(f"""<?xml version="1.0" ?>
<!-- Generated by scripts/generate_terrain_world.py; edit the script, not this file. -->
<sdf version="1.9">
  <world name="terrain_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- Listing any plugin disables Gazebo's defaults, so all needed systems are named here.
         The Sensors system is loaded by the rover URDF itself. -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.75 0.62 0.50 1</background>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>model://rover_terrain</uri>
      <name>terrain</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <model name="rocks">
      <static>true</static>
      <link name="link">
{rock_xml}      </link>
    </model>

{markers}  </world>
</sdf>
""")


def write_marker_config(path):
    """goal_generator parameters: marker_poses as flat [id, x, y, yaw, ...], in marker order."""
    flat = []
    for i, (_, _, x, y, yaw) in enumerate(MARKERS):
        flat += [float(i), x, y, yaw]
    with open(path, 'w') as f:
        f.write('# Generated by scripts/generate_terrain_world.py; edit the script, not this file.\n')
        f.write('# Fallback goals for goal_generator: [id, x, y, yaw] per marker, flattened.\n')
        f.write('goal_generator:\n  ros__parameters:\n')
        f.write(f'    marker_poses: [{", ".join(f"{v:.2f}" for v in flat)}]\n')


def main():
    n_v, n_f = write_terrain_model(os.path.join(PKG_DIR, 'models', 'rover_terrain'))
    rocks = make_rocks()
    write_world(os.path.join(PKG_DIR, 'worlds', 'terrain_world.sdf'), rocks)
    write_marker_config(os.path.join(PKG_DIR, 'config', 'terrain_markers.yaml'))

    print(f'Terrain mesh: {n_v} vertices, {n_f} triangles, '
          f'x[{X_MIN},{X_MAX}] y[{Y_MIN},{Y_MAX}] @ {RES} m')
    for cx, cy, h, s in BUMPS:
        kind = 'crater' if h < 0 else 'hill'
        print(f'  {kind:6s} at ({cx:5.1f},{cy:5.1f}) h={h:+.2f} m  max slope {max_slope_deg(h, s):4.1f} deg')
    for tx, y0, y1, d, b, w in TRENCHES:
        width = b + 2 * d / math.tan(math.radians(w))
        print(f'  trench at x={tx:.1f}, y[{y0},{y1}]: {d} m deep, {width:.1f} m wide at the top, '
              f'{w:.0f} deg walls')
    print(f'Rocks: {len(rocks)} (garden + boulders)')
    for _, name, x, y, _ in MARKERS:
        print(f'  {name:16s} ({x}, {y}) ground z = {float(height(x, y)):+.3f}')
    print(f'Start (0,0) ground z = {float(height(0.0, 0.0)):+.3f}')
    legs = [('start', 0.0, 0.0)] + [(name, x, y) for _, name, x, y, _ in MARKERS]
    for (a, x0, y0), (b, x1, y1) in zip(legs, legs[1:]):
        c = sight_clearance(x0, y0, x1, y1)
        print(f'Line of sight {a} -> {b}: min clearance {c:+.2f} m{"  BLOCKED" if c < 0.05 else ""}')


if __name__ == '__main__':
    main()
