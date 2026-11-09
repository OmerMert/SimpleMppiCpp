"""
generate_paths.py - Generate simple paths for MPPI/pure pursuit testing

Generated path format: CSV (x, y, yaw, ref_v) — same format as ovalpath.csv

Options:
  1) wide_oval     — 100m x 60m smooth oval (30m radius turns)
  2) straight      — 150m straight line (simplest test)
  3) gentle_sine   — 120m length gentle S-curve
  4) large_oval    — 150m x 80m very smooth oval (50m radius turns)

Every path starts at (0,0) and heads in the +x direction.
"""
import math
import csv

RESOLUTION = 0.1   # metres/waypoint (same as ovalpath)
REF_VELOCITY = 4.0  # m/s (path ref_v - calibrated value)


def normalize_angle(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def generate_straight(length_m=150.0):
    """Straight line - simplest test. From (0,0) in the +x direction."""
    n = int(length_m / RESOLUTION)
    points = []
    for i in range(n + 1):
        x = i * RESOLUTION
        y = 0.0
        yaw = 0.0
        points.append((x, y, yaw, REF_VELOCITY))
    return points


def generate_wide_oval(length=100.0, width=60.0):
    """
    Smooth oval. Turns have radius width/2.
    Viewed from above: like a racing circuit with two semicircles + two straight segments.

    (0, 0) -> (length, 0) straight (bottom)
    (length, 0) -> (length, width) semicircle (right, 180 deg)
    (length, width) -> (0, width) straight (top, -x direction)
    (0, width) -> (0, 0) semicircle (left, 180 deg)
    """
    R = width / 2.0        # turn radius
    straight_len = length - 2 * R  # length of straight segment

    points = []

    # Segment 1: bottom straight, (0,0) -> (straight_len, 0), yaw=0
    n1 = int(straight_len / RESOLUTION)
    for i in range(n1):
        x = i * RESOLUTION
        points.append((x, 0.0, 0.0, REF_VELOCITY))

    # Segment 2: right semicircle, center (straight_len, R)
    # yaw: 0 -> pi
    arc_len = math.pi * R
    n2 = int(arc_len / RESOLUTION)
    cx, cy = straight_len, R
    for i in range(n2):
        theta = (i / n2) * math.pi      # 0 -> pi
        # Point on circle: center + R * (cos(theta - pi/2), sin(theta - pi/2))
        # At start: (cx + R*cos(-pi/2), cy + R*sin(-pi/2)) = (cx, cy - R) = (straight_len, 0)
        angle_from_center = theta - math.pi / 2
        x = cx + R * math.cos(angle_from_center)
        y = cy + R * math.sin(angle_from_center)
        # Yaw = tangent to circle in CCW direction
        yaw = theta
        points.append((x, y, yaw, REF_VELOCITY))

    # Segment 3: top straight, (straight_len, width) -> (0, width), yaw=pi
    for i in range(n1):
        x = straight_len - i * RESOLUTION
        points.append((x, width, math.pi, REF_VELOCITY))

    # Segment 4: left semicircle, center (0, R)
    # yaw: pi -> 2*pi (= 0)
    cx, cy = 0.0, R
    for i in range(n2):
        theta = math.pi + (i / n2) * math.pi   # pi -> 2*pi
        angle_from_center = theta - math.pi / 2
        x = cx + R * math.cos(angle_from_center)
        y = cy + R * math.sin(angle_from_center)
        yaw = normalize_angle(theta)
        points.append((x, y, yaw, REF_VELOCITY))

    return points


def generate_gentle_sine(length=120.0, amplitude=5.0, periods=1.5):
    """
    S-curve (sinusoidal). Path is gentle but still has turns.
    """
    n = int(length / RESOLUTION)
    points = []
    k = 2 * math.pi * periods / length

    for i in range(n + 1):
        x = i * RESOLUTION
        y = amplitude * math.sin(k * x)
        # Yaw = tangent direction = atan(dy/dx)
        dy_dx = amplitude * k * math.cos(k * x)
        yaw = math.atan(dy_dx)
        points.append((x, y, yaw, REF_VELOCITY))

    return points


def generate_large_oval(length=150.0, width=80.0):
    """Very wide oval with 50m radius turns."""
    return generate_wide_oval(length, width)


def generate_figure_eight(a=30.0, ref_v=REF_VELOCITY):
    """Figure-eight (Bernoulli lemniscate), resampled to uniform RESOLUTION spacing.

    Harder than the oval for two reasons: curvature is never constant (the vehicle is
    always either tightening or opening the turn, so there is no steady state to settle
    into), and the path crosses itself, which the controller must handle purely from the
    forward-only waypoint window.

    The self-crossing is safe for that window: from the crossing the path only returns
    after a full lobe, far beyond the 200-point (20 m) search horizon, so the nearest
    waypoint can never jump to the other branch.

    Curvature: R_min = a/3, so a=30 m gives a 10 m minimum radius against the vehicle's
    ~4.75 m limit (L / tan(max_steer) = 2.82 / tan(0.534)).

    The whole curve is rigidly transformed so it starts at (0,0) heading +x, matching how
    the bridge spawns the car.
    """
    # dense parametric sampling, then uniform resampling by arc length
    M = 20000
    xs, ys = [], []
    for k in range(M + 1):
        t = 2.0 * math.pi * k / M
        s2 = math.sin(t) ** 2
        xs.append(a * math.cos(t) / (1.0 + s2))
        ys.append(a * math.sin(t) * math.cos(t) / (1.0 + s2))

    # cumulative arc length
    cum = [0.0]
    for i in range(1, len(xs)):
        cum.append(cum[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
    total = cum[-1]

    n = int(total / RESOLUTION)
    rx, ry = [], []
    j = 0
    for i in range(n):
        target = i * RESOLUTION
        while j < len(cum) - 2 and cum[j + 1] < target:
            j += 1
        span = cum[j + 1] - cum[j]
        f = 0.0 if span <= 0 else (target - cum[j]) / span
        rx.append(xs[j] + f * (xs[j + 1] - xs[j]))
        ry.append(ys[j] + f * (ys[j + 1] - ys[j]))

    # rigid transform: start at the origin, heading +x
    yaw0 = math.atan2(ry[1] - ry[0], rx[1] - rx[0])
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    x0, y0 = rx[0], ry[0]
    px, py = [], []
    for x, y in zip(rx, ry):
        dx, dy = x - x0, y - y0
        px.append(c * dx - s * dy)
        py.append(s * dx + c * dy)

    points = []
    for i in range(len(px)):
        nxt = (i + 1) % len(px)
        yaw = normalize_angle(math.atan2(py[nxt] - py[i], px[nxt] - px[i]))
        points.append((round(px[i], 4), round(py[i], 4), round(yaw, 6), ref_v))
    return points


def path_report(points, label):
    """Length, curvature and self-approach report - is this path drivable?"""
    n = len(points)
    length = sum(math.hypot(points[(i + 1) % n][0] - points[i][0],
                            points[(i + 1) % n][1] - points[i][1]) for i in range(n))
    # curvature from the yaw rate along the path
    rmin, kmax = float("inf"), 0.0
    for i in range(n):
        j = (i + 5) % n
        dpsi = abs(normalize_angle(points[j][2] - points[i][2]))
        ds = 5 * RESOLUTION
        k = dpsi / ds
        if k > kmax:
            kmax = k
    if kmax > 0:
        rmin = 1.0 / kmax
    print(f"  {label}: {n} waypoints, length {length:.1f} m, "
          f"min turn radius ~{rmin:.1f} m")
    return length, rmin


def save_path(points, filename):
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw", "ref_v"])
        for p in points:
            writer.writerow(p)
    print(f"  {filename}: {len(points)} waypoints, "
          f"x=[{min(p[0] for p in points):.1f}, {max(p[0] for p in points):.1f}], "
          f"y=[{min(p[1] for p in points):.1f}, {max(p[1] for p in points):.1f}]")


if __name__ == "__main__":
    import os
    os.makedirs("data", exist_ok=True)

    print("Generated paths:")

    # 1. Straight line (sanity check)
    p = generate_straight(length_m=150.0)
    save_path(p, "data/path_straight.csv")

    # 2. Gentle sine (light turns)
    p = generate_gentle_sine(length=120.0, amplitude=5.0, periods=1.5)
    save_path(p, "data/path_sine.csv")

    # 3. Wide oval (30m radius turns, 2x smoother than original)
    p = generate_wide_oval(length=100.0, width=60.0)
    save_path(p, "data/path_wide_oval.csv")

    # 4. Very wide oval (50m radius, very easy for the vehicle)
    p = generate_large_oval(length=150.0, width=80.0)
    save_path(p, "data/path_large_oval.csv")

    print("\nUsage:")
    print("  Set the PATH_CSV variable in beamng_bridge.py to the desired path:")
    print('    PATH_CSV = "data/path_straight.csv"     # simplest')
    print('    PATH_CSV = "data/path_sine.csv"         # gentle')
    print('    PATH_CSV = "data/path_wide_oval.csv"    # medium')
    print('    PATH_CSV = "data/path_large_oval.csv"   # medium-easy')
    print('    PATH_CSV = "data/ovalpath.csv"          # original (hard)')
