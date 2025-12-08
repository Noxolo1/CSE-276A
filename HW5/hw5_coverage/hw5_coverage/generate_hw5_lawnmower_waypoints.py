#!/usr/bin/env python3
"""
Generate HW5 lawnmower coverage waypoints based on the AprilTag map.

Assumptions:
- The map/world frame origin (0, 0) is at the physical center of the 8x8ft
  workspace where you place the robot at the start.
- apriltags_position.yaml gives tag positions in that same map frame.

Usage example:

  cd ~/ros2_ws/rubikpi_ros2/hw5_coverage/hw5_coverage
  python3 generate_hw5_lawnmower_waypoints.py \
      --yaml apriltags_position.yaml \
      --output hw5_waypoints_lawnmower.json \
      --margin 0.15 \
      --stripe_spacing 0.20
"""

import argparse
import json
import math
import os
import yaml


def load_tag_map(yaml_path):
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    tags = data.get("apriltags", [])
    xs, ys = [], []
    for tag in tags:
        xs.append(float(tag["x"]))
        ys.append(float(tag["y"]))
    if not xs or not ys:
        raise RuntimeError("No apriltags found in YAML.")

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    return x_min, x_max, y_min, y_max


def generate_lawnmower_waypoints(x_min, x_max, y_min, y_max,
                                 margin=0.15, stripe_spacing=0.20):
    """
    Generate a boustrophedon (lawnmower) pattern.

    - Shrinks workspace by 'margin' from each side.
    - First stripe is centered in Y (i.e., through the workspace center).
    - Stripes then alternate above/below that center line.
    - Even stripes go left->right in X; odd stripes right->left.
    """

    # Shrink bounds by margin
    x_min_in = x_min + margin
    x_max_in = x_max - margin
    y_min_in = y_min + margin
    y_max_in = y_max - margin

    if x_min_in >= x_max_in or y_min_in >= y_max_in:
        raise RuntimeError("Margin too large; no interior workspace remains.")

    # Compute "center line" in Y (approx origin if map is centered at (0,0))
    y_center = 0.5 * (y_min_in + y_max_in)

    # Build stripe Y-values: center, then ± stripe_spacing
    ys = []
    ys.append(y_center)
    k = 1
    while True:
        added = False
        y_up = y_center + k * stripe_spacing
        y_down = y_center - k * stripe_spacing

        if y_up <= y_max_in + 1e-6:
            ys.append(y_up)
            added = True
        if y_down >= y_min_in - 1e-6:
            ys.append(y_down)
            added = True

        if not added:
            break
        k += 1

    waypoints = []

    for i, row_y in enumerate(ys):
        even = (i % 2 == 0)
        if even:
            x_start = x_min_in
            x_end = x_max_in
            theta_row = 0.0        # facing +x
        else:
            x_start = x_max_in
            x_end = x_min_in
            theta_row = math.pi    # facing -x

        if i == 0:
            # First stripe: go from one side to the other at the center line.
            # Robot starts near (0, 0); first command will be to drive toward
            # (x_start, y_center).
            waypoints.append({"x": x_start, "y": row_y, "theta": theta_row})
        else:
            # Connector: move vertically from previous stripe to this row_y,
            # then align along stripe direction.
            prev_wp = waypoints[-1]

            # Vertical move to new row_y (same x as previous)
            theta_vertical = math.pi / 2.0 if row_y > prev_wp["y"] else -math.pi / 2.0
            waypoints.append({"x": prev_wp["x"], "y": row_y, "theta": theta_vertical})

            # Rotate to stripe direction at x_start
            waypoints.append({"x": x_start, "y": row_y, "theta": theta_row})

        # Main sweep of this stripe
        waypoints.append({"x": x_end, "y": row_y, "theta": theta_row})

    return waypoints


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", required=True,
                        help="Path to apriltags_position.yaml")
    parser.add_argument("--output", required=True,
                        help="Path to output JSON waypoint file")
    parser.add_argument("--margin", type=float, default=0.15,
                        help="Margin from each boundary (m)")
    parser.add_argument("--stripe_spacing", type=float, default=0.20,
                        help="Spacing between lawnmower stripes (m)")
    args = parser.parse_args()

    x_min, x_max, y_min, y_max = load_tag_map(args.yaml)
    print(f"Workspace from tags: x in [{x_min:.3f}, {x_max:.3f}], "
          f"y in [{y_min:.3f}, {y_max:.3f}]")

    wps = generate_lawnmower_waypoints(
        x_min, x_max, y_min, y_max,
        margin=args.margin,
        stripe_spacing=args.stripe_spacing
    )
    print(f"Generated {len(wps)} waypoints.")

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    with open(args.output, "w") as f:
        json.dump(wps, f, indent=2)

    print(f"Saved waypoints to: {args.output}")


if __name__ == "__main__":
    main()
