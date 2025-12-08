# generates lawnmower coverage waypoints based on apriltag locations 
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
        raise RuntimeError("no apriltags found in yaml")

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    return x_min, x_max, y_min, y_max


def generate_lawnmower_waypoints(x_min, x_max, y_min, y_max, margin=0.15, stripe_spacing=0.20):
    # generates lawnmower pattern

    # optional boundary padding w/ margin
    x_min_in = x_min + margin
    x_max_in = x_max - margin
    y_min_in = y_min + margin
    y_max_in = y_max - margin

    y_center = 0.5 * (y_min_in + y_max_in)

    # build stripe y vals: center, then +- stripe_spacing
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
            theta_row = 0.0
        else:
            x_start = x_max_in
            x_end = x_min_in
            theta_row = math.pi

        if i == 0:
            # first stripe go from one side to the other at the center line
            waypoints.append({"x": x_start, "y": row_y, "theta": theta_row})
        else:
            # move vertically from previous stripe to this row_y then align
            prev_wp = waypoints[-1]

            # move to new row_y 
            theta_vertical = math.pi / 2.0 if row_y > prev_wp["y"] else -math.pi / 2.0
            waypoints.append({"x": prev_wp["x"], "y": row_y, "theta": theta_vertical})

            # rotate to stripe direction at x_start
            waypoints.append({"x": x_start, "y": row_y, "theta": theta_row})

        # main sweep of this stripe
        waypoints.append({"x": x_end, "y": row_y, "theta": theta_row})

    return waypoints


def main():
    # parameterized for quick adjustments
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", required=True,
                        help="path to apriltags_position.yaml")
    parser.add_argument("--output", required=True,
                        help="path to output JSON waypoint file")
    parser.add_argument("--margin", type=float, default=0.15,
                        help="margin from each boundary ")
    parser.add_argument("--stripe_spacing", type=float, default=0.20,
                        help="spacing between lawnmower stripes ")
    args = parser.parse_args()

    x_min, x_max, y_min, y_max = load_tag_map(args.yaml)
    print(f"workspace from tags: x in [{x_min:.3f}, {x_max:.3f}], "
          f"y in [{y_min:.3f}, {y_max:.3f}]")

    wps = generate_lawnmower_waypoints(
        x_min, x_max, y_min, y_max,
        margin=args.margin,
        stripe_spacing=args.stripe_spacing
    )
    print(f"generated {len(wps)} waypoints")

    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    with open(args.output, "w") as f:
        json.dump(wps, f, indent=2)

    print(f"saved waypoints to: {args.output}")


if __name__ == "__main__":
    main()
