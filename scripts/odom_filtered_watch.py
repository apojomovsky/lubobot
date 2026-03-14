#!/usr/bin/env python3

import math
import sys


def main() -> int:
    values: dict[str, float] = {}
    mode = None

    for raw_line in sys.stdin:
        line = raw_line.strip()

        if line == "position:":
            mode = "pos"
            continue
        if line == "orientation:":
            mode = "ori"
            continue

        if line.startswith("x:"):
            values[f"{mode}_x"] = float(line.split(":", 1)[1])
            continue
        if line.startswith("y:"):
            values[f"{mode}_y"] = float(line.split(":", 1)[1])
            continue
        if line.startswith("z:"):
            values[f"{mode}_z"] = float(line.split(":", 1)[1])
            continue
        if line.startswith("w:"):
            values[f"{mode}_w"] = float(line.split(":", 1)[1])

            x = values.get("ori_x", 0.0)
            y = values.get("ori_y", 0.0)
            z = values.get("ori_z", 0.0)
            w = values.get("ori_w", 1.0)

            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            print(
                f"x={values.get('pos_x', 0.0): .3f} "
                f"y={values.get('pos_y', 0.0): .3f} "
                f"yaw={yaw: .3f}"
            )
            values = {}

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
