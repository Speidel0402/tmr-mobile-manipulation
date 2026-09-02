#!/usr/bin/env python3
"""Plate specialization of the proven streamed left-arm pick cycle."""

import argparse
import sys

import numpy as np

import run_streamed_live_pick_cycle as pick
from detect_plate import detect_plate


# Initial plate calibration value.  It is deliberately exposed as a bounded
# argument so the first physical plate trial can adjust only Z without
# changing the proven open-align-down-close-lift ordering.
PLATE_DESCENT_M = 0.375
MIN_PLATE_DESCENT_M = 0.340
MAX_PLATE_DESCENT_M = 0.380


def plate_right_once(image):
    best, candidates = detect_plate(image)
    point = np.asarray([best["right_rim_x"], best["right_rim_y"]], dtype=float)
    return point, {"plate": best, "candidate_count": len(candidates)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--descent-m",
        type=float,
        default=PLATE_DESCENT_M,
        help="fixed vertical plate descent after visual alignment",
    )
    parser.add_argument(
        "--force-restore-top",
        action="store_true",
        help="restore the recorded successful top joints before alignment",
    )
    args = parser.parse_args()
    if not MIN_PLATE_DESCENT_M <= args.descent_m <= MAX_PLATE_DESCENT_M:
        parser.error(
            f"--descent-m must be within "
            f"[{MIN_PLATE_DESCENT_M:.3f}, {MAX_PLATE_DESCENT_M:.3f}]"
        )

    pick.cup_right_once = plate_right_once
    pick.DESCENT_M = float(args.descent_m)
    pick.GRASP_Z = pick.REFERENCE_Z - pick.DESCENT_M
    sys.argv = [sys.argv[0]] + (
        ["--force-restore-top"] if args.force_restore_top else []
    )
    pick.main()


if __name__ == "__main__":
    main()
