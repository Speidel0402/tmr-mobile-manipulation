#!/usr/bin/env python3
"""Food-bowl specialization of the proven streamed left-arm pick cycle."""

import numpy as np

import run_streamed_live_pick_cycle as pick
from detect_food_bowl import detect_food_bowl


FOOD_BOWL_DESCENT_M = 0.360


def food_bowl_right_once(image):
    best, candidates = detect_food_bowl(image)
    point = np.asarray([best["right_rim_x"], best["right_rim_y"]], dtype=float)
    return point, {"food_bowl": best, "candidate_count": len(candidates)}


def main():
    # Keep the cup workflow unchanged while substituting only the target
    # detector and the empirically requested food-bowl descent.
    pick.cup_right_once = food_bowl_right_once
    pick.DESCENT_M = FOOD_BOWL_DESCENT_M
    pick.GRASP_Z = pick.REFERENCE_Z - FOOD_BOWL_DESCENT_M
    pick.main()


if __name__ == "__main__":
    main()
