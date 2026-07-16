#!/usr/bin/env python3
# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def split_segments(times: np.ndarray, max_gap: float) -> list[np.ndarray]:
    """
    Split pose indices into contiguous segments at tracking dropouts.

    :param times: Pose timestamps in seconds.
    :param max_gap: Time gap treated as a tracking dropout, in seconds.
    :return: Index arrays, one per contiguous tracking segment.
    """
    if len(times) == 0:
        return []
    breaks = np.where(np.diff(times) > max_gap)[0] + 1
    return np.split(np.arange(len(times)), breaks)


def clean_poses(
    data: np.ndarray, max_attitude: float, max_gap: float, min_duration: float
) -> tuple[np.ndarray, np.ndarray]:
    """
    Drop physically implausible poses and too-short tracking segments.

    :param data: TUM pose rows, t x y z qx qy qz qw.
    :param max_attitude: Largest plausible |roll| or |pitch|, in degrees.
    :param max_gap: Time gap treated as a tracking dropout, in seconds.
    :param min_duration: Shortest tracking segment to keep, in seconds.
    :return: The kept rows, and a keep mask over the input rows.
    """
    _, pitch, roll = Rotation.from_quat(data[:, 4:8]).as_euler("zyx", degrees=True).T
    keep = (np.abs(roll) <= max_attitude) & (np.abs(pitch) <= max_attitude)

    kept = data[keep]
    keep_kept = np.ones(len(kept), dtype=bool)
    for segment in split_segments(kept[:, 0], max_gap):
        if kept[segment[-1], 0] - kept[segment[0], 0] < min_duration:
            keep_kept[segment] = False

    mask = np.zeros(len(data), dtype=bool)
    mask[np.flatnonzero(keep)[keep_kept]] = True
    return data[mask], mask


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="ENU/FLU ground truth TUM file")
    parser.add_argument("output", type=Path, help="Cleaned TUM file")
    parser.add_argument(
        "--max-attitude",
        type=float,
        default=20.0,
        help="Largest plausible |roll| or |pitch| in degrees",
    )
    parser.add_argument(
        "--max-gap",
        type=float,
        default=0.5,
        help="Time gap treated as a tracking dropout in seconds",
    )
    parser.add_argument(
        "--min-duration",
        type=float,
        default=2.0,
        help="Shortest tracking segment to keep in seconds",
    )
    args = parser.parse_args()

    data = np.loadtxt(args.input, comments="#", ndmin=2)
    kept, mask = clean_poses(data, args.max_attitude, args.max_gap, args.min_duration)

    t0 = data[0, 0]
    for segment in split_segments(data[~mask][:, 0], args.max_gap):
        start, end = data[~mask][segment[[0, -1]], 0] - t0
        print(f"Dropped {len(segment)} poses at t={start:.1f}s - {end:.1f}s")

    np.savetxt(args.output, kept, fmt="%.9f")
    print(f"Kept {len(kept)}/{len(data)} poses: {args.input} -> {args.output}")


if __name__ == "__main__":
    main()
