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
import math
from pathlib import Path

from scipy.spatial.transform import Rotation

_Q_ENU_NED = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0])
_Q_FRD_FLU = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


def convert_line(line: str, body_only: bool = False) -> str:
    """
    Convert one NED/FRD TUM pose line to ENU/FLU.

    :param line: Input line, t x y z qx qy qz qw.
    :param body_only: Flip only the FRD body frame, keeping the world frame.
    :return: The converted line in the same format.
    """
    t, x, y, z, qx, qy, qz, qw = map(float, line.split())

    q = Rotation.from_quat([qx, qy, qz, qw])
    if body_only:
        q = q * _Q_FRD_FLU
    else:
        # Convert NED -> ENU
        q = _Q_ENU_NED * q * _Q_FRD_FLU
        x, y, z = y, x, -z

    qx, qy, qz, qw = q.as_quat()
    return " ".join(f"{v:.9f}" for v in (t, x, y, z, qx, qy, qz, qw))


def convert_file(input_path: Path, output_path: Path, body_only: bool = False) -> int:
    """
    Convert a NED/FRD TUM trajectory file to ENU/FLU.

    :param input_path: Input trajectory file, one TUM pose per line.
    :param output_path: Destination for the converted trajectory.
    :param body_only: Flip only the FRD body frame, keeping the world frame.
    :return: Number of poses converted.
    """
    poses = 0
    with input_path.open() as infile, output_path.open("w") as outfile:
        for line in infile:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            outfile.write(convert_line(line, body_only) + "\n")
            poses += 1
    return poses


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="NED/FRD TUM trajectory file")
    parser.add_argument("output", type=Path, help="Converted ENU/FLU TUM file")
    parser.add_argument(
        "--body-only",
        action="store_true",
        help="Flip only the FRD body frame (for MoCap GT)",
    )
    args = parser.parse_args()

    poses = convert_file(args.input, args.output, args.body_only)
    print(f"Converted {poses} poses: {args.input} -> {args.output}")


if __name__ == "__main__":
    main()
