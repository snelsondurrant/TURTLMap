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
import json
import shutil
from pathlib import Path

from rosbags.convert.converter import create_connections_converters, generate_message_converter
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

SRC_DVL_MSGTYPE = "waterlinked_a50_ros_driver/msg/DVL"
DST_DVL_MSGTYPE = "dvl_msgs/msg/DVL"
DVL_JSON_TOPIC = "/dvl/json_data"

# Fallback for the two dvl_msgs/DVL timestamp fields when they can't be recovered from /dvl/json_data
MISSING_TIMESTAMP = -1


def recover_dvl_timestamps(bags: list[Path], expected_count: int) -> list[tuple[int, int]] | None:
    """
    Recover (time_of_validity, time_of_transmission) pairs for each DVL velocity report.

    :param bags: Source ROS1 .bag files to read /dvl/json_data from.
    :param expected_count: Number of /dvl/data messages that need a pair.
    :return: One pair per DVL message in publish order, or None if they can't be recovered.
    """
    with AnyReader(bags) as reader:
        json_conns = [c for c in reader.connections if c.topic == DVL_JSON_TOPIC]
        if not json_conns:
            return None

        timestamps = []
        for _, _, data in reader.messages(connections=json_conns):
            report = json.loads(reader.deserialize(data, json_conns[0].msgtype).data)
            if report.get("type") == "velocity":
                timestamps.append((report["time_of_validity"], report["time_of_transmission"]))

    if len(timestamps) != expected_count:
        print(f"Found {len(timestamps)} velocity reports for {expected_count} DVL messages.")
        return None
    return timestamps


def build_dvl_typestore(dvl_msgs_dir: Path):
    """
    Build a typestore with the dvl_msgs message definitions registered.

    :param dvl_msgs_dir: Directory of dvl_msgs/msg/*.msg files.
    :return: A typestore usable as the destination for DVL messages.
    """
    typestore = get_typestore(Stores.LATEST)
    for msg_path in sorted(dvl_msgs_dir.glob("*.msg")):
        name = f"dvl_msgs/msg/{msg_path.stem}"
        typestore.register(get_types_from_msg(msg_path.read_text(), name))
    return typestore


def make_dvl_converter(reader: AnyReader, dst_typestore, timestamps: list[tuple[int, int]] | None):
    """
    Build a converter from waterlinked_a50_ros_driver/DVL to dvl_msgs/DVL.

    :param reader: Reader the source messages will come from.
    :param dst_typestore: Typestore with dvl_msgs registered.
    :param timestamps: (time_of_validity, time_of_transmission) pairs, one per message.
    :return: A callable converting raw ROS1 bytes to serialized CDR bytes.
    """
    src_typestore = reader.typestore
    migrate = generate_message_converter(
        src_typestore,
        dst_typestore,
        SRC_DVL_MSGTYPE,
        DST_DVL_MSGTYPE,
        {},
        src_is2=False,
        dst_is2=True,
    )
    DVL = dst_typestore.types[DST_DVL_MSGTYPE]
    timestamps_iter = iter(timestamps) if timestamps is not None else None

    def convert(data: bytes) -> memoryview:
        migrated = dst_typestore.deserialize_cdr(migrate(data), DST_DVL_MSGTYPE)
        src_msg = src_typestore.deserialize_ros1(data, SRC_DVL_MSGTYPE)
        time_of_validity, time_of_transmission = (
            next(timestamps_iter) if timestamps_iter is not None else (MISSING_TIMESTAMP, MISSING_TIMESTAMP)
        )
        fixed = DVL(
            header=migrated.header,
            time=migrated.time,
            velocity=migrated.velocity,
            fom=migrated.fom,
            covariance=src_msg.velocity_covariance,
            altitude=migrated.altitude,
            beams=migrated.beams,
            velocity_valid=migrated.velocity_valid,
            status=migrated.status,
            time_of_validity=time_of_validity,
            time_of_transmission=time_of_transmission,
            form=migrated.form,
        )
        return dst_typestore.serialize_cdr(fixed, DST_DVL_MSGTYPE, little_endian=True)

    return convert


def convert_bags(bags: list[Path], output: Path, dvl_msgs_dir: Path) -> None:
    """
    Merge one or more ROS1 bags into a single ROS2 mcap bag.

    :param bags: Source ROS1 .bag files to merge, in any order.
    :param output: Destination ROS2 bag directory (overwritten if it exists).
    :param dvl_msgs_dir: Directory of dvl_msgs/msg/*.msg files.
    """
    if output.exists():
        shutil.rmtree(output)

    dst_typestore = build_dvl_typestore(dvl_msgs_dir)

    with AnyReader(bags) as probe:
        dvl_count = sum(c.msgcount for c in probe.connections if c.msgtype == SRC_DVL_MSGTYPE)
    timestamps = recover_dvl_timestamps(bags, dvl_count) if dvl_count else None

    with AnyReader(bags) as reader, Writer(output, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        connections = list(reader.connections)
        dvl_connections = [c for c in connections if c.msgtype == SRC_DVL_MSGTYPE]
        other_connections = [c for c in connections if c.msgtype != SRC_DVL_MSGTYPE]

        connmap, convmap = create_connections_converters(other_connections, None, reader, writer)

        if dvl_connections:
            convmap[SRC_DVL_MSGTYPE] = make_dvl_converter(reader, dst_typestore, timestamps)
            for conn in dvl_connections:
                existing = next(
                    (c for c in writer.connections if c.topic == conn.topic and c.msgtype == DST_DVL_MSGTYPE),
                    None,
                )
                if existing is None:
                    existing = writer.add_connection(conn.topic, DST_DVL_MSGTYPE, typestore=dst_typestore)
                connmap[(conn.id, conn.owner)] = existing

        for rconn, timestamp, data in reader.messages(connections=other_connections + dvl_connections):
            writer.write(connmap[(rconn.id, rconn.owner)], timestamp, convmap[rconn.msgtype](data))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Destination ROS2 bag directory")
    parser.add_argument("bags", type=Path, nargs="+", help="Source ROS1 .bag files to merge")
    parser.add_argument(
        "--dvl-msgs-dir",
        type=Path,
        required=True,
        help="Directory of dvl_msgs/msg/*.msg files",
    )
    args = parser.parse_args()

    convert_bags(args.bags, args.output, args.dvl_msgs_dir)
    print(f"Converted {len(args.bags)} bag(s) -> {args.output}")


if __name__ == "__main__":
    main()
