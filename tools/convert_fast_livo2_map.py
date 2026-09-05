#!/usr/bin/env python3
"""Convert FAST-LIVO2 keyframes and optimized IMU poses for FAST-LOCALIZATION.

FAST-LIVO2 stores local clouds in the LiDAR frame and optimized poses as
T_map_imu. FAST-LOCALIZATION expects sequential local PCDs and one T_map_lidar
pose per line in the order: tx ty tz qw qx qy qz.
"""

import argparse
import csv
import json
import math
import os
import re
import shutil
import sys
from pathlib import Path


def fail(message):
    raise RuntimeError(message)


def numeric_list(metadata_text, key, expected_count):
    pattern = re.compile(r"^\s*" + re.escape(key) + r"\s*:\s*\[([^]]+)\]\s*$")
    for line in metadata_text.splitlines():
        match = pattern.match(line)
        if not match:
            continue
        values = [float(value.strip()) for value in match.group(1).split(",")]
        if len(values) != expected_count:
            fail("{} must contain {} values, found {}".format(key, expected_count, len(values)))
        if not all(math.isfinite(value) for value in values):
            fail("{} contains a non-finite value".format(key))
        return values
    fail("Missing {} in metadata file".format(key))


def normalize_quaternion_xyzw(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        fail("Invalid zero or non-finite quaternion")
    return tuple(value / norm for value in quaternion), abs(norm - 1.0)


def quaternion_xyzw_to_matrix(quaternion):
    x, y, z, w = quaternion
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def matrix_to_quaternion_xyzw(matrix):
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = ((m21 - m12) / scale, (m02 - m20) / scale,
                      (m10 - m01) / scale, 0.25 * scale)
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m00 - m11 - m22)) * 2.0
        quaternion = (0.25 * scale, (m01 + m10) / scale,
                      (m02 + m20) / scale, (m21 - m12) / scale)
    elif m11 > m22:
        scale = math.sqrt(max(0.0, 1.0 + m11 - m00 - m22)) * 2.0
        quaternion = ((m01 + m10) / scale, 0.25 * scale,
                      (m12 + m21) / scale, (m02 - m20) / scale)
    else:
        scale = math.sqrt(max(0.0, 1.0 + m22 - m00 - m11)) * 2.0
        quaternion = ((m02 + m20) / scale, (m12 + m21) / scale,
                      0.25 * scale, (m10 - m01) / scale)
    return normalize_quaternion_xyzw(quaternion)[0]


def validate_rotation_matrix(matrix, tolerance=1e-4):
    for row in range(3):
        row_norm = sum(matrix[row][column] ** 2 for column in range(3))
        if abs(row_norm - 1.0) > tolerance:
            fail("T_imu_lidar rotation row {} is not unit length".format(row))
        for other_row in range(row + 1, 3):
            dot = sum(matrix[row][column] * matrix[other_row][column]
                      for column in range(3))
            if abs(dot) > tolerance:
                fail("T_imu_lidar rotation rows {} and {} are not orthogonal".format(
                    row, other_row))
    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]))
    if abs(determinant - 1.0) > tolerance:
        fail("T_imu_lidar rotation determinant must be +1, found {:.9f}".format(determinant))


def multiply_quaternion_xyzw(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    result = (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )
    return normalize_quaternion_xyzw(result)[0]


def rotate_vector(matrix, vector):
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3))
                 for row in range(3))


def load_poses(path):
    records = []
    seen_ids = set()
    max_quaternion_deviation = 0.0
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) != 9:
                fail("{}:{} expected 9 fields, found {}".format(path, line_number, len(fields)))
            keyframe_id = int(fields[0])
            if keyframe_id in seen_ids:
                fail("Duplicate keyframe id {} in {}".format(keyframe_id, path))
            seen_ids.add(keyframe_id)
            values = [float(value) for value in fields[1:]]
            if not all(math.isfinite(value) for value in values):
                fail("{}:{} contains a non-finite value".format(path, line_number))
            timestamp, tx, ty, tz, qx, qy, qz, qw = values
            quaternion, deviation = normalize_quaternion_xyzw((qx, qy, qz, qw))
            max_quaternion_deviation = max(max_quaternion_deviation, deviation)
            records.append({
                "id": keyframe_id,
                "timestamp": timestamp,
                "translation": (tx, ty, tz),
                "quaternion": quaternion,
            })
    if not records:
        fail("No poses found in {}".format(path))
    records.sort(key=lambda record: record["id"])
    return records, max_quaternion_deviation


def find_keyframe_cloud(keyframes, keyframe_id):
    candidates = (keyframes / ("{:06d}.pcd".format(keyframe_id)),
                  keyframes / ("{}.pcd".format(keyframe_id)))
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    fail("Missing PCD for keyframe {} (checked {})".format(
        keyframe_id, ", ".join(str(candidate) for candidate in candidates)))


def validate_pcd(path):
    with path.open("rb") as handle:
        header = handle.read(4096).decode("ascii", "ignore")
    fields_line = next((line for line in header.splitlines() if line.startswith("FIELDS ")), "")
    fields = fields_line.split()[1:]
    if not {"x", "y", "z"}.issubset(set(fields)):
        fail("PCD does not contain x/y/z fields: {}".format(path))
    if "DATA " not in header:
        fail("PCD header is incomplete: {}".format(path))
    return fields


def materialize_cloud(source, destination, mode):
    if mode == "copy":
        shutil.copy2(str(source), str(destination))
    elif mode == "hardlink":
        os.link(str(source), str(destination))
    else:
        relative_target = os.path.relpath(str(source), str(destination.parent))
        destination.symlink_to(relative_target)


def convert(args):
    keyframes = args.keyframes.resolve()
    poses_path = args.poses.resolve()
    metadata_path = (args.metadata or keyframes / "metadata.yaml").resolve()
    output = args.output.resolve()

    if not keyframes.is_dir():
        fail("Keyframe directory does not exist: {}".format(keyframes))
    if not poses_path.is_file():
        fail("Pose file does not exist: {}".format(poses_path))
    if not metadata_path.is_file():
        fail("Metadata file does not exist: {}".format(metadata_path))
    if output.exists() and any(output.iterdir()):
        fail("Output directory is not empty; choose a new directory: {}".format(output))

    metadata_text = metadata_path.read_text(encoding="utf-8")
    extrinsic_translation = numeric_list(metadata_text, "T_imu_lidar_translation", 3)
    extrinsic_rotation_values = numeric_list(metadata_text, "T_imu_lidar_rotation_row_major", 9)
    extrinsic_rotation = tuple(tuple(extrinsic_rotation_values[row * 3 + column]
                                     for column in range(3)) for row in range(3))
    validate_rotation_matrix(extrinsic_rotation)
    extrinsic_quaternion = matrix_to_quaternion_xyzw(extrinsic_rotation)

    poses, max_quaternion_deviation = load_poses(poses_path)
    prepared = []
    common_fields = None
    for output_id, pose in enumerate(poses):
        cloud = find_keyframe_cloud(keyframes, pose["id"])
        fields = validate_pcd(cloud)
        if common_fields is None:
            common_fields = fields
        elif fields != common_fields:
            fail("Inconsistent PCD fields in {}: {} != {}".format(cloud, fields, common_fields))

        rotation_map_imu = quaternion_xyzw_to_matrix(pose["quaternion"])
        rotated_extrinsic_translation = rotate_vector(rotation_map_imu, extrinsic_translation)
        translation_map_lidar = tuple(
            pose["translation"][axis] + rotated_extrinsic_translation[axis] for axis in range(3))
        quaternion_map_lidar = multiply_quaternion_xyzw(
            pose["quaternion"], extrinsic_quaternion)
        prepared.append((output_id, pose, cloud, translation_map_lidar, quaternion_map_lidar))

    output.mkdir(parents=True, exist_ok=True)
    pcd_output = output / "pcd"
    pcd_output.mkdir()

    pose_output = output / "pose.json"
    manifest_output = output / "manifest.csv"
    with pose_output.open("w", encoding="utf-8") as pose_handle, \
            manifest_output.open("w", encoding="utf-8", newline="") as manifest_handle:
        manifest = csv.writer(manifest_handle)
        manifest.writerow(("output_id", "source_id", "timestamp", "source_pcd", "output_pcd"))
        for output_id, pose, cloud, translation, quaternion in prepared:
            qx, qy, qz, qw = quaternion
            pose_handle.write(
                "{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(
                    translation[0], translation[1], translation[2], qw, qx, qy, qz))
            destination = pcd_output / ("{}.pcd".format(output_id))
            materialize_cloud(cloud, destination, args.mode)
            manifest.writerow((output_id, pose["id"], "{:.9f}".format(pose["timestamp"]),
                               str(cloud), str(destination)))

    report = {
        "format": "FAST-LOCALIZATION map",
        "keyframe_count": len(prepared),
        "cloud_mode": args.mode,
        "source_keyframes": str(keyframes),
        "source_poses": str(poses_path),
        "source_metadata": str(metadata_path),
        "pose_composition": "T_map_lidar = T_map_imu * T_imu_lidar",
        "pose_output_order": "tx ty tz qw qx qy qz",
        "pcd_fields": common_fields,
        "max_input_quaternion_norm_deviation": max_quaternion_deviation,
        "T_imu_lidar_translation": extrinsic_translation,
        "T_imu_lidar_rotation_row_major": extrinsic_rotation_values,
    }
    (output / "conversion_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def parse_arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keyframes", required=True, type=Path,
                        help="FAST-LIVO2 local LiDAR keyframe directory")
    parser.add_argument("--poses", required=True, type=Path,
                        help="optimized_keyframe_poses_imu.txt")
    parser.add_argument("--metadata", type=Path,
                        help="metadata.yaml (defaults to KEYFRAMES/metadata.yaml)")
    parser.add_argument("--output", required=True, type=Path,
                        help="new FAST-LOCALIZATION map directory")
    parser.add_argument("--mode", choices=("symlink", "hardlink", "copy"), default="symlink",
                        help="how to materialize PCDs (default: symlink)")
    return parser.parse_args(argv)


def main(argv=None):
    try:
        report = convert(parse_arguments(argv))
    except (OSError, ValueError, RuntimeError) as error:
        print("error: {}".format(error), file=sys.stderr)
        return 1
    print("Converted {} keyframes using {} mode.".format(
        report["keyframe_count"], report["cloud_mode"]))
    print("Pose composition: {}".format(report["pose_composition"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
