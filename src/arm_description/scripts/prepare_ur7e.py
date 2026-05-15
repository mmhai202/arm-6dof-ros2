#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
from typing import Dict, Iterable, List, Tuple

import numpy as np
import yaml
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool


ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION_ROOT = ROOT / "src" / "arm_description"
DEFAULT_STEP_FILE = ROOT / "UR7e.step" / "UR7e.step"
DEFAULT_OUTPUT_DIR = DESCRIPTION_ROOT / "meshes" / "ur7e"
DEFAULT_MANIFEST = DEFAULT_OUTPUT_DIR / "ur7e_manifest.yaml"
DEFAULT_REPORT_DIR = DEFAULT_OUTPUT_DIR / "reports"

PI = math.pi
KINEMATICS = {
    "shoulder_z": 0.1625,
    "upper_arm_x": -0.425,
    "forearm_x": -0.3922,
    "forearm_z": 0.1333,
    "wrist_1_y": -0.0997,
    "wrist_2_y": 0.0996,
}
DEFAULT_REFERENCE_JOINTS = {
    "joint_1": 0.0,
    "joint_2": -PI / 2.0,
    "joint_3": 0.0,
    "joint_4": -PI / 2.0,
    "joint_5": 0.0,
    "joint_6": 0.0,
}
STEP_TO_ROS_ROT = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)
ROS_TO_STEP_ROT = STEP_TO_ROS_ROT.T
STEP_TO_ROS_RPY = [-PI / 2.0, 0.0, 0.0]

DEFAULT_MANIFEST_DATA = {
    "metadata": {
        "robot_name": "ur7e",
        "step_file": "UR7e.step/UR7e.step",
        "units": "mm",
        "step_to_ros_rpy": STEP_TO_ROS_RPY,
        "reference_pose": DEFAULT_REFERENCE_JOINTS,
    },
    "links": [
        {
            "link_name": "base_link",
            "component_index": 1,
            "source_step_name": "C-1000257",
            "raw_mesh_path": "raw/component_01.stl",
            "local_mesh_path": "local/base_link.stl",
            "joint_frame_xyz": [0.0, 0.0, 0.0],
            "joint_frame_rpy": [0.0, 0.0, 0.0],
            "status": "locked",
            "notes": "Base pedestal visual from STEP assembly.",
        },
        {
            "link_name": "link_1",
            "component_index": 2,
            "source_step_name": "C-1000248",
            "raw_mesh_path": "raw/component_02.stl",
            "local_mesh_path": "local/link_1.stl",
            "joint_frame_xyz": [0.0, 0.0, 0.1625],
            "joint_frame_rpy": [0.0, 0.0, 0.0],
            "status": "locked",
            "notes": "Shoulder housing visual.",
        },
        {
            "link_name": "link_2",
            "component_index": 3,
            "source_step_name": "C-1000249",
            "raw_mesh_path": "raw/component_03.stl",
            "local_mesh_path": "local/link_2.stl",
            "joint_frame_xyz": [0.0, 0.0, 0.0],
            "joint_frame_rpy": [PI / 2.0, 0.0, 0.0],
            "status": "locked",
            "notes": "Upper-arm casting visual.",
        },
        {
            "link_name": "link_3",
            "component_index": 4,
            "source_step_name": "C-1000250",
            "raw_mesh_path": "raw/component_04.stl",
            "local_mesh_path": "local/link_3.stl",
            "joint_frame_xyz": [-0.425, 0.0, 0.0],
            "joint_frame_rpy": [0.0, 0.0, 0.0],
            "status": "locked",
            "notes": "Forearm casting visual.",
        },
        {
            "link_name": "link_4",
            "component_index": 5,
            "source_step_name": "C-1000251",
            "raw_mesh_path": "raw/component_05.stl",
            "local_mesh_path": "local/link_4.stl",
            "joint_frame_xyz": [-0.3922, 0.0, 0.1333],
            "joint_frame_rpy": [0.0, 0.0, 0.0],
            "status": "locked",
            "notes": "Wrist-1 visual.",
        },
        {
            "link_name": "link_5",
            "component_index": 7,
            "source_step_name": "C-2007861",
            "raw_mesh_path": "raw/component_07.stl",
            "local_mesh_path": "local/link_5.stl",
            "joint_frame_xyz": [0.0, -0.0997, 0.0],
            "joint_frame_rpy": [PI / 2.0, 0.0, 0.0],
            "status": "locked",
            "notes": "Wrist-2 visual.",
        },
        {
            "link_name": "link_6",
            "component_index": 6,
            "source_step_name": "C-2007038",
            "raw_mesh_path": "raw/component_06.stl",
            "local_mesh_path": "local/link_6.stl",
            "joint_frame_xyz": [0.0, 0.0996, 0.0],
            "joint_frame_rpy": [PI / 2.0, PI, PI],
            "status": "locked",
            "notes": "Wrist-3 visual.",
        },
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare exact UR7e visual meshes from the STEP assembly."
    )
    parser.add_argument("--step", type=Path, default=DEFAULT_STEP_FILE)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT_DIR)
    parser.add_argument(
        "--component-index",
        type=int,
        help="Only export one assembly component as a raw STL.",
    )
    parser.add_argument(
        "--force-export",
        action="store_true",
        help="Re-export raw STEP meshes even if the raw STL already exists.",
    )
    parser.add_argument(
        "--skip-local",
        action="store_true",
        help="Export raw meshes without generating link-local visual meshes.",
    )
    return parser.parse_args()


def ensure_default_manifest(manifest_path: Path) -> None:
    if manifest_path.exists():
        return
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    with open(manifest_path, "w", encoding="utf-8") as file_handle:
        yaml.safe_dump(DEFAULT_MANIFEST_DATA, file_handle, sort_keys=False)


def read_manifest(manifest_path: Path) -> dict:
    ensure_default_manifest(manifest_path)
    with open(manifest_path, "r", encoding="utf-8") as file_handle:
        manifest = yaml.safe_load(file_handle) or {}
    if "links" not in manifest or not manifest["links"]:
        raise ValueError(f"Manifest {manifest_path} does not define any links.")
    return manifest


def resolve_artifact_path(base_dir: Path, artifact_path: str | Path) -> Path:
    candidate = Path(artifact_path)
    if candidate.is_absolute():
        return candidate
    return base_dir / candidate


def rotation_x(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cosine, -sine],
            [0.0, sine, cosine],
        ],
        dtype=float,
    )


def rotation_y(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [
            [cosine, 0.0, sine],
            [0.0, 1.0, 0.0],
            [-sine, 0.0, cosine],
        ],
        dtype=float,
    )


def rotation_z(angle: float) -> np.ndarray:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def transform_matrix(
    xyz: Iterable[float] = (0.0, 0.0, 0.0),
    rpy: Iterable[float] = (0.0, 0.0, 0.0),
) -> np.ndarray:
    roll, pitch, yaw = rpy
    rotation = rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(tuple(xyz), dtype=float)
    return matrix


def rotation_about_local_z(angle: float) -> np.ndarray:
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation_z(angle)
    return matrix


def compute_reference_link_frames(reference_pose: Dict[str, float]) -> Dict[str, np.ndarray]:
    shoulder_z = KINEMATICS["shoulder_z"]
    upper_arm_x = KINEMATICS["upper_arm_x"]
    forearm_x = KINEMATICS["forearm_x"]
    forearm_z = KINEMATICS["forearm_z"]
    wrist_1_y = KINEMATICS["wrist_1_y"]
    wrist_2_y = KINEMATICS["wrist_2_y"]

    q1 = reference_pose["joint_1"]
    q2 = reference_pose["joint_2"]
    q3 = reference_pose["joint_3"]
    q4 = reference_pose["joint_4"]
    q5 = reference_pose["joint_5"]
    q6 = reference_pose["joint_6"]

    frames: Dict[str, np.ndarray] = {"base_link": np.eye(4, dtype=float)}
    current = transform_matrix((0.0, 0.0, shoulder_z)) @ rotation_about_local_z(q1)
    frames["link_1"] = current.copy()

    current = current @ transform_matrix((0.0, 0.0, 0.0), (PI / 2.0, 0.0, 0.0)) @ rotation_about_local_z(q2)
    frames["link_2"] = current.copy()

    current = current @ transform_matrix((upper_arm_x, 0.0, 0.0)) @ rotation_about_local_z(q3)
    frames["link_3"] = current.copy()

    current = current @ transform_matrix((forearm_x, 0.0, forearm_z)) @ rotation_about_local_z(q4)
    frames["link_4"] = current.copy()

    current = current @ transform_matrix((0.0, wrist_1_y, 0.0), (PI / 2.0, 0.0, 0.0)) @ rotation_about_local_z(q5)
    frames["link_5"] = current.copy()

    current = current @ transform_matrix((0.0, wrist_2_y, 0.0), (PI / 2.0, PI, PI)) @ rotation_about_local_z(q6)
    frames["link_6"] = current.copy()
    return frames


def load_step_components(step_file: Path) -> List[dict]:
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("ur7e"))
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    status = reader.ReadFile(str(step_file))
    if int(status) != 1:
        raise RuntimeError(f"Could not read STEP file {step_file}. status={int(status)}")
    reader.Transfer(doc)

    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    if roots.Length() < 1:
        raise RuntimeError(f"No free shapes found in {step_file}.")

    root = roots.Value(1)
    components = TDF_LabelSequence()
    shape_tool.GetComponents_s(root, components)
    extracted = []
    for index in range(1, components.Length() + 1):
        component_label = components.Value(index)
        referred_label = components.Value(index)
        shape_tool.GetReferredShape_s(component_label, referred_label)
        shape = shape_tool.GetShape_s(component_label)
        extracted.append(
            {
                "component_index": index,
                "shape": shape,
            }
        )
    return extracted


def export_component_mesh(shape, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    mesh = BRepMesh_IncrementalMesh(shape, 0.8, False, 0.5, False)
    mesh.Perform()
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    writer.Write(shape, str(destination))


def parse_ascii_stl_text(stl_text: str, mesh_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    normals: List[List[float]] = []
    vertices: List[List[float]] = []
    current_vertices: List[List[float]] = []
    current_normal = [0.0, 0.0, 0.0]

    for line in stl_text.splitlines():
        stripped = line.strip()
        if stripped.startswith("facet normal"):
            parts = stripped.split()
            current_normal = [float(parts[-3]), float(parts[-2]), float(parts[-1])]
        elif stripped.startswith("vertex"):
            parts = stripped.split()
            current_vertices.append(
                [float(parts[1]), float(parts[2]), float(parts[3])]
            )
        elif stripped == "endloop":
            if len(current_vertices) != 3:
                raise ValueError(f"Malformed STL loop in {mesh_path}.")
            vertices.append(current_vertices)
            normals.append(current_normal)
            current_vertices = []

    if not vertices:
        raise ValueError(f"No triangles were parsed from {mesh_path}.")

    return np.asarray(normals, dtype=float), np.asarray(vertices, dtype=float)


def parse_binary_stl_bytes(binary_data: bytes, mesh_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    if len(binary_data) < 84:
        raise ValueError(f"Binary STL header is too short in {mesh_path}.")

    triangle_count = struct.unpack("<I", binary_data[80:84])[0]
    expected_size = 84 + triangle_count * 50
    if len(binary_data) < expected_size:
        raise ValueError(
            f"Binary STL {mesh_path} is truncated. expected={expected_size}, got={len(binary_data)}"
        )

    normals = np.zeros((triangle_count, 3), dtype=float)
    vertices = np.zeros((triangle_count, 3, 3), dtype=float)

    offset = 84
    for index in range(triangle_count):
        chunk = binary_data[offset : offset + 50]
        values = struct.unpack("<12fH", chunk)
        normals[index] = values[0:3]
        vertices[index, 0] = values[3:6]
        vertices[index, 1] = values[6:9]
        vertices[index, 2] = values[9:12]
        offset += 50

    return normals, vertices


def parse_stl(mesh_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    binary_data = mesh_path.read_bytes()
    if len(binary_data) >= 84:
        triangle_count = struct.unpack("<I", binary_data[80:84])[0]
        expected_size = 84 + triangle_count * 50
        if expected_size == len(binary_data):
            return parse_binary_stl_bytes(binary_data, mesh_path)

    try:
        ascii_text = binary_data.decode("utf-8")
    except UnicodeDecodeError:
        return parse_binary_stl_bytes(binary_data, mesh_path)
    return parse_ascii_stl_text(ascii_text, mesh_path)


def write_binary_stl(mesh_path: Path, solid_name: str, normals: np.ndarray, vertices: np.ndarray) -> None:
    mesh_path.parent.mkdir(parents=True, exist_ok=True)
    with open(mesh_path, "wb") as file_handle:
        header = f"OpenAI UR7e {solid_name}".encode("ascii", errors="ignore")[:80]
        file_handle.write(header.ljust(80, b"\0"))
        file_handle.write(struct.pack("<I", int(vertices.shape[0])))
        for normal, triangle in zip(normals, vertices):
            file_handle.write(
                struct.pack(
                    "<12fH",
                    float(normal[0]),
                    float(normal[1]),
                    float(normal[2]),
                    float(triangle[0][0]),
                    float(triangle[0][1]),
                    float(triangle[0][2]),
                    float(triangle[1][0]),
                    float(triangle[1][1]),
                    float(triangle[1][2]),
                    float(triangle[2][0]),
                    float(triangle[2][1]),
                    float(triangle[2][2]),
                    0,
                )
            )


def transform_stl_to_local(
    raw_mesh_path: Path,
    local_mesh_path: Path,
    link_world_frame: np.ndarray,
    link_name: str,
) -> dict:
    normals_step, vertices_step_mm = parse_stl(raw_mesh_path)
    vertices_ros_world = vertices_step_mm / 1000.0
    vertices_ros_world = np.einsum("ij,tvj->tvi", STEP_TO_ROS_ROT, vertices_ros_world)
    normals_ros_world = np.einsum("ij,tj->ti", STEP_TO_ROS_ROT, normals_step)

    inverse_link = np.linalg.inv(link_world_frame)
    rotation_local = inverse_link[:3, :3]
    translation_local = inverse_link[:3, 3]
    vertices_local = np.einsum("ij,tvj->tvi", rotation_local, vertices_ros_world) + translation_local
    normals_local = np.einsum("ij,tj->ti", rotation_local, normals_ros_world)

    write_binary_stl(local_mesh_path, local_mesh_path.stem, normals_local, vertices_local)

    return {
        "raw_bbox_step_mm": bounding_box(vertices_step_mm),
        "local_bbox_ros_m": bounding_box(vertices_local),
        "triangle_count": int(vertices_local.shape[0]),
        "vertex_count": int(vertices_local.shape[0] * 3),
    }


def bounding_box(vertices: np.ndarray) -> dict:
    flat = vertices.reshape(-1, 3)
    minimum = flat.min(axis=0)
    maximum = flat.max(axis=0)
    centroid = flat.mean(axis=0)
    return {
        "min": [float(value) for value in minimum],
        "max": [float(value) for value in maximum],
        "centroid": [float(value) for value in centroid],
    }


def link_frame_in_step_mm(link_world_frame: np.ndarray) -> List[float]:
    ros_position_m = link_world_frame[:3, 3]
    step_position_mm = ROS_TO_STEP_ROT @ (ros_position_m * 1000.0)
    return [float(value) for value in step_position_mm]


def within_bbox(point: Iterable[float], bbox: dict, margin_mm: float = 25.0) -> bool:
    coords = list(point)
    lower = bbox["min"]
    upper = bbox["max"]
    return all(lower[i] - margin_mm <= coords[i] <= upper[i] + margin_mm for i in range(3))


def generate_report(
    manifest_path: Path,
    manifest: dict,
    report_dir: Path,
    link_frames: Dict[str, np.ndarray],
    raw_stats: Dict[str, dict],
) -> Path:
    report_dir.mkdir(parents=True, exist_ok=True)
    report_path = report_dir / "ur7e_report.yaml"
    link_reports = []
    for link_entry in manifest["links"]:
        link_name = link_entry["link_name"]
        stats = raw_stats[link_name]
        reference_origin_step_mm = link_frame_in_step_mm(link_frames[link_name])
        link_reports.append(
            {
                "link_name": link_name,
                "component_index": int(link_entry["component_index"]),
                "source_step_name": link_entry["source_step_name"],
                "raw_mesh_path": str(
                    resolve_artifact_path(manifest_path.parent, link_entry["raw_mesh_path"]).relative_to(ROOT)
                ),
                "local_mesh_path": str(
                    resolve_artifact_path(manifest_path.parent, link_entry["local_mesh_path"]).relative_to(ROOT)
                ),
                "reference_origin_step_mm": reference_origin_step_mm,
                "reference_origin_near_bbox": within_bbox(
                    reference_origin_step_mm,
                    stats["raw_bbox_step_mm"],
                ),
                "raw_bbox_step_mm": stats["raw_bbox_step_mm"],
                "local_bbox_ros_m": stats["local_bbox_ros_m"],
                "triangle_count": stats["triangle_count"],
                "vertex_count": stats["vertex_count"],
            }
        )

    report = {
        "step_file": str(DEFAULT_STEP_FILE.relative_to(ROOT)),
        "manifest": str(manifest_path.relative_to(ROOT)),
        "reference_pose": manifest.get("metadata", {}).get("reference_pose", DEFAULT_REFERENCE_JOINTS),
        "step_to_ros_rpy": manifest.get("metadata", {}).get("step_to_ros_rpy", STEP_TO_ROS_RPY),
        "links": link_reports,
    }
    with open(report_path, "w", encoding="utf-8") as file_handle:
        yaml.safe_dump(report, file_handle, sort_keys=False)
    return report_path


def export_single_component(step_file: Path, out_dir: Path, component_index: int) -> Path:
    components = load_step_components(step_file)
    component_map = {entry["component_index"]: entry for entry in components}
    if component_index not in component_map:
        raise ValueError(f"Component index {component_index} is out of range.")
    target = out_dir / "raw" / f"component_{component_index:02d}.stl"
    export_component_mesh(component_map[component_index]["shape"], target)
    return target


def run_pipeline(args: argparse.Namespace) -> int:
    ensure_default_manifest(args.manifest)
    manifest = read_manifest(args.manifest)
    reference_pose = manifest.get("metadata", {}).get("reference_pose", DEFAULT_REFERENCE_JOINTS)
    link_frames = compute_reference_link_frames(reference_pose)

    components = load_step_components(args.step)
    component_map = {entry["component_index"]: entry for entry in components}

    raw_stats: Dict[str, dict] = {}
    for link_entry in manifest["links"]:
        component_index = int(link_entry["component_index"])
        if component_index not in component_map:
            raise ValueError(
                f"Manifest requests component {component_index}, but STEP only exported {sorted(component_map)}."
            )

        raw_mesh_path = resolve_artifact_path(args.manifest.parent, link_entry["raw_mesh_path"])
        local_mesh_path = resolve_artifact_path(args.manifest.parent, link_entry["local_mesh_path"])

        if args.force_export or not raw_mesh_path.exists():
            export_component_mesh(component_map[component_index]["shape"], raw_mesh_path)

        if not args.skip_local:
            raw_stats[link_entry["link_name"]] = transform_stl_to_local(
                raw_mesh_path=raw_mesh_path,
                local_mesh_path=local_mesh_path,
                link_world_frame=link_frames[link_entry["link_name"]],
                link_name=link_entry["link_name"],
            )

    if not args.skip_local:
        report_path = generate_report(
            manifest_path=args.manifest,
            manifest=manifest,
            report_dir=args.report_dir,
            link_frames=link_frames,
            raw_stats=raw_stats,
        )
        print(f"[report] {report_path}")

    print(f"[ok] prepared UR7e visuals in {args.out_dir}")
    return 0


def main() -> int:
    args = parse_args()
    args.manifest = args.manifest.resolve()
    args.out_dir = args.out_dir.resolve()
    args.report_dir = args.report_dir.resolve()
    args.step = args.step.resolve()

    if args.component_index is not None:
        target = export_single_component(args.step, args.out_dir, args.component_index)
        print(f"[ok] exported {target}")
        return 0

    return run_pipeline(args)


if __name__ == "__main__":
    raise SystemExit(main())
