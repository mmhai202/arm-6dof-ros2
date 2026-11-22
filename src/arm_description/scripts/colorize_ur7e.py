#!/usr/bin/env python3
"""Add UR-style materials to the checked-in CAD geometry, without CAD dependencies.

STL has no material channels. Export the surface in COLLADA with material regions
in each link's local frame (metres), plus crease-aware smooth normals. Triangles
are split at finish boundaries so long CAD facets cannot produce jagged colors.
The palette follows the silver tubes, graphite housings and pale blue joint caps
in the product reference. It is an appearance approximation, not STEP metadata.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


LOCAL_MESHES = Path(__file__).resolve().parents[1] / "meshes" / "ur7e" / "local"
# diffuse RGB, specular RGB, shininess; all surfaces remain fully opaque.
MATERIALS = {
    "graphite": ((0.22, 0.23, 0.24), (0.30, 0.30, 0.30), 35),
    "black": ((0.075, 0.08, 0.085), (0.15, 0.15, 0.15), 20),
    "silver": ((0.80, 0.82, 0.84), (0.80, 0.82, 0.84), 65),
    "blue": ((0.56, 0.79, 0.90), (0.40, 0.50, 0.55), 40),
}

# Every coordinate threshold used by surface_materials must also cut the mesh.
# Axis indices: X=0, Y=1, Z=2. Splitting changes tessellation, not the CAD surface.
MATERIAL_PLANES = {
    "base_link": ((1, 0.080), (2, 0.085)),
    "link_1": ((2, -0.050), (1, -0.061)),
    "link_2": ((0, -0.350), (0, -0.075), (2, 0.085),
               (2, 0.191), (0, -0.355), (0, -0.070)),
    "link_3": ((0, -0.350), (0, -0.065), (0, -0.060), (2, 0.061), (2, 0.052)),
    "link_4": ((2, -0.068), (1, -0.039)),
    "link_5": ((2, -0.045), (1, 0.038)),
    "link_6": ((2, -0.018),),
}


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Expected binary STL: {path}")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(data, dtype=dtype, offset=84)["vertices"].astype(float)


def surface_materials(link: str, triangles: np.ndarray) -> np.ndarray:
    """Assign finishes using the existing CAD housings, caps and tube extents."""
    x, y, z = triangles.mean(axis=1).T
    result = np.full(len(triangles), "graphite", dtype="<U8")
    if link == "base_link":
        result[:] = "black"
        result[(y < 0.080) & (z < 0.085)] = "silver"
    elif link == "link_1":
        result[z < -0.050] = "black"
        result[y < -0.061] = "blue"
    elif link == "link_2":
        result[(x > -0.350) & (x < -0.075)] = "silver"
        result[z < 0.085] = "black"
        result[(z > 0.191) & ((x < -0.355) | (x > -0.070))] = "blue"
    elif link == "link_3":
        result[(x > -0.350) & (x < -0.065)] = "silver"
        result[(x > -0.060) & (z > 0.061)] = "blue"
        result[(x > -0.060) & (z > 0.052) & (z <= 0.061)] = "black"
    elif link == "link_4":
        result[z < -0.068] = "blue"
        result[y < -0.039] = "black"
    elif link == "link_5":
        result[z < -0.045] = "blue"
        result[y > 0.038] = "black"
    elif link == "link_6":
        result[:] = "black"
        result[z > -0.018] = "silver"
    else:
        raise ValueError(f"Unknown UR7e link: {link}")
    return result


def smooth_normals(triangles: np.ndarray) -> np.ndarray:
    edges = triangles[:, 1:] - triangles[:, :1]
    face = np.cross(edges[:, 0], edges[:, 1])
    face /= np.maximum(np.linalg.norm(face, axis=1, keepdims=True), 1e-20)
    # Weld only for shading. Positions and triangle topology are left unchanged.
    _, weld = np.unique(np.round(triangles.reshape(-1, 3), 6), axis=0, return_inverse=True)
    order = np.argsort(weld)
    groups = np.split(order, np.flatnonzero(np.diff(weld[order])) + 1)
    corners = np.repeat(face, 3, axis=0)
    normals = np.empty_like(corners)
    for group in groups:
        local = corners[group]
        # Smooth curved surfaces, preserving edges sharper than 45 degrees.
        weights = (local @ local.T) > np.cos(np.pi / 4)
        averaged = weights @ local
        normals[group] = averaged / np.maximum(np.linalg.norm(averaged, axis=1, keepdims=True), 1e-20)
    return normals.reshape(-1, 3, 3)


def split_at_plane(triangles, normals, axis, cut):
    """Clip crossing facets into non-overlapping triangles on both half-spaces.

    Interpolate the original corner normals at each new vertex. Recomputing
    normals after clipping would change the shading beside material seams.
    """
    crossing = ((triangles[:, :, axis].min(axis=1) < cut - 1e-12)
                & (triangles[:, :, axis].max(axis=1) > cut + 1e-12))
    pieces, normal_pieces = [], []
    for triangle, corner_normals in zip(triangles[crossing], normals[crossing]):
        polygon = np.concatenate((triangle, corner_normals), axis=1)
        for side in (-1, 1):
            clipped = []
            for previous, current in zip(np.roll(polygon, 1, axis=0), polygon):
                previous_inside = side * (previous[axis] - cut) >= 0
                current_inside = side * (current[axis] - cut) >= 0
                if previous_inside != current_inside:
                    # Always interpolate from the low end for identical results
                    # on shared edges, irrespective of triangle winding.
                    low, high = ((previous, current) if previous[axis] < current[axis]
                                 else (current, previous))
                    fraction = (cut - low[axis]) / (high[axis] - low[axis])
                    intersection = low + fraction * (high - low)
                    intersection[axis] = cut
                    clipped.append(intersection)
                if current_inside:
                    clipped.append(current)
            for i in range(1, len(clipped) - 1):
                fragment = np.array((clipped[0], clipped[i], clipped[i + 1]))
                edges = fragment[1:, :3] - fragment[:1, :3]
                if np.linalg.norm(np.cross(edges[0], edges[1])) > 1e-20:
                    pieces.append(fragment[:, :3])
                    normal_pieces.append(fragment[:, 3:])
    if not pieces:
        return triangles[~crossing], normals[~crossing]
    return (np.concatenate((triangles[~crossing], pieces)),
            np.concatenate((normals[~crossing], normal_pieces)))


def split_material_boundaries(link, triangles, normals):
    for axis, cut in MATERIAL_PLANES[link]:
        triangles, normals = split_at_plane(triangles, normals, axis, cut)
    normals = normals / np.maximum(np.linalg.norm(normals, axis=2, keepdims=True), 1e-20)
    return triangles, normals


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        node.text = text
    return node


def float_source(mesh, name, values):
    source = element(mesh, "source", id=name)
    element(source, "float_array", " ".join(f"{v:.8g}" for v in values.flat),
            id=f"{name}-array", count=values.size)
    accessor = element(element(source, "technique_common"), "accessor",
                       source=f"#{name}-array", count=len(values), stride=3)
    for axis in "XYZ":
        element(accessor, "param", name=axis, type="float")


def write_collada(path: Path, triangles: np.ndarray, finishes: np.ndarray, corner_normals: np.ndarray):
    root = ET.Element("COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema", version="1.4.1")
    asset = element(root, "asset")
    element(asset, "created", "2026-11-22T00:00:00Z")
    element(asset, "modified", "2026-11-22T00:00:00Z")
    element(asset, "unit", name="meter", meter=1)
    element(asset, "up_axis", "Z_UP")
    effects = element(root, "library_effects")
    materials = element(root, "library_materials")
    used = [name for name in MATERIALS if np.any(finishes == name)]
    for name in used:
        diffuse, specular, shininess = MATERIALS[name]
        effect = element(effects, "effect", id=f"{name}-effect")
        phong = element(element(element(effect, "profile_COMMON"), "technique", sid="common"), "phong")
        for kind, rgb in (("ambient", tuple(v * 0.35 for v in diffuse)),
                          ("diffuse", diffuse), ("specular", specular)):
            element(element(phong, kind), "color", " ".join(str(v) for v in (*rgb, 1)))
        element(element(phong, "shininess"), "float", str(shininess))
        material = element(materials, "material", id=f"{name}-material", name=name)
        element(material, "instance_effect", url=f"#{name}-effect")

    mesh = element(element(element(root, "library_geometries"), "geometry", id="cad-mesh"), "mesh")
    points, vertex_ids = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    normals, normal_ids = np.unique(np.round(corner_normals.reshape(-1, 3), 6),
                                   axis=0, return_inverse=True)
    float_source(mesh, "positions", points)
    float_source(mesh, "normals", normals)
    element(element(mesh, "vertices", id="vertices"), "input", semantic="POSITION", source="#positions")
    indices = np.stack((vertex_ids, normal_ids), axis=1).reshape(-1, 3, 2)
    for name in used:
        selected = indices[finishes == name]
        group = element(mesh, "triangles", count=len(selected), material=name)
        element(group, "input", semantic="VERTEX", source="#vertices", offset=0)
        element(group, "input", semantic="NORMAL", source="#normals", offset=1)
        element(group, "p", " ".join(str(v) for v in selected.flat))
    scene = element(element(root, "library_visual_scenes"), "visual_scene", id="Scene")
    instance = element(element(scene, "node", id="link"), "instance_geometry", url="#cad-mesh")
    bindings = element(element(instance, "bind_material"), "technique_common")
    for name in used:
        element(bindings, "instance_material", symbol=name, target=f"#{name}-material")
    element(element(root, "scene"), "instance_visual_scene", url="#Scene")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def colorize_mesh(source: Path):
    triangles = read_stl(source)
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1)
    triangles = triangles[area > 1e-20]
    # Some CAD exports repeat the same face with opposite winding.
    _, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    _, unique = np.unique(np.sort(indices.reshape(-1, 3), axis=1), axis=0, return_index=True)
    triangles = triangles[np.sort(unique)]
    normals = smooth_normals(triangles)
    triangles, normals = split_material_boundaries(source.stem, triangles, normals)
    finishes = surface_materials(source.stem, triangles)
    target = source.with_suffix(".dae")
    write_collada(target, triangles, finishes, normals)
    print(f"[color] {target.name}: {len(triangles)} triangles, {', '.join(np.unique(finishes))}")


def colorize_directory(directory: Path):
    for link in ("base_link", *(f"link_{i}" for i in range(1, 7))):
        colorize_mesh(directory / f"{link}.stl")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh-dir", type=Path, default=LOCAL_MESHES)
    args = parser.parse_args()
    colorize_directory(args.mesh_dir)


if __name__ == "__main__":
    main()
