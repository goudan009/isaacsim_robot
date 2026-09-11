#!/usr/bin/env python3
"""Decimate selected Mesh prims in a text USDA file.

This intentionally edits only mesh geometry arrays on existing Mesh prims. It
leaves prim paths, schemas, transforms, materials, physics attributes, sensors,
and relationships untouched.
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pymeshlab


DEFAULT_TARGETS = {
    "femto_bolt_camera": 15000,
    "d435_link": 15000,
    "lift_link": 40000,
    "lift_link_1": 12000,
}


@dataclass
class MeshBlock:
    name: str
    start: int
    end: int
    text: str


def _find_mesh_block(text: str, name: str) -> MeshBlock:
    needle = f'def Mesh "{name}"'
    pos = text.find(needle)
    if pos < 0:
        raise ValueError(f"Mesh prim not found: {name}")

    start = text.rfind("\n", 0, pos) + 1
    open_brace = text.find("{", pos)
    if open_brace < 0:
        raise ValueError(f"Mesh prim has no body: {name}")

    depth = 0
    for index in range(open_brace, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return MeshBlock(name=name, start=start, end=index + 1, text=text[start : index + 1])
    raise ValueError(f"Mesh prim body is not closed: {name}")


def _find_array_property_span(block: str, property_name: str) -> tuple[int, int, int, int]:
    match = re.search(rf"^[ \t]*[A-Za-z0-9_:]+\[\]\s+{re.escape(property_name)}\s*=\s*\[", block, re.M)
    if not match:
        raise ValueError(f"Array property not found: {property_name}")

    equals = block.find("=", match.start(), match.end() + 16)
    if equals < 0:
        raise ValueError(f"Array property has no assignment: {property_name}")
    array_start = block.find("[", equals)
    depth = 0
    array_end = None
    for index in range(array_start, len(block)):
        char = block[index]
        if char == "[":
            depth += 1
        elif char == "]":
            depth -= 1
            if depth == 0:
                array_end = index + 1
                break
    if array_end is None:
        raise ValueError(f"Array property is not closed: {property_name}")

    property_end = array_end
    cursor = property_end
    while cursor < len(block) and block[cursor] in " \t\r\n":
        cursor += 1
    if cursor < len(block) and block[cursor] == "(":
        depth = 0
        for index in range(cursor, len(block)):
            char = block[index]
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    property_end = index + 1
                    break
    else:
        line_end = block.find("\n", array_end)
        property_end = len(block) if line_end < 0 else line_end

    if property_end < len(block) and block[property_end] == "\n":
        property_end += 1

    return match.start(), property_end, array_start, array_end


def _extract_numbers(array_text: str, dtype: type[float] | type[int]) -> np.ndarray:
    if dtype is int:
        pattern = r"-?\d+"
        return np.fromiter((int(v) for v in re.findall(pattern, array_text)), dtype=np.int64)

    pattern = r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?"
    return np.fromiter((float(v) for v in re.findall(pattern, array_text)), dtype=np.float64)


def _extract_mesh_arrays(block: str) -> tuple[np.ndarray, np.ndarray]:
    _, _, points_start, points_end = _find_array_property_span(block, "points")
    _, _, counts_start, counts_end = _find_array_property_span(block, "faceVertexCounts")
    _, _, indices_start, indices_end = _find_array_property_span(block, "faceVertexIndices")

    points = _extract_numbers(block[points_start:points_end], float).reshape((-1, 3))
    counts = _extract_numbers(block[counts_start:counts_end], int)
    indices = _extract_numbers(block[indices_start:indices_end], int)
    if not np.all(counts == 3):
        raise ValueError("Only triangulated meshes are supported by this tool")
    faces = indices.reshape((-1, 3))
    return points, faces


def _decimate(points: np.ndarray, faces: np.ndarray, target_faces: int) -> tuple[np.ndarray, np.ndarray]:
    if len(faces) <= target_faces:
        return points, faces

    mesh = pymeshlab.Mesh(vertex_matrix=points.astype(np.float64), face_matrix=faces.astype(np.int32))
    meshset = pymeshlab.MeshSet()
    meshset.add_mesh(mesh, "input")
    meshset.apply_filter(
        "meshing_decimation_quadric_edge_collapse",
        targetfacenum=int(target_faces),
        preservenormal=True,
        preserveboundary=True,
        optimalplacement=True,
        planarquadric=True,
    )
    result = meshset.current_mesh()
    return result.vertex_matrix(), result.face_matrix().astype(np.int64)


def _vertex_normals(points: np.ndarray, faces: np.ndarray) -> np.ndarray:
    normals = np.zeros_like(points, dtype=np.float64)
    triangles = points[faces]
    face_normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(face_normals, axis=1)
    valid = lengths > 1e-18
    face_normals[valid] /= lengths[valid, None]
    for corner in range(3):
        np.add.at(normals, faces[:, corner], face_normals)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-18
    normals[valid] /= lengths[valid, None]
    normals[~valid] = np.array([0.0, 0.0, 1.0])
    return normals


def _format_float(value: float) -> str:
    if not math.isfinite(float(value)):
        raise ValueError(f"Non-finite mesh value: {value}")
    return f"{float(value):.8g}"


def _format_vec3_array(type_name: str, property_name: str, values: np.ndarray, indent: str, metadata: str | None = None) -> str:
    items = ", ".join(
        f"({_format_float(x)}, {_format_float(y)}, {_format_float(z)})" for x, y, z in values
    )
    line = f"{indent}{type_name}[] {property_name} = [{items}]"
    if metadata:
        line += f" (\n{indent}    {metadata}\n{indent})"
    return line + "\n"


def _format_int_array(property_name: str, values: np.ndarray, indent: str) -> str:
    items = ", ".join(str(int(v)) for v in values)
    return f"{indent}int[] {property_name} = [{items}]\n"


def _replace_array_property(block: str, property_name: str, replacement: str) -> str:
    prop_start, prop_end, _, _ = _find_array_property_span(block, property_name)
    return block[:prop_start] + replacement + block[prop_end:]


def _remove_array_property_if_present(block: str, property_name: str) -> str:
    try:
        prop_start, prop_end, _, _ = _find_array_property_span(block, property_name)
    except ValueError:
        return block
    return block[:prop_start] + block[prop_end:]


def _update_mesh_block(block: str, points: np.ndarray, faces: np.ndarray) -> str:
    indent_match = re.search(r"^([ \t]*)point3f\[\]\s+points\s*=", block, re.M)
    if not indent_match:
        raise ValueError("points indentation not found")
    indent = indent_match.group(1)

    extent = np.array([points.min(axis=0), points.max(axis=0)])
    normals = _vertex_normals(points, faces)
    counts = np.full((len(faces),), 3, dtype=np.int64)
    indices = faces.reshape((-1,))

    block = _replace_array_property(
        block,
        "extent",
        _format_vec3_array("float3", "extent", extent, indent),
    )
    block = _replace_array_property(
        block,
        "faceVertexCounts",
        _format_int_array("faceVertexCounts", counts, indent),
    )
    block = _replace_array_property(
        block,
        "faceVertexIndices",
        _format_int_array("faceVertexIndices", indices, indent),
    )
    block = _replace_array_property(
        block,
        "points",
        _format_vec3_array("point3f", "points", points, indent),
    )
    block = _replace_array_property(
        block,
        "primvars:normals",
        _format_vec3_array("normal3f", "primvars:normals", normals, indent, 'interpolation = "vertex"'),
    )
    block = _remove_array_property_if_present(block, "primvars:normals:indices")
    return block


def _parse_targets(raw_targets: list[str]) -> dict[str, int]:
    targets = dict(DEFAULT_TARGETS)
    for raw in raw_targets:
        if "=" not in raw:
            raise ValueError(f"Target must be name=faces: {raw}")
        name, value = raw.split("=", 1)
        targets[name] = int(value)
    return targets


def decimate_usda(usda_path: Path, targets: dict[str, int], backup_dir: Path | None, dry_run: bool) -> None:
    text = usda_path.read_text()
    original_size = usda_path.stat().st_size
    replacements: list[tuple[int, int, str]] = []

    for name, target_faces in targets.items():
        block = _find_mesh_block(text, name)
        points, faces = _extract_mesh_arrays(block.text)
        new_points, new_faces = _decimate(points, faces, target_faces)
        updated = _update_mesh_block(block.text, new_points, new_faces)
        replacements.append((block.start, block.end, updated))
        print(
            f"{name}: points {len(points)} -> {len(new_points)}, "
            f"faces {len(faces)} -> {len(new_faces)}"
        )

    if dry_run:
        print("dry run: no files written")
        return

    if backup_dir:
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup_path = backup_dir / f"{usda_path.stem}.before_decimate{usda_path.suffix}"
        if not backup_path.exists():
            shutil.copy2(usda_path, backup_path)
            print(f"backup: {backup_path}")
        else:
            print(f"backup already exists: {backup_path}")

    for start, end, replacement in sorted(replacements, reverse=True):
        text = text[:start] + replacement + text[end:]
    usda_path.write_text(text)
    print(f"size: {original_size} -> {usda_path.stat().st_size} bytes")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("usda", type=Path)
    parser.add_argument("--target", action="append", default=[], help="Override as mesh_name=target_faces")
    parser.add_argument("--backup-dir", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    targets = _parse_targets(args.target)
    decimate_usda(args.usda, targets, args.backup_dir, args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
