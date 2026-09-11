#!/usr/bin/env python3
"""Audit an Isaac-generated URDF without changing the authoritative robot model."""

from __future__ import annotations

import argparse
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def _mesh_stats(path: Path) -> tuple[int, int]:
    """Return (triangles, vertices) using trimesh when available."""
    try:
        import trimesh

        mesh = trimesh.load(path, force="mesh", process=False)
        if hasattr(mesh, "geometry"):
            meshes = list(mesh.geometry.values())
            return sum(len(m.faces) for m in meshes), sum(len(m.vertices) for m in meshes)
        return len(mesh.faces), len(mesh.vertices)
    except Exception:
        # Lightweight fallback for installations without trimesh.  STL files
        # expose one `facet` per triangle; Collada exposes triangle counts in
        # <triangles count="..."> elements.
        try:
            suffix = path.suffix.lower()
            text = path.read_text(errors="ignore")
            if suffix == ".stl":
                triangles = len(re.findall(r"\bfacet\b", text, flags=re.IGNORECASE))
                if triangles == 0:
                    triangles = len(re.findall(r"\btriangles\b", text, flags=re.IGNORECASE))
                if triangles == 0:
                    raw = path.read_bytes()
                    if len(raw) >= 84:
                        import struct
                        count = struct.unpack_from("<I", raw, 80)[0]
                        if 84 + 50 * count <= len(raw):
                            triangles = count
                return triangles, triangles * 3
            if suffix == ".dae":
                triangles = sum(int(value) for value in re.findall(r"<triangles[^>]*\bcount=[\"'](\d+)", text))
                return triangles, 0
        except (OSError, ValueError):
            pass
        return 0, 0


def audit(urdf_path: Path) -> dict[str, object]:
    root = ET.parse(urdf_path).getroot()
    links: dict[str, dict[str, object]] = {}
    total_visual_triangles = 0
    total_collision_triangles = 0
    visual_meshes = 0
    collision_meshes = 0
    collision_primitives = 0
    unresolved = 0

    for link in root.findall("link"):
        name = link.get("name", "")
        result = {"visual_meshes": 0, "collision_meshes": 0, "collision_primitives": 0,
                  "visual_triangles": 0, "collision_triangles": 0}
        for kind in ("visual", "collision"):
            for geometry in link.findall(f"{kind}/geometry"):
                mesh = geometry.find("mesh")
                if mesh is not None and mesh.get("filename"):
                    target = Path(mesh.get("filename", "").replace("file://", ""))
                    if not target.is_absolute():
                        target = urdf_path.parent / target
                    triangles, _ = _mesh_stats(target)
                    if not target.exists():
                        unresolved += 1
                    result[f"{kind}_meshes"] += 1
                    result[f"{kind}_triangles"] += triangles
                elif geometry.find("box") is not None or geometry.find("cylinder") is not None or geometry.find("sphere") is not None:
                    if kind == "collision":
                        result["collision_primitives"] += 1
        links[name] = result
        visual_meshes += int(result["visual_meshes"])
        collision_meshes += int(result["collision_meshes"])
        collision_primitives += int(result["collision_primitives"])
        total_visual_triangles += int(result["visual_triangles"])
        total_collision_triangles += int(result["collision_triangles"])

    return {
        "urdf": str(urdf_path),
        "links": len(links),
        "visual_meshes": visual_meshes,
        "collision_meshes": collision_meshes,
        "collision_primitives": collision_primitives,
        "visual_triangles": total_visual_triangles,
        "collision_triangles": total_collision_triangles,
        "unresolved_meshes": unresolved,
        "links_by_name": links,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit(args.urdf.expanduser().resolve())
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
