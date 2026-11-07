#!/usr/bin/env python3
"""Turn a bounded part of a PCD snapshot into a static Gazebo collision world.

The output consists only of occupied voxels from the input PCD. Missing points
stay empty; this tool does not invent a floor or silently fill gaps.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


_TYPES = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1",
          ("U", 2): "<u2", ("U", 4): "<u4", ("I", 1): "i1",
          ("I", 2): "<i2", ("I", 4): "<i4"}
_FACES = [
    ((1, 0, 0), (1, 2, 6, 5)), ((-1, 0, 0), (0, 4, 7, 3)),
    ((0, 1, 0), (2, 3, 7, 6)), ((0, -1, 0), (0, 1, 5, 4)),
    ((0, 0, 1), (4, 5, 6, 7)), ((0, 0, -1), (0, 3, 2, 1)),
]
_CORNERS = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
            (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]


def read_pcd_chunks(path, chunk_points=250000):
    """Yield finite XYZ arrays from ASCII or uncompressed binary PCD."""
    with open(path, "rb") as stream:
        header = {}
        lines = 0
        while True:
            raw = stream.readline()
            if not raw or lines > 100:
                raise ValueError("Invalid PCD header")
            lines += 1
            entry = raw.decode("ascii").strip().split()
            if not entry or entry[0].startswith("#"):
                continue
            header[entry[0].upper()] = entry[1:]
            if entry[0].upper() == "DATA":
                break
        fields = header["FIELDS"]
        sizes = list(map(int, header["SIZE"]))
        kinds = header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        if len({len(fields), len(sizes), len(kinds), len(counts)}) != 1:
            raise ValueError("Inconsistent PCD field layout")
        if not all(axis in fields for axis in "xyz"):
            raise ValueError("PCD needs x, y and z fields")
        points = int(header["POINTS"][0])
        mode = header["DATA"][0].lower()
        if mode == "binary":
            formats = []
            for kind, size, count in zip(kinds, sizes, counts):
                dtype = _TYPES.get((kind, size))
                if dtype is None:
                    raise ValueError(f"Unsupported PCD field type {kind}{size}")
                formats.append((dtype, (count,)) if count > 1 else dtype)
            dtype = np.dtype({"names": fields, "formats": formats})
            offset = stream.tell()
            expected = offset + points * dtype.itemsize
            if path.stat().st_size < expected:
                raise ValueError("Truncated binary PCD")
            cloud = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(points,))
            for start in range(0, points, chunk_points):
                part = cloud[start:start + chunk_points]
                xyz = np.column_stack([part[axis] for axis in "xyz"])
                yield xyz[np.isfinite(xyz).all(axis=1)]
        elif mode == "ascii":
            columns = [sum(counts[:fields.index(axis)]) for axis in "xyz"]
            batch = []
            for raw in stream:
                if not raw.strip():
                    continue
                values = raw.split()
                batch.append([float(values[column]) for column in columns])
                if len(batch) == chunk_points:
                    xyz = np.asarray(batch)
                    yield xyz[np.isfinite(xyz).all(axis=1)]
                    batch.clear()
            if batch:
                xyz = np.asarray(batch)
                yield xyz[np.isfinite(xyz).all(axis=1)]
        else:
            raise ValueError("Only ASCII and uncompressed binary PCD are supported")


def occupied_voxels(path, bounds, resolution, max_voxels):
    lower = np.asarray(bounds[::2], dtype=float)
    upper = np.asarray(bounds[1::2], dtype=float)
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or not np.all(lower < upper):
        raise ValueError("Bounds must be finite min/max pairs")
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("Voxel resolution must be positive")
    cells = set()
    for xyz in read_pcd_chunks(path):
        selected = xyz[np.all((xyz >= lower) & (xyz < upper), axis=1)]
        indexes = np.floor((selected - lower) / resolution).astype(np.int32)
        cells.update(map(tuple, indexes))
        if len(cells) > max_voxels:
            raise ValueError("Too many occupied voxels; crop a smaller region or increase --resolution")
    if not cells:
        raise ValueError("The selected PCD region contains no points")
    return cells, lower


def write_obj(path, cells, lower, resolution):
    vertex = 0
    with open(path, "w", encoding="ascii") as stream:
        stream.write("# Occupied PCD voxels; coordinates remain in the PCD frame\n")
        for cell in sorted(cells):
            for neighbor, corners in _FACES:
                adjacent = tuple(cell[i] + neighbor[i] for i in range(3))
                if adjacent in cells:
                    continue
                for corner_index in corners:
                    corner = _CORNERS[corner_index]
                    xyz = lower + resolution * (np.asarray(cell) + corner)
                    stream.write(f"v {xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}\n")
                stream.write(f"f {vertex + 1} {vertex + 2} {vertex + 3} {vertex + 4}\n")
                vertex += 4


def write_world(path, mesh):
    sdf = ET.Element("sdf", version="1.8")
    world = ET.SubElement(sdf, "world", name="scan_pcd")
    for filename, name in (
        ("libignition-gazebo-physics-system.so", "ignition::gazebo::systems::Physics"),
        ("libignition-gazebo-user-commands-system.so", "ignition::gazebo::systems::UserCommands"),
        ("libignition-gazebo-scene-broadcaster-system.so", "ignition::gazebo::systems::SceneBroadcaster"),
    ):
        ET.SubElement(world, "plugin", filename=filename, name=name)
    sensors = ET.SubElement(world, "plugin", filename="libignition-gazebo-sensors-system.so",
                            name="ignition::gazebo::systems::Sensors")
    ET.SubElement(sensors, "render_engine").text = "ogre2"
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    light = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"
    physics = ET.SubElement(world, "physics", name="1ms", type="ignored")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    model = ET.SubElement(world, "model", name="pcd_environment")
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name="geometry")
    for element in ("collision", "visual"):
        geometry = ET.SubElement(ET.SubElement(link, element, name=element), "geometry")
        ET.SubElement(ET.SubElement(geometry, "mesh"), "uri").text = mesh.resolve().as_uri()
    ET.indent(sdf)
    ET.ElementTree(sdf).write(path, encoding="unicode", xml_declaration=True)


def write_manifest(path, pcd, mesh, bounds, resolution, count):
    source = pcd.resolve()
    source_stat = source.stat()
    mesh_stat = mesh.stat()
    metadata = {
        "pcd_file": str(source), "pcd_size": source_stat.st_size,
        "pcd_mtime_ns": source_stat.st_mtime_ns,
        "mesh_file": str(mesh.resolve()), "mesh_size": mesh_stat.st_size,
        "bounds": bounds, "resolution": resolution, "occupied_voxels": count,
    }
    path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Output SDF file; OBJ is written beside it")
    parser.add_argument("--bounds", type=float, nargs=6, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"), required=True)
    parser.add_argument("--resolution", type=float, default=0.12)
    parser.add_argument("--max-voxels", type=int, default=80000)
    args = parser.parse_args()
    if args.max_voxels <= 0 or not args.pcd.is_file():
        parser.error("PCD must exist and max-voxels must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cells, lower = occupied_voxels(args.pcd, args.bounds, args.resolution, args.max_voxels)
    mesh = args.output.with_suffix(".obj")
    write_obj(mesh, cells, lower, args.resolution)
    write_world(args.output, mesh)
    write_manifest(args.output, args.pcd, mesh, args.bounds, args.resolution, len(cells))
    print(f"Wrote {args.output}, {mesh} and metadata from {len(cells)} occupied voxels")


if __name__ == "__main__":
    main()
