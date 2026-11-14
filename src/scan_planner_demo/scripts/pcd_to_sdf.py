#!/usr/bin/env python3
"""Turn a whole PCD or a bounded region into a static Gazebo collision world.

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


def cell_size(resolution):
    size = np.asarray(resolution, dtype=float)
    if size.ndim == 0:
        size = np.repeat(size, 3)
    if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
        raise ValueError("Voxel resolution must contain three positive finite sizes")
    return size


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
    resolution = cell_size(resolution)
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or not np.all(lower < upper):
        raise ValueError("Bounds must be finite min/max pairs")
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


def full_pcd_bounds(path, resolution):
    """Return grid-aligned bounds that include every finite PCD point."""
    resolution = cell_size(resolution)
    minimum = np.full(3, np.inf)
    maximum = np.full(3, -np.inf)
    count = 0
    for xyz in read_pcd_chunks(path):
        if len(xyz):
            minimum = np.minimum(minimum, xyz.min(axis=0))
            maximum = np.maximum(maximum, xyz.max(axis=0))
            count += len(xyz)
    if not count:
        raise ValueError("PCD contains no finite XYZ points")
    lower = np.floor(minimum / resolution) * resolution
    # occupied_voxels uses an exclusive upper bound. Add one grid cell so that
    # even a point exactly on the maximum grid edge is included.
    upper = (np.floor(maximum / resolution) + 1) * resolution
    bounds = [float(value) for pair in zip(lower, upper) for value in pair]
    return bounds, count


def write_obj(path, cells, lower, resolution):
    resolution = cell_size(resolution)
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
                for _ in corners:
                    stream.write(f"vn {neighbor[0]} {neighbor[1]} {neighbor[2]}\n")
                stream.write("f " + " ".join(
                    f"{vertex + offset}//{vertex + offset}" for offset in range(1, 5)) + "\n")
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
    for element, name in (("collision", "collision"), ("visual", "visual"),
                          ("visual", "voxel_edges")):
        shape = ET.SubElement(link, element, name=name)
        if name == "voxel_edges":
            ET.SubElement(shape, "pose").text = "0 0 0.001 0 0 0"
            ET.SubElement(shape, "wireframe").text = "true"
            ET.SubElement(shape, "transparency").text = "0.35"
        geometry = ET.SubElement(shape, "geometry")
        ET.SubElement(ET.SubElement(geometry, "mesh"), "uri").text = mesh.resolve().as_uri()
        if name != "collision":
            material = ET.SubElement(shape, "material")
            if name == "visual":
                ET.SubElement(material, "ambient").text = "0.22 0.35 0.43 1"
                ET.SubElement(material, "diffuse").text = "0.35 0.53 0.62 1"
                ET.SubElement(material, "specular").text = "0.02 0.02 0.02 1"
            else:
                ET.SubElement(material, "ambient").text = "0.06 0.10 0.13 1"
                ET.SubElement(material, "diffuse").text = "0.08 0.15 0.18 1"
    ET.indent(sdf)
    ET.ElementTree(sdf).write(path, encoding="unicode", xml_declaration=True)


def write_manifest(path, pcd, mesh, bounds, resolution, count, coverage="bounded", finite_points=None):
    source = pcd.resolve()
    source_stat = source.stat()
    mesh_stat = mesh.stat()
    metadata = {
        "pcd_file": str(source), "pcd_size": source_stat.st_size,
        "pcd_mtime_ns": source_stat.st_mtime_ns,
        "mesh_file": str(mesh.resolve()), "mesh_size": mesh_stat.st_size,
        "bounds": bounds,
        "resolution": float(resolution) if np.asarray(resolution).ndim == 0 else cell_size(resolution).tolist(),
        "occupied_voxels": count,
        "coverage": coverage,
    }
    if finite_points is not None:
        metadata["finite_points"] = finite_points
    path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="Output SDF file; OBJ is written beside it")
    extent = parser.add_mutually_exclusive_group(required=True)
    extent.add_argument("--bounds", type=float, nargs=6,
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"))
    extent.add_argument("--all", action="store_true", help="Include every finite XYZ point in the PCD")
    parser.add_argument("--resolution", type=float, default=0.12)
    parser.add_argument("--resolution-xyz", type=float, nargs=3,
                        metavar=("X", "Y", "Z"),
                        help="Per-axis voxel sizes; overrides --resolution")
    parser.add_argument("--max-voxels", type=int, default=80000)
    args = parser.parse_args()
    if args.max_voxels <= 0 or not args.pcd.is_file():
        parser.error("PCD must exist and max-voxels must be positive")
    resolution = args.resolution_xyz if args.resolution_xyz is not None else args.resolution
    try:
        cell_size(resolution)
    except ValueError as error:
        parser.error(str(error))
    bounds, finite_points = full_pcd_bounds(args.pcd, resolution) if args.all else (args.bounds, None)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cells, lower = occupied_voxels(args.pcd, bounds, resolution, args.max_voxels)
    mesh = args.output.with_suffix(".obj")
    write_obj(mesh, cells, lower, resolution)
    write_world(args.output, mesh)
    write_manifest(args.output, args.pcd, mesh, bounds, resolution, len(cells),
                   coverage="all_finite_points" if args.all else "bounded", finite_points=finite_points)
    print(f"Wrote {args.output}, {mesh} and metadata from {len(cells)} occupied voxels"
          + (f" covering all {finite_points} finite points" if args.all else ""))


if __name__ == "__main__":
    main()
