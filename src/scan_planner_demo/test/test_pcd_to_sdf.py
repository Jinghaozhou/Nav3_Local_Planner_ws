"""Check that conversion preserves occupied PCD cells and their world coordinates."""

import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "pcd_to_sdf.py"
SPEC = importlib.util.spec_from_file_location("pcd_to_sdf", SCRIPT)
converter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(converter)


class TestPcdToSdf(unittest.TestCase):
    def test_binary_pcd_and_separated_voxels(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pcd = root / "map.pcd"
            header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z _\n"
                      "SIZE 4 4 4 1\nTYPE F F F U\nCOUNT 1 1 1 4\n"
                      "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary\n")
            with pcd.open("wb") as stream:
                stream.write(header.encode("ascii"))
                for xyz in ((1.05, 2.05, 0.05), (1.06, 2.04, 0.04), (1.35, 2.05, 0.05)):
                    stream.write(struct.pack("<fff4B", *xyz, 0, 0, 0, 0))
            bounds = [1.0, 2.0, 2.0, 3.0, 0.0, 1.0]
            cells, lower = converter.occupied_voxels(pcd, bounds, 0.1, 10)
            self.assertEqual(cells, {(0, 0, 0), (3, 0, 0)})
            mesh = root / "world.obj"
            world = root / "world.sdf"
            converter.write_obj(mesh, cells, lower, 0.1)
            converter.write_world(world, mesh)
            converter.write_manifest(world, pcd, mesh, bounds, 0.1, len(cells))
            data = mesh.read_text()
            self.assertEqual(data.count("\nf "), 12)
            self.assertEqual(data.count("\nv "), data.count("\nvn "))
            self.assertIn("f 1//1 2//2 3//3 4//4", data)
            self.assertIn("v 1.000000 2.000000 0.000000", data)
            tree = ET.parse(world)
            self.assertEqual(tree.findtext(".//model/static"), "true")
            self.assertEqual(tree.findtext(".//collision/geometry/mesh/uri"), mesh.as_uri())
            self.assertEqual(len(tree.findall(".//collision")), 1)
            self.assertEqual(tree.findtext(".//visual[@name='visual']/material/diffuse"),
                             "0.35 0.53 0.62 1")
            self.assertEqual(tree.findtext(".//visual[@name='voxel_edges']/wireframe"),
                             "true")
            manifest = json.loads(world.with_suffix(".json").read_text())
            self.assertEqual(manifest["pcd_file"], str(pcd))
            self.assertEqual(manifest["occupied_voxels"], 2)

    def test_ascii_rejects_empty_region_and_missing_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            pcd = Path(directory) / "map.pcd"
            pcd.write_text("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                           "TYPE F F F\nWIDTH 1\nHEIGHT 1\nPOINTS 1\n"
                           "DATA ascii\n0 0 0\n")
            cells, _ = converter.occupied_voxels(pcd, [-1, 1, -1, 1, -1, 1], 0.1, 10)
            self.assertEqual(len(cells), 1)
            with self.assertRaisesRegex(ValueError, "contains no points"):
                converter.occupied_voxels(pcd, [2, 3, 2, 3, 2, 3], 0.1, 10)

    def test_full_bounds_include_extreme_points(self):
        with tempfile.TemporaryDirectory() as directory:
            pcd = Path(directory) / "map.pcd"
            pcd.write_text("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                           "TYPE F F F\nWIDTH 4\nHEIGHT 1\nPOINTS 4\n"
                           "DATA ascii\n-0.4 0 0\n0 0 0\n0.4 1 1\nnan 0 0\n")
            bounds, count = converter.full_pcd_bounds(pcd, 0.2)
            self.assertEqual(count, 3)
            cells, lower = converter.occupied_voxels(pcd, bounds, 0.2, 10)
            self.assertEqual(len(cells), 3)
            self.assertLessEqual(bounds[0], -0.4)
            self.assertGreater(bounds[1], 0.4)
            self.assertEqual(lower[0], bounds[0])

    def test_anisotropic_cells_preserve_five_centimeter_height(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pcd = root / "steps.pcd"
            pcd.write_text("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
                           "TYPE F F F\nWIDTH 2\nHEIGHT 1\nPOINTS 2\n"
                           "DATA ascii\n0.01 0.01 0.01\n0.16 0.01 0.16\n")
            resolution = [0.15, 0.15, 0.05]
            bounds, _ = converter.full_pcd_bounds(pcd, resolution)
            cells, lower = converter.occupied_voxels(pcd, bounds, resolution, 10)
            self.assertEqual(cells, {(0, 0, 0), (1, 0, 3)})
            mesh = root / "steps.obj"
            converter.write_obj(mesh, cells, lower, resolution)
            vertices = [line for line in mesh.read_text().splitlines() if line.startswith("v ")]
            self.assertIn("v 0.150000 0.000000 0.150000", vertices)
            self.assertIn("v 0.300000 0.150000 0.200000", vertices)


if __name__ == "__main__":
    unittest.main()
