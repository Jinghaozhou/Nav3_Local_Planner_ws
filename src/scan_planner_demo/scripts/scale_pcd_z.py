#!/usr/bin/env python3
"""Write a binary XYZ PCD whose heights are scaled about a fixed datum."""

import argparse
from pathlib import Path
import sys

import numpy as np

from pcd_to_sdf import read_pcd_chunks


def scale_pcd(source: Path, target: Path, factor: float, datum: float) -> int:
    if not source.is_file() or source.resolve() == target.resolve():
        raise ValueError("Source must exist and differ from the output PCD")
    if target.exists():
        raise FileExistsError(f"Output already exists: {target}")
    if not np.isfinite(factor) or factor <= 0 or not np.isfinite(datum):
        raise ValueError("Scale must be positive and datum finite")

    count = sum(len(chunk) for chunk in read_pcd_chunks(source))
    if count == 0:
        raise ValueError("Input PCD has no finite XYZ points")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    try:
        with temporary.open("wb") as output:
            output.write((
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                f"COUNT 1 1 1\nWIDTH {count}\nHEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {count}\nDATA binary\n"
            ).encode("ascii"))
            for xyz in read_pcd_chunks(source):
                xyz[:, 2] = datum + factor * (xyz[:, 2] - datum)
                output.write(xyz.astype("<f4", copy=False).tobytes())
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--factor", type=float, default=0.75)
    parser.add_argument("--datum", type=float, default=0.0)
    args = parser.parse_args()
    try:
        count = scale_pcd(args.source, args.output, args.factor, args.datum)
    except (ValueError, FileExistsError) as error:
        parser.error(str(error))
    print(f"Wrote {count} points to {args.output}; z'={args.datum}+{args.factor}*(z-{args.datum})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
