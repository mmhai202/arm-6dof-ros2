#!/usr/bin/env python3

from __future__ import annotations

import sys
from pathlib import Path

from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool


ROOT = Path(__file__).resolve().parents[3]
STEP_FILE = ROOT / "UR5eUR7e.step" / "UR7e.step"
OUTPUT_DIR = ROOT / "src" / "arm_description" / "meshes" / "ur7e_raw"


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: extract_ur7e_component.py <component_index>", file=sys.stderr)
        return 2

    index = int(sys.argv[1])
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("ur7e"))
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.ReadFile(str(STEP_FILE))
    reader.Transfer(doc)

    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    components = TDF_LabelSequence()
    shape_tool.GetComponents_s(roots.Value(1), components)

    if index < 1 or index > components.Length():
        print(f"component index out of range: {index}", file=sys.stderr)
        return 2

    shape = shape_tool.GetShape_s(components.Value(index))
    target = OUTPUT_DIR / f"component_{index:02d}.stl"

    print(f"[mesh] component {index}", flush=True)
    mesh = BRepMesh_IncrementalMesh(shape, 2.0, False, 1.0, False)
    mesh.Perform()

    print(f"[write] {target}", flush=True)
    writer = StlAPI_Writer()
    writer.Write(shape, str(target))
    print(f"[ok] {target}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
