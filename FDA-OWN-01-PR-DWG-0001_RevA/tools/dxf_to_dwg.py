"""Write DWG (R2000 / AC1015) copies of the CAD export DXF files with ezdwg's native writer.

Dimensions are exploded to their rendered lines, arrow solids and text, so the DWG carries the
drawn appearance but not associative dimension objects; the DXF keeps the live DIMENSIONs.
3D MESH entities are written as 3DFACEs (one per triangle).

Usage:  python tools/dxf_to_dwg.py FILE.dxf [FILE.dxf ...]   -> FILE.dwg next to each, plus a
        read-back check (entity counts by type, model extents) printed as JSON.
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

import ezdwg
import ezdxf


def _attrs(e, layer=None):
    a = {"layer": layer or e.dxf.layer}
    if e.dxf.hasattr("color") and e.dxf.color not in (0, 256):
        a["color_index"] = int(e.dxf.color)
    return a


def _emit(msp, e, counts, layer=None):
    t = e.dxftype()
    at = _attrs(e, layer)
    if t == "LINE":
        msp.add_line(tuple(e.dxf.start), tuple(e.dxf.end), **at)
    elif t == "LWPOLYLINE":
        pts = [(p[0], p[1]) for p in e.get_points("xy")]
        if e.closed and pts and pts[0] != pts[-1]:
            pts.append(pts[0])
        msp.add_lwpolyline(pts, **at)
    elif t == "CIRCLE":
        msp.add_circle(tuple(e.dxf.center), e.dxf.radius, **at)
    elif t == "ARC":
        msp.add_arc(tuple(e.dxf.center), e.dxf.radius, e.dxf.start_angle, e.dxf.end_angle, **at)
    elif t == "TEXT":
        msp.add_text(e.dxf.text, tuple(e.dxf.insert), e.dxf.height, **at)
    elif t == "MTEXT":
        msp.add_text(e.plain_text(), tuple(e.dxf.insert), e.dxf.char_height, **at)
    elif t == "SOLID":
        msp.add_solid([tuple(e.dxf.get(f"vtx{i}")) for i in range(4)], **at)
    elif t == "DIMENSION":
        for v in e.virtual_entities():
            _emit(msp, v, counts, layer=e.dxf.layer)
        counts["DIMENSION(exploded)"] += 1
        return
    elif t == "INSERT":                     # dimension arrowheads
        for v in e.virtual_entities():
            _emit(msp, v, counts, layer=layer or e.dxf.layer)
        return
    elif t == "POINT":                      # DEFPOINTS of dimensions
        counts["POINT(defpoint, omitted)"] += 1
        return
    elif t == "HATCH":
        for path in e.paths.rendering_paths(e.dxf.hatch_style):
            pts = [tuple(v)[:2] for v in path.flattening(0.01)] if hasattr(path, "flattening") else []
            if len(pts) >= 3:
                msp.add_solid([pts[0], pts[1], pts[2], pts[-1] if len(pts) > 3 else pts[2]], **at)
        counts["HATCH->SOLID"] += 1
        return
    elif t == "MESH":
        V = [tuple(v) for v in e.vertices]
        for f in e.faces:
            q = [V[i] for i in f]
            if len(q) == 3:
                q.append(q[2])
            msp.add_3dface(q[:4], **at)
            counts["3DFACE"] += 1
        return
    else:
        counts[f"skipped:{t}"] += 1
        return
    counts[t] += 1


def convert(dxf_path: Path) -> dict:
    src = ezdxf.readfile(dxf_path)
    dwg = ezdwg.new()
    for lay in src.layers:
        if lay.dxf.name != "0":
            dwg.layers[lay.dxf.name] = {"color_index": abs(int(lay.dxf.color))}
    dwg.header["insunits"] = 4      # millimetres
    dwg.header["lunits"] = 2
    dwg.header["luprec"] = 0
    msp = dwg.modelspace()
    counts: Counter = Counter()
    for e in src.modelspace():
        _emit(msp, e, counts)
    out = dxf_path.with_suffix(".dwg")
    res = dwg.save(str(out), strict=False)
    back = ezdwg.read(str(out))
    got = Counter(e.dxftype for e in back.modelspace().query())
    return {"dwg": out.name, "bytes": out.stat().st_size, "version": "AC1015 (R2000)",
            "written": dict(counts), "read_back": dict(got),
            "diagnostics": [str(d) for d in getattr(res, "diagnostics", ())][:5]}


if __name__ == "__main__":
    report = {p: convert(Path(p)) for p in sys.argv[1:]}
    print(json.dumps(report, indent=1))
