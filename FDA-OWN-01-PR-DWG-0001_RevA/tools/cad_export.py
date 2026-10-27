"""CAD export of the Singas EL-GLR fluid-domain geometries: STEP, STL, DXF (2D GA + 3D mesh).

Design-intent geometry (wetted internal volume), built from the fdafoam Geometry parameters:
  * 2:1 semi-ellipsoidal bottom head, riser cylinder, conical transition, separator cylinder
    with a flat top (degassing boundary);
  * round downcomer loop (diameter dc_d) - not the equal-area square duct of the block mesher;
  * loop layout per variant: "mitred"  - right-angle corners (C2d26 / SCR block-mesh layout)
                             "elbow"   - 90 deg elbows, centreline radius r/D x dc_d (C8, C8SR)
                             "capsule" - fdafoam implicit model (snappy cases): inclined lower
                                         return, rounded joints (C2 / C2a BoD, C0 pilot)
Units: model and STL/STEP in metres (CFD convention); DXF in millimetres ($INSUNITS = 4).

Usage:  python tools/cad_export.py OUTDIR
"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import asdict, replace
from pathlib import Path

import cadquery as cq
import ezdxf
import numpy as np
from ezdxf.enums import TextEntityAlignment

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fdafoam.spec import Geometry  # noqa: E402

DOC_NO = "FDA-OWN-01-PR-DWG-0001"
REV = "A"
DATE = "2026-10-27"

BASE = Geometry()
P_TOP = {"G6_C0_pilot": 101325.0}
P_TOP = {k: P_TOP.get(k, 5.5e5) for k in ["G1_C2d26", "G2_SCR1", "G3_C8SR", "G4_C8", "G5_C2_BoD", "G6_C0_pilot"]}
VARIANTS = {
    # id: (geometry, layout, cases, fill level m, sheet scale, title)
    "G1_C2d26": (BASE, "mitred", "C1, C2b-C2d26, C2d26L, C2d26LD, SCR0, DP5/DP6/DP600f", 26.5, 50,
                 "Design point, 6.5 m separator, right-angle loop corners (block-mesh layout)"),
    "G2_SCR1": (replace(BASE, sep_d=7.5), "mitred", "SCR1", 26.5, 50,
                "Separator screening S-3: 7.5 m separator, right-angle loop corners"),
    "G3_C8SR": (replace(BASE, dc_x=5.4, dc_elbow_r_over_d=1.0), "elbow", "C8SR", 26.5, 50,
                "Short-radius 1.0D elbows (R 1.6 m), downcomer at 5.4 m"),
    "G4_C8": (replace(BASE, dc_x=7.2, dc_elbow_r_over_d=1.5), "elbow", "C8", 26.5, 50,
              "Long-radius 1.5D elbows (R 2.4 m), downcomer at 7.2 m"),
    "G5_C2_BoD": (BASE, "capsule", "C2, C2a, C3, C4, C6a/b (snappyHexMesh)", 25.0, 50,
                  "BoD implicit-surface model: inclined lower return (RPT-0001)"),
    "G6_C0_pilot": (Geometry(riser_d=0.19, riser_h=2.20, cone_h=0.10, sep_d=0.52, sep_h=0.65, dc_d=0.14,
                             dc_x=0.36, dc_top_z=2.45, dc_len=2.00, return_z=0.12, sparger_ring_d=0.12,
                             sparger_band_w=0.03), "capsule", "C0a-C0d (Young 1991 pilot validation)", 2.52, 5,
                    "Pilot validation reactor (Young 1991), implicit-surface layout"),
}


# ------------------------------------------------------------------ solid model
def _ellipse_pts(Rr, hd, r_from, r_to, n=24):
    t0, t1 = math.acos(min(r_from / Rr, 1.0)), math.acos(min(r_to / Rr, 1.0))
    return [(Rr * math.cos(t), -hd * math.sin(t)) for t in np.linspace(t0, t1, n)]


def vessel_profile(g: Geometry):
    """Half-section (r, z) of the vessel, head split at the sparger band edges."""
    Rr, Rs, hd = g.riser_d / 2, g.sep_d / 2, g.head_depth
    r1 = g.sparger_ring_d / 2 - g.sparger_band_w / 2
    r2 = g.sparger_ring_d / 2 + g.sparger_band_w / 2
    return Rr, Rs, hd, r1, r2


def vessel_solid(g: Geometry) -> cq.Workplane:
    Rr, Rs, hd, r1, r2 = vessel_profile(g)
    wp = cq.Workplane("XZ").moveTo(0, -hd)
    for a, b in ((0.0, r1), (r1, r2), (r2, Rr)):
        pts = _ellipse_pts(Rr, hd, a, b)
        wp = wp.spline(pts[1:], includeCurrent=True)
    wp = (wp.lineTo(Rr, g.riser_h).lineTo(Rs, g.sep_bottom_z).lineTo(Rs, g.top_z)
          .lineTo(0, g.top_z).close())
    return wp.revolve(360, (0, 0, 0), (0, 1, 0))


def _cyl(r, p0, p1):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    return cq.Solid.makeCylinder(r, float(np.linalg.norm(d)), cq.Vector(*p0), cq.Vector(*(d / np.linalg.norm(d))))


def _sphere(r, c):
    return cq.Solid.makeSphere(r, cq.Vector(*c), angleDegrees1=-90, angleDegrees2=90)


def _elbow(r, R, centre, start_dir, sweep_sign):
    """Quarter torus about the y axis through `centre`; tube radius r, centreline radius R."""
    cx, cz = centre
    sx, sz = start_dir                      # unit vector from centre to the start of the bend
    p = cq.Vector(cx + R * sx, 0, cz + R * sz)
    tangent = cq.Vector(-sz, 0, sx) * sweep_sign
    circ = cq.Wire.makeCircle(r, p, tangent)
    face = cq.Face.makeFromWires(circ)
    return cq.Solid.revolve(face, 90.0 * sweep_sign, cq.Vector(cx, -1, cz), cq.Vector(cx, 1, cz))


def loop_solids(g: Geometry, layout: str):
    r = g.dc_d / 2
    Rr, Rs = g.riser_d / 2, g.sep_d / 2
    zt, zr = g.dc_top_z, g.return_z
    x_in_t = 0.6 * min(Rr, Rs)              # pipes start well inside the vessel (union overlap)
    x_in_b = 0.6 * Rr
    parts, info = [], {}
    if layout == "mitred":
        parts += [_cyl(r, (g.dc_x, 0, zr - r), (g.dc_x, 0, zt + r)),
                  _cyl(r, (x_in_t, 0, zt), (g.dc_x, 0, zt)),
                  _cyl(r, (x_in_b, 0, zr), (g.dc_x, 0, zr))]
        info.update(straight_leg_m=zt - zr, top_connector_free_length_m=g.dc_x - r - Rs,
                    bottom_connector_free_length_m=g.dc_x - r - Rr)
    elif layout == "elbow":
        R = g.dc_elbow_r_over_d * g.dc_d
        xc = g.dc_x - R
        zct, zcb = zt - R, zr + R
        parts += [_cyl(r, (x_in_t, 0, zt), (xc, 0, zt)),
                  _elbow(r, R, (xc, zct), (0, 1), 1),
                  _cyl(r, (g.dc_x, 0, zcb), (g.dc_x, 0, zct)),
                  _elbow(r, R, (xc, zcb), (1, 0), 1),
                  _cyl(r, (xc, 0, zr), (x_in_b, 0, zr))]
        info.update(elbow_R_m=R, straight_leg_m=zct - zcb, top_stub_free_m=xc - Rs,
                    bottom_stub_free_m=xc - Rr, centreline_length_m=(xc - Rs) + (xc - Rr) + math.pi * R + (zct - zcb))
    elif layout == "capsule":
        zb = zt - g.dc_len
        x_top_end = Rs - 0.5
        x_bot_end = Rr - 0.5
        if x_bot_end < 0 or x_top_end < 0:
            # fdafoam.geometry.implicit hard-codes the 0.5 m inset: on small vessels the capsule runs
            # past the axis and out through the far wall. Clamp to the axis here (design intent).
            info["implicit_model_defect"] = (
                f"fdafoam implicit(): lower return ends at x = {x_bot_end:.3f} m and top connector at "
                f"x = {x_top_end:.3f} m (hard-coded 0.5 m inset); clamped to the vessel axis in this CAD")
            x_top_end, x_bot_end = max(x_top_end, 0.0), max(x_bot_end, 0.0)
        segs = [((g.dc_x, 0, zb), (g.dc_x, 0, zt)), ((g.dc_x, 0, zt), (x_top_end, 0, zt)),
                ((g.dc_x, 0, zb), (x_bot_end, 0, zr))]
        for a, b in segs:
            parts.append(_cyl(r, a, b))
        for c in {(g.dc_x, 0, zb), (g.dc_x, 0, zt)}:
            parts.append(_sphere(r, c))
        info.update(straight_leg_m=g.dc_len, return_incline_deg=math.degrees(
            math.atan2(zb - zr, g.dc_x - x_bot_end)))
    else:
        raise ValueError(layout)
    return parts, info


def fluid_solid(g: Geometry, layout: str):
    body = vessel_solid(g).val()
    parts, info = loop_solids(g, layout)
    for p in parts:
        body = body.fuse(p)
    body = body.clean()
    # trim anything above the flat top (capsule caps cannot reach it, but keep the top exact)
    return cq.Workplane().add(body), info


# ------------------------------------------------------------------ STL with CFD patches
def tess(shape, tol, ang=0.15):
    v, f = shape.tessellate(tol, ang)
    return np.array([[p.x, p.y, p.z] for p in v]), np.array(f, dtype=np.int64).reshape(-1, 3)


def write_stl(path, name, V, F):
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-30
    rec = np.zeros(len(F), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    rec["n"], rec["v"] = n, np.stack([a, b, c], axis=1)
    with open(path, "wb") as fh:
        fh.write(name.encode()[:80].ljust(80, b" "))
        fh.write(np.uint32(len(F)).tobytes())
        fh.write(rec.tobytes())


def patch_faces(g: Geometry, solid: cq.Shape):
    _, _, _, r1, r2 = vessel_profile(g)
    groups = {"walls": [], "inlet": [], "outlet": []}
    for face in solid.Faces():
        bb = face.BoundingBox()
        if face.geomType() == "PLANE" and abs(bb.zmin - g.top_z) < 1e-6 and abs(bb.zmax - g.top_z) < 1e-6:
            groups["outlet"].append(face)
            continue
        if bb.zmax < 1e-6:
            if abs(bb.xmax - r2) < 1e-3 and abs(bb.xmin + r2) < 1e-3:  # revolved band r1..r2 on the head
                groups["inlet"].append(face)
                continue
        groups["walls"].append(face)
    return groups


def export_stl(g, solid, outdir: Path, stem: str, tol, ang=0.05):
    """One consistent surface mesh of the solid, labelled by patch, oriented outward."""
    import trimesh
    outdir.mkdir(parents=True, exist_ok=True)
    groups = patch_faces(g, solid)          # classify on exact geometry, before meshing
    solid.tessellate(tol, ang)              # mesh all faces together -> shared edge discretisation
    names = list(groups)
    Vs, Fs, L, o = [], [], [], 0
    for k, name in enumerate(names):
        for fc in groups[name]:
            V, F = tess(fc, tol, ang)
            if len(F) == 0:
                continue
            Vs.append(V); Fs.append(F + o); L.append(np.full(len(F), k)); o += len(V)
    Vall, Fall, L = np.vstack(Vs), np.vstack(Fs), np.concatenate(L)
    _, first, inv = np.unique(np.round(Vall, 7), axis=0, return_index=True, return_inverse=True)
    Fall = inv.reshape(-1)[Fall]
    keep = (Fall[:, 0] != Fall[:, 1]) & (Fall[:, 1] != Fall[:, 2]) & (Fall[:, 0] != Fall[:, 2])
    m = trimesh.Trimesh(Vall[first], Fall[keep], process=False)
    L = L[keep]
    lab = {tuple(sorted(f)): l for f, l in zip(m.faces.tolist(), L.tolist())}
    trimesh.repair.fix_normals(m)
    if m.volume < 0:
        m.invert()
    V, F = np.asarray(m.vertices), np.asarray(m.faces)
    L = np.array([lab[tuple(sorted(f))] for f in F.tolist()])   # fix_normals may reorder faces
    pdir = outdir / f"{stem}_CFD_patches"
    pdir.mkdir(exist_ok=True)
    stats = {}
    for k, name in enumerate(names):
        Fk = F[L == k]
        if len(Fk) == 0:
            raise RuntimeError(f"{stem}: patch {name} is empty")
        write_stl(pdir / f"{name}.stl", name, V, Fk)
        a, b, c = V[Fk[:, 0]], V[Fk[:, 1]], V[Fk[:, 2]]
        stats[f"area_{name}_m2"] = float(0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum())
        stats[f"tri_{name}"] = int(len(Fk))
    write_stl(outdir / f"{stem}_fluid_domain_m.stl", stem, V, F)
    write_stl(outdir / f"{stem}_fluid_domain_mm.stl", stem + "_mm", V * 1000.0, F)
    return V, F, stats


def stl_check(V, F):
    import trimesh
    m = trimesh.Trimesh(V, F, process=False)
    return {"watertight": bool(m.is_watertight), "winding_consistent": bool(m.is_winding_consistent),
            "stl_volume_m3": float(m.volume), "triangles": int(len(m.faces)),
            "euler_number": int(m.euler_number), "bodies": int(m.body_count)}


# ------------------------------------------------------------------ DXF drawing
LAYERS = {"OUTLINE": 7, "HIDDEN": 8, "CENTRE": 1, "DIM": 3, "TEXT": 7, "BORDER": 7, "HATCH_LIQ": 5,
          "PATCH_INLET": 6, "PATCH_OUTLET": 4, "MESH_3D": 2}


def _new_doc():
    doc = ezdxf.new("R2018", setup=True)
    doc.header["$INSUNITS"] = 4
    doc.header["$MEASUREMENT"] = 1
    for name, col in LAYERS.items():
        lay = doc.layers.add(name, color=col)
        if name == "HIDDEN":
            lay.dxf.linetype = "DASHED"
        if name == "CENTRE":
            lay.dxf.linetype = "CENTER"
    return doc


def _section_outline(g, layout):
    """Elevation (x-z) outline in mm: list of polylines; uses an x-z slice of the solid."""
    return None


def draw_ga(path: Path, vid: str, g: Geometry, layout: str, cases: str, fill: float, scale: int, title: str,
            solid: cq.Shape, props: dict):
    doc = _new_doc()
    msp = doc.modelspace()
    S = 1000.0  # m -> mm
    th = 3.5 * scale  # text height for 3.5 mm on paper
    dimstyle = doc.dimstyles.duplicate_entry("EZDXF", "FDA")
    dimstyle.dxf.dimscale = scale
    dimstyle.dxf.dimtxt = 3.5
    dimstyle.dxf.dimasz = 2.5
    dimstyle.dxf.dimdec = 0
    dimstyle.dxf.dimlfac = 1.0
    dimstyle.dxf.dimtad = 1
    dimstyle.dxf.dimexe = 1.5
    dimstyle.dxf.dimexo = 1.0

    # ---- elevation: section of the solid on the y = 0 plane
    sec = solid.intersect(cq.Solid.makeBox(1e3, 1e-3, 1e3, cq.Vector(-500, -0.5e-3, -500)))
    for face in sec.Faces():
        if abs(face.normalAt().y) < 0.99:
            continue
        for e in face.Edges():              # each edge drawn on its own: no wire-order assumptions
            if e.geomType() == "LINE":
                ps = [e.startPoint(), e.endPoint()]
            else:
                ps = [e.positionAt(t) for t in np.linspace(0, 1, 64)]
            msp.add_lwpolyline(_dedupe([(p.x * S, p.z * S) for p in ps]), dxfattribs={"layer": "OUTLINE"})

    Rr, Rs, hd, r1, r2 = vessel_profile(g)
    # centre lines
    u = g.riser_d / 10.0  # layout unit, scales with the vessel
    msp.add_line((0, (-hd - u) * S), (0, (g.top_z + 1.6 * u) * S), dxfattribs={"layer": "CENTRE"})
    z_dc0 = g.return_z if layout != "capsule" else g.dc_top_z - g.dc_len
    msp.add_line((g.dc_x * S, (z_dc0 - g.dc_d) * S), (g.dc_x * S, (g.dc_top_z + g.dc_d) * S),
                 dxfattribs={"layer": "CENTRE"})
    # liquid level and sparger band
    msp.add_line((-Rs * S, fill * S), (Rs * S, fill * S), dxfattribs={"layer": "HATCH_LIQ", "linetype": "DASHED"})
    _txt(msp, f"INITIAL (UNGASSED) LIQUID LEVEL EL +{fill * S:.0f}", (-0.95 * Rs * S, fill * S + 0.6 * th), th, "HATCH_LIQ")
    zr1 = -hd * math.sqrt(max(1 - (r1 / Rr) ** 2, 0)) * S
    zr2 = -hd * math.sqrt(max(1 - (r2 / Rr) ** 2, 0)) * S
    for sgn in (-1, 1):
        msp.add_line((sgn * r1 * S, zr1), (sgn * r2 * S, zr2), dxfattribs={"layer": "PATCH_INLET", "lineweight": 70})
    msp.add_line((-Rs * S, g.top_z * S), (Rs * S, g.top_z * S), dxfattribs={"layer": "PATCH_OUTLET", "lineweight": 70})
    _txt(msp, "PATCH 'outlet' (DEGASSING, p = p_top)", (-Rs * S, (g.top_z * S) + 0.8 * th), th, "PATCH_OUTLET")
    _txt(msp, f"PATCH 'inlet' SPARGER BAND D{g.sparger_ring_d * S:.0f} x {g.sparger_band_w * S:.0f} WIDE",
         (-Rs * S, (-hd * S) - 3 * th), th, "PATCH_INLET")

    # dimensions (elevation)
    xd = -Rs * S - 6 * th
    dims = [(-hd, 0), (0, g.riser_h), (g.riser_h, g.sep_bottom_z), (g.sep_bottom_z, g.top_z)]
    for z0, z1 in dims:
        _vdim(msp, xd, z0 * S, z1 * S)
    _vdim(msp, xd - 6 * th, -hd * S, g.top_z * S)
    xr = max(Rs, g.dc_x + g.dc_d / 2) * S + 6 * th
    _vdim(msp, xr, 0, g.return_z * S)
    _vdim(msp, xr, 0, g.dc_top_z * S, off=6 * th)
    zt = (g.top_z + 0.0) * S + 5 * th
    _hdim(msp, -Rs * S, Rs * S, zt)
    _hdim(msp, -Rr * S, Rr * S, -hd * S - 7 * th)
    _hdim(msp, 0, g.dc_x * S, (g.dc_top_z * S) + (g.dc_d * S) + 5 * th)
    _hdim(msp, (g.dc_x - g.dc_d / 2) * S, (g.dc_x + g.dc_d / 2) * S, (g.riser_h / 2) * S)
    _txt(msp, f"DOWNCOMER ID {g.dc_d * S:.0f}", ((g.dc_x + g.dc_d / 2) * S + th, (g.riser_h / 2 - 3 * u) * S), th, "TEXT")
    _txt(msp, f"RISER ID {g.riser_d * S:.0f}", (-0.4 * Rr * S, g.riser_h * 0.55 * S), th, "TEXT")
    _txt(msp, f"SEPARATOR ID {g.sep_d * S:.0f}", (-0.4 * Rs * S, (g.sep_bottom_z + g.sep_h * 0.5) * S), th, "TEXT")
    _txt(msp, "2:1 SE HEAD", (-0.3 * Rr * S, -hd * 0.5 * S), th, "TEXT")
    if layout == "elbow":
        R = g.dc_elbow_r_over_d * g.dc_d
        _txt(msp, f"90 DEG ELBOW CL R{R * S:.0f} ({g.dc_elbow_r_over_d:g}D)",
             ((g.dc_x - R) * S, (g.dc_top_z + g.dc_d) * S + 1.5 * th), th, "TEXT")
    _txt(msp, "ELEVATION (SECTION ON LOOP PLANE y = 0)", (-Rs * S, -hd * S - 14 * th), 1.6 * th, "TEXT")

    # ---- plan view to the right
    px0 = xr + 25 * th + Rs * S
    pz0 = g.top_z * 0.55 * S
    msp.add_circle((px0, pz0), Rs * S, dxfattribs={"layer": "OUTLINE"})
    msp.add_circle((px0, pz0), Rr * S, dxfattribs={"layer": "HIDDEN"})
    msp.add_circle((px0, pz0), r1 * S, dxfattribs={"layer": "PATCH_INLET"})
    msp.add_circle((px0, pz0), r2 * S, dxfattribs={"layer": "PATCH_INLET"})
    msp.add_circle((px0 + g.dc_x * S, pz0), g.dc_d / 2 * S, dxfattribs={"layer": "OUTLINE"})
    rd = g.dc_d / 2 * S
    x_wall = math.sqrt(max((Rs * S) ** 2 - rd ** 2, 0.0))
    for sgn in (-1, 1):  # top connector band (stub + elbow seen from above), separator wall to downcomer
        msp.add_line((px0 + x_wall, pz0 + sgn * rd), (px0 + g.dc_x * S, pz0 + sgn * rd),
                     dxfattribs={"layer": "OUTLINE"})
    if layout != "capsule":
        msp.add_line((px0 + g.dc_x * S, pz0 + rd), (px0 + g.dc_x * S, pz0 - rd), dxfattribs={"layer": "HIDDEN"})
    msp.add_line((px0 - (Rs + u) * S, pz0), (px0 + (g.dc_x + g.dc_d + u) * S, pz0), dxfattribs={"layer": "CENTRE"})
    msp.add_line((px0, pz0 - (Rs + u) * S), (px0, pz0 + (Rs + u) * S), dxfattribs={"layer": "CENTRE"})
    _hdim(msp, px0, px0 + g.dc_x * S, pz0 - Rs * S - 5 * th)
    _txt(msp, "PLAN (DOWNCOMER LOOP ON +x)", (px0 - Rs * S, pz0 - Rs * S - 14 * th), 1.6 * th, "TEXT")

    # ---- data table
    tx, tz = px0 - Rs * S, pz0 - Rs * S - 22 * th
    rows = [("CASES", cases), ("LOOP LAYOUT", layout.upper()),
            ("FLUID DOMAIN VOLUME", f"{props['volume_m3']:.2f} m3"),
            (f"UNGASSED LIQUID TO EL {fill:.2f} m", f"{props['liquid_volume_m3']:.2f} m3"),
            ("RISER / DOWNCOMER AREA RATIO Ad/Ar", f"{(g.dc_d / g.riser_d) ** 2:.4f}"),
            ("OUTLET PATCH AREA", f"{props['area_outlet_m2']:.3f} m2"),
            ("INLET PATCH AREA", f"{props['area_inlet_m2']:.3f} m2")]
    for k, v in props.get("loop", {}).items():
        rows.append((k.upper().replace("_", " "), f"{v:.3f}" if isinstance(v, float) else str(v)))
    for i, (k, v) in enumerate(rows):
        _txt(msp, k, (tx, tz - i * 2 * th), th, "TEXT")
        _txt(msp, v, (tx + 45 * th, tz - i * 2 * th), th, "TEXT")

    notes = ["NOTES",
             "1. WETTED (FLUID) DOMAIN, DESIGN INTENT. NOT A FABRICATION DRAWING: NO WALL THICKNESS,",
             "   NOZZLES, FLANGES, SUPPORTS OR INTERNALS. VESSEL DESIGN TO PRESSURE-VESSEL WORKFLOW.",
             "2. ELEVATIONS FROM RISER BOTTOM TANGENT LINE (EL 0). DIMENSIONS IN mm.",
             "3. DOWNCOMER LOOP IS ROUND (ID AS SHOWN). THE CFD BLOCK MESH USES AN EQUAL-AREA SQUARE",
             "   DUCT AND A VOLUME-EQUIVALENT FLAT BOTTOM; SEE fda_case.json OF EACH RUN.",
             (f"4. HEADSPACE {props['p_top_bar_a']:.3f} bar a" + (" (DESIGN POINT); LIMIT 6.0 bar a (CFD ACCEPTANCE CRITERIA REV B)."
                 if props['p_top_bar_a'] > 2 else " (PILOT VALIDATION, ATMOSPHERIC).")),
             "5. DIMENSION FLAGS (RPT-0001 S3): dc_d, dc_x, cone_h, SPARGER = ASSUMED (HOLD 1).",
             "6. ADVISORY UNTIL SIGNED BY THE ENGINEER OF RECORD."]
    for i, n in enumerate(notes):
        _txt(msp, n, (tx, tz - (len(rows) + 2 + i) * 2 * th), th, "TEXT")

    # ---- border and title block (A0 landscape at sheet scale)
    W, H = 1189 * scale, 841 * scale
    ext = _extents(msp)
    cx, cz = (ext[0] + ext[2]) / 2, (ext[1] + ext[3]) / 2
    x0, z0 = cx - W / 2, cz - H / 2 - 30 * scale
    msp.add_lwpolyline([(x0, z0), (x0 + W, z0), (x0 + W, z0 + H), (x0, z0 + H)], close=True,
                       dxfattribs={"layer": "BORDER"})
    m = 10 * scale
    msp.add_lwpolyline([(x0 + m, z0 + m), (x0 + W - m, z0 + m), (x0 + W - m, z0 + H - m), (x0 + m, z0 + H - m)],
                       close=True, dxfattribs={"layer": "BORDER", "lineweight": 50})
    tbw, tbh = 180 * scale, 60 * scale
    bx, bz = x0 + W - m - tbw, z0 + m
    msp.add_lwpolyline([(bx, bz), (bx + tbw, bz), (bx + tbw, bz + tbh), (bx, bz + tbh)], close=True,
                       dxfattribs={"layer": "BORDER"})
    lines = [("SINGAS EL-GLR  -  FERMENTOR DESIGN AGENT", 4.0),
             (f"FLUID DOMAIN GA: {vid}", 3.5), (title.upper()[:70], 2.5),
             (f"DOC {DOC_NO}-{vid.split('_')[0][1:]:0>2}   REV {REV}   {DATE}", 3.0),
             (f"SCALE 1:{scale} (A0)   UNITS mm   PROJECTION: FIRST ANGLE", 2.5),
             ("PREPARED: CLAUDE   CHECKED: [PENDING]   APPROVED: [PENDING]", 2.5),
             ("STATUS: ISSUED FOR REVIEW (ADVISORY)", 2.5)]
    for i, (s, h) in enumerate(lines):
        _txt(msp, s, (bx + 4 * scale, bz + tbh - (8 + i * 7.5) * scale), h * scale, "BORDER")
    doc.saveas(path)


def _dedupe(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if abs(p[0] - out[-1][0]) > 1e-6 or abs(p[1] - out[-1][1]) > 1e-6:
            out.append(p)
    return out


def _txt(msp, s, at, h, layer):
    msp.add_text(s, height=h, dxfattribs={"layer": layer, "style": "OpenSans"}).set_placement(
        at, align=TextEntityAlignment.LEFT)


def _vdim(msp, x, z0, z1, off=0.0):
    msp.add_linear_dim(base=(x + off, (z0 + z1) / 2), p1=(x + 1, z0), p2=(x + 1, z1), angle=90,
                       dimstyle="FDA", dxfattribs={"layer": "DIM"}).render()


def _hdim(msp, x0, x1, z):
    msp.add_linear_dim(base=((x0 + x1) / 2, z), p1=(x0, z - 1), p2=(x1, z - 1), angle=0,
                       dimstyle="FDA", dxfattribs={"layer": "DIM"}).render()


def _extents(msp):
    from ezdxf import bbox
    b = bbox.extents(msp)
    return b.extmin.x, b.extmin.y, b.extmax.x, b.extmax.y


def write_3d_dxf(path: Path, V, F):
    doc = _new_doc()
    msp = doc.modelspace()
    mesh = msp.add_mesh(dxfattribs={"layer": "MESH_3D"})
    with mesh.edit_data() as md:
        md.vertices = [tuple(p * 1000.0) for p in V]
        md.faces = [tuple(int(i) for i in f) for f in F]
    doc.saveas(path)


# ------------------------------------------------------------------ driver
def build(vid, outroot: Path):
    g, layout, cases, fill, scale, title = VARIANTS[vid]
    out = outroot / vid
    out.mkdir(parents=True, exist_ok=True)
    wp, loop = fluid_solid(g, layout)
    solid = wp.val()
    assert solid.isValid(), vid
    vol = solid.Volume()
    cut = cq.Solid.makeBox(100, 100, fill + 50, cq.Vector(-50, -50, -50))
    vliq = solid.intersect(cut).Volume()
    cq.exporters.export(wp, str(out / f"{vid}_fluid_domain_m.step"))
    tol = 0.01 if g.riser_d > 1 else 0.001
    V, F, stats = export_stl(g, solid, out, vid, tol)
    chk = stl_check(V, F)
    props = {"case_id": vid, "doc_no": f"{DOC_NO}-{vid.split('_')[0][1:]:0>2}", "rev": REV, "cases": cases,
             "layout": layout, "title": title, "geometry": asdict(g), "volume_m3": vol,
             "liquid_fill_z_m": fill, "p_top_bar_a": P_TOP[vid] / 1e5, "liquid_volume_m3": vliq, "loop": loop, **stats, "stl_check": chk,
             "units": {"STEP/STL": "m (plus *_mm.stl in mm)", "DXF": "mm"}}
    draw_ga(out / f"{vid}_GA.dxf", vid, g, layout, cases, fill, scale, title, solid, props)
    # coarser mesh for the 3D DXF (keeps the DWG/DXF light)
    Vc, Fc = tess(solid, tol * 3, 0.3)
    write_3d_dxf(out / f"{vid}_3D_mesh.dxf", Vc, Fc)
    (out / f"{vid}_properties.json").write_text(json.dumps(props, indent=2))
    return props


if __name__ == "__main__":
    outroot = Path(sys.argv[1] if len(sys.argv) > 1 else "cad_out")
    only = sys.argv[2:] or list(VARIANTS)
    summary = {v: build(v, outroot) for v in only}
    for v, p in summary.items():
        print(f"{v:12s} V={p['volume_m3']:8.3f} m3  Vliq={p['liquid_volume_m3']:8.3f} m3  "
              f"inlet={p['area_inlet_m2']:.3f} outlet={p['area_outlet_m2']:.3f}  {p['stl_check']}  {p['loop']}")
