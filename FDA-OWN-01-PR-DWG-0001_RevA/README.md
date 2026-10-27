# FDA-OWN-01-PR-DWG-0001 Rev A — Singas EL-GLR fluid-domain CAD set (DWG, DXF, STL, STEP)

Status: issued for review (advisory). Prepared: Claude, 2026-10-27. Checked / approved: [Pending].

Wetted (fluid) domain of every geometry in the CFD case matrix (`cases/singas_elglr.yaml`, branch
`claude/openfoam-cfd-agent-ntq0m2` at ccfeb6b), built from the fdafoam `Geometry` parameters.
Design intent: 2:1 SE bottom head, round downcomer loop. Not a fabrication model: no wall
thickness, nozzles, flanges, supports or internals (those belong to the pressure-vessel workflow).

## Sheets and cases

| Sheet | Folder | Geometry | Cases using it | Volume m³ | Ungassed liquid m³ (fill EL) |
|---|---|---|---|---|---|
| -01 | `G1_C2d26` | 6.5 m separator, right-angle loop corners, dc_x 4.55 m | C1, C2b–C2d26, C2d26L, C2d26LD, SCR0, DP5/6/600f | 818.81 | 619.71 (26.5 m) |
| -02 | `G2_SCR1` | 7.5 m separator (S-3), otherwise G1 | SCR1 | 905.25 | 640.18 (26.5 m) |
| -03 | `G3_C8SR` | 1.0D short-radius elbows (R 1.6 m), dc_x 5.4 m | C8SR | 818.98 | 619.88 (26.5 m) |
| -04 | `G4_C8` | 1.5D long-radius elbows (R 2.4 m), dc_x 7.2 m | C8 | 824.84 | 625.74 (26.5 m) |
| -05 | `G5_C2_BoD` | RPT-0001 implicit-surface model, inclined lower return (62°) | C2, C2a, C3, C4, C6a/b (snappyHexMesh) | 813.22 | 560.97 (25.0 m) |
| -06 | `G6_C0_pilot` | Young (1991) pilot, 0.19 m riser / 0.14 m downcomer | C0a–C0d | 0.250 | 0.159 (2.52 m) |

Ad/Ar = 0.1024 for G1–G5 (0.5429 for the pilot).

## Files in each folder

| File | Format / units | Use |
|---|---|---|
| `*_GA.dxf` | DXF R2018, mm, A0 sheet (1:50; pilot 1:5) | Elevation section on the loop plane, plan, dimensions, data table, notes, title block. Live DIMENSION entities, layered (OUTLINE, CENTRE, DIM, PATCH_INLET, PATCH_OUTLET, HATCH_LIQ, BORDER) |
| `*_GA.dwg` | DWG R2000 (AC1015), mm | Same sheet. Dimensions exploded to lines/text; linetypes not carried (centre/dashed lines plot continuous) |
| `*_3D_mesh.dxf` / `.dwg` | DXF MESH / DWG 3DFACE, mm | 3D model of the fluid domain for CAD viewing and clash checks |
| `*_fluid_domain_m.step` | STEP AP214, m | Exact B-rep solid (best CAD import) |
| `*_fluid_domain_m.stl` / `_mm.stl` | binary STL, m / mm | Closed, outward-oriented surface; m for CFD, mm for CAD/print |
| `*_CFD_patches/{walls,inlet,outlet}.stl` | binary STL, m | snappyHexMesh-ready patches, names match the fdafoam case templates (`constant/triSurface/`) |
| `*_properties.json` | JSON | Parameters, volumes, patch areas, STL checks, loop dimensions |

## Checks

- **Solids:** all six valid, one solid each. STL watertight and winding-consistent, Euler number 0 (genus 1: the external loop closes a ring, as it should).
- **STL vs exact volume:** within 0.012 %.
- **Patches:** outlet area = π/4·D_sep² exactly (33.18 m², 44.17 m² for SCR1). Inlet band 2.452 m² (Ø3.5 m × 0.2 m band measured on the head surface).
- **Pilot:** 0.159 m³ ungassed at EL 2.52 m, against about 160 L published (Young 1991).
- **Against the CFD block meshes** (checkMesh total volume): G1 818.81 vs 815.42 m³ (C2d26), G2 905.25 vs 901.6, G3 818.98 vs 816.07, G4 824.84 vs 821.93. The CAD is +3.4 to +3.7 m³ (0.4 %). About 3.0 m³ is the flat nozzle strip the block mesher cuts along the +x side of the vessel. The rest is square-duct versus round-pipe corner detail.
- **DWG:** written natively (ezdwg R2000 writer), read back with matching entity counts per type, and re-rendered. Not yet opened in AutoCAD.

## Engineering findings

1. **SCR1 (sheet -02):** with a round 1.6 m pipe at dc_x 4.55 m, the downcomer outer wall is tangent to the 7.5 m separator shell. The top connector free length is 0.00 m. The CFD square duct shows 0.16 m only because of the flat strip. There is no room for a nozzle, reinforcement pad or flange. Restoring the G1 stub (0.5 m round) needs dc_x ≈ 5.05 m. That is a second variable, so it should be its own screening case after SCR1.
2. **Pilot (sheet -06):** `fdafoam.geometry.implicit()` places the loop ends at `R − 0.5 m`. On the pilot that is x = −0.405 m (lower return) and −0.240 m (top connector), so the snappy surface carries the return pipe through the riser axis and about 0.3–0.4 m out through the far wall. The CAD clamps both ends to the axis. Fix `implicit()` (inset as a fraction of R) before any C0 run.
3. **Loop resistance:** the elbow variants cut the per-bend loss coefficient from about 0.57 (mitred, 60 f_T) to 0.19 (1.0D) and 0.13 (1.5D) (Crane TP-410, as recorded in RPT-0003 Att. D). C8 adds 6.0 m³ and moves the downcomer 2.65 m out, which is a layout and support-frame cost.

## Regeneration

`python tools/cad_export.py OUTDIR` then `python tools/dxf_to_dwg.py OUTDIR/*/*.dxf`
(cadquery 2.8, ezdxf 1.4.5, trimesh, ezdwg 0.17). Both scripts are in `tools/` of this package.

Every result is advisory until signed by the engineer of record.
