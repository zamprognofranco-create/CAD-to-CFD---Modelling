# CAD-to-CFD---Modelling
Model Geometries - Gas Lift Fermentor (Singas external-loop gas-lift reactor, EL-GLR)

CAD of the fluid domains used by the Fermentor Design Agent CFD work
(repository `zamprognofranco-create/fermentor-cfd`, case matrix `cases/singas_elglr.yaml`).

| Document | Rev | Content | Status |
|---|---|---|---|
| [FDA-OWN-01-PR-DWG-0001](FDA-OWN-01-PR-DWG-0001_RevA/README.md) | A | Fluid-domain CAD set for six geometries (G1 C2d26, G2 SCR1, G3 C8SR, G4 C8, G5 C2 BoD, G6 C0 pilot): GA sheets DXF/DWG, 3D models DXF/DWG, STEP, STL (m, mm, CFD patches) | Issued for review (advisory), 2026-10-27 |

Units: STEP and STL in metres (plus `*_mm.stl`), DXF/DWG in millimetres.
Regenerate with `tools/cad_export.py` and `tools/dxf_to_dwg.py` inside the document folder. `cad_export.py` imports
`fdafoam.spec.Geometry` from Fermentor-CFD: run it from a Fermentor-CFD checkout (the scripts are also in its `tools/`),
or put that repository on `PYTHONPATH`.
Every result is advisory until signed by the engineer of record.
