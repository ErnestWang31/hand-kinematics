"""
Swept motion envelope: the volume each digit can occupy anywhere in its
constrained range of motion. Drop these into your assembly as keep-out bodies
and a linkage that stays clear of them is clear of the hand in every pose,
instead of only in the poses you remembered to check.

    python3 hand_envelope.py --subject m95
    python3 hand_envelope.py --subject m95 --res 1.0 --margin 1.5 --faces 2500

Method: sample each digit's joint space under the same biomechanical
constraints the bench enforces (DIP coupled to PIP, abduction authority falling
off with MCP flexion), rasterise every bone as a capsule into an occupancy
grid, then surface it. Unlike the hand solids in hand_export.py these are
faceted, not analytic B-rep — a swept volume has no closed form. They are
deliberately conservative: --margin inflates every capsule so the result
contains the true envelope despite voxel and decimation error.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import time

import numpy as np

from hand_kinematics import FINGERS, HandModel

DIGITS = ["thumb"] + FINGERS
BONE_KEYS = {
    "thumb": ["carpal", "mc", "pp", "dp"],
    **{f: ["mc", "pp", "mp", "dp"] for f in FINGERS},
}


# --------------------------------------------------------------------------- #
# joint-space sampling, under the same constraints the bench enforces
# --------------------------------------------------------------------------- #
def samples(model: HandModel, digit: str, n: int) -> list[dict]:
    j = model.cfg["joints"]
    out = []
    if digit == "thumb":
        L = j
        grid = (
            np.linspace(*L["thumb_cmc_flexion"], max(n - 2, 3)),
            np.linspace(*L["thumb_cmc_abduction"], max(n - 1, 4)),
            np.linspace(*L["thumb_cmc_rotation"], 3),
            np.linspace(*L["thumb_mcp_flexion"], max(n - 3, 3)),
            np.linspace(*L["thumb_ip_flexion"], max(n - 3, 3)),
        )
        for cf in grid[0]:
            for ca in grid[1]:
                for cr in grid[2]:
                    for mc in grid[3]:
                        for ip in grid[4]:
                            out.append(dict(thumb_cmc_flex=cf, thumb_cmc_abd=ca,
                                            thumb_cmc_rot=cr, thumb_mcp_flex=mc,
                                            thumb_ip_flex=ip))
        return out

    f = digit
    for mcp in np.linspace(*j["finger_mcp_flexion"][f], n):
        # abduction authority collapses as the MCP flexes — sampling the full
        # +-20 deg at 90 deg of flexion would invent an envelope no hand reaches
        a = j["finger_mcp_abduction"][f][1] * max(0.0, 1 - max(mcp, 0) / 90)
        for abd in ([0.0] if a < 1 else [-a, 0.0, a]):
            for pip in np.linspace(*j["finger_pip_flexion"][f], n):
                out.append({f"{f}_mcp_flex": mcp, f"{f}_mcp_abd": abd,
                            f"{f}_pip_flex": pip,
                            f"{f}_dip_flex": float(np.clip(0.67 * pip,
                                                  *j["finger_dip_flexion"][f]))})
    return out


def capsules(model: HandModel, digit: str, qs: list[dict], margin: float):
    """(a, b, r) per bone, in mm, for every sampled pose. Metacarpals excluded —
    they do not move, so they belong to the static hand, not the envelope."""
    HB = model.hand_breadth * 1000.0
    rad = model.cfg["segment_radius_hb"][digit]
    keys = BONE_KEYS[digit]
    start = 2 if digit == "thumb" else 1
    out = []
    for q in qs:
        P = np.asarray(model.fk(q)[digit]) * 1000.0
        for i in range(start, len(P) - 1):
            r = max(rad[keys[i - 1]], rad[keys[i]]) * HB + margin
            out.append((P[i], P[i + 1], r))
    return out


# --------------------------------------------------------------------------- #
# occupancy grid
# --------------------------------------------------------------------------- #
def voxelize(caps, origin, res, shape) -> np.ndarray:
    g = np.zeros(shape, dtype=bool)
    ax = [origin[k] + res * np.arange(shape[k]) for k in range(3)]
    for a, b, r in caps:
        lo = np.minimum(a, b) - r
        hi = np.maximum(a, b) + r
        i0 = np.maximum(((lo - origin) / res).astype(int), 0)
        i1 = np.minimum(((hi - origin) / res).astype(int) + 2, shape)
        if np.any(i1 <= i0):
            continue
        X, Y, Z = np.meshgrid(ax[0][i0[0]:i1[0]], ax[1][i0[1]:i1[1]],
                              ax[2][i0[2]:i1[2]], indexing="ij")
        P = np.stack([X, Y, Z], -1) - a
        ab = b - a
        L2 = float(ab @ ab)
        t = np.clip((P @ ab) / L2, 0, 1)[..., None] if L2 > 1e-9 else 0.0
        d2 = ((P - t * ab) ** 2).sum(-1)
        g[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]] |= d2 <= r * r
    return g


def surface(g: np.ndarray, origin, res, target_faces: int):
    from skimage.measure import marching_cubes
    import trimesh
    pad = np.pad(g.astype(np.float32), 1)
    v, f, _, _ = marching_cubes(pad, level=0.5, spacing=(res, res, res))
    v += origin - res                       # undo the pad, into world mm
    m = trimesh.Trimesh(v, f, process=True)
    if target_faces and len(m.faces) > target_faces:
        m = m.simplify_quadric_decimation(face_count=target_faces)
    m.fix_normals()
    return m


def to_solid(mesh):
    """Sew a triangle soup into a STEP-able solid body."""
    import cadquery as cq
    from cadquery import Vector
    faces = []
    for tri in mesh.vertices[mesh.faces]:
        try:
            w = cq.Wire.makePolygon([Vector(*p) for p in tri] + [Vector(*tri[0])])
            faces.append(cq.Face.makeFromWires(w))
        except Exception:
            continue
    shell = cq.Shell.makeShell(faces)
    try:
        return cq.Solid.makeSolid(shell)
    except Exception:
        return shell


# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--subject", default="m95",
                    choices=["f05", "f50", "f95", "m05", "m50", "m95"],
                    help="default m95 — size the keep-out for the largest hand you support")
    ap.add_argument("--res", type=float, default=1.5, help="voxel size, mm")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="mm added to every bone radius, keeps the result conservative")
    ap.add_argument("--n", type=int, default=9, help="samples per joint axis")
    ap.add_argument("--faces", type=int, default=1200, help="triangles per digit, 0 = no decimation")
    ap.add_argument("--no-step", action="store_true")
    ap.add_argument("--out", default="export")
    a = ap.parse_args()

    out = pathlib.Path(a.out)
    out.mkdir(exist_ok=True)
    model = HandModel.from_yaml(subject=a.subject)
    print(f"envelope for {a.subject}  HL {model.hand_length*1000:.0f} mm  "
          f"res {a.res} mm  margin {a.margin} mm")

    caps = {}
    for d in DIGITS:
        qs = samples(model, d, a.n)
        caps[d] = capsules(model, d, qs, a.margin)
        print(f"  {d:<7} {len(qs):>5} poses  {len(caps[d]):>6} capsules")

    allc = [c for d in DIGITS for c in caps[d]]
    lo = np.min([np.minimum(x[0], x[1]) - x[2] for x in allc], axis=0) - a.res * 2
    hi = np.max([np.maximum(x[0], x[1]) + x[2] for x in allc], axis=0) + a.res * 2
    shape = tuple(int(np.ceil(v)) for v in (hi - lo) / a.res + 1)
    print(f"  grid {shape}  =  {np.prod(shape)/1e6:.1f} M voxels  "
          f"over {(hi-lo)[0]:.0f} x {(hi-lo)[1]:.0f} x {(hi-lo)[2]:.0f} mm")

    grids, meshes = {}, {}
    for d in DIGITS:
        t = time.time()
        grids[d] = voxelize(caps[d], lo, a.res, shape)
        meshes[d] = surface(grids[d], lo, a.res, a.faces)
        vol = grids[d].sum() * a.res ** 3 / 1000.0
        print(f"  {d:<7} {vol:7.1f} cm3   {len(meshes[d].faces):>5} tris   {time.time()-t:5.1f}s")
        meshes[d].export(str(out / f"envelope_{a.subject}_{d}.stl"))

    # --- the number that actually drives the design -------------------------
    report = {"subject": a.subject, "res_mm": a.res, "margin_mm": a.margin,
              "volume_cm3": {d: round(float(grids[d].sum()) * a.res**3 / 1000, 1) for d in DIGITS},
              "pairwise_overlap_cm3": {}}
    print("\n  shared volume — any rigid part here is hit by both digits in some pose:")
    for i, d1 in enumerate(DIGITS):
        for d2 in DIGITS[i + 1:]:
            v = float((grids[d1] & grids[d2]).sum()) * a.res ** 3 / 1000.0
            report["pairwise_overlap_cm3"][f"{d1}/{d2}"] = round(v, 1)
            if v > 0.5:
                print(f"    {d1:<7} {d2:<7} {v:6.1f} cm3")

    union = np.zeros_like(grids["thumb"])
    for d in DIGITS:
        union |= grids[d]
    report["union_cm3"] = round(float(union.sum()) * a.res ** 3 / 1000, 1)
    # one body for "does my exoskeleton frame ever touch the hand at all"
    um = surface(union, lo, a.res, a.faces * 2)
    um.export(str(out / f"envelope_{a.subject}_union.stl"))
    meshes["union"] = um
    (out / f"envelope_{a.subject}_report.json").write_text(json.dumps(report, indent=1))
    print(f"\n  total swept volume {report['union_cm3']:.0f} cm3")

    if not a.no_step:
        import cadquery as cq
        t = time.time()
        assy = cq.Assembly(name=f"envelope_{a.subject}")
        COLOR = {"thumb": (0.77, 0.38, 0.23), "index": (0.16, 0.49, 0.62),
                 "middle": (0.23, 0.56, 0.45), "ring": (0.44, 0.39, 0.69),
                 "little": (0.66, 0.52, 0.12)}
        for d in DIGITS:
            assy.add(to_solid(meshes[d]), name=f"{d}_envelope",
                     color=cq.Color(*COLOR[d], 0.45))
        p = out / f"envelope_{a.subject}.step"
        (getattr(assy, "export", None) or assy.save)(str(p))
        print(f"  {p.name}   {p.stat().st_size/1e6:.1f} MB   {time.time()-t:.1f}s")

    print(f"  {len(meshes)} STL files in {out}/")


if __name__ == "__main__":
    main()
