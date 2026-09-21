"""
Export the parametric hand as STEP B-rep solids for CAD, plus STL meshes.

    python3 hand_export.py --subject m50 --pose "power grasp"
    python3 hand_export.py --all-poses --clearance 2.0
    python3 hand_export.py --pose-file mypose.json      # exported from the web bench

Output per run, in ./export/ :
    <name>.step          assembly — palm + 5 finger stalls, named and coloured
    <name>.stl           triangulated mesh of the same thing
    <name>_frames.json   every joint centre and joint axis, for placing hardware
    <name>_clear<t>.step the same hand inflated by t mm — use it as the cavity to
                         cut a glove liner, or as the keep-out for an exoskeleton

Coordinate frame (all files, millimetres):
    origin  midpoint of the distal wrist crease
    +X      radial, toward the thumb
    +Y      distal, down the middle finger
    +Z      dorsal, out the back of the hand
Right hand. Mirror across X for the left.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib

import cadquery as cq
import numpy as np
from cadquery import Vector

from hand_kinematics import FINGERS, HandModel, POSES, pose

DIGITS = ["thumb"] + FINGERS
BONE_KEYS = {
    "thumb": ["carpal", "mc", "pp", "dp"],
    **{f: ["mc", "pp", "mp", "dp"] for f in FINGERS},
}
COLOR = {
    "palm":   (0.78, 0.78, 0.80),
    "thumb":  (0.77, 0.38, 0.23),
    "index":  (0.16, 0.49, 0.62),
    "middle": (0.23, 0.56, 0.45),
    "ring":   (0.44, 0.39, 0.69),
    "little": (0.66, 0.52, 0.12),
}


# --------------------------------------------------------------------------- #
# primitives
# --------------------------------------------------------------------------- #
def frustum(a: np.ndarray, b: np.ndarray, ra: float, rb: float):
    """Tapered solid from a to b. A true conical/cylindrical B-rep face, not a mesh."""
    v = b - a
    h = float(np.linalg.norm(v))
    if h < 1e-6:
        return None
    axis = Vector(*(v / h))
    if abs(ra - rb) < 1e-6:
        return cq.Solid.makeCylinder(ra, h, Vector(*a), axis)
    return cq.Solid.makeCone(ra, rb, h, Vector(*a), axis)


def ball(c: np.ndarray, r: float):
    return cq.Solid.makeSphere(
        r, Vector(*c), angleDegrees1=-90, angleDegrees2=90, angleDegrees3=360
    )


def fuse_all(solids):
    solids = [s for s in solids if s is not None]
    out = solids[0]
    for s in solids[1:]:
        out = out.fuse(s)
    return out.clean()


# --------------------------------------------------------------------------- #
# the hand
# --------------------------------------------------------------------------- #
def build(model: HandModel, q: dict, clearance: float = 0.0) -> dict:
    """Returns {part name: cq.Solid}, in millimetres."""
    K = {d: np.asarray(P) * 1000.0 for d, P in model.fk(q).items()}
    HB = model.hand_breadth * 1000.0
    rad = model.cfg["segment_radius_hb"]

    def r(d: str, key: str) -> float:
        return rad[d][key] * HB + clearance

    parts: dict[str, cq.Solid] = {}

    # --- finger stalls: MCP to tip. One solid per digit is what you want when
    # --- you are drawing a glove finger around it.
    for d in DIGITS:
        P, keys = K[d], BONE_KEYS[d]
        start = 2 if d == "thumb" else 1       # skip the metacarpal, it lives in the palm
        solids = []
        for i in range(start, len(P) - 1):
            ra, rb = r(d, keys[i - 1]), r(d, keys[i])
            solids.append(frustum(P[i], P[i + 1], ra, rb))
            solids.append(ball(P[i], ra))
        solids.append(ball(P[-1], r(d, keys[-1])))      # rounded fingertip
        parts[d] = fuse_all(solids)

    # --- palm: a lofted mass through three cross-sections, with the metacarpals
    # --- fused in so the knuckle arch and the thenar eminence come through.
    y_knuckle = float(np.mean([K[f][1][1] for f in FINGERS]))
    c = clearance
    sections = [
        (0.0,              0.365 * HB + c, 0.145 * HB + c, 0.0),
        (0.45 * y_knuckle, 0.465 * HB + c, 0.168 * HB + c, 0.0),
        (y_knuckle,        0.505 * HB + c, 0.156 * HB + c, 0.010 * model.hand_length * 1000),
    ]
    wires = [
        cq.Wire.makeEllipse(a, b, Vector(0, y, z), Vector(0, 1, 0), Vector(1, 0, 0))
        for y, a, b, z in sections
    ]
    palm = [cq.Solid.makeLoft(wires)]
    for f in FINGERS:
        palm.append(frustum(K[f][0], K[f][1], r(f, "mc") * 1.25, r(f, "mc")))
        palm.append(ball(K[f][1], r(f, "mc")))
    # start the thenar mass inside the palm, not on the wrist plane, or its end
    # cap pokes out below the wrist
    thenar_root = K["thumb"][0] + 0.35 * (K["thumb"][1] - K["thumb"][0])
    palm.append(frustum(thenar_root, K["thumb"][1],
                        r("thumb", "carpal") * 0.8, r("thumb", "carpal")))
    palm.append(frustum(K["thumb"][1], K["thumb"][2], r("thumb", "carpal"), r("thumb", "mc")))
    palm.append(ball(K["thumb"][2], r("thumb", "mc")))
    parts["palm"] = fuse_all(palm)
    return parts


def frames(model: HandModel, q: dict) -> dict:
    """Joint centres and the axis each joint rotates about, in the export frame.

    This is the part you actually need for hardware: to place a sensor, a
    linkage pivot or a strap anchor you need the axis, not just the point.
    """
    K = {d: np.asarray(P) * 1000.0 for d, P in model.fk(q).items()}
    eps = 1e-4
    out = {"units": "mm", "frame": "X radial, Y distal, Z dorsal; origin at wrist crease",
           "joints": []}
    NAMES = {
        "thumb": [("CMC", 1), ("MCP", 2), ("IP", 3), ("TIP", 4)],
        **{f: [("MCP", 1), ("PIP", 2), ("DIP", 3), ("TIP", 4)] for f in FINGERS},
    }
    dof = {
        "thumb": {"CMC": "thumb_cmc_flex", "MCP": "thumb_mcp_flex", "IP": "thumb_ip_flex"},
        **{f: {"MCP": f"{f}_mcp_flex", "PIP": f"{f}_pip_flex", "DIP": f"{f}_dip_flex"}
           for f in FINGERS},
    }
    for d in DIGITS:
        for name, idx in NAMES[d]:
            rec = {"digit": d, "joint": name, "position": [round(v, 3) for v in K[d][idx]]}
            key = dof[d].get(name)
            if key:
                # numeric axis: perturb the joint and see which way the distal end swings
                q2 = dict(q)
                q2[key] = q.get(key, 0.0) + 0.5
                K2 = {dd: np.asarray(P) * 1000.0 for dd, P in model.fk(q2).items()}
                a, b = K[d][idx + 1] - K[d][idx], K2[d][idx + 1] - K[d][idx]
                ax = np.cross(a, b)
                n = np.linalg.norm(ax)
                if n > eps:
                    rec["flexion_axis"] = [round(v, 5) for v in (ax / n)]
            out["joints"].append(rec)
    return out


# --------------------------------------------------------------------------- #
# poses
# --------------------------------------------------------------------------- #
def pose_from_json(path: str):
    """Read a pose exported by the web bench. Returns (q, hl_m, hb_m, scale)."""
    J = json.load(open(path))
    j = J["joints"]
    q = {}
    for f in FINGERS:
        s = j[f]
        q[f"{f}_mcp_flex"] = s["mcp"]
        q[f"{f}_mcp_abd"] = s["abd"]
        q[f"{f}_pip_flex"] = s["pip"]
        q[f"{f}_dip_flex"] = s["dip"]
    t = j["thumb"]
    q.update(thumb_cmc_flex=t["cmcF"], thumb_cmc_abd=t["cmcA"], thumb_cmc_rot=t["cmcR"],
             thumb_mcp_flex=t["mcp"], thumb_mcp_abd=t.get("abd", 0.0), thumb_ip_flex=t["ip"])
    sub = J.get("subject", {})
    return (q,
            sub.get("hand_length_mm", 194.1) / 1000.0,
            sub.get("hand_breadth_mm", 90.4) / 1000.0,
            J.get("scale"))


# --------------------------------------------------------------------------- #
def write(parts: dict, path: pathlib.Path, stl: bool) -> None:
    assy = cq.Assembly(name=path.name)
    for name in ["palm"] + DIGITS:
        assy.add(parts[name], name=name, color=cq.Color(*COLOR[name]))
    # NB: never with_suffix() here — a clearance like 1.5 puts a dot in the stem
    exporter = getattr(assy, "export", None) or assy.save
    exporter(f"{path}.step")
    print("  ", pathlib.Path(f"{path}.step").name)
    if stl:
        cq.exporters.export(fuse_all(list(parts.values())), f"{path}.stl", tolerance=0.08)
        print("  ", pathlib.Path(f"{path}.stl").name)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--subject", default="m50",
                    choices=["f05", "f50", "f95", "m05", "m50", "m95"])
    ap.add_argument("--pose", default="rest", help="a named pose, case-insensitive")
    ap.add_argument("--all-poses", action="store_true")
    ap.add_argument("--all-subjects", action="store_true",
                    help="same pose across the whole 5th-female to 95th-male span")
    ap.add_argument("--pose-file", help="pose JSON exported from the web bench")
    ap.add_argument("--clearance", type=float, default=0.0,
                    help="mm of offset for a second, inflated solid (glove liner cavity)")
    ap.add_argument("--no-stl", action="store_true")
    ap.add_argument("--out", default="export")
    a = ap.parse_args()

    out = pathlib.Path(a.out)
    out.mkdir(exist_ok=True)

    jobs = []   # (label, model, q)
    if a.pose_file:
        q, hl, hb, scale = pose_from_json(a.pose_file)
        m = HandModel.from_yaml(hand_length=hl, hand_breadth=hb, scale=scale)
        jobs.append((pathlib.Path(a.pose_file).stem, m, q))
    else:
        names = list(POSES) if a.all_poses else [
            next(n for n in POSES if n.lower() == a.pose.lower())]
        subs = ["f05", "f50", "f95", "m05", "m50", "m95"] if a.all_subjects else [a.subject]
        for s in subs:
            m = HandModel.from_yaml(subject=s)
            for n in names:
                jobs.append((f"hand_{s}_{n.replace(' ', '_')}", m, POSES[n]))

    for label, m, q in jobs:
        print(f"{label}  (HL {m.hand_length*1000:.0f} mm, HB {m.hand_breadth*1000:.0f} mm)")
        parts = build(m, q)
        write(parts, out / label, not a.no_stl)
        if a.clearance > 0:
            c = build(m, q, clearance=a.clearance)
            write(c, out / f"{label}_clear{a.clearance:g}", False)
        fp = out / f"{label}_frames.json"
        fp.write_text(json.dumps(frames(m, q), indent=1))
        print("  ", fp.name)


if __name__ == "__main__":
    main()
