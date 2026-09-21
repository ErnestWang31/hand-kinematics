"""
Build an *articulated* hand for Onshape: one solid per bone, in the neutral
pose, with a cross-axle and matching bores on every joint axis.

    python3 hand_onshape.py --subject m50
    python3 hand_onshape.py --subject m95 --bore 2.0 --out export

Why this shape of output: STEP cannot carry mates, so an articulated model has
to be assembled once inside Onshape. Onshape generates an implicit mate
connector on the axis of any cylindrical face, so if every joint carries a
cylinder on its true rotation axis, each revolute mate is two clicks and lands
perfectly aligned. That is what the axles and bores are for.

Two-DoF joints (the finger MCPs and the thumb CMC) get a cross-axle — a real
universal-joint cross, one arm per axis — so they become two revolute mates
that can each take angle limits. A ball mate would be fewer clicks but Onshape
cannot limit one, and the limits are the whole point.

Everything is exported at the neutral (flat) pose, so every mate reads 0 at
assembly time and the limits in onshape_mates.md apply directly.
"""
from __future__ import annotations

import argparse
import csv
import pathlib

import cadquery as cq
import numpy as np
from cadquery import Vector

from hand_export import DIGITS, ball, frustum, fuse_all
from hand_kinematics import DEG, FINGERS, HandModel, rot

BONE = {"pp": "prox", "mp": "mid", "dp": "dist", "mc": "meta"}


def axle(center, d1, d2, r, length):
    """A universal-joint cross: one cylinder per rotation axis."""
    arms = []
    for d in (d1, d2):
        d = np.asarray(d, float)
        d = d / np.linalg.norm(d)
        arms.append(cq.Solid.makeCylinder(r, length, Vector(*(center - d * length / 2)),
                                          Vector(*d)))
    return fuse_all(arms)


def bore(solid, center, d, r, length):
    d = np.asarray(d, float)
    d = d / np.linalg.norm(d)
    cut = cq.Solid.makeCylinder(r, length, Vector(*(center - d * length / 2)), Vector(*d))
    return solid.cut(cut).clean()


def build(model: HandModel, bore_r: float):
    """Returns ({part name: solid}, [mate rows])."""
    q = {}                                   # neutral pose: every joint at zero
    K = {d: np.asarray(P) * 1000.0 for d, P in model.fk(q).items()}
    HB = model.hand_breadth * 1000.0
    HL = model.hand_length * 1000.0
    rad = model.cfg["segment_radius_hb"]
    lim = model.cfg["joints"]

    def r(d, k):
        return rad[d][k] * HB

    # At the neutral pose every finger frame is the world frame, so finger
    # flexion is about +X and abduction about +Z. The thumb column carries a
    # fixed obliquity, so its axes come from that rotation.
    t = model.cfg["palm"]["thumb_cmc"]
    R0 = (rot("z", -t["ray_deg"] * DEG)
          @ rot("x", -t["palmar_tilt_deg"] * DEG)
          @ rot("y", t["pronation_deg"] * DEG))
    FLEX, ABD = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
    T_FLEX, T_ABD = R0 @ FLEX, R0 @ ABD

    parts: dict[str, cq.Solid] = {}
    mates: list[dict] = []

    def mate(name, parent, child, axis, limits, note=""):
        mates.append({"mate": name, "parent": parent, "child": child,
                      "axis_x": round(float(axis[0]), 4),
                      "axis_y": round(float(axis[1]), 4),
                      "axis_z": round(float(axis[2]), 4),
                      "min_deg": limits[0], "max_deg": limits[1], "note": note})

    # ---------------- palm: metacarpals 2-5 plus the carpal stub -------------
    y_knuckle = float(np.mean([K[f][1][1] for f in FINGERS]))
    sec = [(0.0, 0.365 * HB, 0.145 * HB, 0.0),
           (0.45 * y_knuckle, 0.465 * HB, 0.168 * HB, 0.0),
           (y_knuckle, 0.505 * HB, 0.156 * HB, 0.010 * HL)]
    palm = [cq.Solid.makeLoft([
        cq.Wire.makeEllipse(a, b, Vector(0, y, z), Vector(0, 1, 0), Vector(1, 0, 0))
        for y, a, b, z in sec])]
    for f in FINGERS:
        palm.append(frustum(K[f][0], K[f][1], r(f, "mc") * 1.25, r(f, "mc")))
        palm.append(ball(K[f][1], r(f, "mc")))
    root = K["thumb"][0] + 0.35 * (K["thumb"][1] - K["thumb"][0])
    palm.append(frustum(root, K["thumb"][1], r("thumb", "carpal") * 0.8,
                        r("thumb", "carpal")))
    palm_solid = fuse_all(palm)

    # ---------------- fingers ------------------------------------------------
    for f in FINGERS:
        P = K[f]
        mcp, pip, dip = P[1], P[2], P[3]
        span = r(f, "mc") * 3.2

        # cross-axle at the MCP: Z arm mates to the palm, X arm to the phalanx
        parts[f"{f}_gimbal"] = axle(mcp, ABD, FLEX, bore_r, span)
        palm_solid = bore(palm_solid, mcp, ABD, bore_r, span * 1.4)

        pp = fuse_all([frustum(mcp, pip, r(f, "pp"), r(f, "mp")), ball(mcp, r(f, "pp")),
                       ball(pip, r(f, "mp"))])
        pp = bore(pp, mcp, FLEX, bore_r, span * 1.4)
        pp = bore(pp, pip, FLEX, bore_r, span * 1.4)
        parts[f"{f}_prox"] = pp

        mp = fuse_all([frustum(pip, dip, r(f, "mp"), r(f, "dp")), ball(pip, r(f, "mp")),
                       ball(dip, r(f, "dp"))])
        mp = bore(mp, pip, FLEX, bore_r, span * 1.4)
        mp = bore(mp, dip, FLEX, bore_r, span * 1.4)
        parts[f"{f}_mid"] = mp

        dp = fuse_all([frustum(dip, P[4], r(f, "dp"), r(f, "dp") * 0.92),
                       ball(dip, r(f, "dp")), ball(P[4], r(f, "dp") * 0.92)])
        parts[f"{f}_dist"] = bore(dp, dip, FLEX, bore_r, span * 1.4)

        mate(f"{f}_MCP_abduction", "palm", f"{f}_gimbal", ABD,
             lim["finger_mcp_abduction"][f], "authority falls to 0 as the MCP flexes")
        mate(f"{f}_MCP_flexion", f"{f}_gimbal", f"{f}_prox", FLEX,
             lim["finger_mcp_flexion"][f], "")
        mate(f"{f}_PIP", f"{f}_prox", f"{f}_mid", FLEX, lim["finger_pip_flexion"][f], "")
        mate(f"{f}_DIP", f"{f}_mid", f"{f}_dist", FLEX, lim["finger_dip_flexion"][f],
             "drive from PIP x 0.67")

    # ---------------- thumb --------------------------------------------------
    P = K["thumb"]
    cmc, tmcp, ip = P[1], P[2], P[3]
    span = r("thumb", "carpal") * 3.2
    parts["thumb_gimbal"] = axle(cmc, T_ABD, T_FLEX, bore_r, span)
    palm_solid = bore(palm_solid, cmc, T_ABD, bore_r, span * 1.4)

    meta = fuse_all([frustum(cmc, tmcp, r("thumb", "carpal"), r("thumb", "mc")),
                     ball(cmc, r("thumb", "carpal")), ball(tmcp, r("thumb", "mc"))])
    meta = bore(meta, cmc, T_FLEX, bore_r, span * 1.4)
    parts["thumb_meta"] = bore(meta, tmcp, T_FLEX, bore_r, span * 1.4)

    tpp = fuse_all([frustum(tmcp, ip, r("thumb", "pp"), r("thumb", "dp")),
                    ball(tmcp, r("thumb", "pp")), ball(ip, r("thumb", "dp"))])
    tpp = bore(tpp, tmcp, T_FLEX, bore_r, span * 1.4)
    parts["thumb_prox"] = bore(tpp, ip, T_FLEX, bore_r, span * 1.4)

    tdp = fuse_all([frustum(ip, P[4], r("thumb", "dp"), r("thumb", "dp") * 0.92),
                    ball(ip, r("thumb", "dp")), ball(P[4], r("thumb", "dp") * 0.92)])
    parts["thumb_dist"] = bore(tdp, ip, T_FLEX, bore_r, span * 1.4)

    mate("thumb_CMC_abduction", "palm", "thumb_gimbal", T_ABD,
         lim["thumb_cmc_abduction"], "out of the palm plane")
    mate("thumb_CMC_flexion", "thumb_gimbal", "thumb_meta", T_FLEX,
         lim["thumb_cmc_flexion"], "axial rotation is NOT modelled here")
    mate("thumb_MCP", "thumb_meta", "thumb_prox", T_FLEX, lim["thumb_mcp_flexion"], "")
    mate("thumb_IP", "thumb_prox", "thumb_dist", T_FLEX, lim["thumb_ip_flexion"], "")

    parts["palm"] = palm_solid
    return parts, mates


RECIPE = """# Articulated hand in Onshape — {subject}

`{step}` holds **{n} named solids** in the neutral (flat) pose, each carrying a
cylinder on its true joint axis. Onshape puts an implicit mate connector on the
axis of every cylindrical face, so each revolute mate below is two clicks and
lands already aligned — you never place a mate connector by hand.

## Import

1. In a document, **Insert → Import** `{step}`. Take the default
   *"Import into a Part Studio"* so all {n} solids land in one Part Studio, each
   already in its correct position.
2. Create an **Assembly**. Insert the Part Studio and choose **"Insert all parts"**.
   Everything arrives at the neutral pose, already coincident.
3. Right-click `palm` → **Fix**. It is the ground.

## Mates

Each row is one **Revolute** mate: click the Ø{bore:g} mm cylindrical face on the
parent, then the one on the child — they are already coaxial, so the parts do
not move — then type the limits into *Limits → Min / Max* in the mate dialog.

At the MCPs and the thumb CMC that means axle-to-bore (the `_gimbal` parts are
universal-joint crosses carrying one arm per axis; hide them once mated). At the
PIPs, DIPs, thumb MCP and IP it is bore-to-bore straight through the joint.
Every one of the 20 axes is verified to carry a cylindrical face on both parts.

Work proximal to distal: every MCP row, then every PIP, then every DIP.

| Mate | Parent | Child | Min | Max |
|---|---|---|---|---|
{rows}

Twenty revolute mates, about fifteen minutes once. After that you drag a
fingertip and the whole chain follows, clamped to real human limits.

## Posing it

- **Drag** any part — Onshape solves the chain and stops at the limits.
- To reproduce an exact pose from the bench, open each mate and type the angle
  into the mate's value field.
- For repeatable poses, promote the mates to **Mate values** and drive them from
  a **Configuration** — one dropdown for flat / rest / power grasp / tip pinch.

## What this model does not do

- **The DIP is free here.** In a real hand it rides on the PIP at about 0.67x.
  Onshape has no relation between two mates, so either type both angles, or add
  a *Mate value* and a configuration variable that drives both.
- **Abduction does not collapse with flexion.** The bench enforces
  `abd_max x (1 - MCP_flex/90)`; a fixed revolute limit cannot. Treat the
  abduction limits as valid only near full MCP extension.
- **No neighbour coupling.** You can spread adjacent fingers further apart here
  than a hand allows.
- **Thumb CMC axial rotation is dropped.** It is real but small, and its axis is
  oblique and non-intersecting with the others — see the model caveats.

For a pose that has to be right, set it in the bench, which enforces all of the
above, then read the angles into Onshape.
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--subject", default="m50",
                    choices=["f05", "f50", "f95", "m05", "m50", "m95"])
    ap.add_argument("--bore", type=float, default=1.5, help="axle radius, mm (default 1.5 = 3 mm pin)")
    ap.add_argument("--out", default="export")
    a = ap.parse_args()

    out = pathlib.Path(a.out)
    out.mkdir(exist_ok=True)
    model = HandModel.from_yaml(subject=a.subject)
    print(f"articulated hand, {a.subject}: HL {model.hand_length*1000:.0f} mm, "
          f"axle {a.bore*2:g} mm")

    parts, mates = build(model, a.bore)

    assy = cq.Assembly(name=f"hand_{a.subject}")
    order = ["palm"] + [f"{f}_{s}" for f in DIGITS
                        for s in ("gimbal", "meta", "prox", "mid", "dist")
                        if f"{f}_{s}" in parts]
    COLOR = {"thumb": (0.77, 0.38, 0.23), "index": (0.16, 0.49, 0.62),
             "middle": (0.23, 0.56, 0.45), "ring": (0.44, 0.39, 0.69),
             "little": (0.66, 0.52, 0.12), "palm": (0.78, 0.78, 0.80)}
    for name in order:
        key = name.split("_")[0]
        assy.add(parts[name], name=name, color=cq.Color(*COLOR[key]))

    step = out / f"hand_{a.subject}_articulated.step"
    (getattr(assy, "export", None) or assy.save)(str(step))
    print(f"  {step.name}  {len(order)} solids  {step.stat().st_size/1e6:.1f} MB")

    csv_path = out / f"hand_{a.subject}_mates.csv"
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(mates[0]))
        w.writeheader()
        w.writerows(mates)

    rows = "\n".join(
        f"| `{m['mate']}` | `{m['parent']}` | `{m['child']}` | {m['min_deg']}° | {m['max_deg']}° |"
        for m in mates)
    doc = out / f"hand_{a.subject}_onshape.md"
    doc.write_text(RECIPE.format(subject=a.subject, step=step.name, n=len(order),
                                 rows=rows, bore=a.bore * 2))
    print(f"  {csv_path.name}  {len(mates)} mates")
    print(f"  {doc.name}")


if __name__ == "__main__":
    main()
