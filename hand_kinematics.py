"""
Forward kinematics + validity checking for the parametric hand in hand_model.yaml.

    python3 hand_kinematics.py                 # render reference poses
    python3 hand_kinematics.py --workspace     # fingertip reachable cloud
    python3 hand_kinematics.py --subject m95   # pick a size preset

Joint vector layout (26 DoF), all in degrees:
    thumb  : cmc_flex, cmc_abd, cmc_rot, mcp_flex, mcp_abd, ip_flex   (6)
    finger : mcp_flex, mcp_abd, pip_flex, dip_flex                    (4 x 4)
    wrist  : flex, deviation, pronosup                                (3)  [optional]
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field

import numpy as np
import yaml

FINGERS = ["index", "middle", "ring", "little"]
DEG = np.pi / 180.0


# --------------------------------------------------------------------------- #
# small SE(3) helpers
# --------------------------------------------------------------------------- #
def rot(axis: str, a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def se3(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


# --------------------------------------------------------------------------- #
# model
# --------------------------------------------------------------------------- #
@dataclass
class HandModel:
    cfg: dict
    hand_length: float
    hand_breadth: float
    mcp_origin: dict = field(default_factory=dict)
    seg: dict = field(default_factory=dict)
    # per-digit length multiplier, for subjects whose proportions differ from the mean
    scale: dict = field(default_factory=lambda: {d: 1.0 for d in ["thumb"] + FINGERS})

    @classmethod
    def from_yaml(cls, path: str = "hand_model.yaml", subject: str = "m50",
                  hand_length: float | None = None, hand_breadth: float | None = None,
                  scale: dict | None = None) -> "HandModel":
        cfg = yaml.safe_load(open(path))
        s = cfg["subjects"][subject]
        m = cls(cfg=cfg,
                hand_length=hand_length if hand_length is not None else s["hand_length"],
                hand_breadth=hand_breadth if hand_breadth is not None else s["hand_breadth"])
        if scale:
            m.scale.update(scale)
        m._build()
        return m

    def _build(self) -> None:
        HL, HB = self.hand_length, self.hand_breadth
        self.seg = {
            name: {k: v * HL * self.scale.get(name, 1.0) for k, v in d.items()}
            for name, d in self.cfg["segments_hl"].items()
        }
        # MCP centres: radial distance = metacarpal link, lateral offset from HB.
        # Solving for the distal component reproduces the transverse arch
        # instead of laying the knuckles out on a straight line.
        off = self.cfg["palm"]["mcp_lateral_offset_hb"]
        for f in FINGERS:
            x = off[f] * HB
            r = self.seg[f]["mc"]
            y = np.sqrt(max(r * r - x * x, 1e-9))
            # knuckles sit slightly dorsal of the wrist-crease midpoint
            self.mcp_origin[f] = np.array([x, y, 0.010 * HL])

        t = self.cfg["palm"]["thumb_cmc"]
        self.mcp_origin["thumb"] = np.array(
            [t["x_hb"] * HB, t["y_hl"] * HL, t["z_hl"] * HL]
        )

    # ----------------------------------------------------------------- FK ---
    def fk(self, q: dict) -> dict:
        """q maps joint names -> degrees. Returns {digit: (5,3) joint positions}."""
        out = {}
        for f in FINGERS:
            out[f] = self._fk_finger(f, q)
        out["thumb"] = self._fk_thumb(q)
        return out

    def _fk_finger(self, f: str, q: dict) -> np.ndarray:
        s = self.seg[f]
        mcp_f = q.get(f"{f}_mcp_flex", 0.0) * DEG
        mcp_a = q.get(f"{f}_mcp_abd", 0.0) * DEG
        pip = q.get(f"{f}_pip_flex", 0.0) * DEG
        dip = q.get(f"{f}_dip_flex", 0.0) * DEG

        pts = [np.zeros(3), self.mcp_origin[f]]
        # abduction about the dorsal axis, then flexion about the radial axis
        R = rot("z", -mcp_a) @ rot("x", -mcp_f)
        T = se3(R, self.mcp_origin[f])
        for L, ang in ((s["pp"], None), (s["mp"], pip), (s["dp"], dip)):
            if ang is not None:
                T = T @ se3(rot("x", -ang), np.zeros(3))
            T = T @ se3(np.eye(3), np.array([0.0, L, 0.0]))
            pts.append(T[:3, 3].copy())
        return np.array(pts)

    def _fk_thumb(self, q: dict) -> np.ndarray:
        s, t = self.seg["thumb"], self.cfg["palm"]["thumb_cmc"]
        base = self.mcp_origin["thumb"]
        # Fixed obliquity of the thumb column: pronation about the ray, then
        # tilt of the whole plane out of the palm.
        R0 = (
            rot("z", -t["ray_deg"] * DEG)
            @ rot("x", -t["palmar_tilt_deg"] * DEG)
            @ rot("y", t["pronation_deg"] * DEG)
        )
        R = (
            R0
            @ rot("z", -q.get("thumb_cmc_abd", 0.0) * DEG)
            @ rot("x", -q.get("thumb_cmc_flex", 0.0) * DEG)
            @ rot("y", q.get("thumb_cmc_rot", 0.0) * DEG)
        )
        T = se3(R, base)
        pts = [np.zeros(3), base]
        T = T @ se3(np.eye(3), np.array([0.0, s["mc"], 0.0]))
        pts.append(T[:3, 3].copy())
        T = T @ se3(
            rot("z", -q.get("thumb_mcp_abd", 0.0) * DEG)
            @ rot("x", -q.get("thumb_mcp_flex", 0.0) * DEG),
            np.zeros(3),
        )
        T = T @ se3(np.eye(3), np.array([0.0, s["pp"], 0.0]))
        pts.append(T[:3, 3].copy())
        T = T @ se3(rot("x", -q.get("thumb_ip_flex", 0.0) * DEG), np.zeros(3))
        T = T @ se3(np.eye(3), np.array([0.0, s["dp"], 0.0]))
        pts.append(T[:3, 3].copy())
        return np.array(pts)

    # -------------------------------------------------------- constraints ---
    def violations(self, q: dict) -> list[str]:
        """Everything about q that a real hand could not do."""
        j, c = self.cfg["joints"], self.cfg["coupling"]
        bad = []

        def lim(table, key, name):
            lo, hi = j[table][key]
            v = q.get(name, 0.0)
            if not lo - 1e-6 <= v <= hi + 1e-6:
                bad.append(f"{name}={v:.1f} outside [{lo}, {hi}]")

        for f in FINGERS:
            lim("finger_mcp_flexion", f, f"{f}_mcp_flex")
            lim("finger_mcp_abduction", f, f"{f}_mcp_abd")
            lim("finger_pip_flexion", f, f"{f}_pip_flex")
            lim("finger_dip_flexion", f, f"{f}_dip_flex")

            # abduction authority shrinks as the MCP flexes
            mcpf = max(q.get(f"{f}_mcp_flex", 0.0), 0.0)
            abd_max = j["finger_mcp_abduction"][f][1] * max(0.0, 1.0 - mcpf / 90.0)
            if abs(q.get(f"{f}_mcp_abd", 0.0)) > abd_max + 2.0:
                bad.append(
                    f"{f}: |abd|={abs(q.get(f'{f}_mcp_abd', 0.0)):.1f} exceeds "
                    f"{abd_max:.1f} available at {mcpf:.0f} deg MCP flexion"
                )

            # DIP rides on the PIP
            r = c["dip_from_pip"]
            exp = r["ratio"] * q.get(f"{f}_pip_flex", 0.0)
            if abs(q.get(f"{f}_dip_flex", 0.0) - exp) > r["tolerance_deg"]:
                bad.append(
                    f"{f}: DIP {q.get(f'{f}_dip_flex', 0.0):.0f} incompatible with "
                    f"PIP {q.get(f'{f}_pip_flex', 0.0):.0f} (expect ~{exp:.0f})"
                )

        for pair, mx in c["neighbour_mcp_flexion_max_diff_deg"].items():
            a, b = pair.split("_")
            d = abs(q.get(f"{a}_mcp_flex", 0.0) - q.get(f"{b}_mcp_flex", 0.0))
            if d > mx:
                bad.append(f"{a}/{b} MCP flexion differ by {d:.0f} > {mx}")

        for name, table in (
            ("thumb_cmc_flex", "thumb_cmc_flexion"),
            ("thumb_cmc_abd", "thumb_cmc_abduction"),
            ("thumb_cmc_rot", "thumb_cmc_rotation"),
            ("thumb_mcp_flex", "thumb_mcp_flexion"),
            ("thumb_mcp_abd", "thumb_mcp_abduction"),
            ("thumb_ip_flex", "thumb_ip_flexion"),
        ):
            lo, hi = j[table]
            v = q.get(name, 0.0)
            if not lo - 1e-6 <= v <= hi + 1e-6:
                bad.append(f"{name}={v:.1f} outside [{lo}, {hi}]")
        return bad

    def clamp(self, q: dict) -> dict:
        """Nearest pose the hand can actually reach. Use this on raw glove output."""
        j = self.cfg["joints"]
        out = dict(q)
        for f in FINGERS:
            for table, name in (
                ("finger_mcp_flexion", f"{f}_mcp_flex"),
                ("finger_pip_flexion", f"{f}_pip_flex"),
                ("finger_dip_flexion", f"{f}_dip_flex"),
            ):
                lo, hi = j[table][f]
                out[name] = float(np.clip(out.get(name, 0.0), lo, hi))
            mcpf = max(out[f"{f}_mcp_flex"], 0.0)
            a = j["finger_mcp_abduction"][f][1] * max(0.0, 1.0 - mcpf / 90.0)
            out[f"{f}_mcp_abd"] = float(np.clip(out.get(f"{f}_mcp_abd", 0.0), -a, a))
        for name, table in (
            ("thumb_cmc_flex", "thumb_cmc_flexion"),
            ("thumb_cmc_abd", "thumb_cmc_abduction"),
            ("thumb_cmc_rot", "thumb_cmc_rotation"),
            ("thumb_mcp_flex", "thumb_mcp_flexion"),
            ("thumb_mcp_abd", "thumb_mcp_abduction"),
            ("thumb_ip_flex", "thumb_ip_flexion"),
        ):
            lo, hi = j[table]
            out[name] = float(np.clip(out.get(name, 0.0), lo, hi))
        return out


# --------------------------------------------------------------------------- #
# poses
# --------------------------------------------------------------------------- #
def pose(mcp=0.0, pip=0.0, dip=None, abd=0.0, t_flex=0.0, t_abd=0.0,
         t_rot=0.0, t_mcp=0.0, t_ip=0.0) -> dict:
    dip = 0.67 * pip if dip is None else dip
    q = {}
    for i, f in enumerate(FINGERS):
        q[f"{f}_mcp_flex"] = mcp
        q[f"{f}_mcp_abd"] = abd * (1.5 - i)      # fan out radially
        q[f"{f}_pip_flex"] = pip
        q[f"{f}_dip_flex"] = dip
    q.update(thumb_cmc_flex=t_flex, thumb_cmc_abd=t_abd, thumb_cmc_rot=t_rot,
             thumb_mcp_flex=t_mcp, thumb_ip_flex=t_ip)
    return q


POSES = {
    "flat": pose(),
    "spread": pose(abd=12),
    "rest": pose(mcp=25, pip=45, t_abd=30, t_mcp=10, t_ip=15),
    "power grasp": pose(mcp=60, pip=75, dip=45, t_abd=25, t_flex=15, t_mcp=40, t_ip=30),
    "tip pinch": pose(mcp=40, pip=60, dip=40, t_flex=-5, t_abd=45, t_mcp=30, t_ip=30),
    "full fist": pose(mcp=85, pip=105, dip=70, t_abd=10, t_flex=25, t_mcp=50, t_ip=60),
}


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #
def draw(model: HandModel, out: str = "hand_poses.png") -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"thumb": "#c8553d", "index": "#2f6690", "middle": "#3a7d44",
              "ring": "#8367c7", "little": "#d1a13a"}
    fig = plt.figure(figsize=(15, 9))
    for i, (name, q) in enumerate(POSES.items(), 1):
        ax = fig.add_subplot(2, 3, i, projection="3d")
        pts = model.fk(q)
        for d, P in pts.items():
            ax.plot(P[:, 0] * 100, P[:, 1] * 100, P[:, 2] * 100, "-o",
                    color=colors[d], lw=2.2, ms=3.5)
        bad = model.violations(q)
        ax.set_title(f"{name}" + ("" if not bad else f"  ({len(bad)} violations)"),
                     fontsize=11)
        ax.set_xlim(-8, 10); ax.set_ylim(-1, 20); ax.set_zlim(-9, 9)
        ax.set_box_aspect((18, 21, 18))
        ax.set_xlabel("x radial (cm)", fontsize=7)
        ax.set_ylabel("y distal (cm)", fontsize=7)
        ax.set_zlabel("z dorsal (cm)", fontsize=7)
        ax.tick_params(labelsize=6)
        ax.view_init(elev=22, azim=-72)
    fig.suptitle(
        f"Parametric hand — hand length {model.hand_length*100:.1f} cm, "
        f"breadth {model.hand_breadth*100:.1f} cm", fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    return out


def workspace(model: HandModel, n: int = 4000, out: str = "hand_workspace.png") -> str:
    """Fingertip clouds under the full constrained joint space."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rng = np.random.default_rng(0)
    j = model.cfg["joints"]
    clouds = {d: [] for d in FINGERS + ["thumb"]}
    for _ in range(n):
        q = {}
        for f in FINGERS:
            mcp = rng.uniform(*j["finger_mcp_flexion"][f])
            pip = rng.uniform(*j["finger_pip_flexion"][f])
            q[f"{f}_mcp_flex"] = mcp
            q[f"{f}_pip_flex"] = pip
            q[f"{f}_dip_flex"] = np.clip(0.67 * pip + rng.uniform(-12, 12),
                                         *j["finger_dip_flexion"][f])
            a = j["finger_mcp_abduction"][f][1] * max(0.0, 1 - max(mcp, 0) / 90)
            q[f"{f}_mcp_abd"] = rng.uniform(-a, a)
        q.update(
            thumb_cmc_flex=rng.uniform(*j["thumb_cmc_flexion"]),
            thumb_cmc_abd=rng.uniform(*j["thumb_cmc_abduction"]),
            thumb_cmc_rot=rng.uniform(*j["thumb_cmc_rotation"]),
            thumb_mcp_flex=rng.uniform(*j["thumb_mcp_flexion"]),
            thumb_ip_flex=rng.uniform(*j["thumb_ip_flexion"]),
        )
        for d, P in model.fk(q).items():
            clouds[d].append(P[-1])

    colors = {"thumb": "#c8553d", "index": "#2f6690", "middle": "#3a7d44",
              "ring": "#8367c7", "little": "#d1a13a"}
    fig = plt.figure(figsize=(13, 5.5))
    for k, (elev, azim, label) in enumerate(
        [(90, -90, "dorsal view"), (0, -90, "radial view"), (20, -60, "oblique")], 1
    ):
        ax = fig.add_subplot(1, 3, k, projection="3d")
        for d, C in clouds.items():
            C = np.array(C) * 100
            ax.scatter(C[:, 0], C[:, 1], C[:, 2], s=1.2, alpha=0.18, color=colors[d],
                       label=d if k == 1 else None)
        ax.set_xlim(-10, 12); ax.set_ylim(-2, 20); ax.set_zlim(-12, 10)
        ax.set_box_aspect((22, 22, 22))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(label, fontsize=10)
        ax.tick_params(labelsize=6)
        ax.set_xlabel("x (cm)", fontsize=7); ax.set_ylabel("y (cm)", fontsize=7)
        ax.set_zlabel("z (cm)", fontsize=7)
        if k == 1:
            ax.legend(fontsize=7, markerscale=6, loc="upper left")
    fig.suptitle(f"Fingertip workspace, {n} constrained samples", fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--subject", default="m50",
                    choices=["f05", "f50", "f95", "m05", "m50", "m95"])
    ap.add_argument("--workspace", action="store_true")
    a = ap.parse_args()

    model = HandModel.from_yaml(subject=a.subject)
    print(f"subject {a.subject}: HL={model.hand_length*1000:.0f} mm  "
          f"HB={model.hand_breadth*1000:.0f} mm")
    for d in ["thumb"] + FINGERS:
        s = model.seg[d]
        print("  " + d.ljust(7)
              + "  ".join(f"{k}={v*1000:5.1f}" for k, v in s.items()))

    print("\npose checks:")
    for name, q in POSES.items():
        tip = model.fk(q)["index"][-1]
        bad = model.violations(q)
        print(f"  {name:<12} index tip ({tip[0]*100:6.2f},{tip[1]*100:6.2f},"
              f"{tip[2]*100:6.2f}) cm   {'ok' if not bad else bad[0]}")

    tp = model.fk(POSES["tip pinch"])
    gap = np.linalg.norm(tp["thumb"][-1] - tp["index"][-1]) * 1000
    print(f"\ntip-pinch closure (thumb tip to index tip): {gap:.1f} mm")

    print("wrote", draw(model))
    if a.workspace:
        print("wrote", workspace(model))


if __name__ == "__main__":
    main()
