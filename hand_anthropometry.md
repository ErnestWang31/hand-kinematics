# Hand sizing and range of motion — reference for a mocap / UMI glove

Everything below is aimed at two decisions: **what hand sizes the glove has to
fit**, and **what joint angles the model is allowed to produce**. Numbers are
adult, healthy, active-duty-ish populations; sources at the bottom.

---

## 1. The primary dataset

The best public source for hand kinematics is **Greiner (1991), *Hand
Anthropometry of U.S. Army Personnel*, NATICK/TR-92/011** — 1304 women and 1003
men, 86 dimensions, and crucially it reports **link lengths** (joint-centre to
joint-centre) rather than only bone or surface lengths. That is exactly what a
kinematic chain needs. I pulled the full scan and extracted the table into
[data/greiner_hand_1988.csv](data/greiner_hand_1988.csv).

ANSUR II (2012, n=6068) is newer but only carries hand length / breadth /
circumference / palm length — no per-segment breakdown. Use it as a sanity
check on the gross dimensions, not as a source of link lengths.

### Gross hand size (cm)

| Dimension | F mean ± SD | F 5th–95th | M mean ± SD | M 5th–95th |
|---|---|---|---|---|
| Hand length (wrist crease → D3 tip) | 18.07 ± 0.98 | 16.5 – 19.7 | 19.41 ± 0.99 | 17.8 – 21.0 |
| Hand breadth (across MCP heads) | 7.95 ± 0.38 | 7.3 – 8.6 | 9.04 ± 0.42 | 8.4 – 9.7 |
| Hand circumference (at MCP) | 18.65 ± 0.86 | 17.2 – 20.1 | 21.33 ± 0.98 | 19.7 – 22.9 |
| Palm length | 10.09 ± 0.57 | 9.2 – 11.0 | 11.05 | — |
| Wrist circumference | 15.14 ± 0.69 | 14.0 – 16.3 | 17.43 ± 0.82 | 16.2 – 18.8 |
| Wrist → thumb tip | 11.77 ± 0.68 | — | 12.45 ± 0.68 | — |
| Stature | 163.0 ± 6.4 | — | 175.7 ± 6.7 | — |

**The design span you must cover: hand length 165–210 mm** (5th %ile female to
95th %ile male), i.e. a 1.28× ratio end to end.

### The finding that matters most for a glove

Length and girth do **not** scale together across sex:

| | F | M | M/F |
|---|---|---|---|
| Hand length | 18.07 | 19.41 | **1.074** |
| Hand breadth | 7.95 | 9.04 | **1.137** |
| Hand circumference | 18.65 | 21.33 | **1.144** |

A male hand is 7% longer but 14% thicker. Breadth/length is 0.440 in women and
0.466 in men. So **a glove sized on length alone will be loose across the palm
on women and tight on men**, and that translates directly into sensor slip and
a calibration drift you'll chase forever. Size the shell on **circumference**
(the way the glove industry already does — glove size ≈ hand circumference in
inches) and scale the *kinematic model* on **length**. They are two different
parameters and should be decoupled in your fitting code.

### Link lengths, as fractions of hand length

Segment ratios are remarkably stable across sex and size, which is what lets
you drive the whole skeleton from one scalar. MC = wrist → MCP, PP = MCP → PIP,
MP = PIP → DIP, DP = DIP → fingertip (includes the pulp, which is what a glove
or a marker actually sees).

| Digit | MC | PP | MP | DP | Σ (= tip-to-wrist) |
|---|---|---|---|---|---|
| Thumb | 0.232 | 0.165 (MCP→IP) | 0.140 | — | 0.537 |
| Index | 0.435 | 0.247 | 0.117 | 0.141 | 0.940 |
| Middle | 0.432 | 0.275 | 0.139 | 0.141 | 0.987 |
| Ring | 0.396 | 0.268 | 0.126 | 0.144 | 0.934 |
| Little | 0.376 | 0.209 | 0.090 | 0.131 | 0.806 |

**Validation:** for digits 3, 4 and 5 the sum of Greiner's four link lengths
reproduces his independently measured "tip to wrist crease" distance to within
0.3 mm, in both sexes. That is a strong internal consistency check and it is
why I trust these ratios.

**Two corrections I had to make to the raw report — worth knowing about:**

1. **Digit 2 proximal phalanx.** Greiner measures the index MCP centre by the
   *proximal transverse palm crease*, which sits proximal to the actual joint,
   inflating D2 PP to 5.65 cm (longer than the middle finger's 4.97 cm, which
   is anatomically impossible) and overshooting the tip-to-wrist total by
   2.8 mm. I moved the MC/PP boundary distally so that PP(index) = 0.90 ×
   PP(middle), which matches the osteometric ratio, and kept the total fixed.
2. **The thumb decomposition is unusable as published.** Greiner gives D1
   proximal phalanx link = 1.92 cm (F) / 2.11 cm (M), against a real thumb
   proximal phalanx of ~31 mm, and MC + PP + DP overshoots the D1 link length
   by 1.5 cm. His "D1 metacarpal link" runs from the thenar crease and swallows
   most of the proximal phalanx. **Do not use rows `d1_proximal_phalanx_link`
   or `d1_metacarpal_link` for a kinematic chain.**

   The thumb here is anatomical instead — 1st metacarpal 45 mm, proximal
   phalanx 32 mm, distal phalanx plus pulp 27 mm at a 194 mm hand — with the
   trapezium at 0.24 HB radial and 0.12 HL distal. Those five numbers are
   calibrated against the one thumb dimension of Greiner's that *is* reliable,
   the overall tip-to-wrist-crease distance:

   | | model | Greiner | error |
   |---|---|---|---|
   | wrist → thumb tip, 50th %ile female | 125.5 mm | 125.7 mm | −0.2% |
   | wrist → thumb tip, 50th %ile male | 135.5 mm | 137.9 mm | −1.8% |

   For comparison the fingers land at −0.2% to −5.2% on the same measure, so
   the thumb now sits inside the same band rather than outside it. An earlier
   version of this model carried Greiner's segment values through and came out
   **+10.9% — a visibly over-long thumb**, which also inflated every thumb
   figure in section 5.

---

## 2. Range of motion

Two different numbers get quoted and they are not interchangeable. **AAOS
clinical norms** are conservative pass/fail thresholds used to judge
impairment. **Functional maxima** are what a healthy hand actually reaches. For
a simulator you want the functional limits as hard bounds, and the AAOS values
as a plausibility band.

### Fingers (digits 2–5), degrees

| Joint | AAOS norm | Functional range used in the model | Notes |
|---|---|---|---|
| MCP flexion | 0–90 | −30 … +90 (little −35 … +95) | negative = hyperextension, which is real and large |
| MCP abduction | — | ±20 index, ±15 middle, ±20 ring, ±30 little | **only at full MCP extension** — see coupling |
| PIP flexion | 0–100 | −5 … +110 | |
| DIP flexion | 0–70 | −10 … +80 (little −15 … +85) | hyperextension common, esp. ulnar digits |

Ulnar digits are more mobile than radial ones in essentially every axis. If
your glove uses one limit set for all four fingers it will clip the little
finger and over-permit the index.

### Thumb, degrees

| Joint | Range | Notes |
|---|---|---|
| CMC (trapeziometacarpal) flexion/extension | −25 … +30 (≈53° total arc) | in the plane of the palm |
| CMC abduction/adduction | −5 … +60 | palmar abduction, perpendicular to the palm |
| CMC axial rotation | ±20 (≈17° reported) | small, and **essential** — opposition is impossible without it |
| MCP flexion | −10 … +60 | huge inter-subject variance; some people barely move it, some reach 90 |
| MCP abduction | ±10 | |
| IP flexion | −25 … +85 | hyperextension to −20/−25 is normal |

The CMC flexion/extension and abduction/adduction axes are **non-intersecting
and both oblique** to the saddle surface — the flexion axis sits in the
trapezium, the abduction axis in the metacarpal. A clean 2-DoF universal joint
at a single point is an approximation, and it is the main reason naive thumb
retargets look wrong. If your glove's thumb output is the part that never quite
matches, this is why.

### Coupling constraints — the part most models skip

A free 26-DoF model reaches poses no hand can make. These are the constraints
worth enforcing before you trust glove output:

1. **DIP is not independent.** `DIP ≈ 0.67 × PIP` (terminal extensor / FDP
   coupling), tolerance ~±15°. Most gloves don't sense the DIP at all and
   infer it from this — which is fine, as long as you know that's what's
   happening and don't treat it as measured data.
2. **Abduction authority collapses with MCP flexion.** The collateral ligaments
   tighten over the cam-shaped metacarpal head:
   `abd_max_effective = abd_max × max(0, 1 − MCP_flex/90)`. At 90° MCP flexion
   the finger is locked in the frontal plane. A glove that reports 15° of
   spread on a fully flexed finger is reporting sensor cross-talk, not motion.
3. **Juncturae tendinum** tie neighbouring extensors: adjacent MCP flexion
   angles can't differ by more than ~25–30°, adjacent PIP by ~35–45°.
4. **The ulnar metacarpal arch is mobile.** Ring CMC flexes ~15°, little CMC
   ~25°, cupping the palm around an object. Treating the palm as one rigid body
   is the single biggest source of error when retargeting a flat-palm glove
   model onto a real power grasp.

---

## 3. What this means for the glove

- **Calibrate on hand length, fit on hand circumference.** Two parameters, not
  one. See section 1.
- **Three shell sizes** covers the 5F–95M span at ≤8% error if you bin on
  circumference: ~18.0 / 20.0 / 22.0 cm.
- **Calibration pose should be `rest`, not `flat`.** A flat hand pressed on a
  table is at a joint-limit corner (MCP hyperextension varies by ±15° between
  subjects), so it's a terrible zero reference. The relaxed cascade
  (MCP 25 / PIP 45 / DIP 20) is repeatable. Capture `flat` and `full fist` as
  the two span endpoints instead.
- **Don't trust reported DIP.** Either sense it or derive it — never half of
  each.
- **Log the raw sensor values alongside the joint angles.** Once you apply
  coupling constraints the mapping is no longer invertible, and you will want
  to re-fit later against a better model.

---

## Files

- [hand_model.yaml](hand_model.yaml) — the full parameterised model: segment
  ratios, joint limits, coupling rules, size presets (f05…m95), reference poses.
- [hand_kinematics.py](hand_kinematics.py) — forward kinematics, constraint
  checking (`violations`), projection onto the feasible set (`clamp`),
  pose rendering and workspace sampling.
- [data/greiner_hand_1988.csv](data/greiner_hand_1988.csv) — the extracted
  Greiner table, all 1988 measurements, both sexes.
- [hand_export.py](hand_export.py) — STEP / STL / joint-frame export (section 4).
- [hand_envelope.py](hand_envelope.py) — swept motion envelopes (section 5).
- [hand_onshape.py](hand_onshape.py) — articulated, mate-ready export (section 6).
- [onshape_bridge.py](onshape_bridge.py) — push bench poses into Onshape (section 7).
- [build_site.py](build_site.py) — wraps the bench as a standalone page for hosting.
- [hand_explorer.html](hand_explorer.html) — source of the interactive bench.

```bash
python3 hand_kinematics.py --subject m95 --workspace
```

---

## Sources

- [Greiner, T.M. (1991), *Hand Anthropometry of U.S. Army Personnel*, NATICK/TR-92/011 (DTIC ADA244533)](https://archive.org/details/DTIC_ADA244533) — primary source for all link lengths
- [ANSUR II (2012) Anthropometric Survey of U.S. Army Personnel](https://apps.dtic.mil/sti/tr/pdf/ADA611869.pdf/) — gross hand dimensions, n=6068
- [ANSUR II hand summary statistics](https://www.sota2.com/research/sota/hand-anthropometry-on-ansur-ii-combined-male-and-female)
- [AAOS normal range-of-motion values](https://cdn-links.lww.com/permalink/prsgo/b/prsgo_8_6_2020_04_17_hendriks_gox-d-20-00155r2_sdc1.pdf)
- [Necessary MCP range of motion to maintain hand function](https://www.sciencedirect.com/science/article/pii/S1569186114000333)
- [Sub-millimetre accurate human hand kinematics: from surface to skeleton](https://www.tandfonline.com/doi/full/10.1080/10255842.2018.1425996) — the 26-DoF model this follows
- [Constraint study for a hand exoskeleton: human hand kinematics and dynamics](https://pdfs.semanticscholar.org/5a84/3ade1ba00d075c4a9ae93c331dc7ee2c4b70.pdf) — static / intra-finger / inter-finger constraints
- [Efficient human hand kinematics for manipulation tasks (UPM)](https://oa.upm.es/4040/1/INVE_MEM_2008_58022.pdf)
- [In vivo kinematics of the trapeziometacarpal joint](https://www.jhandsurg.org/article/S0363-5023(14)01587-1/abstract) and [thumb CMC joint overview](https://radsource.us/thumb-carpometacarpal-joint/) — thumb CMC axes and ROM
- [Proportions of hand segments, Int. J. Morphol. 28(3)](https://scielo.conicyt.cl/pdf/ijmorphol/v28n3/art15.pdf)
- [DexCap: scalable and portable mocap data collection for dexterous manipulation](https://dex-cap.github.io/) — comparable glove pipeline

---

## 4. CAD export

[hand_export.py](hand_export.py) turns any pose into **STEP B-rep solids** — real
conical, spherical and planar faces you can select, fillet and offset in CAD,
not a triangulated blob wearing a `.step` extension. It uses OpenCascade via
CadQuery (`pip install cadquery`).

```bash
python3 hand_export.py --subject m50 --pose "power grasp" --clearance 2
python3 hand_export.py --all-poses --all-subjects        # 36 hands, the whole design span
python3 hand_export.py --pose-file pose.json             # a pose from the web bench
```

### What comes out

| File | What it's for |
|---|---|
| `<name>.step` | assembly of **6 named solids** — `palm`, `thumb`, `index`, `middle`, `ring`, `little` |
| `<name>_clear<t>.step` | the same hand inflated by *t* mm — subtract it to get a glove liner cavity, or use it as the keep-out volume for an exoskeleton linkage |
| `<name>.stl` | triangulated mesh, for anything that wants a mesh |
| `<name>_frames.json` | every joint centre **and its flexion axis** |

The six-solid split is deliberate: one palm shell and five finger stalls is the
same decomposition a glove has, so you can hide the palm while you draw a
fingertip, or offset one stall without touching the rest.

Face counts stay low — 47 for the palm, 5–7 per finger stall — because each bone
is a single cone or cylinder with spherical joints, fused. That means CAD sees
analytic faces, so `Offset`, `Shell` and `Fillet` behave.

### Coordinate frame

All exports, millimetres:

- origin at the **midpoint of the distal wrist crease**
- **+X** radial, toward the thumb
- **+Y** distal, down the middle finger
- **+Z** dorsal, out the back of the hand

Right hand — mirror across X for the left.

### Joint frames

`_frames.json` carries each joint's position *and* the unit vector it rotates
about, computed numerically by perturbing the joint and measuring which way the
distal segment swings. For hardware this is the part that matters: to place a
linkage pivot, a strap anchor or a sensor you need the axis, not just the point.

```json
{"digit":"index","joint":"PIP","position":[35.2,98.4,-21.7],
 "flexion_axis":[0.997,0.0,-0.071]}
```

### Workflow with the web bench

Pose the hand in [the bench](https://claude.ai/artifact/16XnQo6CQRTK6Yw7yQm7H4),
hit **Copy pose → CAD**, save the JSON as `pose.json`, then run the command it
shows you. The JSON carries hand length, hand breadth, per-digit scaling and all
26 joint angles, so the solid you get is exactly the hand you were looking at.

### Known limits of the solid model

- The palm is a three-section loft with the metacarpals fused in. It captures the
  transverse arch and the thenar eminence, but it is **not** a scanned palm —
  no thenar/hypothenar crease detail, no soft-tissue bulging under load.
- Total volume lands around 350 cm³ for a 50th-percentile male versus ~380–420 cm³
  measured by water displacement, so the model runs slightly lean. If you are
  sizing a tight glove, add 1–2 mm of clearance to compensate.
- Joints are rigid rotations about fixed axes. Real skin slides and bunches at
  the dorsal MCP by several mm in full flexion — budget for it at the knuckles.

---

## 5. Swept motion envelopes

[hand_envelope.py](hand_envelope.py) produces the volume each digit can occupy
**anywhere in its constrained range of motion**. Drop these into the assembly as
keep-out bodies: a linkage that stays clear of them is clear of the hand in
every pose, rather than only in the poses you remembered to check.

```bash
python3 hand_envelope.py --subject m95                    # size for the largest hand you support
python3 hand_envelope.py --subject m95 --res 1.0 --margin 1.5
```

Needs `scikit-image`, `trimesh` and `fast_simplification` alongside CadQuery.

**Method.** Sample each digit's joint space under the same constraints the bench
enforces — DIP coupled to PIP, abduction authority falling off with MCP flexion
— rasterise every bone as a capsule into an occupancy grid, then surface it.
Unlike the hand solids these are **faceted, not analytic**: a swept volume has
no closed form. `--margin` inflates every capsule so the result is conservative,
containing the true envelope despite voxel and decimation error.

Boolean-unioning hundreds of analytic solids is the obvious approach and it does
not work — OpenCascade runs out of memory well before it finishes. The occupancy
grid handles 12,000 capsules in about 20 seconds.

**Output** (95th-percentile male, 1.5 mm voxels, 1 mm margin):

| Digit | Swept volume |
|---|---|
| thumb | 1790 cm³ |
| middle | 978 cm³ |
| ring | 944 cm³ |
| index | 884 cm³ |
| little | 642 cm³ |
| **union** | **3358 cm³** |

Plus `envelope_<subject>_report.json` with every pairwise overlap, and a
`_union.stl` for the single question "does my frame ever touch the hand at all".

### Two things the numbers say

**The thumb eats the palmar half-space.** Its 5-DoF sweep is 1790 cm³ — nearly
twice any finger, and roughly five times the volume of the hand itself. It
overlaps the index envelope by 401 cm³ and still reaches into the *little*
finger's envelope. Practically: there is almost nowhere on the palmar side to
put fixed structure. Anything that has to be rigid belongs on the dorsum.

**Adjacent fingers sweep through each other.** index/middle share 345 cm³,
middle/ring 366 cm³. Inter-finger space is not available for hardware. Anything
between two fingers has to move with one of them or be soft.

Both of these are the kind of thing you would otherwise discover after printing.

---

## 6. Articulated hand in Onshape

[hand_onshape.py](hand_onshape.py) builds a different export for a different
job: **one solid per bone**, in the neutral pose, with a cylinder on every joint
axis.

```bash
python3 hand_onshape.py --subject m50          # 21 solids, 20 mates
python3 hand_onshape.py --subject m95 --bore 2.0
```

STEP cannot carry mates, so an articulated model has to be assembled once inside
Onshape. The thing that makes that fast: **Onshape puts an implicit mate
connector on the axis of any cylindrical face.** So if every joint carries a
cylinder on its true rotation axis, each revolute mate is two clicks and lands
already aligned — you never place a mate connector by hand.

- **PIP, DIP, thumb MCP and IP** — a Ø3 mm bore straight through the joint in
  both adjoining bones. Bore to bore.
- **The four finger MCPs and the thumb CMC** are 2-DoF, so they get a
  `_gimbal` part: a real universal-joint cross, one arm per axis. Palm →
  (abduction) → gimbal → (flexion) → proximal phalanx. Hide the gimbals once
  mated.

A ball mate at those joints would be fewer clicks, but Onshape cannot put angle
limits on a ball mate, and the limits are the point.

The exporter checks its own work: all 20 axes are verified to carry a
cylindrical face on **both** parts, so every mate is click-ready.

Each run also writes `hand_<subject>_onshape.md` — the import recipe with the
full mate table and limits — and `hand_<subject>_mates.csv` with axis vectors
for scripting.

### What the Onshape model cannot enforce

Assembly mates are independent; the bench's constraints are not. Specifically:

- **DIP is free.** Drive it from the PIP with a configuration variable at 0.67×,
  or type both angles.
- **Abduction does not collapse with flexion.** A fixed revolute limit cannot
  express `abd_max × (1 − MCP_flex/90)`, so treat the abduction limits as valid
  only near full MCP extension.
- **No neighbour coupling** — you can spread adjacent fingers further than a
  hand allows.
- **Thumb CMC axial rotation is dropped.**

So: rough things out by dragging in Onshape, but for a pose that has to be
right, set it in the bench — which enforces all of the above — and read the
angles across.

---

## 7. Driving Onshape from Python

[onshape_bridge.py](onshape_bridge.py) pushes a bench pose straight into the
mated assembly, so you stop retyping 26 angles into mate fields.

```bash
export ONSHAPE_ACCESS_KEY=...  ONSHAPE_SECRET_KEY=...   # dev-portal.onshape.com/keys
python3 onshape_bridge.py check
python3 onshape_bridge.py url "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>"
python3 onshape_bridge.py upload export/hand_m50_articulated.step --did D --wid W
python3 onshape_bridge.py mates --did D --wid W --eid E
python3 onshape_bridge.py pose pose.json --did D --wid W --eid E --watch
```

With `--watch`, re-copying a pose out of the bench updates Onshape within a
second. That is the loop: pose in the bench where the constraints are real, see
it in CAD where your parts are.

Endpoints, verified against the live v17 spec:

| | |
|---|---|
| `GET /users/sessioninfo` | credential check |
| `GET /documents`, `GET /documents/d/{did}/w/{wid}/elements` | find ids |
| `POST /translations/d/{did}/w/{wid}` | import the STEP |
| `GET`/`POST /assemblies/d/{did}/w/{wid}/e/{eid}/matevalues` | read and write mate angles |

### Two things worth knowing about the implementation

**It never constructs a mate value.** The POST body is polymorphic and its
concrete subtypes are not publicly documented. So the bridge GETs the current
values, edits the numeric rotation field in place, and POSTs the same objects
back — whatever Onshape calls the type, it gets handed back unchanged. Verified
offline: `jsonType` is echoed untouched, and on a cylindrical mate the
`translationZ` field is left alone while `rotationZ` is set.

**Mate rotation sense is not knowable in advance.** Which way a revolute mate
turns depends on how its mate connector landed when you made it. The first run
writes `onshape_signs.json`, a template of `+1` per mate; flip any that move the
wrong way to `-1`. One-time job, and it prints the list of mates it could not
find alongside the ones the assembly actually has, so typos surface immediately.

Angles go over the wire in radians. Onshape's API is metric-SI throughout.

### Status

The request/response logic is unit-tested offline against a mocked API. It has
**not** been run against a live Onshape document — that needs your API key. Run
`check` first; if anything returns a 4xx the error body is printed in full.

---

## 8. STEP export in the browser

The bench exports STEP directly — **Download STEP**, no Python round-trip. It
loads OpenCascade as WebAssembly (via [replicad](https://replicad.xyz)) on the
first click, ~23 MB fetched once, then builds the same six solids
`hand_export.py` does.

It is the real thing, not a mesh with a `.step` extension. A typical export
carries ~30 conical, ~24 spherical and 5 spline faces — the splines are the
palm's three-section loft, everything else is analytic. Same STEP writer as the
Python path, so `Offset`, `Shell` and `Fillet` behave identically.

About 4–6 seconds per export. The **clearance** field beside the button adds a
uniform offset for a glove-liner cavity, exactly like `--clearance`.

This works on the [hosted bench](https://ernestwang31.github.io/hand-kinematics/).
It cannot work inside a Claude artifact — that sandbox blocks both the
WebAssembly fetch and page-initiated downloads — so there the button falls back
to the clipboard flow and says so.

### Three OpenCascade degeneracies worth knowing about

These cost real debugging time and will bite anyone building solids
programmatically:

1. **A sphere exactly tangent to a cone's base circle fails to union.** When the
   joint ball's radius equals the bone's radius at that joint, the cone's base
   circle lies *on* the sphere, and the boolean fails intermittently. The balls
   are built 0.4% proud — 0.04 mm on a knuckle — which makes every union a
   clean transversal intersection.
2. **Two cones meeting face to face do not merge.** A straight finger puts
   adjacent bones on an identical circle, and the union returns them as separate
   lumps rather than one solid. Each bone is trimmed 0.3 mm at both ends and the
   joint spheres bridge the gap.
3. **A boolean can fail by returning an empty shape rather than throwing**, and
   a shape that meshes perfectly well can still break the STEP writer. Every
   union is checked for an empty result, and if a part cannot be unioned it is
   exported as separate named bodies instead of losing the whole file.

The thumb metacarpal and thenar eminence belong to the **thumb** solid here, not
the palm — they move with the thumb, and it is the shape a glove's thumb stall
wraps. That also leaves the palm completely static, identical in every pose.

Verified across all six poses, clearances of 0–4 mm, and hand lengths from
165 mm (5th-percentile female) to 210 mm (95th-percentile male).
