"""
Push a pose from the bench straight into an Onshape assembly.

    export ONSHAPE_ACCESS_KEY=...  ONSHAPE_SECRET_KEY=...
    python3 onshape_bridge.py check
    python3 onshape_bridge.py url "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>"
    python3 onshape_bridge.py upload export/hand_m50_articulated.step --did D --wid W
    python3 onshape_bridge.py mates --did D --wid W --eid E
    python3 onshape_bridge.py pose pose.json --did D --wid W --eid E [--watch]

The assembly has to be mated once by hand first — see hand_onshape.py. After
that this drives it, and because the pose comes from the bench it has already
been checked against DIP coupling, abduction collapse and neighbour limits,
none of which Onshape's independent mates can enforce.

Endpoints used, from the live v17 spec:
    GET  /users/sessioninfo
    GET  /documents                              list documents
    GET  /documents/d/{did}/w/{wid}/elements     find the assembly's eid
    POST /translations/d/{did}/w/{wid}           import a STEP
    GET  /assemblies/d/{did}/w/{wid}/e/{eid}/matevalues
    POST /assemblies/d/{did}/w/{wid}/e/{eid}/matevalues

On writing mate values: the POST body is polymorphic and its concrete subtypes
are not publicly documented, so this never constructs one. It GETs the current
values, edits the numeric field in place, and POSTs the same objects back.
Whatever Onshape calls the type, we hand it straight back.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import re
import sys
import time

import requests

BASE = os.environ.get("ONSHAPE_BASE", "https://cad.onshape.com/api/v17")
FINGERS = ["index", "middle", "ring", "little"]
META = {"featureId", "jsonType", "mateName", "ownerOccurrencePath"}


# --------------------------------------------------------------------------- #
def keys() -> tuple[str, str]:
    a, s = os.environ.get("ONSHAPE_ACCESS_KEY"), os.environ.get("ONSHAPE_SECRET_KEY")
    if not (a and s):
        f = pathlib.Path.home() / ".onshape_keys.json"
        if f.exists():
            j = json.loads(f.read_text())
            a, s = j.get("access_key"), j.get("secret_key")
    if not (a and s):
        sys.exit("No credentials. Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY, or\n"
                 "write ~/.onshape_keys.json as {\"access_key\": \"...\", \"secret_key\": \"...\"}.\n"
                 "Make a key pair at https://dev-portal.onshape.com/keys "
                 "(needs read and write scope on documents).")
    return a, s


def call(method: str, path: str, **kw):
    r = requests.request(method, BASE + path, auth=keys(), timeout=90,
                         headers={"Accept": "application/json;charset=UTF-8;qs=0.09",
                                  **kw.pop("headers", {})}, **kw)
    if r.status_code >= 400:
        sys.exit(f"{method} {path} -> {r.status_code}\n{r.text[:1200]}")
    return r.json() if r.content and "json" in r.headers.get("content-type", "") else r.text


def parse_url(u: str) -> dict:
    m = re.search(r"/documents/([0-9a-f]+)(?:/[wv]/([0-9a-f]+))?(?:/e/([0-9a-f]+))?", u)
    if not m:
        sys.exit("Could not read a document id out of that URL.")
    return {"did": m.group(1), "wid": m.group(2), "eid": m.group(3)}


# --------------------------------------------------------------------------- #
def target_angles(J: dict) -> dict[str, float]:
    """Bench pose JSON -> {mate name: degrees}, using hand_onshape.py's names."""
    out = {}
    for f in FINGERS:
        s = J["joints"][f]
        out[f"{f}_MCP_abduction"] = s["abd"]
        out[f"{f}_MCP_flexion"] = s["mcp"]
        out[f"{f}_PIP"] = s["pip"]
        out[f"{f}_DIP"] = s["dip"]
    t = J["joints"]["thumb"]
    out["thumb_CMC_abduction"] = t["cmcA"]
    out["thumb_CMC_flexion"] = t["cmcF"]
    out["thumb_MCP"] = t["mcp"]
    out["thumb_IP"] = t["ip"]
    return out


def angle_key(mv: dict) -> str | None:
    """Which field in this mate value holds the rotation."""
    num = [k for k, v in mv.items() if k not in META and isinstance(v, (int, float))
           and not isinstance(v, bool)]
    if not num:
        return None
    rot = [k for k in num if "rot" in k.lower() or "angle" in k.lower()]
    return (rot or num)[0]


def load_signs(path: pathlib.Path, names) -> dict:
    """Mate rotation sense depends on how each mate connector landed, which is
    not knowable ahead of time. First run writes a template of +1s."""
    if path.exists():
        return json.loads(path.read_text())
    tpl = {n: 1 for n in sorted(names)}
    path.write_text(json.dumps(tpl, indent=1))
    print(f"  wrote {path} — flip any mate that moves the wrong way to -1")
    return tpl


def push(args, J: dict, quiet=False) -> None:
    p = f"/assemblies/d/{args.did}/w/{args.wid}/e/{args.eid}/matevalues"
    cur = call("GET", p)
    values = cur.get("mateValues", [])
    by_name = {v.get("mateName"): v for v in values}
    want = target_angles(J)
    signs = load_signs(pathlib.Path(args.signs), want)

    missing = [n for n in want if n not in by_name]
    if missing:
        print(f"  not in this assembly: {', '.join(sorted(missing))}")
        print(f"  assembly has: {', '.join(sorted(n for n in by_name if n))}")

    touched = 0
    for name, deg in want.items():
        mv = by_name.get(name)
        if mv is None:
            continue
        k = angle_key(mv)
        if k is None:
            print(f"  {name}: no numeric field to set, skipped")
            continue
        mv[k] = math.radians(deg * signs.get(name, 1))   # Onshape angles are radians
        touched += 1

    body = {"mateValues": values}
    if args.dry_run:
        print(json.dumps(body, indent=1)[:2500])
        print(f"  [dry run] would set {touched} mates")
        return
    call("POST", p, json=body, headers={"Content-Type": "application/json"})
    if not quiet:
        print(f"  set {touched} mates  ({J.get('pose', 'custom')}, "
              f"HL {J['subject']['hand_length_mm']} mm)")


# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def ids(p, need_eid=True):
        p.add_argument("--did", required=True)
        p.add_argument("--wid", required=True)
        if need_eid:
            p.add_argument("--eid", required=True)

    sub.add_parser("check", help="verify the credentials")
    u = sub.add_parser("url", help="split a pasted Onshape URL into did/wid/eid")
    u.add_argument("url")
    d = sub.add_parser("docs", help="list documents, or one document's elements")
    d.add_argument("--did")
    d.add_argument("--wid")
    up = sub.add_parser("upload", help="import a STEP into a document")
    up.add_argument("file")
    ids(up, need_eid=False)
    m = sub.add_parser("mates", help="list the assembly's mates and current values")
    ids(m)
    m.add_argument("--dump", help="save the raw matevalues JSON here")
    po = sub.add_parser("pose", help="push a bench pose into the assembly")
    po.add_argument("pose_file")
    ids(po)
    po.add_argument("--signs", default="onshape_signs.json")
    po.add_argument("--dry-run", action="store_true")
    po.add_argument("--watch", action="store_true",
                    help="re-push whenever the pose file changes")

    a = ap.parse_args()

    if a.cmd == "check":
        s = call("GET", "/users/sessioninfo")
        print(f"authenticated as {s.get('name')} <{s.get('email')}>")

    elif a.cmd == "url":
        print(json.dumps(parse_url(a.url), indent=1))

    elif a.cmd == "docs":
        if not a.did:
            for x in call("GET", "/documents?filter=0&limit=20").get("items", []):
                print(f"  {x['id']}  {x['name']}")
            print("\n  pass --did to list a document's elements")
        else:
            wid = a.wid or call("GET", f"/documents/{a.did}")["defaultWorkspace"]["id"]
            print(f"  workspace {wid}")
            for e in call("GET", f"/documents/d/{a.did}/w/{wid}/elements"):
                print(f"  {e['id']}  {e['elementType']:<12} {e['name']}")

    elif a.cmd == "upload":
        f = pathlib.Path(a.file)
        r = requests.post(
            f"{BASE}/translations/d/{a.did}/w/{a.wid}", auth=keys(), timeout=600,
            files={"file": (f.name, f.open("rb"), "application/step")},
            data={"formatName": "STEP", "flattenAssemblies": "false",
                  "yAxisIsUp": "false", "translate": "true",
                  "storeInDocument": "true", "importWithinDocument": "true",
                  "unit": "millimeter", "notifyUser": "false"})
        if r.status_code >= 400:
            sys.exit(f"upload -> {r.status_code}\n{r.text[:1200]}")
        t = r.json()
        print(f"  translation {t.get('id')} state {t.get('requestState')}")
        for _ in range(120):
            time.sleep(3)
            s = call("GET", f"/translations/{t['id']}")
            if s.get("requestState") != "ACTIVE":
                print(f"  {s.get('requestState')}  elements: {s.get('resultElementIds')}")
                if s.get("failureReason"):
                    print(f"  {s['failureReason']}")
                return
        print("  still translating; check the document")

    elif a.cmd == "mates":
        r = call("GET", f"/assemblies/d/{a.did}/w/{a.wid}/e/{a.eid}/matevalues")
        vals = r.get("mateValues", [])
        print(f"  {len(vals)} mates")
        for v in vals:
            k = angle_key(v)
            cur = f"{math.degrees(v[k]):7.1f} deg" if k else "        -"
            print(f"  {v.get('mateName',''):<24} {cur}   [{v.get('jsonType')}]")
        if a.dump:
            pathlib.Path(a.dump).write_text(json.dumps(r, indent=1))
            print(f"  raw response -> {a.dump}")

    elif a.cmd == "pose":
        f = pathlib.Path(a.pose_file)
        push(a, json.loads(f.read_text()))
        if a.watch:
            print("  watching for changes, ctrl-C to stop")
            last = f.stat().st_mtime
            while True:
                time.sleep(1)
                if f.stat().st_mtime != last:
                    last = f.stat().st_mtime
                    time.sleep(0.2)
                    try:
                        push(a, json.loads(f.read_text()))
                    except Exception as e:      # a half-written file, try again next tick
                        print(f"  {e}")


if __name__ == "__main__":
    main()
