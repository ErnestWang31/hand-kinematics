"""
Wrap hand_explorer.html into a standalone page for GitHub Pages.

The bench is authored as a fragment: the Artifact platform injects a <head>
with charset, viewport and a small reset. Served raw from Pages it would get
none of that — mojibake on every middle dot, no mobile layout, a default body
margin. This adds an equivalent head so the two render identically.

    python3 build_site.py        # -> docs/index.html
"""
import pathlib
import re

SRC = pathlib.Path("hand_explorer.html")
OUT = pathlib.Path("docs/index.html")

HEAD = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="description" content="Interactive bench for human hand size and joint range of motion, bounded by measured anthropometric limits.">
<style>
  :root{color-scheme:light dark;
        padding-top:env(safe-area-inset-top,0px);
        padding-bottom:env(safe-area-inset-bottom,0px)}
  body{margin:0;font-family:system-ui,-apple-system,sans-serif;font-size:14px}
  img{max-width:100%}
  [hidden]{display:none!important}
</style>
"""

FOOT = """
<noscript><p style="padding:16px">This page needs JavaScript — it computes the
hand's forward kinematics in the browser.</p></noscript>
</body>
</html>
"""


def main() -> None:
    src = SRC.read_text()
    title = re.search(r"<title>(.*?)</title>", src)
    body = src.replace(title.group(0), "", 1) if title else src
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(f"{HEAD}<title>{title.group(1) if title else 'Hand Bench'}</title>\n"
                   f"</head>\n<body>\n{body.lstrip()}{FOOT}")
    print(f"{OUT}  {OUT.stat().st_size/1024:.0f} KB")


if __name__ == "__main__":
    main()
