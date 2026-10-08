import re
from pathlib import Path

DOWNLOADS = Path(r"C:\Users\jackj\Downloads")
PURSUIT = Path(__file__).resolve().parent / "pursuit.cpp"
PER_ROW = 5

files = [f for f in DOWNLOADS.iterdir() if f.is_file()]
if not files:
    print("No files in Downloads")
    exit(1)
source = max(files, key=lambda f: f.stat().st_mtime)
text = source.read_text(encoding="utf-8")

points = []
for raw_line in text.splitlines():
    line = raw_line.strip()
    if not line or line.startswith("#"):
        continue

    parts = [part.strip() for part in line.split(",")]
    if len(parts) < 2:
        continue

    # jerryio exports cm; pursuit code uses mm
    point = f"{{{float(parts[0]) * 10.0:.1f}, {float(parts[1]) * 10.0:.1f}}}"
    # duplicate points make zero-length segments
    if points and points[-1] == point:
        continue
    points.append(point)

if len(points) < 2:
    print(f"Not enough points in {source.name}")
    exit(1)

width = max(len(p) for p in points) + 1
rows = []
for i in range(0, len(points), PER_ROW):
    chunk = points[i:i + PER_ROW]
    cells = [(p + ",").ljust(width + 1) for p in chunk]
    if i + PER_ROW >= len(points):
        cells[-1] = chunk[-1]
    rows.append("    " + "".join(cells).rstrip())
block = "Point path[] = {\n" + "\n".join(rows) + "\n};"

cpp = PURSUIT.read_text(encoding="utf-8")
cpp, count = re.subn(r"Point path\[\] = \{.*?\n\};", lambda _: block, cpp, count=1, flags=re.S)
if count == 0:
    print("Could not find 'Point path[] = {' in pursuit.cpp")
    exit(1)
PURSUIT.write_text(cpp, encoding="utf-8")

print(f"Wrote {len(points)} points from {source.name} to {PURSUIT.name}")
