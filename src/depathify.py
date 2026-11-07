"""Finds the jerryio export in Downloads that a path in paths.cpp came from."""
import sys
from pathlib import Path

from pathify import DOWNLOADS, POINT_RE, load_paths


def cpp_points(body):
    return [(round(float(x), 1), round(float(y), 1)) for x, y in POINT_RE.findall(body)]


def export_points(source):
    """Converts an export the same way pathify.py does (cm -> mm, 1 decimal, dedupe)."""
    points = []
    for raw_line in source.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [part.strip() for part in line.split(",")]
        if len(parts) < 2:
            continue
        try:
            x, y = float(parts[0]), float(parts[1])
        except ValueError:
            continue
        point = (round(float(f"{x * 10.0:.1f}"), 1), round(float(f"{y * 10.0:.1f}"), 1))
        if points and points[-1] == point:
            continue
        points.append(point)
    return points


def find_source(num):
    paths = load_paths()
    if num not in paths:
        print(f"path{num} not found in paths.cpp (have {sorted(paths)})")
        raise SystemExit(1)
    target = cpp_points(paths[num])

    files = [f for f in DOWNLOADS.iterdir() if f.is_file() and f.name.startswith("path.jerryio")]
    files.sort(key=lambda f: f.stat().st_mtime, reverse=True)
    for f in files:
        if export_points(f) == target:
            return f
    return None


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else input("Path number: ").strip()
    if not arg.isdigit():
        print("Path number must be a positive integer.")
        raise SystemExit(1)

    match = find_source(int(arg))
    if match is None:
        print(f"No export in {DOWNLOADS} matches path{arg}.")
        raise SystemExit(1)
    print(match)


if __name__ == "__main__":
    main()
