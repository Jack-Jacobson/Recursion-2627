from pathlib import Path

path_input = input("Enter the jerryio path: ").strip().strip('"').strip("'")
if not path_input:
    print("No path")
    exit(1)

text = Path(path_input).read_text(encoding="utf-8")
point_lines = []

for raw_line in text.splitlines():
    line = raw_line.strip()
    if not line or line.startswith("#"):
        continue

    parts = [part.strip() for part in line.split(",")]
    if len(parts) < 2:
        continue

    x = float(parts[0])
    y = float(parts[1])
    point_lines.append(f"    {{{x:.1f}, {y:.1f}}}")

print("Point path[] = {")
for index, line in enumerate(point_lines):
    suffix = "," if index < len(point_lines) - 1 else ""
    print(f"{line}{suffix}")
print("};")
