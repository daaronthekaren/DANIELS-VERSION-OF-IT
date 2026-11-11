import argparse
import math
import os
import struct
import sys
from dataclasses import dataclass, field

import numpy as np


# -------------------settings-----------------
@dataclass
class Settings:
    layer_height: float = 0.001
    hatch_spacing: float = 0.0005
    outline_passes: int = 1
    write_speed: float = 60.0
    travel_speed: float = 120.0
    max_speed: float = 60.0
    laser_power: float = 0.5
    layer_axis: str = "Z"
    decimals: int = 4
    alternate_hatch: bool = True
    units_to_mm: float = 1.0
    weld_tolerance: float = 1e-7
    allow_open: bool = False

    def validate(self):
        positive = {
            "layer height": self.layer_height,
            "hatch spacing": self.hatch_spacing,
            "write speed": self.write_speed,
            "travel speed": self.travel_speed,
            "max speed": self.max_speed,
            "unit scale": self.units_to_mm,
            "weld tolerance": self.weld_tolerance,
        }
        for name, value in positive.items():
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise SlicerError(f"{name} must be finite and positive, got {value!r}")
        if not 0.0 <= self.laser_power <= 1.0:
            raise SlicerError(f"laser power must be within [0, 1], got {self.laser_power!r}")
        if self.layer_axis not in ("Z", "A"):
            raise SlicerError(f"layer axis must be Z or A, got {self.layer_axis!r}")
        if not isinstance(self.outline_passes, int) or self.outline_passes < 0:
            raise SlicerError("outline passes must be a non-negative integer")
        if not isinstance(self.decimals, int) or not 3 <= self.decimals <= 9:
            raise SlicerError("decimals must be an integer within [3, 9]")
        resolution = 10.0 ** -self.decimals
        if self.hatch_spacing < 2 * resolution or self.layer_height < 2 * resolution:
            raise SlicerError(f"{self.decimals} decimals ({resolution} mm) are too coarse for the "
                              "layer height / hatch spacing; increase --decimals")


class SlicerError(ValueError):
    pass


@dataclass
class SliceStats:
    triangles: int = 0
    degenerate_dropped: int = 0
    layers_total: int = 0
    layers_written: int = 0
    loops: int = 0
    hatch_lines: int = 0
    write_moves: int = 0
    travel_moves: int = 0
    layer_moves: int = 0
    laser_on: int = 0
    laser_off: int = 0
    write_length: float = 0.0
    travel_length: float = 0.0
    sliver_loops_dropped: int = 0
    open_chains_skipped: int = 0
    warnings: list = field(default_factory=list)


# -------------------read stl-----------------
def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    return parse_stl_bytes(data, path)


def parse_stl_bytes(data, name="<stl>"):
    if len(data) == 0:
        raise SlicerError(f"{name}: file is empty")
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if len(data) == 84 + n * 50:
            if n == 0:
                raise SlicerError(f"{name}: binary STL contains no triangles")
            record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            tris = np.frombuffer(data, dtype=record, count=n, offset=84)["v"].astype(np.float64)
            return tris
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError:
        raise SlicerError(f"{name}: not a binary STL (size does not match the triangle count) "
                          "and not ASCII text")
    return _parse_ascii_stl(text, name)


def _parse_ascii_stl(text, name):
    lines = text.splitlines()
    first = next((ln.strip() for ln in lines if ln.strip()), "")
    if not first.lower().startswith("solid"):
        raise SlicerError(f"{name}: ASCII STL must start with 'solid'")
    tris = []
    current = None
    in_loop = False
    for line_no, line in enumerate(lines, start=1):
        parts = line.split()
        if not parts:
            continue
        key = parts[0].lower()
        if key == "facet":
            if current is not None:
                raise SlicerError(f"{name}:{line_no}: 'facet' before previous 'endfacet'")
            current = []
        elif key == "outer":
            if current is None or in_loop:
                raise SlicerError(f"{name}:{line_no}: unexpected 'outer loop'")
            in_loop = True
        elif key == "vertex":
            if current is None or not in_loop:
                raise SlicerError(f"{name}:{line_no}: vertex outside a facet loop")
            if len(parts) != 4:
                raise SlicerError(f"{name}:{line_no}: vertex needs exactly 3 coordinates")
            try:
                current.append([float(v) for v in parts[1:]])
            except ValueError:
                raise SlicerError(f"{name}:{line_no}: vertex coordinate is not a number")
        elif key == "endloop":
            in_loop = False
        elif key == "endfacet":
            if current is None or len(current) != 3:
                raise SlicerError(f"{name}:{line_no}: facet must have exactly 3 vertices")
            tris.append(current)
            current = None
        elif key in ("solid", "endsolid"):
            continue
        else:
            raise SlicerError(f"{name}:{line_no}: unexpected token {parts[0]!r}")
    if current is not None:
        raise SlicerError(f"{name}: last facet is not terminated")
    if not tris:
        raise SlicerError(f"{name}: no triangles found")
    return np.array(tris, dtype=np.float64)


def prepare_triangles(tris, units_to_mm=1.0):
    tris = np.asarray(tris, dtype=np.float64)
    if tris.ndim != 3 or tris.shape[1:] != (3, 3) or len(tris) == 0:
        raise SlicerError("triangle array must have shape (n, 3, 3) with n > 0")
    if not np.all(np.isfinite(tris)):
        raise SlicerError("STL contains non-finite (NaN/inf) coordinates")
    tris = tris * units_to_mm
    area2 = np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    keep = area2 > 0.0
    dropped = int(len(tris) - np.count_nonzero(keep))
    tris = tris[keep]
    if len(tris) == 0:
        raise SlicerError("all triangles are degenerate (zero area)")
    return tris, dropped


# -------------------slice-----------------
def slice_at(tris, z):
    segs = []
    for t in tris:
        d = t[:, 2] - z
        above = d >= 0.0
        if above.all() or not above.any():
            continue
        pts = []
        for a, b in ((0, 1), (1, 2), (2, 0)):
            if above[a] != above[b]:
                k = d[a] / (d[a] - d[b])
                p = t[a] + k * (t[b] - t[a])
                pts.append((float(p[0]), float(p[1])))
        if pts[0] != pts[1]:
            segs.append((pts[0], pts[1]))
    return segs


# -------------------outlines-----------------
class _Welder:
    def __init__(self, tol):
        self.tol = tol
        self.cells = {}
        self.points = []

    def index(self, p):
        cx, cy = int(math.floor(p[0] / self.tol)), int(math.floor(p[1] / self.tol))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for idx in self.cells.get((cx + dx, cy + dy), ()):
                    q = self.points[idx]
                    if abs(q[0] - p[0]) <= self.tol and abs(q[1] - p[1]) <= self.tol:
                        return idx
        idx = len(self.points)
        self.points.append(p)
        self.cells.setdefault((cx, cy), []).append(idx)
        return idx


def loop_area(loop):
    area = 0.0
    for (x1, y1), (x2, y2) in zip(loop, loop[1:]):
        area += x1 * y2 - x2 * y1
    return 0.5 * area


def chain_loops(segs, tol=1e-7, z=None, allow_open=False, stats=None):
    welder = _Welder(tol)
    edge_count = {}
    for a, b in segs:
        ia, ib = welder.index(a), welder.index(b)
        if ia == ib:
            continue
        key = (min(ia, ib), max(ia, ib))
        edge_count[key] = edge_count.get(key, 0) + 1
    edges = [key for key, count in sorted(edge_count.items()) if count % 2 == 1]

    adjacency = {}
    for i, (u, v) in enumerate(edges):
        adjacency.setdefault(u, []).append(i)
        adjacency.setdefault(v, []).append(i)

    where = f" at z={z:.6f}" if z is not None else ""
    dangling = sorted(v for v, inc in adjacency.items() if len(inc) % 2 == 1)
    if dangling and not allow_open:
        p = welder.points[dangling[0]]
        raise SlicerError(f"open contour{where}: {len(dangling)} dangling endpoint(s), first near "
                          f"({p[0]:.6f}, {p[1]:.6f}); the mesh is not watertight")

    used = [False] * len(edges)
    loops = []
    for start in range(len(edges)):
        if used[start]:
            continue
        used[start] = True
        u, v = edges[start]
        chain = [u, v]
        while chain[-1] != chain[0]:
            nxt = next((j for j in adjacency[chain[-1]] if not used[j]), None)
            if nxt is None:
                break
            used[nxt] = True
            a, b = edges[nxt]
            chain.append(b if a == chain[-1] else a)
        if chain[-1] != chain[0]:
            if stats is not None:
                stats.open_chains_skipped += 1
            continue
        loop = [welder.points[i] for i in chain]
        if abs(loop_area(loop)) <= tol * tol or len(chain) < 4:
            if stats is not None:
                stats.sliver_loops_dropped += 1
            continue
        loops.append(loop)
    return loops


# -------------------hatch-----------------
def hatch_lines(loops, spacing, vertical=False):
    if not loops:
        return []
    if vertical:
        loops = [[(p[1], p[0]) for p in lp] for lp in loops]
    ys = [p[1] for lp in loops for p in lp]
    ymin, ymax = min(ys), max(ys)
    lines = []
    n_rows = int(math.floor((ymax - ymin) / spacing + 0.5))
    flip = False
    for row_idx in range(max(n_rows, 0) + 1):
        y = ymin + spacing / 2 + row_idx * spacing
        if y >= ymax:
            break
        xs = []
        for lp in loops:
            for i in range(len(lp) - 1):
                (x1, y1), (x2, y2) = lp[i], lp[i + 1]
                if (y1 <= y < y2) or (y2 <= y < y1):
                    xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        if len(xs) % 2:
            raise SlicerError(f"odd number of crossings ({len(xs)}) on hatch line y={y:.6f}")
        xs.sort()
        row = [((xs[i], y), (xs[i + 1], y)) for i in range(0, len(xs) - 1, 2) if xs[i + 1] > xs[i]]
        if flip:
            row = [(b, a) for a, b in reversed(row)]
        flip = not flip
        lines.extend(row)
    if vertical:
        lines = [((a[1], a[0]), (b[1], b[0])) for a, b in lines]
    return lines


def fmt(v, decimals=4):
    s = f"{v:.{decimals}f}"
    if float(s) == 0:
        s = f"{0:.{decimals}f}"
    return s


def fmt_feed(f):
    if not math.isfinite(f) or f <= 0:
        raise SlicerError(f"feed must be finite and positive, got {f!r}")
    s = f"{f:.3f}"
    if float(s) <= 0:
        raise SlicerError(f"feed {f!r} is below the 0.001 mm/min output resolution")
    return s


# -------------------gcode-----------------
def generate_gcode(tris, settings, preview_layers=None):
    settings.validate()
    stats = SliceStats()
    tris, stats.degenerate_dropped = prepare_triangles(tris, settings.units_to_mm)
    stats.triangles = len(tris)
    dec = settings.decimals

    zmin, zmax = float(tris[:, :, 2].min()), float(tris[:, :, 2].max())
    height = zmax - zmin
    if height <= 0:
        raise SlicerError("model has zero height (all triangles in one plane)")
    n_layers = int(math.ceil(round(height / settings.layer_height, 6)))
    stats.layers_total = n_layers

    write_f = min(settings.write_speed, settings.max_speed)
    travel_f = min(settings.travel_speed, settings.max_speed)
    if write_f < settings.write_speed or travel_f < settings.travel_speed:
        stats.warnings.append(f"speeds capped to {settings.max_speed} mm/min")
    write_fs, travel_fs = fmt_feed(write_f), fmt_feed(travel_f)
    laser_on_cmd = f"M3 S{settings.laser_power:g}"
    laser_off_cmd = "M3 S0"

    g = ["; -------------------start-----------------", "SCALE=1", "G21", "G90", laser_off_cmd]
    state = {"laser": False, "pos": None}

    def laser(on):
        if on != state["laser"]:
            g.append(laser_on_cmd if on else laser_off_cmd)
            state["laser"] = on
            if on:
                stats.laser_on += 1
            else:
                stats.laser_off += 1

    def go_to(x, y):
        laser(False)
        if state["pos"] is not None:
            stats.travel_length += math.hypot(x - state["pos"][0], y - state["pos"][1])
        g.append(f"G1 X{fmt(x, dec)} Y{fmt(y, dec)} F{travel_fs}")
        stats.travel_moves += 1
        state["pos"] = (x, y)

    def write_to(x, y):
        laser(True)
        stats.write_length += math.hypot(x - state["pos"][0], y - state["pos"][1])
        g.append(f"G1 X{fmt(x, dec)} Y{fmt(y, dec)} F{write_fs}")
        stats.write_moves += 1
        state["pos"] = (x, y)

    for i in range(n_layers):
        z_cut = zmin + (i + 0.5) * settings.layer_height
        if z_cut >= zmax:
            break
        loops = chain_loops(slice_at(tris, z_cut), settings.weld_tolerance, z_cut, settings.allow_open, stats)
        if not loops:
            continue
        vertical = settings.alternate_hatch and (i % 2 == 1)
        hatch = hatch_lines(loops, settings.hatch_spacing, vertical)
        stats.layers_written += 1
        stats.loops += len(loops)
        stats.hatch_lines += len(hatch)

        g.append(f"; -------------------layer {i + 1}-----------------")
        laser(False)
        g.append(f"G1 {settings.layer_axis}{fmt(z_cut, dec)} F{travel_fs}")
        stats.layer_moves += 1

        for _ in range(settings.outline_passes):
            for lp in loops:
                go_to(*lp[0])
                for p in lp[1:]:
                    write_to(*p)
        for a, b in hatch:
            if state["pos"] is None or math.hypot(a[0] - state["pos"][0], a[1] - state["pos"][1]) > 1e-9:
                go_to(*a)
            write_to(*b)

        if preview_layers is not None:
            preview_layers.append((i + 1, loops, hatch))

    if stats.layers_written == 0:
        raise SlicerError("no layer produced any closed contour; nothing to write")
    if stats.open_chains_skipped:
        stats.warnings.append(f"{stats.open_chains_skipped} open contour chain(s) skipped (--allow-open)")

    g.append("; -------------------end-----------------")
    g.append(laser_off_cmd)
    if state["laser"]:
        stats.laser_off += 1
    state["laser"] = False
    g.append("M2")
    return g, stats


def main(argv=None):
    parser = argparse.ArgumentParser(description="Slice an STL into 2PP outline + hatch G-code")
    defaults = Settings()
    parser.add_argument("stl")
    parser.add_argument("-o", "--output")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--layer-height", type=float, default=defaults.layer_height)
    parser.add_argument("--hatch-spacing", type=float, default=defaults.hatch_spacing)
    parser.add_argument("--write-speed", type=float, default=defaults.write_speed)
    parser.add_argument("--travel-speed", type=float, default=defaults.travel_speed)
    parser.add_argument("--max-speed", type=float, default=defaults.max_speed)
    parser.add_argument("--power", type=float, default=defaults.laser_power)
    parser.add_argument("--layer-axis", choices=("Z", "A"), default=defaults.layer_axis)
    parser.add_argument("--units", choices=("mm", "um"), default="mm")
    parser.add_argument("--decimals", type=int, default=defaults.decimals)
    parser.add_argument("--allow-open", action="store_true")
    args = parser.parse_args(argv)

    settings = Settings(layer_height=args.layer_height, hatch_spacing=args.hatch_spacing,
                        write_speed=args.write_speed, travel_speed=args.travel_speed,
                        max_speed=args.max_speed, laser_power=args.power, layer_axis=args.layer_axis,
                        decimals=args.decimals, allow_open=args.allow_open,
                        units_to_mm=1.0 if args.units == "mm" else 0.001)
    out_path = args.output or os.path.splitext(args.stl)[0] + ".gcode"

    try:
        tris = read_stl(args.stl)
        all_layers = [] if args.preview else None
        g, stats = generate_gcode(tris, settings, all_layers)
    except SlicerError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(g) + "\n")

    print(f"read {stats.triangles} triangles ({stats.degenerate_dropped} degenerate dropped), "
          f"{stats.layers_total} layers")
    for warning in stats.warnings:
        print(f"warning: {warning}")
    print(f"wrote {out_path}: {len(g)} lines, {stats.layers_written} layers, {stats.loops} contours, "
          f"{stats.hatch_lines} hatch lines")
    write_f = min(settings.write_speed, settings.max_speed)
    travel_f = min(settings.travel_speed, settings.max_speed)
    est_min = stats.write_length / write_f + stats.travel_length / travel_f
    print(f"laser-on path {stats.write_length:.2f} mm, travel {stats.travel_length:.2f} mm, "
          f"~{est_min:.0f} min at the programmed feeds (ignores acceleration and per-line overhead)")

    if all_layers is not None:
        picks = sorted({0, len(all_layers) // 2, len(all_layers) - 1})
        make_preview([all_layers[i] for i in picks], os.path.splitext(out_path)[0] + "_preview.png")
    return 0


# -------------------preview-----------------
def make_preview(layers, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, len(layers), figsize=(5 * len(layers), 5))
    if len(layers) == 1:
        axs = [axs]
    for ax, (num, loops, hatch) in zip(axs, layers):
        step = max(1, len(hatch) // 60)
        for a, b in hatch[::step]:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="tab:blue", lw=0.6)
        for lp in loops:
            xs, ys = zip(*lp)
            ax.plot(xs, ys, color="tab:red", lw=1.5)
        ax.set_title(f"layer {num}  (red = outline, blue = hatch, every {step}th line)")
        ax.set_aspect("equal")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
    plt.tight_layout()
    plt.savefig(path, dpi=120)
    plt.close(fig)
    print("preview saved:", path)


if __name__ == "__main__":
    sys.exit(main())
