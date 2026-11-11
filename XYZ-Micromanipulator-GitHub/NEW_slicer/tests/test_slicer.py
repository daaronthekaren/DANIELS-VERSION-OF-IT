import math
import os
import struct
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import pbm_slicer as ps


# -------------------geometry helpers-----------------
def box(x0, y0, z0, x1, y1, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [[v[a], v[b], v[c]] for a, b, c in faces]


def frame(outer, inner, z1):
    o, i = outer, inner
    tris = []
    tris += box(-o, -o, 0, o, -i, z1)
    tris += box(-o, i, 0, o, o, z1)
    tris += box(-o, -i, 0, -i, i, z1)
    tris += box(i, -i, 0, o, i, z1)
    return tris


def to_ascii(tris):
    out = ["solid test"]
    for t in tris:
        out += ["facet normal 0 0 0", "outer loop"]
        out += [f"vertex {x!r} {y!r} {z!r}" for x, y, z in t]
        out += ["endloop", "endfacet"]
    out.append("endsolid test")
    return "\n".join(out).encode("ascii")


def to_binary(tris, header=b"binary"):
    data = header.ljust(80, b" ") + struct.pack("<I", len(tris))
    for t in tris:
        data += struct.pack("<3f", 0, 0, 0)
        for x, y, z in t:
            data += struct.pack("<3f", x, y, z)
        data += b"\0\0"
    return data


def polygon_area(loop):
    return abs(ps.loop_area(loop))


def settings(**kw):
    base = dict(layer_height=0.25, hatch_spacing=0.1, decimals=6)
    base.update(kw)
    return ps.Settings(**base)


# -------------------reported vertex-on-plane case-----------------
def test_vertex_on_plane_triangle():
    tri = np.array([[(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 1.0, 2.0)]])
    segs = ps.slice_at(tri, 1.0)
    assert len(segs) == 1
    pts = sorted(segs[0])
    assert pts[0] == pytest.approx((0.0, 0.5))
    assert pts[1] == pytest.approx((1.0, 0.0))


def test_strict_sign_test_would_miss_it():
    t = np.array([(0.0, 0.0, 0.0), (1.0, 0.0, 1.0), (0.0, 1.0, 2.0)])
    zs = t[:, 2] - 1.0
    strict = [(a, b) for a, b in ((0, 1), (1, 2), (2, 0)) if zs[a] * zs[b] < 0]
    assert len(strict) == 1


def test_slice_exactly_at_vertex_heights_of_closed_mesh():
    tris = np.array(box(0, 0, 0, 1, 1, 1), dtype=float)
    for z in (0.0, 1.0):
        loops = ps.chain_loops(ps.slice_at(tris, z), z=z)
        assert loops == [] or all(polygon_area(lp) == pytest.approx(1.0) for lp in loops)
    loops = ps.chain_loops(ps.slice_at(tris, 0.5))
    assert len(loops) == 1 and polygon_area(loops[0]) == pytest.approx(1.0)


def test_edge_on_plane_tetrahedron_gives_closed_contour():
    tet = np.array([
        [(0, 0, 0), (1, 0, 0), (0, 1, 0)],
        [(0, 0, 0), (0, 1, 0), (0, 0, 1)],
        [(0, 0, 0), (0, 0, 1), (1, 0, 0)],
        [(1, 0, 0), (0, 0, 1), (0, 1, 0)],
    ], dtype=float)
    loops = ps.chain_loops(ps.slice_at(tet, 0.5), z=0.5)
    assert len(loops) == 1
    assert polygon_area(loops[0]) == pytest.approx(0.125)


def test_coplanar_face_on_slice_plane_does_not_duplicate():
    tris = np.array(box(0, 0, 0, 1, 1, 1) + box(0, 0, 1, 1, 1, 2), dtype=float)
    loops = ps.chain_loops(ps.slice_at(tris, 1.0), z=1.0)
    assert sum(polygon_area(lp) for lp in loops) == pytest.approx(1.0)


# -------------------topology-----------------
def test_simple_block_contour_and_hatch():
    tris = np.array(box(0, 0, 0, 1, 2, 1), dtype=float)
    loops = ps.chain_loops(ps.slice_at(tris, 0.5))
    assert len(loops) == 1 and polygon_area(loops[0]) == pytest.approx(2.0)
    hatch = ps.hatch_lines(loops, 0.1)
    assert len(hatch) == 20
    assert all(abs(a[0] - b[0]) == pytest.approx(1.0) for a, b in hatch)


def test_hole_is_not_filled():
    tris = np.array(frame(1.0, 0.4, 1.0), dtype=float)
    loops = ps.chain_loops(ps.slice_at(tris, 0.5))
    assert loops
    hatch = ps.hatch_lines(loops, 0.05)
    for (x0, y), (x1, _) in hatch:
        lo, hi = sorted((x0, x1))
        if -0.4 < y < 0.4:
            assert hi <= -0.4 + 1e-9 or lo >= 0.4 - 1e-9
    filled = sum(abs(b[0] - a[0]) for a, b in hatch) * 0.05
    assert filled == pytest.approx(4.0 - 0.64, rel=0.03)


def test_prototype_hole_stl_hatch_skips_hole():
    path = os.path.join(os.path.dirname(HERE), "examples", "block_with_hole.stl")
    tris, _ = ps.prepare_triangles(ps.read_stl(path))
    zmin, zmax = tris[:, :, 2].min(), tris[:, :, 2].max()
    loops = ps.chain_loops(ps.slice_at(tris, (zmin + zmax) / 2))
    assert len(loops) == 2
    areas = sorted(polygon_area(lp) for lp in loops)
    hatch = ps.hatch_lines(loops, 0.0005)
    filled = sum(abs(b[0] - a[0]) for a, b in hatch) * 0.0005
    assert filled == pytest.approx(areas[1] - areas[0], rel=0.01)


def test_disconnected_components_both_written():
    tris = np.array(box(0, 0, 0, 1, 1, 1) + box(3, 0, 0, 4, 1, 1), dtype=float)
    loops = ps.chain_loops(ps.slice_at(tris, 0.5))
    assert len(loops) == 2
    hatch = ps.hatch_lines(loops, 0.1)
    xs = {round(min(a[0], b[0]), 6) for a, b in hatch}
    assert xs == {0.0, 3.0}


def test_open_mesh_raises_actionable_error():
    tris = np.array(box(0, 0, 0, 1, 1, 1)[:-2], dtype=float)
    with pytest.raises(ps.SlicerError, match="open contour.*not watertight"):
        ps.chain_loops(ps.slice_at(tris, 0.5), z=0.5)
    with pytest.raises(ps.SlicerError, match="open contour"):
        ps.generate_gcode(tris, settings())


def test_open_mesh_allowed_explicitly_is_reported():
    tris = np.array(box(0, 0, 0, 1, 1, 1)[:-2] + box(3, 0, 0, 4, 1, 1), dtype=float)
    g, stats = ps.generate_gcode(tris, settings(allow_open=True))
    assert stats.open_chains_skipped > 0
    assert any("open contour" in w for w in stats.warnings)


# -------------------input validation-----------------
def test_ascii_and_binary_read_identically(tmp_path):
    tris = box(0, 0, 0, 1, 1, 0.5)
    a = ps.parse_stl_bytes(to_ascii(tris))
    b = ps.parse_stl_bytes(to_binary(tris, header=b"solid but actually binary"))
    assert a.shape == b.shape == (12, 3, 3)
    assert np.allclose(a, b, atol=1e-6)


@pytest.mark.parametrize("data,needle", [
    (b"", "empty"),
    (b"solid x\nendsolid x\n", "no triangles"),
    (b"solid x\nfacet normal 0 0 0\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\n", "3 vertices"),
    (b"solid x\nfacet normal 0 0 0\nouter loop\nvertex 0 0 zz\n", "not a number"),
    (b"hello world", "must start with 'solid'"),
    (b"\xff\xfe" * 100, "not ASCII"),
])
def test_malformed_stl_rejected(data, needle):
    with pytest.raises(ps.SlicerError, match=needle):
        ps.parse_stl_bytes(data)


def test_binary_with_zero_triangles_rejected():
    with pytest.raises(ps.SlicerError, match="no triangles"):
        ps.parse_stl_bytes(b" " * 80 + struct.pack("<I", 0))


def test_nonfinite_coordinates_rejected():
    tris = np.array(box(0, 0, 0, 1, 1, 1), dtype=float)
    tris[0, 0, 0] = np.nan
    with pytest.raises(ps.SlicerError, match="non-finite"):
        ps.generate_gcode(tris, settings())


def test_degenerate_triangles_dropped_and_counted():
    tris = box(0, 0, 0, 1, 1, 1) + [[(0, 0, 0), (1, 1, 1), (2, 2, 2)]]
    g, stats = ps.generate_gcode(np.array(tris, dtype=float), settings())
    assert stats.degenerate_dropped == 1


@pytest.mark.parametrize("kw", [dict(layer_height=0), dict(layer_height=-1), dict(hatch_spacing=float("nan")),
                                dict(laser_power=2.0), dict(max_speed=0), dict(layer_axis="B"),
                                dict(hatch_spacing=0.0001, decimals=3)])
def test_invalid_settings_rejected(kw):
    with pytest.raises(ps.SlicerError):
        ps.generate_gcode(np.array(box(0, 0, 0, 1, 1, 1), dtype=float), settings(**kw))


def test_flat_model_rejected():
    tri = np.array([[(0, 0, 0), (1, 0, 0), (0, 1, 0)]], dtype=float)
    with pytest.raises(ps.SlicerError, match="zero height"):
        ps.generate_gcode(tri, settings())


# -------------------units and output-----------------
def test_unit_scaling_um_to_mm():
    tris_um = np.array(box(0, 0, 0, 100, 100, 20), dtype=float)
    tris_mm = tris_um * 0.001
    g_um, s_um = ps.generate_gcode(tris_um, ps.Settings(units_to_mm=0.001))
    g_mm, s_mm = ps.generate_gcode(tris_mm, ps.Settings())
    assert g_um == g_mm
    assert s_um.layers_written == 20


def test_gcode_structure_and_laser_state():
    tris = np.array(box(0, 0, 0, 1, 1, 1), dtype=float)
    g, stats = ps.generate_gcode(tris, settings(laser_power=0.4, max_speed=60.0, travel_speed=120.0))
    assert g[1:5] == ["SCALE=1", "G21", "G90", "M3 S0"]
    assert g[-2:] == ["M3 S0", "M2"]
    laser = False
    last_layer_z = -1
    for line in g:
        if line.startswith("M3"):
            laser = line != "M3 S0"
        if line.startswith("G1 Z"):
            assert not laser
            z = float(line.split()[1][1:])
            assert z > last_layer_z
            last_layer_z = z
        if line.startswith("G1") and " F" in line:
            assert 0 < float(line.split("F")[1]) <= 60.0
    assert not laser
    assert stats.layers_written == 4
    assert [float(l.split()[1][1:]) for l in g if l.startswith("G1 Z")] == [0.125, 0.375, 0.625, 0.875]


def test_layer_axis_a_and_runner_compatibility():
    sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "EDITED_gui", "source")))
    from gcode_program import GCodeProgram
    tris = np.array(frame(1.0, 0.4, 0.5), dtype=float)
    g, stats = ps.generate_gcode(tris, settings(layer_axis="A"))
    program = GCodeProgram("\n".join(g))
    a_moves = [ln for ln in program.lines if ln.kind == "move" and "A" in ln.params]
    assert len(a_moves) == stats.layers_written == 2
    assert not any("Z" in ln.params for ln in program.lines if ln.kind == "move")


def test_output_is_deterministic(tmp_path):
    tris = np.array(frame(1.0, 0.4, 0.5), dtype=float)
    assert ps.generate_gcode(tris, settings())[0] == ps.generate_gcode(tris, settings())[0]


def test_cli_writes_file_and_reports_errors(tmp_path, capsys):
    stl = tmp_path / "part.stl"
    stl.write_bytes(to_ascii(box(0, 0, 0, 0.01, 0.01, 0.004)))
    assert ps.main([str(stl)]) == 0
    text = (tmp_path / "part.gcode").read_text()
    assert text.startswith("; -------------------start-----------------\nSCALE=1\n")
    bad = tmp_path / "bad.stl"
    bad.write_bytes(to_ascii(box(0, 0, 0, 1, 1, 1)[:-2]))
    assert ps.main([str(bad)]) == 2
    assert "not watertight" in capsys.readouterr().err
