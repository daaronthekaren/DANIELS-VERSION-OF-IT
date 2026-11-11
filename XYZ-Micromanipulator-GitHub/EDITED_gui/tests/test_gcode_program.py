import os

import pytest

from gcode_program import GCodeError, GCodeProgram, parse_line

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


# -------------------comments and tokens-----------------
def test_semicolon_comment_does_not_command_axis():
    line = parse_line("G1 X1 ; Y99 is only a comment", 1)
    assert line.kind == "move"
    assert line.params == {"X": 1.0}


def test_parenthesis_comment_is_removed():
    line = parse_line("G1 X1 (Y99) Z2", 1)
    assert line.params == {"X": 1.0, "Z": 2.0}


@pytest.mark.parametrize("text", ["G1 X1 (Y99", "G1 X1 Y2)", "G1 (a (b) c) X1"])
def test_malformed_comments_rejected(text):
    with pytest.raises(GCodeError):
        parse_line(text, 3)


def test_comment_only_and_blank_lines_are_skipped():
    assert parse_line("; just a comment", 1) is None
    assert parse_line("   ", 2) is None
    assert parse_line("(header)", 3) is None


def test_g1_and_g10_are_distinct():
    assert parse_line("G1 X1", 1).kind == "move"
    with pytest.raises(GCodeError, match="G10"):
        parse_line("G10 X1", 1)


def test_zero_padded_codes():
    assert parse_line("G01 X1 F60", 1).code == ("G", 1)
    assert parse_line("G00 X1", 1).code == ("G", 0)
    off = parse_line("M05", 1)
    assert off.kind == "tool" and off.params == {"tool": 0, "value": 0.0}


def test_words_without_spaces():
    assert parse_line("G1X0.5Y-0.25F60", 1).params == {"X": 0.5, "Y": -0.25, "F": 60.0}


def test_malformed_word_rejected():
    with pytest.raises(GCodeError, match="malformed"):
        parse_line("G1 X1.0.0", 1)
    with pytest.raises(GCodeError, match="malformed"):
        parse_line("G1 Xabc", 1)


def test_duplicate_word_rejected():
    with pytest.raises(GCodeError, match="twice"):
        parse_line("G1 X1 X2", 1)


# -------------------tool commands-----------------
def test_m3_tool_and_value():
    line = parse_line("M3 T1 S0.25", 1)
    assert line.params == {"tool": 1, "value": 0.25}


@pytest.mark.parametrize("text", ["M3 S1000", "M3 S-0.1", "M3", "M3 T2 S0.5", "M5 S1"])
def test_bad_tool_commands_rejected(text):
    with pytest.raises(GCodeError):
        parse_line(text, 1)


def test_tool_selection_alone_rejected():
    with pytest.raises(GCodeError, match="tool selection"):
        parse_line("T1", 1)
    with pytest.raises(GCodeError, match="tool change"):
        parse_line("M6 T1", 1)


# -------------------dwell units-----------------
def test_dwell_units():
    assert parse_line("G4 S0.25", 1).params == {"seconds": 0.25}
    assert parse_line("G4 P500", 1).params == {"seconds": 0.5}
    with pytest.raises(GCodeError):
        parse_line("G4", 1)
    with pytest.raises(GCodeError):
        parse_line("G4 P1 S1", 1)
    with pytest.raises(GCodeError):
        parse_line("G4 S-1", 1)


# -------------------modes-----------------
@pytest.mark.parametrize("text,needle", [
    ("G20", "inch"), ("G91", "relative"), ("G2 X1 Y1 I0.5 J0", "arc"), ("G3 X1 Y1 I0.5 J0", "arc"),
    ("G43 H1", "length"), ("G92 X0", "G92"), ("G55", "unsupported"), ("M4 S0.1", "counter"),
])
def test_unsupported_modes_rejected(text, needle):
    with pytest.raises(GCodeError, match=needle):
        parse_line(text, 1)


def test_modal_noop_preamble_accepted():
    line = parse_line("G17 G54 G40 G49 G80 G90", 1)
    assert line.kind == "modal"


def test_program_termination():
    assert parse_line("M2", 1).kind == "end"
    assert parse_line("M30", 1).kind == "end"
    program = GCodeProgram("SCALE=1\nG1 X1 F60\nM2\nG1 X99 F60\n")
    moves = [resolved["X"] for _, resolved in program.iter_resolved_moves()]
    assert moves == [1.0]


def test_scale_directive_forms():
    assert parse_line("SCALE=0.001", 1).params["X"] == 0.001
    assert parse_line("SCALE X0.5 Y0.25", 1).params == {"X": 0.5, "Y": 0.25}
    with pytest.raises(GCodeError):
        parse_line("SCALE=0", 1)
    with pytest.raises(GCodeError):
        parse_line("SCALE Q1", 1)


def test_g21_sets_mm_unless_scale_given():
    program = GCodeProgram("G21\nG1 X1 F60\n")
    assert [r["X"] for _, r in program.iter_resolved_moves(0.001)] == [1.0]
    program = GCodeProgram("SCALE=0.5\nG21\nG1 X1 F60\n")
    assert [r["X"] for _, r in program.iter_resolved_moves(0.001)] == [0.5]
    program = GCodeProgram("G1 X1000 F60\n")
    assert [r["X"] for _, r in program.iter_resolved_moves(0.001)] == [1.0]


def test_error_reports_line_number():
    with pytest.raises(GCodeError) as info:
        GCodeProgram("G21\nG90\nG1 X1 F60\nG2 X1 Y1 I1 J0\n")
    assert info.value.line_no == 4


# -------------------freecad output-----------------
def test_freecad_milling_output_is_rejected_with_reason():
    path = os.path.join(REPO, "NEW_freecad_study", "block_test_v2.gcode")
    with open(path, encoding="utf-8") as handle:
        text = handle.read()
    with pytest.raises(GCodeError) as info:
        GCodeProgram(text)
    assert info.value.line_no == 5
    assert "M6" in info.value.message


def test_slicer_output_parses():
    path = os.path.join(REPO, "NEW_slicer", "examples", "block_test_v2.gcode")
    if not os.path.exists(path):
        pytest.skip("slicer example not generated")
    with open(path, encoding="utf-8") as handle:
        program = GCodeProgram(handle.read())
    assert len(program) > 1000
