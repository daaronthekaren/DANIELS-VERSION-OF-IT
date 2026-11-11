import math
import re
from dataclasses import dataclass, field
from typing import Optional


# -------------------errors-----------------
class GCodeError(ValueError):
    def __init__(self, line_no, text, message):
        self.line_no = line_no
        self.text = text
        self.message = message
        super().__init__(f"line {line_no}: {message}  [{text.strip()}]")


# -------------------supported commands-----------------
MODAL_NOOP_CODES = {('G', 17), ('G', 21), ('G', 40), ('G', 49), ('G', 54), ('G', 80), ('G', 90)}
MOTION_CODES = {('G', 0), ('G', 1)}
UNSUPPORTED_HINTS = {
    ('G', 2): "arc moves (G2) are not supported; linearize arcs before running",
    ('G', 3): "arc moves (G3) are not supported; linearize arcs before running",
    ('G', 20): "inch units (G20) are not supported; use millimetres (G21) or a SCALE directive",
    ('G', 91): "relative positioning (G91) is not supported; X/Y/Z/A must be absolute",
    ('G', 43): "tool length compensation (G43) is not supported",
    ('G', 41): "cutter radius compensation (G41) is not supported",
    ('G', 42): "cutter radius compensation (G42) is not supported",
    ('G', 18): "plane selection G18 is not supported",
    ('G', 19): "plane selection G19 is not supported",
    ('M', 6): "tool change (M6) is not supported",
    ('M', 4): "counter-clockwise spindle (M4) is not supported",
}
MOTION_PARAMS = set('XYZAFE')
WORD_RE = re.compile(r'([A-Z])([-+]?(?:\d+(?:\.\d*)?|\.\d+))')
SCALE_RE = re.compile(r'^SCALE\s*(?:=\s*|\s+)?(.*)$', re.IGNORECASE)


# -------------------parsed line-----------------
@dataclass(frozen=True)
class GCodeLine:
    line_no: int
    text: str
    kind: str
    code: Optional[tuple] = None
    params: dict = field(default_factory=dict)


def strip_comments(raw, line_no):
    text = raw.split(';', 1)[0]
    out = []
    depth = 0
    for ch in text:
        if ch == '(':
            if depth:
                raise GCodeError(line_no, raw, "nested parenthesis comments are not supported")
            depth = 1
        elif ch == ')':
            if not depth:
                raise GCodeError(line_no, raw, "unbalanced ')' in comment")
            depth = 0
        elif not depth:
            out.append(ch)
    if depth:
        raise GCodeError(line_no, raw, "unterminated '(' comment")
    return ''.join(out).strip()


def _parse_number(text, line_no, raw):
    value = float(text)
    if not math.isfinite(value):
        raise GCodeError(line_no, raw, f"non-finite number {text!r}")
    return value


def tokenize(clean, line_no, raw):
    compact = re.sub(r'\s+', '', clean.upper())
    words = []
    pos = 0
    for match in WORD_RE.finditer(compact):
        if match.start() != pos:
            raise GCodeError(line_no, raw, f"malformed word near {compact[pos:match.start() + 1]!r}")
        words.append((match.group(1), _parse_number(match.group(2), line_no, raw)))
        pos = match.end()
    if pos != len(compact):
        raise GCodeError(line_no, raw, f"malformed word near {compact[pos:]!r}")
    return words


def _parse_scale(rest, line_no, raw):
    rest = rest.strip()
    if not rest:
        raise GCodeError(line_no, raw, "SCALE directive needs a value")
    try:
        value = float(rest)
    except ValueError:
        value = None
    if value is not None:
        if not math.isfinite(value) or value <= 0:
            raise GCodeError(line_no, raw, "SCALE must be finite and positive")
        return {k: value for k in 'XYZAF'}
    scales = {}
    for letter, number in tokenize(rest, line_no, raw):
        if letter not in 'XYZAF':
            raise GCodeError(line_no, raw, f"SCALE does not support axis {letter}")
        if letter in scales:
            raise GCodeError(line_no, raw, f"SCALE axis {letter} given twice")
        if number <= 0:
            raise GCodeError(line_no, raw, f"SCALE {letter} must be positive")
        scales[letter] = number
    return scales


def _code_of(letter, number, line_no, raw):
    if number != int(number):
        raise GCodeError(line_no, raw, f"unsupported command {letter}{number:g}")
    return (letter, int(number))


def parse_line(raw, line_no):
    stripped = raw.strip()
    scale_match = SCALE_RE.match(strip_comments(stripped, line_no)) if stripped.upper().startswith('SCALE') else None
    if scale_match:
        return GCodeLine(line_no, raw, 'scale', params=_parse_scale(scale_match.group(1), line_no, raw))

    clean = strip_comments(raw, line_no)
    if clean in ('', '%'):
        return None

    words = tokenize(clean, line_no, raw)
    words = [(l, v) for l, v in words if l != 'N']
    if not words:
        return None

    codes = [_code_of(l, v, line_no, raw) for l, v in words if l in 'GM']
    params = {}
    for letter, value in words:
        if letter in 'GM':
            continue
        if letter in params:
            raise GCodeError(line_no, raw, f"word {letter} given twice")
        params[letter] = value

    if not codes:
        if 'T' in params:
            raise GCodeError(line_no, raw, "tool selection without M3/M5 is not supported")
        raise GCodeError(line_no, raw, "line has parameters but no G or M command")

    for code in codes:
        if code in UNSUPPORTED_HINTS:
            raise GCodeError(line_no, raw, UNSUPPORTED_HINTS[code])

    if all(code in MODAL_NOOP_CODES for code in codes):
        if params:
            raise GCodeError(line_no, raw, f"unexpected parameters {sorted(params)} on modal command")
        return GCodeLine(line_no, raw, 'modal', code=tuple(codes))

    if len(codes) != 1:
        raise GCodeError(line_no, raw, "only one non-modal command per line is supported")
    code = codes[0]

    if code in MOTION_CODES:
        extra = set(params) - MOTION_PARAMS
        if extra:
            raise GCodeError(line_no, raw, f"unsupported parameters {sorted(extra)} for {code[0]}{code[1]}")
        if 'F' in params and params['F'] <= 0:
            raise GCodeError(line_no, raw, "feed F must be positive")
        return GCodeLine(line_no, raw, 'move', code, params)

    if code == ('G', 4):
        extra = set(params) - {'P', 'S'}
        if extra or len(params) != 1:
            raise GCodeError(line_no, raw, "G4 needs exactly one of P (milliseconds) or S (seconds)")
        seconds = params['S'] if 'S' in params else params['P'] * 0.001
        if seconds < 0:
            raise GCodeError(line_no, raw, "dwell time must not be negative")
        return GCodeLine(line_no, raw, 'dwell', code, {'seconds': seconds})

    if code == ('G', 92):
        if set(params) != {'E'}:
            raise GCodeError(line_no, raw, "G92 is only supported for resetting the extruder position (E)")
        return GCodeLine(line_no, raw, 'set_e', code, params)

    if code in (('M', 3), ('M', 5)):
        allowed = {'S', 'T'} if code == ('M', 3) else {'T'}
        extra = set(params) - allowed
        if extra:
            raise GCodeError(line_no, raw, f"unsupported parameters {sorted(extra)} for M{code[1]}")
        tool = params.get('T', 0.0)
        if tool != int(tool) or not 0 <= int(tool) <= 1:
            raise GCodeError(line_no, raw, "tool index T must be 0 or 1")
        if code == ('M', 3):
            if 'S' not in params:
                raise GCodeError(line_no, raw, "M3 needs an S value (tool output 0..1)")
            if not 0.0 <= params['S'] <= 1.0:
                raise GCodeError(line_no, raw, "tool output S must be within [0, 1]")
            return GCodeLine(line_no, raw, 'tool', code, {'tool': int(tool), 'value': params['S']})
        return GCodeLine(line_no, raw, 'tool', code, {'tool': int(tool), 'value': 0.0})

    if code in (('M', 82), ('M', 83)):
        if params:
            raise GCodeError(line_no, raw, f"M{code[1]} takes no parameters")
        return GCodeLine(line_no, raw, 'extrude_mode', code, {'relative': code == ('M', 83)})

    if code in (('M', 2), ('M', 30)):
        if params:
            raise GCodeError(line_no, raw, f"M{code[1]} takes no parameters")
        return GCodeLine(line_no, raw, 'end', code)

    raise GCodeError(line_no, raw, f"unsupported command {code[0]}{code[1]}")


# -------------------program-----------------
class GCodeProgram:
    def __init__(self, text):
        self.text = text
        self.raw_lines = text.splitlines()
        self.lines = []
        for idx, raw in enumerate(self.raw_lines, start=1):
            parsed = parse_line(raw, idx)
            if parsed is not None:
                self.lines.append(parsed)

    def __len__(self):
        return len(self.lines)

    def iter_resolved_moves(self, initial_scale=0.001):
        resolver = UnitResolver(initial_scale)
        state = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'A': 0.0, 'F': None}
        for line in self.lines:
            resolver.apply(line)
            if line.kind == 'end':
                return
            if line.kind != 'move':
                continue
            for key in 'XYZAF':
                if key in line.params:
                    state[key] = line.params[key]
            yield line, {k: (state[k] * resolver.scale[k] if state[k] is not None else None) for k in 'XYZAF'}


class UnitResolver:
    def __init__(self, initial_scale):
        self.scale = {k: float(initial_scale) for k in 'XYZAF'}
        self.scale_directive_seen = False

    def apply(self, line):
        if line.kind == 'scale':
            self.scale.update(line.params)
            self.scale_directive_seen = True
        elif line.kind == 'modal' and ('G', 21) in line.code and not self.scale_directive_seen:
            self.scale = {k: 1.0 for k in 'XYZAF'}
