#!/usr/bin/env python3
"""ArtPrint: translate Unlayered Slicer G-code into motor coordinates for Klipper.

Unlayered Slicer writes every point as the nozzle tip in the BED frame plus the B (head tilt)
and C (bed rotation) angles, and leaves the five-axis compensation to the Unlayered fork of
RepRapFirmware (src/Movement/Kinematics/FiveAxisKinematics.cpp, CartesianToMotorSteps, M669 K16).
Klipper has no such kinematics, so this script does the same computation ahead of time and writes
moves Klipper can execute as plain independent axes.

Pipeline for every move:
    bed frame (X Y Z B C)  --rotate by C, add the B-arm offset-->  machine frame
    machine frame          --differential belt matrix (optional)--> motor positions
Moves whose B and C stay constant map to straight lines (the transform is affine), so they are
written as one move. Moves that change B or C are split so that each piece turns by at most
max_segment_deg and travels at most max_segment_mm.

Usage:
    python unlayered_to_klipper.py input.gcode [-o output.gcode] [--machine machine.json]
Without -o the file is rewritten in place, which is what the slicer's
"Post-processing scripts" option expects.
"""

import argparse
import json
import math
import os
import re
import sys

DEFAULT_MACHINE = {
    # "corexbyc": Unlayered's belt layout. Motors 0/3 drive X and B together, motors 1/4 drive Y
    # and C together (M669 K16). "cartesian": one motor per axis, output is X Y Z B C directly.
    "kinematics": "corexbyc",
    # B-arm geometry (M669 A and D): a5 and d6 in mm, plus bSkew (M669 B). Defaults are the
    # firmware's defaults for the Unlayered printer.
    "a5": 2.5,
    "d6": 46.4,
    "b_skew": 0.0,
    # B angle at which the nozzle points straight down (Unlayered: -90). With zero_offset_at_home
    # the B-arm offset is measured from this pose, so a vertical nozzle needs no correction.
    "b_home": -90.0,
    "zero_offset_at_home": True,
    # Belt reductions of the rotary axes (M669 R and Q). 1/9 mm of belt per degree per unit ratio:
    # a 20-tooth GT2 pulley moves 40 mm per turn.
    "b_ratio": 3.0,
    "c_ratio": 5.5,
    # Names of the output axes, one per motor (corexbyc) or per axis (cartesian).
    "output_axes": ["X", "Y", "Z", "U", "V"],
    # Segmentation of moves that change B or C.
    "max_segment_deg": 1.0,
    "max_segment_mm": 1.0,
    # Feed rate cap for the translated moves, mm/min.
    "max_feed": 30000.0,
}

WORD_RE = re.compile(r"([A-Za-z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def load_machine(path):
    machine = dict(DEFAULT_MACHINE)
    if path:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        unknown = set(data) - set(DEFAULT_MACHINE) - {"_comment"}
        if unknown:
            raise ValueError("unknown machine settings: " + ", ".join(sorted(unknown)))
        machine.update({k: v for k, v in data.items() if k != "_comment"})
    if machine["kinematics"] not in ("corexbyc", "cartesian"):
        raise ValueError("kinematics must be 'corexbyc' or 'cartesian'")
    if len(machine["output_axes"]) != 5:
        raise ValueError("output_axes needs five names")
    return machine


def arm_offset(b_deg, m):
    """B-arm displacement of the nozzle tip, as in FiveAxisKinematics (rotationMatrix2, K16)."""
    b = math.radians(b_deg)
    cos_m1, s = math.cos(b) - 1.0, math.sin(b)
    return (
        -s * m["a5"] + cos_m1 * m["d6"],
        cos_m1 * m["b_skew"],
        -cos_m1 * m["a5"] - s * m["d6"],
    )


def bed_to_machine(x, y, z, b_deg, c_deg, m):
    """Bed-frame nozzle position -> machine frame (rotationMatrix1 then rotationMatrix2, K16)."""
    c = math.radians(c_deg)
    cc, sc = math.cos(c), math.sin(c)
    ox, oy, oz = arm_offset(b_deg, m)
    if m["zero_offset_at_home"]:
        hx, hy, hz = arm_offset(m["b_home"], m)
        ox, oy, oz = ox - hx, oy - hy, oz - hz
    return (cc * x + sc * y + ox, -sc * x + cc * y + oy, z + oz)


def machine_to_motors(xm, ym, zm, b_deg, c_deg, m):
    """Machine frame -> the five output coordinates (inverseMatrix of K16 without skews)."""
    if m["kinematics"] == "cartesian":
        return (xm, ym, zm, b_deg, c_deg)
    kb = 4.0 * m["b_ratio"] / 36.0
    kc = 4.0 * m["c_ratio"] / 36.0
    return (
        xm - kb * b_deg,    # motor 0
        -ym - kc * c_deg,   # motor 1
        zm,                 # motor 2
        -xm - kb * b_deg,   # motor 3
        ym - kc * c_deg,    # motor 4
    )


def bed_to_motors(p, m):
    x, y, z, b, c = p
    return machine_to_motors(*bed_to_machine(x, y, z, b, c, m), b, c, m)


def fmt(v, digits=4):
    s = f"{v:.{digits}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


class Translator:
    def __init__(self, machine):
        self.m = machine
        self.pos = [0.0, 0.0, 0.0, machine["b_home"], 0.0]   # bed frame X Y Z B C
        self.known = [False, False, False, True, True]
        self.e = 0.0
        self.feed = 3000.0
        self.abs_xyz = True
        self.abs_e = False
        self.stats = {"moves": 0, "segments": 0, "arcs": 0}

    # --- helpers -------------------------------------------------------------------------------

    def _emit_move(self, cmd, target, e_amount, feed, out):
        """Write one straight move to bed-frame `target`."""
        motors = bed_to_motors(target, self.m)
        words = [cmd]
        for name, value in zip(self.m["output_axes"], motors):
            words.append(name + fmt(value))
        if e_amount is not None:
            words.append("E" + fmt(e_amount, 5))
        if feed is not None:
            words.append("F" + fmt(min(feed, self.m["max_feed"]), 1))
        out.append(" ".join(words))

    def _output_feed(self, start, end, duration):
        """F that makes the move from `start` to `end` (bed frame) last `duration` minutes.
        Klipper times a move on its XYZ distance, or on the extra axes when XYZ does not move."""
        a, b = bed_to_motors(start, self.m), bed_to_motors(end, self.m)
        d_xyz = math.dist(a[:3], b[:3])
        d = d_xyz if d_xyz > 1e-6 else math.dist(a[3:], b[3:])
        if duration <= 0 or d <= 1e-9:
            return None
        return d / duration

    def _move(self, cmd, target, e_word, out):
        start = list(self.pos)
        if not all(self.known[:3]):
            # Position not known yet (start of file, after G28): nothing to interpolate from.
            for i in range(5):
                if target[i] is not None:
                    start[i] = target[i]
        end = [start[i] if target[i] is None else target[i] for i in range(5)]
        for i in range(5):
            if target[i] is not None:
                self.known[i] = True
        self.stats["moves"] += 1

        lin = math.dist(start[:3], end[:3])
        ang = max(abs(end[3] - start[3]), abs(end[4] - start[4]))
        # Same timing rule as RepRapFirmware: F is over the linear distance, or deg/min when
        # only the rotary axes move.
        duration = (lin if lin > 1e-9 else ang) / self.feed if self.feed > 0 else 0.0

        if ang > 1e-9 and all(self.known[:3]):
            n = max(1, math.ceil(ang / self.m["max_segment_deg"]), math.ceil(lin / self.m["max_segment_mm"]))
        else:
            n = 1

        e_total = None
        if e_word is not None:
            e_total = e_word - self.e if self.abs_e else e_word
        prev = start
        for k in range(1, n + 1):
            t = k / n
            p = [s + (q - s) * t for s, q in zip(start, end)]
            e_out = None
            if e_total is not None:
                e_out = (self.e + e_total * t) if self.abs_e else e_total / n
            feed = self._output_feed(prev, p, duration / n)
            self._emit_move(cmd, p, e_out, feed, out)
            prev = p
        self.stats["segments"] += n
        if e_total is not None and self.abs_e:
            self.e += e_total
        self.pos = end

    def _arc(self, clockwise, words, out):
        """Linearise G2/G3 in the bed-frame XY plane (Unlayered turns arc fitting off, but be safe)."""
        self.stats["arcs"] += 1
        x0, y0, z0 = self.pos[0], self.pos[1], self.pos[2]
        x1 = self._axis_target(words, "X", 0)
        y1 = self._axis_target(words, "Y", 1)
        z1 = self._axis_target(words, "Z", 2)
        x1 = x0 if x1 is None else x1
        y1 = y0 if y1 is None else y1
        z1 = z0 if z1 is None else z1
        if "R" in words:
            raise ValueError("G2/G3 with R is not supported, use I/J")
        cx, cy = x0 + words.get("I", 0.0), y0 + words.get("J", 0.0)
        r = math.hypot(x0 - cx, y0 - cy)
        a0 = math.atan2(y0 - cy, x0 - cx)
        a1 = math.atan2(y1 - cy, x1 - cx)
        sweep = a1 - a0
        if clockwise and sweep >= 0:
            sweep -= 2 * math.pi
        elif not clockwise and sweep <= 0:
            sweep += 2 * math.pi
        n = max(1, math.ceil(abs(sweep) * r / 0.5))
        e = words.get("E")
        e0 = self.e
        for k in range(1, n + 1):
            t = k / n
            a = a0 + sweep * t
            px, py = (x1, y1) if k == n else (cx + r * math.cos(a), cy + r * math.sin(a))
            e_k = None
            if e is not None:
                e_k = (e0 + (e - e0) * t) if self.abs_e else e / n
            self._move("G1", [px, py, z0 + (z1 - z0) * t, None, None], e_k, out)

    def _axis_target(self, words, letter, index):
        if letter not in words:
            return None
        v = words[letter]
        return v if self.abs_xyz else self.pos[index] + v

    # --- main loop -----------------------------------------------------------------------------

    def translate_line(self, line, out):
        code, sep, comment = line.partition(";")
        stripped = code.strip()
        if not stripped:
            out.append(line.rstrip("\n"))
            return
        head = stripped.split()[0].upper()
        words = {}
        for letter, value in WORD_RE.findall(stripped[len(head):]):
            words[letter.upper()] = float(value)
        tail = (" ;" + comment.rstrip("\n")) if sep else ""

        if head in ("G0", "G1", "G00", "G01"):
            if "F" in words:
                self.feed = words["F"]
            target = [
                self._axis_target(words, "X", 0),
                self._axis_target(words, "Y", 1),
                self._axis_target(words, "Z", 2),
                self._axis_target(words, "B", 3),
                self._axis_target(words, "C", 4),
            ]
            if all(t is None for t in target):
                out.append(line.rstrip("\n"))   # E-only or F-only move: pass through unchanged
                if "E" in words and self.abs_e:
                    self.e = words["E"]
                return
            cmd = "G0" if head in ("G0", "G00") else "G1"
            if not all(self.known[:3]) and any(target[i] is None for i in range(3)):
                # Without a known X/Y/Z (e.g. right after G28) the motor positions cannot be
                # computed: keep the line for the start G-code to deal with, and track B/C.
                for i in (3, 4):
                    if target[i] is not None:
                        self.pos[i] = target[i]
                for i in range(3):
                    if target[i] is not None:
                        self.pos[i] = target[i]
                        self.known[i] = True
                out.append("; ArtPrint: position unknown, line kept as is")
                out.append(line.rstrip("\n"))
                return
            n0 = len(out)
            self._move(cmd, target, words.get("E"), out)
            if tail and len(out) > n0:
                out[n0] += tail
        elif head in ("G2", "G3", "G02", "G03"):
            if "F" in words:
                self.feed = words["F"]
            self._arc(head in ("G2", "G02"), words, out)
        elif head == "G90":
            self.abs_xyz = True
            out.append(line.rstrip("\n"))
        elif head == "G91":
            self.abs_xyz = False
            out.append(line.rstrip("\n"))
        elif head == "M82":
            self.abs_e = True
            out.append(line.rstrip("\n"))
        elif head == "M83":
            self.abs_e = False
            out.append(line.rstrip("\n"))
        elif head == "G28":
            # After homing the bed-frame position is unknown until the next absolute move.
            self.known = [False, False, False, True, True]
            self.pos[3], self.pos[4] = self.m["b_home"], 0.0
            out.append(line.rstrip("\n"))
        elif head == "G92":
            if "E" in words:
                self.e = words["E"]
            moved = [k for k in "XYZBC" if k in words]
            if moved:
                for i, k in enumerate("XYZBC"):
                    if k in words:
                        self.pos[i] = words[k]
                        self.known[i] = True
                motors = bed_to_motors(self.pos, self.m)
                w = ["G92"] + [n + fmt(v) for n, v in zip(self.m["output_axes"], motors)]
                if "E" in words:
                    w.append("E" + fmt(words["E"], 5))
                out.append(" ".join(w) + tail)
            else:
                out.append(line.rstrip("\n"))
        else:
            out.append(line.rstrip("\n"))

    def translate(self, lines):
        out = [
            "; ArtPrint: translated from Unlayered bed-frame G-code to motor coordinates",
            "; kinematics=%s a5=%s d6=%s b_ratio=%s c_ratio=%s axes=%s" % (
                self.m["kinematics"], self.m["a5"], self.m["d6"], self.m["b_ratio"],
                self.m["c_ratio"], "".join(self.m["output_axes"])),
        ]
        for line in lines:
            self.translate_line(line, out)
        return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("input")
    parser.add_argument("-o", "--output", help="output file (default: rewrite the input)")
    parser.add_argument("--machine", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "machine.json"),
                        help="machine settings JSON (default: machine.json next to this script)")
    args = parser.parse_args(argv)

    machine = load_machine(args.machine if os.path.exists(args.machine) else None)
    with open(args.input, encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    tr = Translator(machine)
    out = tr.translate(lines)
    target = args.output or args.input
    tmp = target + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    os.replace(tmp, target)
    print("ArtPrint: %d moves -> %d lines of motion (%d arcs linearised) written to %s" % (
        tr.stats["moves"], tr.stats["segments"], tr.stats["arcs"], target), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
