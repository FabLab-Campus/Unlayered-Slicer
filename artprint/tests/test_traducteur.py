"""Tests for artprint/traducteur/unlayered_to_klipper.py. Run: python -m unittest discover artprint/tests"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "traducteur"))
import unlayered_to_klipper as t  # noqa: E402

COS, SIN, NCOS, NSIN = 2, 3, 4, 5


def firmware_reference(x, y, z, b, c, a5=2.5, d6=46.4, bskew=0.0, bratio=3.0, cratio=5.5):
    """Straight transcription of FiveAxisKinematics::CartesianToMotorSteps for M669 K16
    (matrix loops kept as in the C++), used to check the translator's closed-form maths."""
    n = 5
    r1 = [[1 if i == j else 0 for j in range(n)] for i in range(n)]
    r1[0][0], r1[0][1], r1[1][0], r1[1][1] = COS, SIN, NSIN, COS
    r2 = [[0] * 3 for _ in range(n)]
    r2[0][0], r2[0][2], r2[1][1], r2[2][0], r2[2][2] = NSIN, COS, COS, NCOS, NSIN

    def val(code, cs, sn):
        return {1: 1.0, COS: cs, SIN: sn, NCOS: -cs, NSIN: -sn}.get(code, 0.0)

    inv = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    inv[0][0], inv[0][3], inv[1][1], inv[2][2], inv[1][4] = 1.0, -1.0, -1.0, 1.0, 1.0
    inv[3][0] = inv[3][3] = -4.0 * bratio / 36.0
    inv[4][1] = inv[4][4] = -4.0 * cratio / 36.0

    pos = [x, y, z, b, c]
    c5, s5 = math.cos(math.radians(b)), math.sin(math.radians(b))
    c1, s1 = math.cos(math.radians(c)), math.sin(math.radians(c))
    rot = [0.0] * n
    for i in range(n):
        for j in range(n):
            rot[i] += val(r1[i][j], c1, s1) * pos[j]
        rot[i] += val(r2[i][0], c5 - 1, s5) * a5
        rot[i] += val(r2[i][1], c5 - 1, s5) * bskew
        rot[i] += val(r2[i][2], c5 - 1, s5) * d6
    return [sum(inv[a][m] * rot[a] for a in range(n)) for m in range(n)]


def machine(**kw):
    m = dict(t.DEFAULT_MACHINE)
    m.update(kw)
    return m


class Kinematics(unittest.TestCase):
    def test_matches_firmware(self):
        m = machine(zero_offset_at_home=False)
        for p in [(10, 20, 5, -90, 0), (35.5, -12, 40, -45, 30), (-60, 70, 12, -120, -170), (0, 0, 0, 0, 0)]:
            got = t.bed_to_motors(p, m)
            ref = firmware_reference(*p)
            for g, r in zip(got, ref):
                self.assertAlmostEqual(g, r, places=9, msg=str(p))

    def test_home_pose_is_identity(self):
        m = machine(kinematics="cartesian")
        self.assertEqual(tuple(round(v, 9) for v in t.bed_to_motors((12, -7, 3, -90, 0), m)), (12, -7, 3, -90, 0))

    def test_bed_rotation(self):
        m = machine(kinematics="cartesian")
        x, y, _, _, _ = t.bed_to_motors((10, 0, 0, -90, 90), m)
        self.assertAlmostEqual(x, 0, places=9)
        self.assertAlmostEqual(y, -10, places=9)

    def test_corexbyc_pure_x_move(self):
        m = machine()
        a = t.bed_to_motors((0, 0, 0, -90, 0), m)
        b = t.bed_to_motors((10, 0, 0, -90, 0), m)
        self.assertAlmostEqual(b[0] - a[0], 10)
        self.assertAlmostEqual(b[3] - a[3], -10)
        self.assertAlmostEqual(b[1] - a[1], 0)


class Translation(unittest.TestCase):
    def run_lines(self, text, **kw):
        tr = t.Translator(machine(**kw))
        return tr.translate(text.strip().splitlines())

    def moves(self, out):
        return [l for l in out if l.startswith(("G0", "G1"))]

    def test_constant_angles_not_split(self):
        out = self.run_lines("M83\nG1 X0 Y0 Z0.2 F3000\nG1 X50 Y0 E2 F1200", kinematics="cartesian")
        self.assertEqual(len(self.moves(out)), 2)
        self.assertIn("E2", out[-1])

    def test_rotation_change_is_split_and_e_preserved(self):
        out = self.run_lines("M83\nG1 X10 Y0 Z1 F3000\nG1 X10 Y0 Z1 C90 E9 F600", kinematics="cartesian")
        segs = self.moves(out)[1:]
        self.assertEqual(len(segs), 90)
        e = sum(float(s.split(" E")[1].split()[0]) for s in segs)
        self.assertAlmostEqual(e, 9, places=4)
        last = segs[-1].split()
        self.assertEqual(last[1], "X0")
        self.assertEqual(last[2], "Y-10")

    def test_absolute_e(self):
        out = self.run_lines("M82\nG92 E0\nG1 X0 Y0 Z1 F3000\nG1 X0 Y0 Z1 B-80 E5", kinematics="cartesian")
        self.assertIn("E5", self.moves(out)[-1])

    def test_arc_endpoint(self):
        out = self.run_lines("M83\nG1 X10 Y0 Z1 F3000\nG3 X0 Y10 I-10 J0 E1", kinematics="cartesian")
        last = self.moves(out)[-1].split()
        self.assertEqual(last[1], "X0")
        self.assertEqual(last[2], "Y10")
        e = sum(float(s.split(" E")[1].split()[0]) for s in self.moves(out)[1:])
        self.assertAlmostEqual(e, 1, places=4)

    def test_unknown_position_line_kept(self):
        out = self.run_lines("G28\nG1 B-90 C0 F3600\nG1 X1 Y2 Z3", kinematics="cartesian")
        self.assertIn("G1 B-90 C0 F3600", out)
        self.assertEqual(self.moves(out)[-1].split()[1:4], ["X1", "Y2", "Z3"])

    def test_comments_and_other_lines_kept(self):
        out = self.run_lines("; hello\nM104 S200\nG1 X1 Y1 Z1 ; travel")
        self.assertIn("; hello", out)
        self.assertIn("M104 S200", out)
        self.assertTrue(out[-1].endswith("; travel"))


if __name__ == "__main__":
    unittest.main()
