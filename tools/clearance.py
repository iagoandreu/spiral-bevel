"""Swept-volume clearance of the spiral bevel pair against the two helical gears.

    py -3.10 tools/clearance.py

Gears that are not meant to mesh must keep their SWEPT volumes apart - what a
gear occupies over a full turn is a solid of revolution, so the test is
axisymmetric and needs only tooth 0. Each gear's swept outline comes from
sb_math's own tooth sections and blank; each helical gear is its tip cylinder.

Layout assumed (the "context" pair, from the screenshot):
    A  the helical gear the pinion continues: axis z, end face at z = 0, body
       from z = -thickness to 0, tip radius d/2 + mn.
    B  the crossed helical it drives at 90 deg: same gear, centre distance
       (dA + dB)/2 on the x axis, its axis along y, its mid-plane through the
       middle of A's face width.
Distances are signed: negative means the swept volumes overlap by that much.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, 'tests', 'stub'))
sys.path.insert(0, HERE)

import sb_math as M                                          # noqa: E402

THICK = 0.5          # helical gear thickness, mm (screenshot)
N_ANG = 360


def swept(G, which, origin, fx, fy, fz, neck):
    return M.swept_outline(G, which, neck), (origin, fx, fy, fz)


def layout(G, gap, position_deg):
    """Pinion and gear frames in world mm, A's end face at z = 0."""
    z_heel = M.heel_face_z(G, 'pinion', neck=gap)
    apex = (0.0, 0.0, -z_heel)
    fr = M.mesh_frame(G, math.radians(position_deg))
    return apex, fr


def helicals(G):
    r_tip = 0.5 * G['pinion']['d'] + G['mn']
    A = ((0.0, 0.0, -0.5 * THICK), (0.0, 0.0, 1.0), r_tip, 0.5 * THICK)
    B = ((G['pinion']['d'], 0.0, -0.5 * THICK), (0.0, 1.0, 0.0), r_tip,
         0.5 * THICK)
    return A, B


def pinion_vs_B(G, gap):
    apex, _fr = layout(G, gap, 0.0)
    _A, B = helicals(G)
    out, fr = swept(G, 'pinion', apex, (1, 0, 0), (0, 1, 0), (0, 0, 1), gap)
    return M.swept_clearance(out, fr, B, N_ANG)


def gear_clearances(G, gap, position_deg):
    apex, fr = layout(G, gap, position_deg)
    A, B = helicals(G)
    out, f = swept(G, 'gear', apex, fr['x'], fr['y'], fr['z'], 0.0)
    return (M.swept_clearance(out, f, A, N_ANG), M.swept_clearance(out, f, B, N_ANG))


def min_gap(G, margin=0.0):
    """Smallest neck that keeps the pinion's swept volume `margin` clear of B."""
    if pinion_vs_B(G, 0.0) >= margin:
        return 0.0
    lo, hi = 0.0, 5.0
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        if pinion_vs_B(G, mid) >= margin:
            hi = mid
        else:
            lo = mid
    return hi


def main():
    print('pinion vs B (crossed helical), gear vs A and B; mm, negative = overlap')
    print()
    print(' cone  z2   pinion-B at gap 0   gap needed   gear-A   gear-B by position (deg)')
    positions = (0, 45, 90, 135, 180, 225, 270, 315)
    print(' ' * 58 + '  '.join('%5d' % p for p in positions))
    for d1 in (20, 30, 40, 45, 50, 60, 70):
        G = M.design({'by': 'cone', 'delta1': d1})
        c0 = pinion_vs_B(G, 0.0)
        g = min_gap(G)
        cells = []
        for pos in positions:
            ca, cb = gear_clearances(G, g, pos)
            cells.append('%5.2f' % cb)
        ca, _cb = gear_clearances(G, g, 180)
        print('%5.1f %3d   %8.3f          %8.3f    %6.3f   %s'
              % (math.degrees(G['pinion']['delta']), G['z2'], c0, g, ca,
                 '  '.join(cells)))


if __name__ == '__main__':
    main()
