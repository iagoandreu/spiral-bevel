"""Offline tests for the pair geometry. No Fusion.

    py -3.10 tests/test_math.py

Every expected value is worked out independently of the code under test - from
the screenshot's gear, from gear-geometry identities, or by measuring the built
3D points - never recorded from a previous run.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import sb_math as M                                         # noqa: E402

FAILED = []


def check(name, cond, detail=''):
    print('  %s %s%s' % ('ok  ' if cond else 'FAIL', name,
                         '' if cond else '   %s' % (detail,)))
    if not cond:
        FAILED.append(name)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --------------------------------------------------------------------------- #
print('the pair, from the helical gear in the screenshot')
# --------------------------------------------------------------------------- #

G = M.design()
check('transverse module 0.1414 / cos 45 = 0.2 mm', close(G['mt'], 0.19997, 1e-4),
      G['mt'])
check('pinion pitch diameter = helical pitch diameter, 25 x 0.2 = 5 mm',
      close(G['pinion']['d'], 25 * G['mt'], 1e-12) and close(G['pinion']['d'],
                                                              5.0, 2e-3))
check('a 45 deg pinion on a 90 deg shaft angle is a mitre pair, z2 = 25',
      G['z2'] == 25, G['z2'])
check('and both cones are 45 deg',
      close(G['pinion']['delta'], math.pi / 4, 1e-12)
      and close(G['gear']['delta'], math.pi / 4, 1e-12))
check('cone distance Re = r1 / sin(delta1)',
      close(G['Re'], 2.5 / math.sin(math.pi / 4), 2e-3), G['Re'])
check('face width defaults to Re / 3', close(G['face'], G['Re'] / 3.0))

for d1, sig in ((30.0, 90.0), (60.0, 90.0), (35.0, 75.0), (50.0, 110.0)):
    Gs = M.design({'by': 'cone', 'delta1': d1, 'sigma': sig})
    p, g = Gs['pinion'], Gs['gear']
    check('d1 %g, shaft %g: the cones add up to the shaft angle' % (d1, sig),
          close(p['delta'] + g['delta'], math.radians(sig), 1e-12))
    check('d1 %g, shaft %g: both pitch circles lie on one cone distance'
          % (d1, sig),
          close(p['r'] / math.sin(p['delta']), g['r'] / math.sin(g['delta']),
                1e-9))
    check('d1 %g, shaft %g: rounding z2 moves the cone angle less than half a '
          'tooth would' % (d1, sig),
          abs(math.degrees(p['delta']) - d1) < 2.5,
          (math.degrees(p['delta']), Gs['z2']))

GT = M.design({'z1': 12, 'z2': 25})
check('by teeth, 12 : 25 on 90 deg: tan(delta1) = 12/25',
      GT['z1'] == 12 and GT['z2'] == 25
      and close(GT['pinion']['delta'], math.atan2(12, 25), 1e-12),
      math.degrees(GT['pinion']['delta']))
check('and the gear cone is the rest of the shaft angle',
      close(GT['gear']['delta'], math.pi / 2 - math.atan2(12, 25), 1e-12))
GT = M.design({'z1': 12, 'z2': 25, 'sigma': 75.0})
check('by teeth on 75 deg: both pitch circles on one cone distance',
      close(GT['pinion']['r'] / math.sin(GT['pinion']['delta']),
            GT['gear']['r'] / math.sin(GT['gear']['delta']), 1e-9))
check('the default is by teeth, 25 : 25', M.design()['z2'] == 25
      and M.DEFAULTS['by'] == 'teeth')
check('90 deg with a 30 deg pinion: z2 = round(25 cot 30) = 43',
      M.design({'by': 'cone', 'delta1': 30.0})['z2'] == 43)

for bad in ({'by': 'cone', 'delta1': 2.0}, {'by': 'cone', 'delta1': 89.0},
            {'z2': 3}, {'mn': 0.0}, {'z1': 3},
            {'face': 5.0}):
    try:
        M.design(bad)
        check('refuses %s' % bad, False, 'no error')
    except M.GearError as exc:
        check('refuses %s' % bad, True, exc)


# --------------------------------------------------------------------------- #
print('the tooth in one section')
# --------------------------------------------------------------------------- #

p = G['pinion']
sec = M.section_2d(G, p, 1.0)
# at the pitch radius the half thickness must be theta_p: find it on the flank
rv = sec['rv']
pts = [(math.hypot(u + rv, v), math.atan2(v, u + rv)) for u, v in sec['right']]
for (r0, t0), (r1, t1) in zip(pts, pts[1:]):
    if r0 <= rv <= r1:
        th = t0 + (t1 - t0) * (rv - r0) / (r1 - r0)
        break
want = (0.5 * math.pi * G['mt'] - 0.5 * G['backlash']) / (2.0 * rv)
check('tooth thickness at the pitch circle is half a pitch less half the '
      'backlash', close(th, want, 2e-5), (th, want))
check('the tip is one addendum above the pitch point',
      close(sec['tip'][0], G['ha'], 1e-12) and not sec['pointed'])
check('the root is one dedendum below it', close(sec['rf'], -G['hf'], 1e-12))
check('flank radii rise monotonically root to tip',
      all(b_[0] > a_[0] for a_, b_ in zip(pts, pts[1:])))
half = M.section_2d(G, p, 0.5)
check('at half the cone distance every dimension is half',
      close(half['tip'][0], 0.5 * sec['tip'][0], 1e-12)
      and close(half['right'][-1][1], 0.5 * sec['right'][-1][1], 1e-12))


# --------------------------------------------------------------------------- #
print('the spiral')
# --------------------------------------------------------------------------- #

for which in ('pinion', 'gear'):
    g = G[which]
    worst = 0.0
    for k in range(1, 20):
        R = G['Re'] - G['face'] * k / 20.0
        h = 1e-6
        a = M.tooth_centre(G, which, R - h)
        b_ = M.tooth_centre(G, which, R + h)
        tang = M.unit(M.sub(b_, a))
        gen = M.unit(M.tooth_centre(G, which, R))
        ang = math.acos(max(-1.0, min(1.0, M.dot(tang, gen))))
        worst = max(worst, abs(ang - G['beta']))
    check('%s: the tooth line keeps the spiral angle 45 deg to the generator '
          'across the whole face' % which, worst < 1e-6,
          '%.2e rad off' % worst)

# continuation of a RIGHT hand helix: moving along +z (away from the helical
# gear, towards the toe) the azimuth about +z must increase, at tan(beta)/r per
# unit length along the tooth's projection on the axis direction of travel
R = G['Re']
phi0 = M.spiral_psi(G, p, R)
phi1 = M.spiral_psi(G, p, R - 1e-6)
check('right hand: from heel to toe the tooth turns positively about +z, like '
      'the helix it continues', phi1 > phi0)
rate_slant = (phi1 - phi0) / 1e-6
check('at the rate tan(beta)/r per unit slant, the helix\'s per unit length',
      close(rate_slant, math.tan(G['beta']) / p['r'], 1e-6),
      (rate_slant, math.tan(G['beta']) / p['r']))
GL = M.design({'hand': M.LEFT})
check('left hand turns the other way',
      M.spiral_psi(GL, GL['pinion'], R - 1e-3) < 0.0)


# --------------------------------------------------------------------------- #
print('the gear against the pinion, in 3D')
# --------------------------------------------------------------------------- #


def rolling_errors(G, contact_phi, flip_hand=False):
    """Rolling without slip, checked exactly, at eleven cone distances.

    At cone distance R the pitch cones roll on each other along the contact
    generator. Walk from it round each pitch circle to the nearest pinion tooth
    centre, and to the nearest gear tooth-space centre: the two arcs must be the
    same arc in space (same length, same side) modulo the circular pitch at R.
    Comparing 3D distances instead would compare two different cones, which part
    quadratically off the generator whatever the hand - that measures curvature,
    not mesh.
    """
    fr = M.mesh_frame(G, contact_phi)
    if flip_hand:
        G['gear']['hs'] = -G['gear']['hs']
    p, g = G['pinion'], G['gear']
    pitch1 = 2.0 * math.pi / p['z']
    pitch2 = 2.0 * math.pi / g['z']
    contact_gear = -fr['gamma']            # gear-local azimuth of the contact
    errs = []
    for i in range(11):
        R = G['Re'] - G['face'] * i / 10.0
        a1 = p['r'] * R / G['Re'] * M.wrap(
            M.spiral_psi(G, p, R) - contact_phi, pitch1)
        a2 = g['r'] * R / G['Re'] * M.wrap(
            M.spiral_psi(G, g, R) + 0.5 * pitch2 - contact_gear, pitch2)
        circ = math.pi * G['mt'] * R / G['Re']
        errs.append(abs(M.wrap(a1 - fr['sign'] * a2, circ)))
    return errs, fr


def heel_tangents(G):
    """Angle between the pinion tooth line and the gear space line where both
    cross the contact generator at the heel - contact_phi 0 puts them there."""
    fr = M.mesh_frame(G, 0.0)
    h = 1e-6
    Re = G['Re']
    t1 = M.unit(M.sub(M.tooth_centre(G, 'pinion', Re),
                      M.tooth_centre(G, 'pinion', Re - h)))
    k = [k for k in range(G['z2'])
         if M.norm(M.sub(M.local_to(fr, M.space_centre(G, 'gear', Re, k)),
                         M.tooth_centre(G, 'pinion', Re))) < 1e-9]
    if not k:
        return None, None
    t2 = M.unit(M.sub(M.local_to(fr, M.space_centre(G, 'gear', Re, k[0])),
                      M.local_to(fr, M.space_centre(G, 'gear', Re - h, k[0]))))
    return math.degrees(math.acos(max(-1.0, min(1.0, M.dot(t1, t2))))), k[0]


for d1, sig, phi in ((45.0, 90.0, 0.0), (30.0, 90.0, 0.37), (60.0, 90.0, 1.9),
                     (40.0, 75.0, -0.8), (50.0, 110.0, 2.6), ('12:25', 90.0, 0.2)):
    if d1 == '12:25':
        Gm = M.design({'z1': 12, 'z2': 25, 'sigma': sig})
        errs, fr = rolling_errors(Gm, phi)
        wrong, _ = rolling_errors(M.design({'z1': 12, 'z2': 25}), phi,
                                  flip_hand=True)
        check('12 : 25 by teeth: rolls without slip (worst %.1e mm; wrong hand '
              '%.3f mm)' % (max(errs), max(wrong)),
              max(errs) < 1e-9 and max(wrong) > 0.05)
        continue
    Gm = M.design({'by': 'cone', 'delta1': d1, 'sigma': sig})
    errs, fr = rolling_errors(Gm, phi)
    check('d1 %g shaft %g: the gear axis is at the shaft angle to the pinion axis'
          % (d1, sig),
          close(math.degrees(math.acos(M.dot(fr['z'], (0.0, 0.0, 1.0)))), sig,
                1e-9))
    wrong, _ = rolling_errors(M.design({'by': 'cone', 'delta1': d1, 'sigma': sig}), phi,
                              flip_hand=True)
    check('d1 %g shaft %g: the pitch cones roll without slip across the whole '
          'face (worst %.1e mm; the wrong hand slips %.3f mm)'
          % (d1, sig, max(errs), max(wrong)),
          max(errs) < 1e-9 and max(wrong) > 0.05, (errs, wrong))
    ang, k = heel_tangents(M.design({'by': 'cone', 'delta1': d1, 'sigma': sig}))
    check('d1 %g shaft %g: at the heel contact a gear space sits on the pinion '
          'tooth, and their tooth lines are parallel' % (d1, sig),
          ang is not None and ang < 1e-4, ang)


# --------------------------------------------------------------------------- #
print('sections and blank')
# --------------------------------------------------------------------------- #

secs = M.tooth_sections(G, 'pinion')
check('nine sections, toe to heel',
      len(secs) == 9 and secs[0]['R'] < secs[-1]['R'])
ok = all(s['right'][-1] == s['tip'][0] and s['tip'][-1] == s['left'][0]
         and s['left'][-1] == s['bottom'][0] and s['bottom'][-1] == s['right'][0]
         for s in secs)
check('every section outline closes, piece to piece', ok)
heel = secs[-1]
tipd = math.hypot(heel['tip'][1][0], heel['tip'][1][1]) * 2.0
check('the heel tip diameter is d + 2 ha cos(delta)',
      close(tipd, p['d'] + 2.0 * G['ha'] * math.cos(p['delta']), 1e-9),
      (tipd, p['d'] + 2.0 * G['ha'] * math.cos(p['delta'])))
bl = M.blank_profile(G, 'pinion', neck=0.3)
check('the blank starts and ends on the axis', bl[0][0] == 0.0 and bl[-1][0] == 0.0)
check('the neck is as long as asked',
      close(bl[-2][1] - bl[-3][1], -0.3, 1e-12))
check('and no wider than the helical root less 0.25 mn clearance, 2.5 - 1.5 mn',
      close(bl[-2][0], 0.5 * p['d'] - 1.5 * G['mn'], 1e-12), bl[-2][0])
lowest = min(z for s in secs for pt in s['bottom'] for z in [pt[2]])
check('no tooth reaches below the heel face',
      lowest >= M.heel_face_z(G, 'pinion') - 1e-12,
      (lowest, M.heel_face_z(G, 'pinion')))


# --------------------------------------------------------------------------- #
print('swept clearance')
# --------------------------------------------------------------------------- #

CYL = ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 2.0, 1.0)
check('a point outside the side of a cylinder is its radial distance away',
      close(M.cyl_distance((3.5, 0.0, 0.3), CYL), 1.5))
check('past an end face, the axial distance',
      close(M.cyl_distance((1.0, 0.0, 1.75), CYL), 0.75))
check('past the rim, the distance to the rim circle',
      close(M.cyl_distance((5.0, 0.0, 5.0), CYL), 5.0))
check('inside, negative by the depth to the nearest wall',
      close(M.cyl_distance((1.5, 0.0, 0.0), CYL), -0.5))
# a ring of radius 1 at z = 3, revolved about z, is 2 above the cylinder's top;
# the same ring tilted onto its side about x reaches down to z = 2 at radius 1
ring = [(1.0, 0.0)]
up = ((0.0, 0.0, 3.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
check('a revolved ring above the cylinder clears it by the gap',
      close(M.swept_clearance(ring, up, CYL, 360), 2.0, 1e-12))
side = ((0.0, 0.0, 3.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))
check('and turned on its side it comes down to the nearest point of its circle',
      close(M.swept_clearance(ring, side, CYL, 360), 1.0, 1e-12))

print()
if FAILED:
    print('%d FAILED: %s' % (len(FAILED), ', '.join(FAILED)))
    raise SystemExit(1)
print('all tests passed')
