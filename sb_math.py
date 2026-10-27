"""Spiral bevel pair geometry. NO adsk IMPORTS, NO numpy - stdlib only, so every
number here can be proved with `py -3.10 tests/test_math.py` and no CAD in the loop.

Units are MILLIMETRES and radians throughout. sb_core converts to Fusion's
centimetres at the boundary, in one place.

WHAT IS BEING MADE
    A pinion that is the conical continuation of an existing helical gear, and the
    bevel gear it drives through the shaft angle SIGMA. The pinion keeps the
    helical gear's normal module, normal pressure angle and helix angle (as its
    spiral angle, same hand). With the same tooth count its heel pitch diameter
    is the helical pitch diameter and the teeth line up tooth for tooth.

    The pair is defined either by BOTH tooth counts (by = 'teeth'; the cone
    angles follow from the ratio and the shaft angle, tan d1 = sin S / (z2/z1 +
    cos S)), or by the pinion's count and cone angle (by = 'cone'; then z2 =
    z1 (sin S / tan d1 - cos S), rounded, and the cone angle actually built is
    recomputed from that whole number).

TOOTH FORM - Tredgold, on sections, tapered depth
    Each transverse section is taken in the plane perpendicular to the pitch-cone
    generator through the tooth centre, which is the plane tangent to the back cone
    there. In it the tooth is the involute of the VIRTUAL spur gear of radius
    r / cos(delta), with the transverse pressure angle of a helical tooth of the
    same spiral angle. That is Tredgold's approximation; its error is second order
    in the tooth's half width over the virtual radius - about 3 % of a module at
    this size, well under a print's resolution.

    Every dimension scales with the cone distance R (depth tapers to the apex), so
    the root and face cones share the pitch apex.

SPIRAL - constant spiral angle, i.e. a logarithmic spiral
    The tooth line keeps the same angle beta to the pitch-cone generator at every
    cone distance. Developed flat that is a logarithmic spiral, and on the cone the
    tooth centre sits at azimuth

        psi(R) = -hand * tan(beta) * ln(R / Re) / sin(delta)

    Its azimuth rate per unit slant, tan(beta) / r, is the helix's per unit axial
    length - which is what makes it a continuation of the helical tooth. The
    usual Gleason face-milled arc has a spiral angle that varies across the face;
    it was not used because it cannot match a constant helix.

LOCAL FRAME of every gear
    Apex at the origin, the cone opening towards -z (heel at the most negative z),
    tooth 0 centred at azimuth 0 AT THE HEEL. +z therefore points from the heel to
    the apex, and for the pinion that is AWAY from the helical gear.
"""

import math

RIGHT = 1
LEFT = -1


class GearError(Exception):
    """The parameters cannot make the pair asked for."""


# --------------------------------------------------------------------------- #
# vectors - plain tuples
# --------------------------------------------------------------------------- #

def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    if n < 1e-300:
        raise GearError('zero-length vector')
    return scale(a, 1.0 / n)


def rotate(v, axis, ang):
    """Rodrigues: v rotated by ang about the unit axis."""
    c, s = math.cos(ang), math.sin(ang)
    return add(add(scale(v, c), scale(cross(axis, v), s)),
               scale(axis, dot(axis, v) * (1.0 - c)))


def inv(a):
    """The involute function."""
    return math.tan(a) - a


# --------------------------------------------------------------------------- #
# the pair
# --------------------------------------------------------------------------- #

DEFAULTS = {
    'mn': 0.1414,          # normal module at the heel, mm
    'alpha_n': 25.0,       # normal pressure angle, deg
    'beta': 45.0,          # spiral angle = the helix angle it continues, deg
    'hand': RIGHT,         # the helical gear's hand; the pinion keeps it
    'z1': 25,
    'z2': 25,              # gear teeth - used when by == 'teeth'
    'by': 'teeth',         # 'teeth': z1 and z2 given; 'cone': z1 and delta1
    'delta1': 45.0,        # requested pinion pitch cone angle, deg
    'sigma': 90.0,         # shaft angle, deg
    'face': 0.0,           # face width, mm; 0 = Re / 3
    'backlash': 0.02,      # total circular backlash at the heel, mm
    'ha': 1.0,             # addendum, in normal modules
    'hf': 1.25,            # dedendum, in normal modules
    'gap': 0.0,            # helical end face to the pinion's heel face, mm
}


def design(p=None):
    """Everything about the pair that does not depend on where it is put."""
    q = dict(DEFAULTS)
    q.update(p or {})
    mn = float(q['mn'])
    an = math.radians(float(q['alpha_n']))
    beta = math.radians(float(q['beta']))
    sig = math.radians(float(q['sigma']))
    z1 = int(q['z1'])
    d1_req = math.radians(float(q['delta1']))
    hand = RIGHT if int(q['hand']) >= 0 else LEFT
    if mn <= 0.0:
        raise GearError('the module must be greater than zero')
    if z1 < 5:
        raise GearError('the pinion needs at least 5 teeth')
    if not 0.0 <= beta < math.radians(70.0):
        raise GearError('the spiral angle must be between 0 and 70 deg')
    if not math.radians(10.0) <= sig <= math.radians(170.0):
        raise GearError('the shaft angle must be between 10 and 170 deg')
    mt = mn / math.cos(beta)
    d1 = z1 * mt
    by = str(q.get('by', 'teeth'))
    if by == 'teeth':
        # both counts chosen: the cones follow from the ratio and the shaft
        z2 = int(q['z2'])
        if z2 < 5:
            raise GearError('the gear needs at least 5 teeth')
        d1_req = math.atan2(math.sin(sig), z2 / float(z1) + math.cos(sig))
    else:
        if not 0.0 < d1_req < sig:
            raise GearError('the pinion cone angle must lie between 0 and the '
                            'shaft angle')
        u_req = math.sin(sig) / math.tan(d1_req) - math.cos(sig)
        z2 = int(round(u_req * z1))
        if z2 < 5:
            raise GearError('that cone angle leaves the gear %d teeth - make the '
                            'pinion cone angle smaller' % z2)
    delta1 = math.atan2(math.sin(sig), z2 / float(z1) + math.cos(sig))
    delta2 = sig - delta1
    for dl, name in ((delta1, 'pinion'), (delta2, 'gear')):
        if not math.radians(5.0) < dl < math.radians(85.0):
            raise GearError('the %s cone angle comes out at %.1f deg; it has to '
                            'stay between 5 and 85 deg (a crown or near-crown '
                            'gear has no virtual spur gear to take a profile from)'
                            % (name, math.degrees(dl)))
    Re = d1 / (2.0 * math.sin(delta1))
    face = float(q['face'])
    if face <= 0.0:
        face = Re / 3.0
    if face >= 0.6 * Re:
        raise GearError('a face width of %.3f mm is more than 0.6 of the cone '
                        'distance (%.3f mm) - the toe would be too small'
                        % (face, Re))

    at = math.atan(math.tan(an) / math.cos(beta))
    ha = float(q['ha']) * mn
    hf = float(q['hf']) * mn

    def gear(z, delta, hs):
        return {'z': z, 'delta': delta, 'hs': hs,
                'd': z * mt, 'r': 0.5 * z * mt,
                'rv': 0.5 * z * mt / math.cos(delta),
                'zv': z / math.cos(delta)}

    return {
        'mn': mn, 'mt': mt, 'alpha_n': an, 'alpha_t': at, 'beta': beta,
        'sigma': sig, 'hand': hand, 'z1': z1, 'z2': z2,
        'delta1_requested': d1_req, 'Re': Re, 'face': face,
        'face_recommended': Re / 3.0, 'ha': ha, 'hf': hf,
        'theta_a': math.atan(ha / Re), 'theta_f': math.atan(hf / Re),
        'backlash': float(q['backlash']), 'gap': float(q['gap']),
        # hs is the spiral's sign in the gear's own frame. The pinion's follows
        # the helical hand; the gear's is set by mesh_frame(), which is the only
        # place that knows how the two frames sit against each other.
        'pinion': gear(z1, delta1, -hand),
        'gear': gear(z2, delta2, hand),
        'station_count': 9,
    }


# --------------------------------------------------------------------------- #
# one tooth
# --------------------------------------------------------------------------- #

def spiral_psi(G, g, R):
    """Azimuth of the tooth centre at cone distance R (0 at the heel)."""
    return (g['hs'] * math.tan(G['beta']) * math.log(R / G['Re'])
            / math.sin(g['delta']))


def section_2d(G, g, s, flank_points=12):
    """The tooth at scale s = R / Re, in its section plane.

    u runs along the tooth depth, measured from the pitch point, outward; v runs
    round the gear. Returns the right flank root->tip (v > 0), the tip point on
    the centreline, and the bottom depth the closing lines drop to - below the
    root, so the tooth overlaps the blank and joins it cleanly.
    """
    rv = g['rv'] * s
    at = G['alpha_t']
    rb = rv * math.cos(at)
    ra = rv + G['ha'] * s
    rf = rv - G['hf'] * s
    t = 0.5 * math.pi * G['mt'] * s - 0.5 * G['backlash'] * s
    if t <= 0.0:
        raise GearError('the backlash is larger than the tooth')
    theta_p = t / (2.0 * rv)

    def half(rho):
        return theta_p + inv(at) - inv(math.acos(min(1.0, rb / rho)))

    pointed = False
    if half(ra) <= 0.0:
        # the flanks meet below the tip circle: stop the tooth where they do
        lo, hi = max(rb, rf), ra
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if half(mid) > 0.0:
                lo = mid
            else:
                hi = mid
        ra = lo
        pointed = True

    rho0 = max(rb, rf)
    # spaced evenly in roll angle, which puts the points where the involute bends
    e0 = math.sqrt(max(0.0, (rho0 / rb) ** 2 - 1.0))
    e1 = math.sqrt(max(0.0, (ra / rb) ** 2 - 1.0))
    right = []
    if rf < rb:
        right.append((rf, half(rb)))            # radial line below the base circle
    for i in range(flank_points):
        e = e0 + (e1 - e0) * i / (flank_points - 1)
        rho = rb * math.sqrt(1.0 + e * e)
        right.append((rho, half(rho)))
    pts = [(rho * math.cos(th) - rv, rho * math.sin(th)) for rho, th in right]
    return {'right': pts, 'tip': (ra - rv, 0.0), 'rf': rf - rv,
            'bottom': (rf - rv) - 0.25 * G['hf'] * s, 'pointed': pointed,
            'rv': rv}


def frame_at(g, R, phi):
    """Pitch point, depth direction and round direction at cone distance R and
    azimuth phi, in the gear's local frame."""
    sd, cd = math.sin(g['delta']), math.cos(g['delta'])
    cp, sp = math.cos(phi), math.sin(phi)
    gen = (sd * cp, sd * sp, -cd)
    e_h = (cd * cp, cd * sp, sd)
    e_t = (-sp, cp, 0.0)
    return scale(gen, R), e_h, e_t


def to_local(P, e_h, e_t, uv):
    return add(P, add(scale(e_h, uv[0]), scale(e_t, uv[1])))


def tooth_sections(G, which):
    """Every section of tooth 0, toe to heel, as closed 3D outlines in the gear's
    local frame. Each outline is a dict of polylines that share end points:

        right   right flank, root -> tip
        tip     three points, right tip -> centre -> left tip
        left    left flank, tip -> root
        bottom  left root -> down -> across -> up -> right root
    """
    g = G[which]
    Re, b = G['Re'], G['face']
    n = G['station_count']
    out = []
    for k in range(n):
        R = (Re - b) + b * k / (n - 1)
        s = R / Re
        sec = section_2d(G, g, s)
        P, e_h, e_t = frame_at(g, R, spiral_psi(G, g, R))
        right = [to_local(P, e_h, e_t, uv) for uv in sec['right']]
        left = [to_local(P, e_h, e_t, (u, -v)) for u, v in reversed(sec['right'])]
        tip = [right[-1], to_local(P, e_h, e_t, sec['tip']), left[0]]
        r0 = sec['right'][0]
        down_l = to_local(P, e_h, e_t, (sec['bottom'], -r0[1]))
        down_r = to_local(P, e_h, e_t, (sec['bottom'], r0[1]))
        bottom = [left[-1], down_l, down_r, right[0]]
        out.append({'R': R, 'right': right, 'tip': tip, 'left': left,
                    'bottom': bottom, 'pointed': sec['pointed']})
    return out


def blank_profile(G, which, neck=0.0):
    """Half-section (r, z) of the body the teeth sit on, closed polygon.

    Root cone from toe to heel, then down the back cone just far enough to cover
    the bottom of the teeth, then - for the pinion - a neck of length `neck` back
    to the helical gear. The neck is no wider than the helical gear's root less
    the standard 0.25 mn clearance: the crossed helical that meshes with it sweeps
    its tips down to that root, right across the end face, and a neck as wide as
    the bevel's heel would run into them.
    """
    g = G[which]
    Re, b, hf = G['Re'], G['face'], G['hf']
    sd, cd = math.sin(g['delta']), math.cos(g['delta'])

    def root(R):
        s = R / Re
        return (R * sd - hf * s * cd, -R * cd - hf * s * sd)

    toe = root(Re - b)
    heel = root(Re)
    drop = 0.5 * hf                       # twice the tooth's own overlap
    heel2 = (heel[0] - drop * cd, heel[1] - drop * sd)
    pts = [(0.0, toe[1]), toe, heel, heel2]
    if neck > 0.0:
        r_neck = min(heel2[0], neck_radius(G))
        if r_neck < heel2[0]:
            pts.append((r_neck, heel2[1]))
        pts.append((r_neck, heel2[1] - neck))
        pts.append((0.0, heel2[1] - neck))
    else:
        pts.append((0.0, heel2[1]))
    return pts


def neck_radius(G):
    """The helical gear's root radius less the 0.25 mn tip clearance."""
    return G['pinion']['r'] - (G['hf'] / G['mn'] + 0.25) * G['mn']


def heel_face_z(G, which, neck=0.0):
    """z of the flat heel face in the local frame - where the helical gear ends."""
    return blank_profile(G, which, neck)[-1][1]


# --------------------------------------------------------------------------- #
# the pair against each other
# --------------------------------------------------------------------------- #

def wrap(a, period):
    """a folded into [-period/2, period/2)."""
    return (a + 0.5 * period) % period - 0.5 * period


def mesh_frame(G, contact_phi):
    """Where the gear sits, in the PINION's local frame, and how it is turned.

    contact_phi is the pinion-local azimuth of the contact generator - the side of
    the pinion the gear is on. Returns the gear's local axes (x, y, z, as vectors
    in pinion coordinates, apex shared at the origin) with its tooth phase already
    folded into x and y, and sets G['gear']['hs'].

    Mating is checked on the pitch surfaces at the contact generator, where both
    cones touch: a pinion tooth centre there must be a gear tooth-space centre,
    and the two tooth lines must leave it the same way. Both are proved by
    tests/test_math.py in 3D, not argued here.
    """
    p, gr = G['pinion'], G['gear']
    sig = G['sigma']
    w = (math.cos(contact_phi), math.sin(contact_phi), 0.0)
    a1 = (0.0, 0.0, -1.0)                              # pinion, apex -> heel
    a2 = add(scale(a1, math.cos(sig)), scale(w, math.sin(sig)))
    z_c = scale(a2, -1.0)
    gen, _eh, e_t1 = frame_at(p, 1.0, contact_phi)
    x_c = unit(sub(gen, scale(a2, dot(gen, a2))))
    y_c = cross(z_c, x_c)
    sgn = 1.0 if dot(e_t1, y_c) > 0.0 else -1.0

    # the gear's tooth line must leave the contact the way the pinion's does
    gr['hs'] = p['hs'] * sgn

    # the pinion tooth nearest the contact is `off` behind it in azimuth; the
    # gear's tooth SPACE must be the same arc behind, measured on its own circle
    pitch1 = 2.0 * math.pi / p['z']
    off = wrap(contact_phi, pitch1)
    gamma = -sgn * off * p['z'] / gr['z'] + math.pi / gr['z']
    x_g = rotate(x_c, z_c, gamma)
    y_g = cross(z_c, x_g)
    return {'x': x_g, 'y': y_g, 'z': z_c, 'sign': sgn, 'gamma': gamma}


def local_to(frame, p):
    """A point in a child frame (x, y, z axes, shared origin) to the parent."""
    return add(add(scale(frame['x'], p[0]), scale(frame['y'], p[1])),
               scale(frame['z'], p[2]))


def tooth_centre(G, which, R, k=0):
    """Pitch point of tooth k's centre line at cone distance R, gear-local."""
    g = G[which]
    phi = spiral_psi(G, g, R) + 2.0 * math.pi * k / g['z']
    return frame_at(g, R, phi)[0]


def space_centre(G, which, R, k=0):
    g = G[which]
    phi = spiral_psi(G, g, R) + 2.0 * math.pi * (k + 0.5) / g['z']
    return frame_at(g, R, phi)[0]


# --------------------------------------------------------------------------- #
# swept-volume clearance against gears it must NOT mesh with
# --------------------------------------------------------------------------- #
#
# What a gear occupies over a full turn is a solid of revolution, so two gears
# that are not meant to mesh must keep those apart - whatever their phase. That
# makes the test axisymmetric: tooth 0's outline, revolved, is the whole gear.
# The helical gears are taken as their tip cylinders, which is exactly what they
# sweep. Distances are signed: negative is overlap.

def cyl_distance(p, cyl):
    """Signed distance from p to a finite solid cylinder
    cyl = (centre, unit axis, radius, half length)."""
    centre, axis, radius, half_len = cyl
    v = sub(p, centre)
    a = dot(v, axis)
    rho = norm(sub(v, scale(axis, a)))
    dx, dy = rho - radius, abs(a) - half_len
    if dx < 0.0 and dy < 0.0:
        return max(dx, dy)
    return math.hypot(max(dx, 0.0), max(dy, 0.0))


def swept_outline(G, which, neck=0.0):
    """(rho, z) points on the boundary of the gear's swept volume, gear-local."""
    pts = []
    for sec in tooth_sections(G, which):
        for key in ('right', 'tip', 'left'):
            pts.extend((math.hypot(p[0], p[1]), p[2]) for p in sec[key])
    prof = blank_profile(G, which, neck)
    for (r0, z0), (r1, z1) in zip(prof, prof[1:]):
        for i in range(8):
            t = i / 8.0
            pts.append((r0 + (r1 - r0) * t, z0 + (z1 - z0) * t))
    return pts


def swept_clearance(outline, frame, cyl, n_ang=180):
    """Smallest signed distance from a revolved outline to a cylinder.

    frame = (origin, x, y, z) of the gear in the cylinder's coordinates.
    Sampled at n_ang angles, then the closest few samples are refined by golden
    section within one step either side: a coarse grid alone can only
    OVERSTATE a clearance, by up to the chord sag r (1 - cos(pi / n_ang)).
    """
    o, fx, fy, fz = frame

    def at(rho, z, a):
        p = add(add(o, scale(fz, z)), add(scale(fx, rho * math.cos(a)),
                                          scale(fy, rho * math.sin(a))))
        return cyl_distance(p, cyl)

    step = 2.0 * math.pi / n_ang
    cands = []
    for rho, z in outline:
        best = min((at(rho, z, k * step), k) for k in range(n_ang))
        cands.append((best[0], rho, z, best[1] * step))
    cands.sort()
    out = cands[0][0]
    g = (math.sqrt(5.0) - 1.0) / 2.0
    for d0, rho, z, a0 in cands[:6]:
        lo, hi = a0 - step, a0 + step
        for _ in range(30):
            m1 = hi - g * (hi - lo)
            m2 = lo + g * (hi - lo)
            if at(rho, z, m1) < at(rho, z, m2):
                hi = m2
            else:
                lo = m1
        out = min(out, d0, at(rho, z, 0.5 * (lo + hi)))
    return out


def summary(G):
    """The numbers the dialog shows."""
    p, g = G['pinion'], G['gear']
    out = {}
    for key, gg in (('pinion', p), ('gear', g)):
        dl = gg['delta']
        out[key] = {
            'z': gg['z'], 'cone_deg': math.degrees(dl),
            'pitch_d': gg['d'],
            'tip_d': gg['d'] + 2.0 * G['ha'] * math.cos(dl),
            'face_cone_deg': math.degrees(dl + G['theta_a']),
            'root_cone_deg': math.degrees(dl - G['theta_f']),
            'virtual_z': gg['zv'],
        }
    out['Re'] = G['Re']
    out['face'] = G['face']
    out['face_recommended'] = G['face_recommended']
    out['ratio'] = g['z'] / float(p['z'])
    out['delta1_requested_deg'] = math.degrees(G['delta1_requested'])
    out['spiral_twist_deg'] = math.degrees(
        abs(spiral_psi(G, p, G['Re'] - G['face'])))
    return out
