"""Spiral Bevel - all the working code. RE-IMPORTED ON EVERY COMMAND EVENT.

Nothing may keep state in a module global between events; anything the dialog
decides has to be readable back off its inputs.

The geometry is sb_math's and is proved there without Fusion. This file only
reads what the pair is placed on (any flat face or construction plane, noticing
when a face is a gear's end face), puts two frames in the model, and turns
sb_math's outlines into features:

    blank   revolve of sb_math.blank_profile about the component's z axis
    tooth   loft through sb_math.tooth_sections - nine closed 3D outlines, toe
            to heel, each drawn as fitted splines and lines joined at shared
            sketch points and handed to the loft as a Path (they are not on any
            construction plane, so they cannot be sketch profiles)
    teeth   circular pattern of that one tooth, then a join onto the blank

Each gear is its own component, placed by its occurrence transform, so every
feature is built in the gear's local frame - apex at the origin, heel towards -z.

After building, every new body is intersected with every other visible body and
the interference volume reported. The pinion against the gear is in that list on
purpose: with backlash they must not touch, so a non-zero volume there means the
mesh itself is wrong, not the layout.
"""

import importlib
import json
import math
import os
import traceback

import adsk.core
import adsk.fusion

import sb_math as M

importlib.reload(M)

ADDIN_DIR = os.path.dirname(os.path.realpath(__file__))
RES_DIR = os.path.join(ADDIN_DIR, 'resources', 'SpiralBevel')
LOG_PATH = os.path.join(ADDIN_DIR, 'SpiralBevel.log')
SETTINGS = os.path.join(ADDIN_DIR, 'last_settings.json')

CMD_ID = 'SpiralBevelCreate'
CMD_NAME = 'Spiral Bevel Gear'
VERSION = '0.3.0'

CM = 10.0          # internal cm -> mm
MM = 0.1           # mm -> internal cm
ATTR = 'SpiralBevel'

DEFAULTS = {
    'mn': 0.1414, 'alpha_n': 25.0, 'beta': 45.0, 'hand': 0, 'by': 0,
    'z1': 25, 'z2': 25, 'backlash': 0.02, 'delta1': 45.0, 'sigma': 90.0,
    'face': 0.0, 'gap': 0.0, 'position': 180.0, 'flip': False,
    'replace': True, 'check': True, 'preview': True,
}

HANDS = [('Right', M.RIGHT), ('Left', M.LEFT)]


def log(msg):
    try:
        with open(LOG_PATH, 'a', encoding='utf-8') as fh:
            fh.write('%s\n' % msg)
    except Exception:
        pass


def short_error(tb):
    for line in reversed(tb.strip().splitlines()):
        line = line.strip()
        if line and not line.startswith(('File "', 'Traceback', '^', '~')):
            return line
    return tb.strip().splitlines()[-1]


def load_settings():
    s = dict(DEFAULTS)
    try:
        with open(SETTINGS, encoding='utf-8') as fh:
            s.update(json.load(fh))
    except Exception:
        pass
    return s


def save_settings(s):
    try:
        with open(SETTINGS, 'w', encoding='utf-8') as fh:
            json.dump(s, fh, indent=2)
    except Exception:
        pass


def _v(p):
    return (p.x, p.y, p.z)


def _pt(t):
    return adsk.core.Point3D.create(t[0], t[1], t[2])


def _vec(t):
    return adsk.core.Vector3D.create(t[0], t[1], t[2])


def _cm(t):
    return (t[0] * MM, t[1] * MM, t[2] * MM)


# --------------------------------------------------------------------------- #
# what the pair is placed on, and what it must keep clear of
# --------------------------------------------------------------------------- #

def outward_normal(face):
    ok, param = face.evaluator.getParameterAtPoint(face.pointOnFace)
    ok2, nrm = face.evaluator.getNormalAtParameter(param)
    n = M.unit(_v(nrm))
    if face.isParamReversed:
        n = M.scale(n, -1.0)
    return n


def reference_x(n):
    """A direction in the face plane to measure angles from: world X, or world Y
    when the axis is near X."""
    ref = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    return M.unit(M.sub(ref, M.scale(n, M.dot(ref, n))))


def outline_points(face):
    """The outer loop of the face as a polyline, in cm."""
    pts = []
    for loop in face.loops:
        if not loop.isOuter:
            continue
        for edge in loop.edges:
            ev = edge.evaluator
            ok, p0, p1 = ev.getParameterExtents()
            if not ok:
                continue
            ok, strokes = ev.getStrokes(p0, p1, 0.0005)
            if ok:
                pts.extend(_v(p) for p in strokes)
    return pts


def gear_outline(face, n, c, x0):
    """If the face's outline is a toothed ring, how many teeth and how big.

    Teeth are the runs of outline points above half depth. A face counts as a
    gear with 8 or more of them and a tooth depth under 30 % of its radius -
    which a rectangle (4 corners) or a hexagon (6) never passes.
    """
    y0 = M.cross(n, x0)
    polar = []
    for p in outline_points(face):
        v = M.sub(p, c)
        polar.append((math.hypot(M.dot(v, x0), M.dot(v, y0)),
                      math.atan2(M.dot(v, y0), M.dot(v, x0))))
    if len(polar) < 20:
        return None
    rmax = max(r for r, _a in polar)
    rmin = min(r for r, _a in polar)
    if rmax <= 0.0 or (rmax - rmin) / rmax > 0.3:
        return None
    mid = 0.5 * (rmax + rmin)
    ordered = sorted(polar, key=lambda ra: ra[1])
    count = 0
    prev = ordered[-1][0] > mid
    for r, _a in ordered:
        up = r > mid
        if up and not prev:
            count += 1
        prev = up
    if count < 8:
        return None
    return {'count': count, 'polar': polar, 'rmax': rmax, 'rmin': rmin,
            'tip_d_mm': 2.0 * rmax * CM, 'root_d_mm': 2.0 * rmin * CM}


def tooth_phase(g, z):
    """Angle of a tooth CENTRE: the tip region of every tooth is symmetric about
    it, so the circular mean of z * angle near the tip circle is z times it."""
    lim = g['rmax'] - 0.3 * (g['rmax'] - g['rmin'])
    tips = [a for r, a in g['polar'] if r > lim]
    C = sum(math.cos(z * a) for a in tips)
    S = sum(math.sin(z * a) for a in tips)
    return math.atan2(S, C) / z if (C or S) else 0.0


def read_base(ent, z1, flip=False):
    """Where the pinion goes, from a flat face or a construction plane.

    A face gives its centroid and outward normal; a construction plane its origin
    and normal; the pinion grows along that normal (away from the face) unless
    flipped. If the face is a gear's end face, that is noticed rather than
    required: its tip cylinder is then checked against the gear's sweep, and when
    it has the pinion's tooth count the teeth are lined up with it, tooth for
    tooth - the only thing a gear face gives that any flat face does not.
    """
    face = adsk.fusion.BRepFace.cast(ent)
    cp = adsk.fusion.ConstructionPlane.cast(ent)
    if face:
        if adsk.core.Plane.cast(face.geometry) is None:
            raise M.GearError('pick a flat face or a construction plane')
        n = outward_normal(face)
        c = _v(face.centroid)
        body = face.body
        kind = 'the centre of the face'
    elif cp:
        pl = cp.geometry
        n = M.unit(_v(pl.normal))
        c = _v(pl.origin)
        face, body = None, None
        kind = 'the origin of the plane'
    else:
        raise M.GearError('pick a flat face or a construction plane')
    if flip:
        n = M.scale(n, -1.0)
    x0 = reference_x(n)
    base = {'n': n, 'centre': c, 'x0': x0, 'phase': 0.0, 'body': body,
            'kind': kind, 'gear': None, 'envelope': None, 'aligned': False}
    if face is not None:
        g = gear_outline(face, n, c, x0)
        if g is not None:
            base['gear'] = g
            base['kind'] = 'the axis of the gear it sits on'
            try:
                base['envelope'] = read_envelope(face)
            except M.GearError:
                pass
            if g['count'] == z1:
                base['phase'] = tooth_phase(g, z1)
                base['aligned'] = True
    return base


def read_envelope(face):
    """A gear's swept volume as its tip cylinder, from one flat end face, in mm.

    Axis = the face normal, radius = the farthest outline point from the face
    centroid, length = the distance to the body's opposite parallel flat face.
    Returns (centre, axis, radius, half length) for sb_math.cyl_distance.
    """
    if adsk.core.Plane.cast(face.geometry) is None:
        raise M.GearError('pick a flat END face of the gear')
    n = outward_normal(face)
    c = _v(face.centroid)
    r = 0.0
    for p in outline_points(face):
        v = M.sub(p, c)
        r = max(r, M.norm(M.sub(v, M.scale(n, M.dot(v, n)))))
    t = 0.0
    for f in face.body.faces:
        pl = adsk.core.Plane.cast(f.geometry)
        if pl and f != face:
            try:
                m = outward_normal(f)
            except Exception:
                continue
            if M.dot(m, n) < -0.9999:
                t = max(t, M.dot(M.sub(c, _v(f.centroid)), n))
    if t <= 0.0:
        raise M.GearError('%s has no opposite end face to measure its thickness '
                          'from' % face.body.name)
    centre = M.scale(M.sub(c, M.scale(n, 0.5 * t)), CM)
    return (centre, n, r * CM, 0.5 * t * CM)


N_ANG_LIVE = 72


def _mm_frame(fr):
    return (M.scale(fr['origin'], CM), fr['x'], fr['y'], fr['z'])


def swept_report(P, others):
    """Swept-volume clearances, mm, negative = overlap.

    gear_base   the gear against the gear the pinion sits on (they turn on
                different shafts); None when the base is not a gear
    others      [(name, gear clearance, pinion clearance)] for every gear to keep
                clear of
    gap_needed  the smallest neck that clears the pinion of all of them
    """
    G, H = P['G'], P['H']
    g_out = M.swept_outline(G, 'gear')
    gf = _mm_frame(P['gear_frame'])
    out = {'gear_base': None, 'others': [], 'gap_needed': 0.0}
    if H['envelope'] is not None:
        out['gear_base'] = M.swept_clearance(g_out, gf, H['envelope'], N_ANG_LIVE)
    if not others:
        return out
    n = H['n']
    J = M.scale(H['centre'], CM)
    fx, fy = P['pinion_frame']['x'], P['pinion_frame']['y']

    def pinion(gap, env):
        o = M.add(J, M.scale(n, -M.heel_face_z(G, 'pinion', neck=gap)))
        return M.swept_clearance(M.swept_outline(G, 'pinion', gap),
                                 (o, fx, fy, n), env, N_ANG_LIVE)

    for name, env in others:
        out['others'].append((name, M.swept_clearance(g_out, gf, env, N_ANG_LIVE),
                              pinion(G['gap'], env)))

    def worst(gap):
        return min(pinion(gap, env) for _n, env in others)
    if worst(0.0) < 0.0:
        lo, hi = 0.0, 3.0
        if worst(hi) < 0.0:
            out['gap_needed'] = None
        else:
            for _ in range(12):
                mid = 0.5 * (lo + hi)
                if worst(mid) >= 0.0:
                    hi = mid
                else:
                    lo = mid
            out['gap_needed'] = hi
    return out


def clear_envelopes(inputs):
    sel = inputs.itemById('selClear')
    out = []
    for i in range(sel.selectionCount):
        f = adsk.fusion.BRepFace.cast(sel.selection(i).entity)
        if f:
            out.append((f.body.name, read_envelope(f)))
    return out


# --------------------------------------------------------------------------- #
# dialog values
# --------------------------------------------------------------------------- #

BY = [('teeth', 'Tooth counts'), ('cone', 'Pinion teeth + cone angle')]


def params_from(inputs):
    """sb_math parameters, in mm and degrees, off the dialog."""
    deg = math.degrees
    return {
        'mn': inputs.itemById('mn').value * CM,
        'alpha_n': deg(inputs.itemById('alpha_n').value),
        'beta': deg(inputs.itemById('beta').value),
        'hand': HANDS[inputs.itemById('hand').selectedItem.index][1],
        'by': BY[inputs.itemById('by').selectedItem.index][0],
        'z1': inputs.itemById('z1').value,
        'z2': inputs.itemById('z2').value,
        'backlash': inputs.itemById('backlash').value * CM,
        'delta1': deg(inputs.itemById('delta1').value),
        'sigma': deg(inputs.itemById('sigma').value),
        'face': inputs.itemById('face').value * CM,
        'gap': inputs.itemById('gap').value * CM,
    }


def settings_from(inputs):
    s = params_from(inputs)
    s['hand'] = inputs.itemById('hand').selectedItem.index
    s['by'] = inputs.itemById('by').selectedItem.index
    s['position'] = math.degrees(inputs.itemById('position').value)
    for k in ('replace', 'check', 'preview', 'flip'):
        s[k] = inputs.itemById(k).value
    return s


def selected_base(inputs):
    sel = inputs.itemById('selFace')
    if sel.selectionCount == 0:
        return None
    return sel.selection(0).entity


def resolve(inputs):
    """The pair and where it goes, or a GearError saying what is missing."""
    G = M.design(params_from(inputs))
    ent = selected_base(inputs)
    if ent is None:
        raise M.GearError('pick the face or plane to put the pinion on')
    H = read_base(ent, G['z1'], inputs.itemById('flip').value)
    return place(G, H, inputs.itemById('position').value)


def place(G, H, position):
    """World frames of both gears (cm), from the pair and its base.

    position is the gear's azimuth round the pinion axis, measured from the
    base's reference direction - the knob that moves it away from whatever else
    turns next to the pinion.
    """
    n, x0 = H['n'], H['x0']
    xp = M.rotate(x0, n, H['phase'])
    yp = M.cross(n, xp)
    z_heel = M.heel_face_z(G, 'pinion', neck=G['gap'])          # mm, negative
    apex = M.add(H['centre'], M.scale(n, -z_heel * MM))
    pin = {'origin': apex, 'x': xp, 'y': yp, 'z': n}
    fr = M.mesh_frame(G, position - H['phase'])

    def world(v):
        return M.add(M.add(M.scale(xp, v[0]), M.scale(yp, v[1])), M.scale(n, v[2]))
    gear = {'origin': apex, 'x': world(fr['x']), 'y': world(fr['y']),
            'z': world(fr['z'])}
    return {'G': G, 'H': H, 'pinion_frame': pin, 'gear_frame': gear,
            'position': position}


# --------------------------------------------------------------------------- #
# building
# --------------------------------------------------------------------------- #

def _matrix(fr):
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(_pt(fr['origin']), _vec(fr['x']), _vec(fr['y']),
                              _vec(fr['z']))
    return m


def build_blank(comp, G, which, neck):
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = 'blank'
    prof = M.blank_profile(G, which, neck)
    pts = [sk.modelToSketchSpace(_pt(_cm((r, 0.0, z)))) for r, z in prof]
    lines = sk.sketchCurves.sketchLines
    first = prev = None
    for i in range(len(pts)):
        a = prev.endSketchPoint if prev else pts[0]
        b = first.startSketchPoint if (i == len(pts) - 1 and first) else \
            pts[(i + 1) % len(pts)]
        ln = lines.addByTwoPoints(a, b)
        first = first or ln
        prev = ln
    if sk.profiles.count == 0:
        raise M.GearError('the blank outline did not close')
    rev = comp.features.revolveFeatures
    ri = rev.createInput(sk.profiles.item(0), comp.zConstructionAxis,
                         adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    return rev.add(ri).bodies.item(0)


def _section_curves(sk, sec):
    """One closed outline as six sketch curves, each starting on the previous
    one's own end point - that shared SketchPoint is what makes them connected.
    Handing a spline a separately made SketchPoint as a fit point does NOT."""
    splines = sk.sketchCurves.sketchFittedSplines
    lines = sk.sketchCurves.sketchLines

    def spline(start, rest):
        col = adsk.core.ObjectCollection.create()
        col.add(start)
        for p in rest:
            col.add(_pt(_cm(p)))
        return splines.add(col)

    right = spline(_pt(_cm(sec['right'][0])), sec['right'][1:])
    tip = spline(right.endSketchPoint, sec['tip'][1:])
    left = spline(tip.endSketchPoint, sec['left'][1:])
    l1 = lines.addByTwoPoints(left.endSketchPoint, _pt(_cm(sec['bottom'][1])))
    l2 = lines.addByTwoPoints(l1.endSketchPoint, _pt(_cm(sec['bottom'][2])))
    l3 = lines.addByTwoPoints(l2.endSketchPoint, right.startSketchPoint)
    col = adsk.core.ObjectCollection.create()
    for c in (right, tip, left, l1, l2, l3):
        col.add(c)
    return col


def build_tooth(comp, G, which):
    secs = M.tooth_sections(G, which)
    loft = comp.features.loftFeatures
    li = loft.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    li.isSolid = True
    paths = []
    for k, sec in enumerate(secs):
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = 'tooth section %d' % (k + 1)
        sk.isComputeDeferred = True
        curves = _section_curves(sk, sec)
        sk.isComputeDeferred = False
        path = comp.features.createPath(curves, False)
        paths.append((path.count, path.isClosed))
        li.loftSections.add(path)
    if any(n != 6 or not closed for n, closed in paths):
        raise M.GearError('a tooth section did not close into one outline: %s'
                          % paths)
    return loft.add(li).bodies.item(0), any(s['pointed'] for s in secs)


def build_gear(comp, G, which, neck=0.0):
    blank = build_blank(comp, G, which, neck)
    tooth, pointed = build_tooth(comp, G, which)
    ents = adsk.core.ObjectCollection.create()
    ents.add(tooth)
    cp = comp.features.circularPatternFeatures
    ci = cp.createInput(ents, comp.zConstructionAxis)
    ci.quantity = adsk.core.ValueInput.createByReal(G[which]['z'])
    ci.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    ci.isSymmetric = False
    pat = cp.add(ci)
    tools = adsk.core.ObjectCollection.create()
    tools.add(tooth)
    for b in pat.bodies:
        if b != tooth:
            tools.add(b)
    cb = comp.features.combineFeatures
    cbi = cb.createInput(blank, tools)
    cbi.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    cbi.isKeepToolBodies = False
    cb.add(cbi)
    # the section and blank sketches are construction, not design: nine sets of
    # 3D splines left showing would clutter the model with dotted outlines
    for sk in comp.sketches:
        sk.isVisible = False
    return blank, pointed


def remove_previous(des):
    """Delete the occurrences an earlier run of this command made - and only
    those: they are found by the attribute this command writes on them."""
    gone = 0
    for a in list(des.findAttributes(ATTR, 'role')):
        occ = adsk.fusion.Occurrence.cast(a.parent)
        if occ and occ.isValid:
            occ.deleteMe()
            gone += 1
    return gone


def build_pair(des, P):
    G = P['G']
    root = des.rootComponent
    out = {}
    for which, fr, neck in (('pinion', P['pinion_frame'], G['gap']),
                            ('gear', P['gear_frame'], 0.0)):
        occ = root.occurrences.addNewComponent(_matrix(fr))
        comp = occ.component
        g = G[which]
        comp.name = ('Spiral bevel %s z%d %.2fdeg'
                     % (which, g['z'], math.degrees(g['delta'])))
        body, pointed = build_gear(comp, G, which, neck)
        body.name = comp.name
        occ.attributes.add(ATTR, 'role', which)
        occ.attributes.add(ATTR, 'params', json.dumps(M.summary(G)))
        out[which] = {'occ': occ, 'body': body.createForAssemblyContext(occ),
                      'pointed': pointed}
    return out


# --------------------------------------------------------------------------- #
# interference
# --------------------------------------------------------------------------- #

def _boxes_overlap(a, b):
    return (a.minPoint.x <= b.maxPoint.x and b.minPoint.x <= a.maxPoint.x
            and a.minPoint.y <= b.maxPoint.y and b.minPoint.y <= a.maxPoint.y
            and a.minPoint.z <= b.maxPoint.z and b.minPoint.z <= a.maxPoint.z)


def world_copy(body):
    """A temporary copy of a body WHERE IT IS in the assembly.

    TemporaryBRepManager.copy of an occurrence proxy does not reliably carry the
    occurrence's placement - measured: a gear proxy spanning z -0.218..5.578 mm
    copied to z 0.101..5.287 - so copy the native body and place it explicitly.
    """
    tbm = adsk.fusion.TemporaryBRepManager.get()
    occ = body.assemblyContext
    native = body.nativeObject if occ else body
    c = tbm.copy(native)
    if occ:
        tbm.transform(c, occ.transform2)
    return c


def tbm_bool(a, b, kind):
    return adsk.fusion.TemporaryBRepManager.get().booleanOperation(a, b, kind)


def overlap_mm3(a, b):
    """Volume of a AND b, in mm3; 0 when they only touch or are apart."""
    if not _boxes_overlap(a.boundingBox, b.boundingBox):
        return 0.0
    ta = world_copy(a)
    tb = world_copy(b)
    if not tbm_bool(ta, tb,
                    adsk.fusion.BooleanTypes.IntersectionBooleanType):
        return 0.0
    return max(0.0, ta.volume * 1000.0)


def visible_bodies(des, skip):
    """Every visible solid body in world context, except those in `skip`."""
    skip_keys = set(b.entityToken for b in skip)
    out = []
    root = des.rootComponent
    for b in root.bRepBodies:
        if b.isVisible and b.isSolid and b.entityToken not in skip_keys:
            out.append(b)
    for occ in root.allOccurrences:
        if not occ.isVisible:
            continue
        for b in occ.bRepBodies:
            if b.isVisible and b.isSolid and b.entityToken not in skip_keys:
                out.append(b)
    return out


def interference_report(des, made, helical_body):
    """[(what, against, mm3)] for every new body against everything else."""
    pin, gear = made['pinion']['body'], made['gear']['body']
    rows = [('pinion', 'gear (the mesh itself)', overlap_mm3(pin, gear))]
    others = visible_bodies(des, [pin, gear])
    for b in others:
        rows.append(('gear', b.name, overlap_mm3(gear, b)))
    for b in others:
        # the pinion is MEANT to sit on the helical gear's end face
        if helical_body is not None and b.entityToken == helical_body.entityToken:
            continue
        rows.append(('pinion', b.name, overlap_mm3(pin, b)))
    return rows


# --------------------------------------------------------------------------- #
# dialog
# --------------------------------------------------------------------------- #

def _esc(t):
    return (str(t).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))


def _rows(pairs):
    out = ['<table cellpadding="0" cellspacing="0">']
    for k, v in pairs:
        out.append('<tr><td width="120"><b>%s</b></td><td>%s</td></tr>'
                   % (_esc(k), v))
    out.append('</table>')
    return ''.join(out)


def on_created(app, cmd):
    s = load_settings()
    cmd.isRepeatable = False
    cmd.okButtonText = 'Create'
    inputs = cmd.commandInputs
    rl = adsk.core.ValueInput.createByReal

    sel = inputs.addSelectionInput('selFace', 'Place on',
                                   "A flat face (centred on it) or a construction "
                                   "plane (on its origin). A gear's end face is "
                                   "recognised: the teeth line up with it when "
                                   "the counts match.")
    sel.addSelectionFilter('PlanarFaces')
    sel.addSelectionFilter('ConstructionPlanes')
    sel.setSelectionLimits(1, 1)
    inputs.addBoolValueInput('flip', 'Flip direction', True, '', bool(s['flip']))
    clr = inputs.addSelectionInput('selClear', 'Keep clear of',
                                   'End faces of other gears that turn next to '
                                   'this pair - each is checked as the cylinder '
                                   'its teeth sweep over a full turn')
    clr.addSelectionFilter('PlanarFaces')
    clr.setSelectionLimits(0, 0)

    t = inputs.addGroupCommandInput('gTeeth', 'Teeth')
    ti = t.children
    by = ti.addDropDownCommandInput('by', 'Define by',
                                    adsk.core.DropDownStyles.TextListDropDownStyle)
    for i, (_k, lab) in enumerate(BY):
        by.listItems.add(lab, i == int(s['by']))
    ti.addIntegerSpinnerCommandInput('z1', 'Pinion teeth', 5, 400, 1, int(s['z1']))
    ti.addIntegerSpinnerCommandInput('z2', 'Gear teeth', 5, 400, 1, int(s['z2']))
    ti.addValueInput('delta1', 'Pinion cone angle', 'deg',
                     rl(math.radians(float(s['delta1']))))
    ti.addValueInput('sigma', 'Shaft angle', 'deg',
                     rl(math.radians(float(s['sigma']))))

    g = inputs.addGroupCommandInput('gSystem', 'Tooth system')
    gi = g.children
    gi.addValueInput('mn', 'Normal module', 'mm', rl(float(s['mn']) * MM))
    gi.addValueInput('alpha_n', 'Pressure angle', 'deg',
                     rl(math.radians(float(s['alpha_n']))))
    gi.addValueInput('beta', 'Spiral angle', 'deg',
                     rl(math.radians(float(s['beta']))))
    hd = gi.addDropDownCommandInput('hand', 'Pinion hand',
                                    adsk.core.DropDownStyles.TextListDropDownStyle)
    for i, (lab, _v2) in enumerate(HANDS):
        hd.listItems.add(lab, i == int(s['hand']))
    gi.addValueInput('backlash', 'Backlash', 'mm', rl(float(s['backlash']) * MM))
    gi.addValueInput('face', 'Face width (0 = Re/3)', 'mm',
                     rl(float(s['face']) * MM))

    pg = inputs.addGroupCommandInput('gPlace', 'Placement')
    pi = pg.children
    pi.addValueInput('gap', 'Neck (gap)', 'mm', rl(float(s['gap']) * MM))
    pi.addValueInput('position', 'Gear position', 'deg',
                     rl(math.radians(float(s['position']))))

    inputs.addBoolValueInput('replace', 'Replace the pair made before', True, '',
                             bool(s['replace']))
    inputs.addBoolValueInput('check', 'Measure interference', True, '',
                             bool(s['check']))
    inputs.addBoolValueInput('preview', 'Preview', True, '', bool(s['preview']))
    info = inputs.addTextBoxCommandInput('info', '', '', 14, True)
    info.isFullWidth = True
    refresh(inputs)


def refresh(inputs):
    by = BY[inputs.itemById('by').selectedItem.index][0]
    inputs.itemById('z2').isVisible = by == 'teeth'
    inputs.itemById('delta1').isVisible = by == 'cone'
    inputs.itemById('info').formattedText = describe(inputs)


def _guard(pairs, fn):
    try:
        pairs.extend(fn())
    except M.GearError as exc:
        pairs.append(('', _esc(exc)))
    except Exception:
        log(traceback.format_exc())
        pairs.append(('', _esc(short_error(traceback.format_exc()))))


def describe(inputs):
    try:
        G = M.design(params_from(inputs))
    except M.GearError as exc:
        return _rows([('', _esc(exc))])
    except Exception:
        log(traceback.format_exc())
        return _rows([('', _esc(short_error(traceback.format_exc())))])
    S = M.summary(G)
    p, g = S['pinion'], S['gear']
    cone = 'pinion %.3f&deg;, gear %.3f&deg;' % (p['cone_deg'], g['cone_deg'])
    if params_from(inputs)['by'] == 'cone':
        cone = ('pinion %.3f&deg; (asked %.3f&deg;), gear %.3f&deg;'
                % (p['cone_deg'], S['delta1_requested_deg'], g['cone_deg']))
    pairs = [
        ('teeth', 'pinion %d : gear %d &mdash; ratio %.4f'
         % (p['z'], g['z'], S['ratio'])),
        ('cone angles', cone),
        ('pinion', 'pitch &#8960; %.3f, tip &#8960; %.3f mm at the heel'
         % (p['pitch_d'], p['tip_d'])),
        ('gear', 'pitch &#8960; %.3f, tip &#8960; %.3f mm at the heel'
         % (g['pitch_d'], g['tip_d'])),
        ('cone distance', '%.3f mm, face %.3f mm (Re/3 = %.3f)'
         % (S['Re'], S['face'], S['face_recommended'])),
        ('spiral', 'turns %.1f&deg; across the pinion face'
         % S['spiral_twist_deg']),
    ]
    if S['face'] > S['face_recommended'] * 1.0001:
        pairs.append(('', '<b>face wider than Re/3</b> - the toe teeth get small'))
    if selected_base(inputs) is not None:
        _guard(pairs, lambda: base_rows(inputs, G))
        _guard(pairs, lambda: swept_rows(inputs))
    return _rows(pairs)


def base_rows(inputs, G):
    H = read_base(selected_base(inputs), G['z1'], inputs.itemById('flip').value)
    rows = [('placed on', _esc(H['kind']))]
    g = H['gear']
    if g is not None:
        rows.append(('that gear', '%d teeth, tip &#8960; %.3f mm'
                     % (g['count'], g['tip_d_mm'])))
        if H['aligned']:
            rows.append(('', "same tooth count - the pinion's teeth are lined up "
                             "with it"))
            want = G['pinion']['d'] + 2.0 * G['ha']
            if abs(g['tip_d_mm'] - want) > 0.05 * want:
                rows.append(('', "<b>its tip &#8960; is not this module's %.3f mm"
                                 "</b> - check the module and spiral angle"
                             % want))
        else:
            rows.append(('', 'a different tooth count - centred on it, not '
                             'lined up'))
    return rows


def _mm_or_overlap(d):
    if d >= 0.0:
        return 'clear by %.3f mm' % d
    return '<b>OVERLAPS by %.3f mm</b>' % -d


def swept_rows(inputs):
    """Dialog rows for the swept-volume check - what each gear occupies over a
    full turn, so it holds whatever phase the teeth happen to be at."""
    P = resolve(inputs)
    others = clear_envelopes(inputs)
    r = swept_report(P, others)
    rows = []
    if r['gear_base'] is not None:
        rows.append(('gear vs base', _mm_or_overlap(r['gear_base'])))
    for name, cg, cp in r['others']:
        rows.append(('gear vs %s' % name[:14], _mm_or_overlap(cg)))
        rows.append(('pinion vs %s' % name[:12], _mm_or_overlap(cp)))
    if others:
        need = r['gap_needed']
        if need is None:
            rows.append(('', '<b>no neck up to 3 mm clears the pinion</b>'))
        elif need > P['G']['gap'] + 1e-6:
            rows.append(('neck needed', '<b>%.3f mm</b>, plus the margin you want'
                         % need))
        else:
            rows.append(('neck needed', '%.3f mm - the neck set is enough' % need))
    return rows


def on_input_changed(app, inputs, changed_id):
    refresh(inputs)


def on_validate(inputs):
    try:
        resolve(inputs)
        return True
    except Exception:
        return False


# --------------------------------------------------------------------------- #
# preview appearance - the glass Horological Gears uses, so the two add-ins'
# previews read the same: a real GLASS base (only glass renders see-through in
# the shaded view; "Plastic - Translucent" looks solid unless ray-traced), with
# its transmitted-light colour forced to bright cyan.
# --------------------------------------------------------------------------- #

PREVIEW_APPEARANCE_NAME = 'Spiral Bevel Preview (glass)'
_PREVIEW_CANDIDATES = ('Glass (Green)', 'Glass (Blue)', 'Glass (Clear)',
                       'Glass - Light Color (Blue)', 'Glass - Medium Color (Blue)')
_PREVIEW_COLOR = (51, 251, 255)
_PREVIEW_ROUGHNESS = 1.0
_PREVIEW_ABSORPTION_MM = 20.0
_PREVIEW_IOR = 1.52


def _by_name(container, name):
    try:
        return container.itemByName(name)
    except Exception:
        return None


def _set_prop(prop, value):
    try:
        prop.value = value
    except Exception:
        pass


def _tune_preview_appearance(appearance):
    """A glass appearance has TWO ColorProperty entries: the white surface colour
    first, and the transmitted-light tint - what the UI swatch edits - last."""
    props = appearance.appearanceProperties
    colors = []
    for k in range(props.count):
        p = props.item(k)
        kind = p.objectType.split('::')[-1]
        if kind == 'ColorProperty':
            colors.append(p)
        elif p.name == 'Roughness':
            _set_prop(p, _PREVIEW_ROUGHNESS)
        elif p.name == 'Transparent Distance':
            _set_prop(p, _PREVIEW_ABSORPTION_MM)
        elif p.name == 'IOR':
            _set_prop(p, _PREVIEW_IOR)
    if colors:
        r, g, b = _PREVIEW_COLOR
        _set_prop(colors[-1], adsk.core.Color.create(r, g, b, 255))


def preview_appearance(des):
    existing = _by_name(des.appearances, PREVIEW_APPEARANCE_NAME)
    if existing is not None:
        return existing
    app = adsk.core.Application.get()
    for i in range(app.materialLibraries.count):
        lib = app.materialLibraries.item(i)
        if lib.appearances.count == 0:
            continue
        for candidate in _PREVIEW_CANDIDATES:
            base = _by_name(lib.appearances, candidate)
            if base is None:
                continue
            appearance = des.appearances.addByCopy(base, PREVIEW_APPEARANCE_NAME)
            _tune_preview_appearance(appearance)
            return appearance
    return None


def apply_preview_appearance(des, made):
    """Cosmetic only: never let a missing appearance break the preview."""
    try:
        appearance = preview_appearance(des)
    except Exception:
        log(traceback.format_exc())
        return None
    if appearance is None:
        return None
    for m in made.values():
        try:
            m['body'].nativeObject.appearance = appearance
        except Exception:
            log(traceback.format_exc())
    return appearance.name


def on_preview(app, inputs, args):
    if not inputs.itemById('preview').value:
        return
    des = adsk.fusion.Design.cast(app.activeProduct)
    P = resolve(inputs)
    if inputs.itemById('replace').value:
        remove_previous(des)
    apply_preview_appearance(des, build_pair(des, P))


def on_execute(app, ui, inputs):
    des = adsk.fusion.Design.cast(app.activeProduct)
    P = resolve(inputs)
    save_settings(settings_from(inputs))
    if inputs.itemById('replace').value:
        remove_previous(des)
    made = build_pair(des, P)
    msg = ['Spiral bevel pair: pinion z%d at %.3f deg, gear z%d at %.3f deg.'
           % (P['G']['z1'], math.degrees(P['G']['pinion']['delta']),
              P['G']['z2'], math.degrees(P['G']['gear']['delta']))]
    if made['pinion']['pointed'] or made['gear']['pointed']:
        msg.append('Some teeth come to a point before the tip circle.')
    try:
        msg.extend(['', 'Swept volumes (a full turn):']
                   + ['  %s: %s' % (k, v.replace('<b>', '').replace('</b>', ''))
                      for k, v in swept_rows(inputs) if k])
    except Exception:
        log(traceback.format_exc())
    if inputs.itemById('check').value:
        rows = interference_report(des, made, P['H']['body'])
        bad = [r for r in rows if r[2] > 1e-6]
        msg.append('')
        if not bad:
            msg.append('No interference: %d pairs checked.' % len(rows))
        for what, other, v in bad:
            msg.append('INTERFERENCE  %s with %s: %.5f mm3' % (what, other, v))
        log('%s delta1 %.3f position %.2f: %s'
            % (VERSION, math.degrees(P['G']['pinion']['delta']),
               math.degrees(P['position']), rows))
    ui.messageBox('\n'.join(msg), CMD_NAME)
    return made


def on_destroy(app):
    pass


# --------------------------------------------------------------------------- #
# toolbar
# --------------------------------------------------------------------------- #

TOOLTIP = ('Build a spiral bevel pinion and the gear it drives.\n\n'
           'Choose both tooth counts (or the pinion teeth and its cone angle) and '
           'the shaft angle. Place the pinion on any flat face or construction '
           "plane; on a gear's end face with the same tooth count its teeth line "
           'up with that gear, so it continues it. Pick the gears it must keep '
           'clear of and the dialog shows, live, the clearance over a full turn '
           'and the neck length needed.')

PANELS = ['SolidCreatePanel']


def install(app, ui, created_handler_factory):
    old = ui.commandDefinitions.itemById(CMD_ID)
    if old:
        old.deleteMe()
    folder = RES_DIR if os.path.isdir(RES_DIR) and os.listdir(RES_DIR) else ''
    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, TOOLTIP,
                                                        folder)
    cmd_def.commandCreated.add(created_handler_factory())
    placed = []
    for pid in PANELS:
        panel = ui.allToolbarPanels.itemById(pid)
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)
            placed.append(pid)
    return placed


def uninstall(ui):
    for pid in PANELS:
        panel = ui.allToolbarPanels.itemById(pid)
        if panel:
            c = panel.controls.itemById(CMD_ID)
            if c:
                c.deleteMe()
    d = ui.commandDefinitions.itemById(CMD_ID)
    if d:
        d.deleteMe()


# --------------------------------------------------------------------------- #
# dev bridge - everything that builds runs in a NEW document, never the user's
# --------------------------------------------------------------------------- #

def dev_ping(app, job):
    doc = app.activeDocument
    return {'version': VERSION, 'document': doc.name if doc else None}


def _stand_in_helical(des, G, thickness_mm=0.5):
    """A cylinder the size of the helical gear's tip circle - its envelope - with
    the end face at z = 0 facing +z. Enough to place a pair on and to measure
    interference against conservatively; it has no teeth, so no phase."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    r = 0.5 * (G['pinion']['d'] + 2.0 * G['mn']) * MM
    tmp = tbm.createCylinderOrCone(_pt((0, 0, -thickness_mm * MM)), r,
                                   _pt((0, 0, 0)), r)
    root = des.rootComponent
    base = root.features.baseFeatures.add()
    base.startEdit()
    body = root.bRepBodies.add(tmp, base)
    base.finishEdit()
    body = root.bRepBodies.item(root.bRepBodies.count - 1)
    body.name = 'helical envelope'
    top = max((f for f in body.faces if adsk.core.Plane.cast(f.geometry)),
              key=lambda f: f.centroid.z)
    return body, top


def _in_scratch(app, job, fn):
    prev = app.activeDocument
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        return fn(adsk.fusion.Design.cast(
            doc.products.itemByProductType('DesignProductType')))
    finally:
        if not job.get('keep'):
            doc.close(False)
            if prev:
                prev.activate()


def _envelope_H(G, face):
    n = outward_normal(face)
    return {'n': n, 'centre': _v(face.centroid), 'x0': reference_x(n),
            'phase': 0.0, 'body': face.body, 'kind': 'dev', 'gear': None,
            'envelope': None, 'aligned': False}


def dev_build(app, job):
    """Build one pair on a stand-in helical gear and measure it."""
    def run(des):
        G = M.design(job.get('params'))
        env, top = _stand_in_helical(des, G)
        P = place(G, _envelope_H(G, top), math.radians(job.get('position', 0.0)))
        made = build_pair(des, P)
        out = {'summary': M.summary(G), 'bodies': {}}
        for k, m in made.items():
            b = m['body']
            bb = b.boundingBox
            out['bodies'][k] = {'solid': b.isSolid, 'volume_mm3': b.volume * 1000.0,
                                'faces': b.faces.count,
                                'bbox_mm': [round(v * CM, 4) for v in
                                            _v(bb.minPoint) + _v(bb.maxPoint)]}
        out['interference'] = interference_report(des, made, None)
        return out
    return _in_scratch(app, job, run)


def dev_sweep(app, job):
    """Interference against the stand-in helical gear across cone angles."""
    def run(des):
        rows = []
        # the envelope depends only on the helical gear, not the cone angle
        env, top = _stand_in_helical(des, M.design(job.get('params')))
        for d1 in job['angles']:
            params = dict(job.get('params') or {})
            params['delta1'] = d1
            try:
                G = M.design(params)
            except M.GearError as exc:
                rows.append({'delta1_asked': d1, 'error': str(exc)})
                continue
            P = place(G, _envelope_H(G, top),
                      math.radians(job.get('position', 0.0)))
            made = build_pair(des, P)
            rep = interference_report(des, made, env)
            rows.append({'delta1_asked': d1,
                         'delta1': math.degrees(G['pinion']['delta']),
                         'z2': G['z2'], 'rows': rep})
            for m in made.values():
                m['occ'].deleteMe()
        return rows
    return _in_scratch(app, job, run)


def _body_info(b):
    out = {}
    for k, f in (('name', lambda: b.name), ('solid', lambda: b.isSolid),
                 ('volume_mm3', lambda: b.volume * 1000.0),
                 ('faces', lambda: b.faces.count), ('shells', lambda: b.shells.count),
                 ('bbox_mm', lambda: [round(v * CM, 4) for v in
                                      _v(b.boundingBox.minPoint)
                                      + _v(b.boundingBox.maxPoint)])):
        try:
            out[k] = f()
        except Exception:
            out[k] = 'ERR ' + short_error(traceback.format_exc())
    return out


def dev_probe(app, job):
    """One gear, step by step, reporting each body - to see which step breaks."""
    def run(des):
        G = M.design(job.get('params'))
        if job.get('stations'):
            G['station_count'] = int(job['stations'])
        which = job.get('which', 'pinion')
        comp = des.rootComponent.occurrences.addNewComponent(
            adsk.core.Matrix3D.create()).component
        out = {}
        blank = build_blank(comp, G, which, 0.0)
        out['blank'] = _body_info(blank)
        tooth, _p = build_tooth(comp, G, which)
        out['tooth'] = _body_info(tooth)
        tbm = adsk.fusion.TemporaryBRepManager.get()
        t1 = tbm.copy(blank)
        out['tooth_blank_union_tbm'] = tbm.booleanOperation(
            t1, tbm.copy(tooth), adsk.fusion.BooleanTypes.UnionBooleanType)
        out['tooth_blank_overlap_mm3'] = overlap_mm3(tooth, blank)
        tools = adsk.core.ObjectCollection.create()
        tools.add(tooth)
        cb = comp.features.combineFeatures
        cbi = cb.createInput(blank, tools)
        cbi.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        cbi.isKeepToolBodies = True
        try:
            cb.add(cbi)
            out['join_one_tooth'] = _body_info(blank)
        except Exception:
            out['join_one_tooth'] = short_error(traceback.format_exc())
        return out
    return _in_scratch(app, job, run)


def dev_checkcheck(app, job):
    """Does overlap_mm3 see bodies where they are? Against known answers."""
    def run(des):
        G = M.design(job.get('params'))
        env, top = _stand_in_helical(des, G)
        P = place(G, _envelope_H(G, top), 0.0)
        made = build_pair(des, P)
        tbm = adsk.fusion.TemporaryBRepManager.get()
        out = {}
        for k, m in made.items():
            b = m['body']
            c = world_copy(b)
            out[k + '_proxy_bbox_z'] = [b.boundingBox.minPoint.z * CM,
                                        b.boundingBox.maxPoint.z * CM]
            out[k + '_copy_bbox_z'] = [c.boundingBox.minPoint.z * CM,
                                       c.boundingBox.maxPoint.z * CM]
            out[k + '_volume'] = b.volume * 1000.0
            out[k + '_self_overlap'] = overlap_mm3(b, b)
        # a big box around the gear, in world coordinates, in the root
        bb = made['gear']['body'].boundingBox
        obb = adsk.core.OrientedBoundingBox3D.create(
            _pt(M.scale(M.add(_v(bb.minPoint), _v(bb.maxPoint)), 0.5)),
            adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0),
            bb.maxPoint.x - bb.minPoint.x + 0.1, bb.maxPoint.y - bb.minPoint.y + 0.1,
            bb.maxPoint.z - bb.minPoint.z + 0.1)
        root = des.rootComponent
        base = root.features.baseFeatures.add()
        base.startEdit()
        root.bRepBodies.add(tbm.createBox(obb), base)
        base.finishEdit()
        box = root.bRepBodies.item(root.bRepBodies.count - 1)
        out['gear_in_box'] = overlap_mm3(made['gear']['body'], box)
        # the gear's own slice behind the helical face, against the envelope
        out['gear_vs_envelope'] = overlap_mm3(made['gear']['body'], env)
        return out
    return _in_scratch(app, job, run)


def _fake_inputs(face, clear, s):
    """A stand-in for the dialog's inputs, so resolve() and swept_rows() run on
    real faces without a dialog open. s holds sb_math-style values (mm, deg);
    'by' and 'hand' may be given as the sb_math strings/signs."""
    from types import SimpleNamespace as NS

    def sel(ents):
        return NS(selectionCount=len(ents), selection=lambda i: NS(entity=ents[i]))
    q = dict(DEFAULTS)
    q.update(s)
    by = q['by'] if isinstance(q['by'], int) else [k for k, _l in BY].index(q['by'])
    hand = 0 if int(q['hand']) >= 0 else 1
    d = {'selFace': sel([face] if face else []), 'selClear': sel(list(clear)),
         'hand': NS(selectedItem=NS(index=hand)),
         'by': NS(selectedItem=NS(index=by)),
         'z1': NS(value=int(q['z1'])), 'z2': NS(value=int(q['z2']))}
    for k in ('mn', 'backlash', 'face', 'gap'):
        d[k] = NS(value=float(q[k]) * MM)
    for k in ('alpha_n', 'beta', 'delta1', 'sigma', 'position'):
        d[k] = NS(value=math.radians(float(q[k])))
    for k in ('replace', 'check', 'preview', 'flip'):
        d[k] = NS(value=bool(q[k]))
    return NS(itemById=d.__getitem__)


def _toy_gear(des, z, r_tip, r_root, phase, thick, name):
    """A toothed prism: z trapezoid teeth, tooth 0 CENTRED at `phase`, top face
    at z = 0. Stands in for a real gear's end face with a known answer."""
    root = des.rootComponent
    sk = root.sketches.add(root.xYConstructionPlane)
    pts = []
    half_tip, half_root = 0.22 * math.pi / z, 0.32 * math.pi / z
    for k in range(z):
        c = phase + 2.0 * math.pi * k / z
        for r, a in ((r_root, c - half_root), (r_tip, c - half_tip),
                     (r_tip, c + half_tip), (r_root, c + half_root)):
            pts.append(_pt((r * math.cos(a), r * math.sin(a), 0.0)))
    lines = sk.sketchCurves.sketchLines
    first = prev = None
    for i in range(len(pts)):
        a = prev.endSketchPoint if prev else pts[0]
        b = first.startSketchPoint if i == len(pts) - 1 else pts[i + 1]
        ln = lines.addByTwoPoints(a, b)
        first = first or ln
        prev = ln
    ext = root.features.extrudeFeatures.addSimple(
        sk.profiles.item(0), adsk.core.ValueInput.createByReal(-thick),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = ext.bodies.item(0)
    body.name = name
    return body


def _cylinder_body(des, p0, p1, r, name):
    tbm = adsk.fusion.TemporaryBRepManager.get()
    root = des.rootComponent
    base = root.features.baseFeatures.add()
    base.startEdit()
    root.bRepBodies.add(tbm.createCylinderOrCone(_pt(p0), r, _pt(p1), r), base)
    base.finishEdit()
    body = root.bRepBodies.item(root.bRepBodies.count - 1)
    body.name = name
    return body


def dev_layout(app, job):
    """The dialog's logic and a real build, on a chosen base, against a stand-in
    for the crossed helical gear B.

    base 'toy'    a toothed prism (toy_z teeth, tooth 0 centred at toy_phase rad,
                  top face at z = 0) standing in for the helical gear A
    base 'plane'  the root XY construction plane
    base 'plain'  the top face of a plain box
    """
    def run(des):
        params = dict(job.get('params') or {})
        G = M.design(params)
        rt = (0.5 * G['pinion']['d'] + G['mn']) * MM
        t = 0.5 * MM
        kind = job.get('base', 'toy')
        base_body = None
        if kind == 'toy':
            z = int(job.get('toy_z', G['z1']))
            base_body = _toy_gear(des, z, rt, rt - 2.25 * G['mn'] * MM,
                                  float(job.get('toy_phase', 0.05)), t, 'helical A')
            ent = max((f for f in base_body.faces
                       if adsk.core.Plane.cast(f.geometry)),
                      key=lambda f: f.centroid.z)
        elif kind == 'plane':
            ent = des.rootComponent.xYConstructionPlane
        else:
            tbm = adsk.fusion.TemporaryBRepManager.get()
            obb = adsk.core.OrientedBoundingBox3D.create(
                _pt((0, 0, -0.5 * t)), _vec((1, 0, 0)), _vec((0, 1, 0)),
                1.2, 0.8, t)
            root = des.rootComponent
            bf = root.features.baseFeatures.add()
            bf.startEdit()
            root.bRepBodies.add(tbm.createBox(obb), bf)
            bf.finishEdit()
            base_body = root.bRepBodies.item(root.bRepBodies.count - 1)
            ent = max((f for f in base_body.faces
                       if adsk.core.Plane.cast(f.geometry)),
                      key=lambda f: f.centroid.z)
        clear = []
        if job.get('with_B', True):
            d = G['pinion']['d'] * MM
            b = _cylinder_body(des, (d, -0.5 * t, -0.5 * t),
                               (d, 0.5 * t, -0.5 * t), rt, 'helical B')
            clear.append(next(f for f in b.faces
                              if adsk.core.Plane.cast(f.geometry)))
        params['position'] = job.get('position', 180.0)
        inp = _fake_inputs(ent, clear, params)
        H = read_base(ent, G['z1'], False)
        out = {'rows': base_rows(inp, G) + swept_rows(inp),
               'base': {'kind': H['kind'], 'aligned': H['aligned'],
                        'phase': H['phase'],
                        'count': H['gear']['count'] if H['gear'] else None,
                        'centre_mm': [v * CM for v in H['centre']],
                        'n': H['n']}}
        if job.get('build', True):
            P = resolve(inp)
            made = build_pair(des, P)
            out['interference'] = interference_report(des, made, base_body)
            out['solid'] = {k: m['body'].isSolid for k, m in made.items()}
            out['pinion_heel_z_mm'] = made['pinion']['body'].boundingBox.minPoint.z * CM
            if job.get('glass'):
                out['glass'] = apply_preview_appearance(des, made)
                ap = made['gear']['body'].nativeObject.appearance
                cols = [p for p in ap.appearanceProperties
                        if p.objectType.split('::')[-1] == 'ColorProperty']
                c = cols[-1].value if cols else None
                out['glass_tint'] = [c.red, c.green, c.blue] if c else None
                out['body_appearance'] = ap.name
        return out
    return _in_scratch(app, job, run)


def _library_appearance(des, words):
    """First library appearance whose name contains every word, copied in."""
    app = adsk.core.Application.get()
    for i in range(app.materialLibraries.count):
        lib = app.materialLibraries.item(i)
        for k in range(lib.appearances.count):
            a = lib.appearances.item(k)
            if all(w.lower() in a.name.lower() for w in words):
                try:
                    return des.appearances.addByCopy(a, a.name + ' (icon)')
                except Exception:
                    return a
    return None


def dev_appearances(app, job):
    """Library appearance names containing a word - to choose render looks."""
    out = []
    for i in range(app.materialLibraries.count):
        lib = app.materialLibraries.item(i)
        for k in range(lib.appearances.count):
            n = lib.appearances.item(k).name
            if job.get('word', '').lower() in n.lower():
                out.append(n)
    return out[:int(job.get('limit', 60))]


def dev_render(app, job):
    """Build a pair and save a transparent render of it - the icon source."""
    def run(des):
        G = M.design(job.get('params'))
        H = {'n': (0.0, 0.0, 1.0), 'centre': (0.0, 0.0, 0.0),
             'x0': (1.0, 0.0, 0.0), 'phase': 0.0, 'body': None, 'kind': 'dev',
             'gear': None, 'envelope': None, 'aligned': False}
        P = place(G, H, math.radians(job.get('position', 0.0)))
        made = build_pair(des, P)
        for key, words in (('gear', job.get('gear_look', ['Blue'])),
                           ('pinion', job.get('pinion_look', ['Aluminum']))):
            ap = _library_appearance(des, words)
            if ap is not None:
                made[key]['body'].nativeObject.appearance = ap
        try:
            des.rootComponent.isOriginFolderLightBulbOn = False
        except Exception:
            pass
        vp = app.activeViewport
        cam = vp.camera
        cam.isSmoothTransition = False
        e = job.get('eye', [1.0, -1.3, 0.9])
        t = job.get('target', [0.0, 0.0, 0.0])
        cam.target = _pt(t)
        cam.eye = _pt(e)
        cam.upVector = _vec(job.get('up', [0.0, 0.0, 1.0]))
        vp.camera = cam
        vp.fit()
        adsk.doEvents()
        vp.refresh()
        opts = adsk.core.SaveImageFileOptions.create(job['out'])
        opts.width = int(job.get('size', 1024))
        opts.height = int(job.get('size', 1024))
        opts.isBackgroundTransparent = True
        opts.isAntiAliased = True
        ok = vp.saveAsImageFileWithOptions(opts)
        return {'ok': ok, 'file': job['out'],
                'looks': [made[k]['body'].nativeObject.appearance.name
                          for k in ('gear', 'pinion')]}
    return _in_scratch(app, job, run)


DEV_COMMANDS = {
    'appearances': dev_appearances,
    'render': dev_render,
    'layout': dev_layout,
    'checkcheck': dev_checkcheck,
    'probe': dev_probe,
    'ping': dev_ping,
    'build': dev_build,
    'sweep': dev_sweep,
}
