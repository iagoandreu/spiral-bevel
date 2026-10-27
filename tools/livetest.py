"""Live checks inside Fusion, through the dev bridge. Each job opens a NEW
document and closes it unsaved; the active document is never touched.

    py -3.10 tools/livetest.py

Every check has an answer known in advance: bodies must be solid, the pair must
not overlap at the design backlash but MUST overlap with the teeth fattened (so a
zero proves clearance, not that the gears are apart), and against stand-ins for
the two helical gears the swept check and the real solids must agree.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sb as client                                          # noqa: E402

FAILED = []


def check(name, cond, detail=''):
    print('  %s %s%s' % ('ok  ' if cond else 'FAIL', name,
                         '   %s' % (detail,) if detail != '' else ''))
    if not cond:
        FAILED.append(name)


def job(cmd, payload):
    r = client.send(cmd, payload)
    if not r.get('ok'):
        print(r.get('error'))
        raise SystemExit(1)
    return r['result']


def mesh(res):
    return [v for a, b, v in res['interference'] if 'mesh' in b][0]


def main():
    if not client.alive():
        print('The dev bridge is not answering - is Spiral Bevel running?')
        return 2
    cases = ((({'by': 'cone', 'delta1': 45}), 0, 'cone 45'),
             (({'by': 'cone', 'delta1': 30}), 37, 'cone 30'),
             (({'by': 'cone', 'delta1': 60, 'hand': -1}), 200, 'cone 60 left'),
             (({'by': 'cone', 'delta1': 40, 'sigma': 75}), 37, 'cone 40 shaft 75'),
             (({'z1': 12, 'z2': 25}), 33, 'teeth 12:25'),
             (({'z1': 12, 'z2': 25, 'sigma': 75}), 33, 'teeth 12:25 shaft 75'))
    for p, pos, tag in cases:
        r = job('build', {'params': p, 'position': pos})
        check('%s: both bodies solid' % tag,
              all(b['solid'] for b in r['bodies'].values()))
        check('%s: no overlap at the design backlash' % tag, mesh(r) < 1e-6,
              mesh(r))
        r = job('build', {'params': dict(p, backlash=-0.04), 'position': pos})
        check('%s: fattened teeth DO overlap - the pair is engaged' % tag,
              mesh(r) > 1e-3, mesh(r))

    def num(text):
        return float(text.split('<b>')[-1].split()[0])

    r = job('layout', {'base': 'toy', 'toy_phase': 0.05,
                       'params': {'by': 'cone', 'delta1': 45}})
    rows = dict(r['rows'])
    real = {(a, b): v for a, b, v in r['interference']}
    check('a toothed end face is recognised as a 25-tooth gear',
          r['base']['count'] == 25 and r['base']['aligned'], r['base'])
    check('and its tooth centre is found where it was put, 0.05 rad',
          abs(r['base']['phase'] - 0.05) < 1e-6, r['base']['phase'])
    check('no neck: the swept check says the pinion overlaps B',
          'OVERLAPS' in rows['pinion vs helical B'], rows['pinion vs helical B'])
    check('and the real solids agree', real[('pinion', 'helical B')] > 1e-4,
          real[('pinion', 'helical B')])
    check('the neck needed at 45 deg is the offline 0.567 mm',
          abs(num(rows['neck needed']) - 0.567) < 0.001, rows['neck needed'])
    r = job('layout', {'base': 'toy', 'glass': True,
                       'params': {'by': 'cone', 'delta1': 45, 'gap': 0.62}})
    rows = dict(r['rows'])
    real = {(a, b): v for a, b, v in r['interference']}
    check('with 0.62 mm the pinion clears B, swept and real',
          'clear' in rows['pinion vs helical B']
          and real[('pinion', 'helical B')] < 1e-6, rows['pinion vs helical B'])
    check('the preview glass is on the bodies, tinted 51 251 255',
          r['body_appearance'] == 'Spiral Bevel Preview (glass)'
          and r['glass_tint'] == [51, 251, 255], (r['glass'], r['glass_tint']))
    r = job('layout', {'base': 'toy', 'position': 0,
                       'params': {'by': 'cone', 'delta1': 45, 'gap': 0.62}})
    check('a gear put on B\'s side overlaps it',
          'OVERLAPS' in dict(r['rows'])['gear vs helical B'],
          dict(r['rows'])['gear vs helical B'])
    r = job('layout', {'base': 'toy', 'toy_z': 25, 'params': {'z1': 12, 'z2': 25}})
    check('a gear face with a different count is centred on, not lined up',
          r['base']['count'] == 25 and not r['base']['aligned'], r['base'])
    for base in ('plane', 'plain'):
        r = job('layout', {'base': base, 'with_B': False,
                           'params': {'z1': 12, 'z2': 25}})
        check('on a %s: centred at the origin, heel on it, both solid' % base,
              max(abs(v) for v in r['base']['centre_mm']) < 1e-9
              and abs(r['pinion_heel_z_mm']) < 1e-6 and all(r['solid'].values()),
              (r['base']['kind'], r['pinion_heel_z_mm']))
    print()
    print('%d failed' % len(FAILED) if FAILED else 'all live checks passed')
    return 1 if FAILED else 0


if __name__ == '__main__':
    raise SystemExit(main())
