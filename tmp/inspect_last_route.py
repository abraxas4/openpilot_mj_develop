from collections import Counter
from pathlib import Path

from tools.lib.logreader import LogReader

realdata = Path('/data/media/0/realdata')
routes = sorted({p.name.rsplit('--', 1)[0] for p in realdata.glob('*--*')})
route = routes[-1]
print(f'LAST_ROUTE {route}')

segments = sorted(realdata.glob(f'{route}--*/qlog.zst'))
print('SEGMENTS', [p.parent.name for p in segments])

for qlog in segments:
  seg = qlog.parent.name.rsplit('--', 1)[1]
  print(f'--- SEG {seg} ---')
  counts = Counter()
  alerts = []
  events = []
  errors = []
  car_events = []
  last_selfdrive = None
  last_panda = None
  last_carstate = None

  for m in LogReader(str(qlog)):
    which = m.which()
    counts[which] += 1

    if which == 'selfdriveState':
      s = m.selfdriveState
      last_selfdrive = {
        'enabled': bool(s.enabled),
        'active': bool(s.active),
        'state': str(s.state),
        'alertText1': str(s.alertText1),
        'alertText2': str(s.alertText2),
      }
    elif which == 'onroadEvents':
      try:
        ev_names = [str(e.name) for e in m.onroadEvents]
      except Exception:
        ev_names = [str(e) for e in m.onroadEvents]
      if ev_names:
        events.append(ev_names)
    elif which == 'controlsState':
      cs = m.controlsState
      alerts.append((str(cs.alertType), str(cs.alertText1), str(cs.alertText2)))
    elif which == 'carState':
      cs = m.carState
      last_carstate = {
        'vEgo': float(cs.vEgo),
        'standstill': bool(cs.standstill),
        'cruiseEnabled': bool(cs.cruiseState.enabled),
        'cruiseAvailable': bool(cs.cruiseState.available),
        'brakePressed': bool(cs.brakePressed),
        'gasPressed': bool(cs.gasPressed),
        'gear': str(cs.gearShifter),
      }
      try:
        car_events = [str(e.name) for e in cs.events]
      except Exception:
        pass
    elif which == 'pandaStates':
      ps = m.pandaStates[0]
      last_panda = {
        'controlsAllowed': bool(ps.controlsAllowed),
        'safetyModel': str(ps.safetyModel),
        'ignitionLine': bool(ps.ignitionLine),
        'ignitionCan': bool(ps.ignitionCan),
      }
    elif which == 'errorLogMessage':
      txt = str(m.errorLogMessage)
      if 'unavailable' in txt.lower() or 'ui' in txt.lower() or 'manager' in txt.lower() or 'control' in txt.lower():
        errors.append(txt)

  print('COUNTS', counts.most_common(12))
  print('LAST_SELFDRIVE', last_selfdrive)
  print('LAST_PANDA', last_panda)
  print('LAST_CARSTATE', last_carstate)
  print('CAR_EVENTS', car_events)
  if alerts:
    print('LAST_ALERTS', alerts[-5:])
  if events:
    print('LAST_ONROAD_EVENTS', events[-5:])
  if errors:
    print('ERRORS', errors[-10:])
  print()
