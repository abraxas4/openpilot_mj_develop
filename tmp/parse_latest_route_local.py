import os
import sys
import types
from collections import Counter

if not hasattr(os, 'register_at_fork'):
  os.register_at_fork = lambda *args, **kwargs: None

fcntl = types.ModuleType('fcntl')
fcntl.flock = lambda *args, **kwargs: None
fcntl.LOCK_EX = 0
fcntl.LOCK_NB = 0
fcntl.LOCK_UN = 0
sys.modules['fcntl'] = fcntl

from tools.lib.logreader import LogReader

route = '00000091--b58bf9f4c5'
base = r'S:/github/openpilot_mj_develop/tmp'
segments = sorted([d for d in os.listdir(base) if d.startswith(route + '--')])
print('ROUTE', route)
print('SEGMENTS', segments)

for segdir in segments:
  qlog = os.path.join(base, segdir, 'qlog.zst')
  if not os.path.exists(qlog):
    continue

  counts = Counter()
  last_selfdrive = None
  last_controls = None
  last_carstate = None
  last_panda = None
  onroad_events = []
  error_lines = []

  for m in LogReader(qlog):
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
    elif which == 'controlsState':
      c = m.controlsState
      last_controls = {
        'alertType': str(c.alertType),
        'alertText1': str(c.alertText1),
        'alertText2': str(c.alertText2),
      }
    elif which == 'carState':
      c = m.carState
      last_carstate = {
        'vEgo': float(c.vEgo),
        'standstill': bool(c.standstill),
        'gear': str(c.gearShifter),
        'cruiseEnabled': bool(c.cruiseState.enabled),
        'cruiseAvailable': bool(c.cruiseState.available),
        'brakePressed': bool(c.brakePressed),
        'gasPressed': bool(c.gasPressed),
      }
    elif which == 'pandaStates' and len(m.pandaStates):
      p = m.pandaStates[0]
      last_panda = {
        'controlsAllowed': bool(p.controlsAllowed),
        'safetyModel': str(p.safetyModel),
        'ignitionLine': bool(p.ignitionLine),
        'ignitionCan': bool(p.ignitionCan),
      }
    elif which == 'onroadEvents':
      try:
        evs = [str(e.name) for e in m.onroadEvents]
      except Exception:
        evs = [str(e) for e in m.onroadEvents]
      if evs:
        onroad_events.append(evs)
    elif which == 'errorLogMessage':
      txt = str(m.errorLogMessage)
      if any(k in txt.lower() for k in ('unavailable', 'controls', 'commissue', 'card', 'selfdrive', 'ui', 'manager')):
        error_lines.append(txt)

  print('\nSEG', segdir.rsplit('--', 1)[1])
  print('COUNTS', counts.most_common(12))
  print('LAST_SELFDRIVE', last_selfdrive)
  print('LAST_CONTROLS', last_controls)
  print('LAST_CARSTATE', last_carstate)
  print('LAST_PANDA', last_panda)
  print('LAST_EVENTS', onroad_events[-5:])
  print('ERRORS', error_lines[-10:])
