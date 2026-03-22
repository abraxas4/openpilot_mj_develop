import os
import sys
import types

# Minimal stubs for unix-only imports pulled in through swaglog on Windows
if not hasattr(os, 'register_at_fork'):
  os.register_at_fork = lambda *args, **kwargs: None

fcntl = types.ModuleType('fcntl')
fcntl.flock = lambda *args, **kwargs: None
fcntl.LOCK_EX = 0
fcntl.LOCK_NB = 0
fcntl.LOCK_UN = 0
sys.modules['fcntl'] = fcntl

from tools.lib.logreader import LogReader

routes = ['0000008a--7c87b1fea7', '0000008b--9879b4faa9']
base = r'S:/github/openpilot_mj_develop/tmp/comma_logs'

for route in routes:
  print('ROUTE', route)
  for seg in range(10):
    path = os.path.join(base, f'{route}--{seg}', 'qlog.zst')
    if not os.path.exists(path):
      continue
    try:
      for m in LogReader(path):
        if m.which() == 'errorLogMessage':
          txt = str(m.errorLogMessage)
          if any(s in txt.lower() for s in ('ui', 'traceback', 'unknownkeyname', 'restart')):
            print('SEG', seg, txt)
    except Exception as e:
      print('SEG', seg, 'ERROR', repr(e))
  print()
