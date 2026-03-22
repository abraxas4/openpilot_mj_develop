from tools.lib.logreader import LogReader

routes = ['0000008a--7c87b1fea7', '0000008b--9879b4faa9']
for route in routes:
  print(f'ROUTE {route}')
  for seg in range(10):
    path = f'/data/media/0/realdata/{route}--{seg}/qlog.zst'
    try:
      lr = LogReader(path)
    except Exception:
      continue
    for m in lr:
      if m.which() == 'errorLogMessage':
        txt = str(m.errorLogMessage)
        if any(s in txt.lower() for s in ('ui', 'traceback', 'unknownkeyname', 'restart')):
          print(f'SEG {seg} {txt}')
  print()
