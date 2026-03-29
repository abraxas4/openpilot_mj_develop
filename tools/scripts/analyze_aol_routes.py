#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bz2
from collections import Counter, defaultdict, deque
from pathlib import Path
import re
import urllib.parse

import capnp
import zstandard as zstd
from cereal import log as capnp_log


SEGMENT_DIR_RE = re.compile(r'(?P<route>.+)--(?P<seg>\d+)$')
FLAT_QLOG_RE = re.compile(r'(?P<route>.+)--(?P<seg>\d+)\.qlog\.(?:zst|bz2)$')


def decompress_stream(data: bytes) -> bytes:
	dctx = zstd.ZstdDecompressor()
	with dctx.stream_reader(data) as reader:
		return reader.read()


class CachedEventReader:
	__slots__ = ('_evt', '_enum')

	def __init__(self, evt: capnp._DynamicStructReader, _enum: str | None = None):
		self._evt = evt
		self._enum = _enum

	def which(self) -> str:
		if self._enum is None:
			self._enum = self._evt.which()
		return self._enum

	def __getattr__(self, name: str):
		return getattr(self._evt, name)


class LocalLogFileReader:
	def __init__(self, fn: str):
		with open(fn, 'rb') as f:
			dat = f.read()

		ext = Path(urllib.parse.urlparse(fn).path).suffix
		if ext == '.bz2' or dat.startswith(b'BZh9'):
			dat = bz2.decompress(dat)
		elif ext == '.zst' or dat.startswith(b'\x28\xB5\x2F\xFD'):
			dat = decompress_stream(dat)

		self._ents: list[CachedEventReader] = []
		ents = capnp_log.Event.read_multiple_bytes(dat)
		try:
			for e in ents:
				self._ents.append(CachedEventReader(e))
		except capnp.KjException:
			pass

	def __iter__(self):
		yield from self._ents


def default_state() -> dict:
	return {
		'seg': None,
		't': None,
		'vEgo': 0.0,
		'standstill': False,
		'brakePressed': False,
		'brakeHoldActive': False,
		'gasPressed': False,
		'steeringPressed': False,
		'steerFaultTemporary': False,
		'steerFaultPermanent': False,
		'cruise_enabled': False,
		'selfdrive_enabled': False,
		'selfdrive_active': False,
		'fp_enabled': False,
		'fp_allowed': False,
		'cc_enabled': False,
		'cc_lat_active': False,
		'controls_allowed': False,
		'faults': [],
		'events': [],
	}


def parse_qlog_path(path: Path) -> tuple[str, int] | None:
	if path.name in {'qlog.zst', 'qlog.bz2'}:
		match = SEGMENT_DIR_RE.match(path.parent.name)
		if match is None:
			return None
		return match.group('route'), int(match.group('seg'))

	match = FLAT_QLOG_RE.match(path.name)
	if match is None:
		return None
	return match.group('route'), int(match.group('seg'))


def discover_routes(data_dir: Path) -> dict[str, dict[int, Path]]:
	route_files: dict[str, dict[int, Path]] = defaultdict(dict)
	for path in data_dir.rglob('*'):
		if not path.is_file():
			continue
		parsed = parse_qlog_path(path)
		if parsed is None:
			continue
		route, seg = parsed
		route_files[route][seg] = path
	return dict(route_files)


def select_routes(route_files: dict[str, dict[int, Path]], requested_routes: list[str] | None, last_routes: int) -> list[str]:
	if requested_routes:
		return [r for r in requested_routes if r in route_files]

	ranked = sorted(
		route_files,
		key=lambda route: max(path.stat().st_mtime for path in route_files[route].values()),
		reverse=True,
	)
	return ranked[:last_routes]


def analyze_route(route: str, seg_map: dict[int, Path], max_cases: int) -> tuple[Counter, list[dict], list[Path]]:
	latest = default_state()
	prev = None
	ring: deque[dict] = deque(maxlen=8)
	summary: Counter = Counter()
	cases: list[dict] = []
	segments = [seg_map[seg] for seg in sorted(seg_map)]

	for qlog in segments:
		seg = parse_qlog_path(qlog)[1]
		for m in LocalLogFileReader(str(qlog)):
			which = m.which()
			mono = getattr(m, 'logMonoTime', None)
			latest['t'] = int(mono) if mono is not None else latest['t']
			latest['seg'] = seg

			if which == 'carState':
				c = m.carState
				latest['vEgo'] = round(float(c.vEgo), 3)
				latest['standstill'] = bool(c.standstill)
				latest['brakePressed'] = bool(c.brakePressed)
				latest['brakeHoldActive'] = bool(c.brakeHoldActive)
				latest['gasPressed'] = bool(c.gasPressed)
				latest['steeringPressed'] = bool(c.steeringPressed)
				latest['steerFaultTemporary'] = bool(c.steerFaultTemporary)
				latest['steerFaultPermanent'] = bool(c.steerFaultPermanent)
				latest['cruise_enabled'] = bool(c.cruiseState.enabled)
			elif which == 'selfdriveState':
				s = m.selfdriveState
				latest['selfdrive_enabled'] = bool(s.enabled)
				latest['selfdrive_active'] = bool(s.active)
			elif which == 'frogpilotCarState':
				f = m.frogpilotCarState
				latest['fp_enabled'] = bool(f.alwaysOnLateralEnabled)
				latest['fp_allowed'] = bool(f.alwaysOnLateralAllowed)
			elif which == 'pandaStates' and len(m.pandaStates):
				p = m.pandaStates[0]
				latest['controls_allowed'] = bool(p.controlsAllowed)
				latest['faults'] = [str(x) for x in p.faults]
			elif which == 'onroadEvents':
				try:
					latest['events'] = [str(e.name) for e in m.onroadEvents]
				except Exception:
					latest['events'] = [str(e) for e in m.onroadEvents]
			elif which == 'carControl':
				c = m.carControl
				latest['cc_enabled'] = bool(c.enabled)
				latest['cc_lat_active'] = bool(c.latActive)
				current = latest.copy()
				ring.append(current.copy())

				if current['cc_lat_active']:
					summary['lat_active_true'] += 1
				if current['steerFaultTemporary']:
					summary['steer_fault_temporary_samples'] += 1
				if current['steeringPressed'] and current['cc_lat_active']:
					summary['steering_pressed_while_lat_active'] += 1
				if 'interruptRateCan2' in current['faults']:
					summary['interrupt_rate_can2_samples'] += 1

				if prev is not None:
					brake_rise = (not prev['brakePressed']) and current['brakePressed']
					lat_drop = prev['cc_lat_active'] and (not current['cc_lat_active'])
					controls_drop = prev['controls_allowed'] and (not current['controls_allowed'])
					enabled_drop = prev['selfdrive_enabled'] and (not current['selfdrive_enabled'])
					temp_fault_rise = (not prev['steerFaultTemporary']) and current['steerFaultTemporary']

					if brake_rise:
						summary['brake_rising_edges'] += 1
					if controls_drop:
						summary['controls_drop'] += 1
					if enabled_drop:
						summary['enabled_drop'] += 1
					if temp_fault_rise:
						summary['temp_fault_rise'] += 1
					if lat_drop:
						summary['lat_drop'] += 1
						reasons = []
						if current['brakePressed'] or prev['brakePressed']:
							reasons.append('brake')
							summary['lat_drop_brake'] += 1
						if current['steeringPressed'] or prev['steeringPressed']:
							reasons.append('driver_steer')
							summary['lat_drop_driver_steer'] += 1
						if current['steerFaultTemporary'] or prev['steerFaultTemporary']:
							reasons.append('temp_fault')
							summary['lat_drop_temp_fault'] += 1
						if current['steerFaultPermanent'] or prev['steerFaultPermanent']:
							reasons.append('perm_fault')
							summary['lat_drop_perm_fault'] += 1
						if current['standstill'] or prev['standstill']:
							reasons.append('standstill')
							summary['lat_drop_standstill'] += 1
						if controls_drop:
							reasons.append('controls_drop')
							summary['lat_drop_controls_drop'] += 1
						if enabled_drop:
							reasons.append('enabled_drop')
							summary['lat_drop_enabled_drop'] += 1
						if 'interruptRateCan2' in current['faults'] or 'interruptRateCan2' in prev['faults']:
							reasons.append('interruptRateCan2')
							summary['lat_drop_interrupt_rate_can2'] += 1
						if not reasons:
							reasons.append('unknown')
							summary['lat_drop_unknown'] += 1

						if len(cases) < max_cases:
							cases.append({
								'reason': reasons,
								'pre': list(ring)[:-1],
								'now': current.copy(),
							})

				prev = current

	return summary, cases, segments


def build_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description='Analyze AOL-related lateral interruptions from local qlog files.')
	parser.add_argument('--data-dir', type=Path, default=Path('/data/media/0/realdata'),
											help='Root directory containing route segment directories or copied *.qlog.zst files.')
	parser.add_argument('--route', action='append', default=None,
											help='Specific route name to analyze. Can be passed multiple times.')
	parser.add_argument('--last-routes', type=int, default=1,
											help='Analyze the most recent N routes by file modification time when --route is omitted.')
	parser.add_argument('--max-cases', type=int, default=12,
											help='Maximum number of lateral-drop cases to print per route.')
	return parser


def main() -> int:
	args = build_parser().parse_args()
	route_files = discover_routes(args.data_dir)
	if not route_files:
		print(f'No qlog files found under {args.data_dir}')
		return 1

	routes = select_routes(route_files, args.route, args.last_routes)
	if not routes:
		print('No matching routes found.')
		return 1

	print('ROUTES', routes)
	for route in routes:
		summary, cases, segments = analyze_route(route, route_files[route], args.max_cases)
		print(f'=== ROUTE {route} ===')
		print('SEGMENTS', [p.name if p.name.startswith(route) else p.parent.name for p in segments])
		print('SUMMARY')
		for key in sorted(summary):
			print(key, summary[key])
		print('CASE_COUNT', len(cases))
		for idx, case in enumerate(cases, 1):
			print(f'--- LAT_DROP_CASE {idx} reason={case["reason"]} ---')
			for row in case['pre'][-4:]:
				print('PRE', row)
			print('NOW', case['now'])
		print()
	return 0


if __name__ == '__main__':
	raise SystemExit(main())
