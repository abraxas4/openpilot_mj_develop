#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import glob
import os
from dataclasses import dataclass

from cereal import car, log
from openpilot.tools.lib.logreader import LogReader


IGNORED_SAFETY_MODELS = {
  car.CarParams.SafetyModel.silent,
  car.CarParams.SafetyModel.noOutput,
}


@dataclass
class Window:
  start_ns: int
  end_ns: int


@dataclass
class Stats:
  frames: int = 0
  enabled_frames: int = 0
  lat_active_frames: int = 0
  command_frames: int = 0
  blocked_enabled_frames: int = 0
  blocked_command_frames: int = 0
  effective_frames: int = 0
  interrupt_rate_can2_fault_frames: int = 0

  def add(self, enabled: bool, lat_active: bool, has_command: bool, blocked: bool, interrupt_rate_fault: bool) -> None:
    self.frames += 1
    if enabled:
      self.enabled_frames += 1
    if lat_active:
      self.lat_active_frames += 1
    if has_command:
      self.command_frames += 1
    if blocked:
      self.blocked_enabled_frames += 1
    if blocked and has_command:
      self.blocked_command_frames += 1
    if has_command and not blocked:
      self.effective_frames += 1
    if interrupt_rate_fault:
      self.interrupt_rate_can2_fault_frames += 1


def pick_latest_rlog(realdata_root: str) -> str:
  candidates = []
  for ext in ("*.zst", "*.bz2", "*.rlog"):
    candidates.extend(glob.glob(f"{realdata_root}/**/rlog.{ext}".replace("..", "."), recursive=True))
    candidates.extend(glob.glob(f"{realdata_root}/**/rlog{ext[1:]}".replace("..", "."), recursive=True))

  cleaned = sorted(set(candidates))
  if not cleaned:
    raise FileNotFoundError(f"No rlog files found under: {realdata_root}")
  return max(cleaned, key=lambda p: (len(p), p))


def in_any_window(t_ns: int, windows: list[Window]) -> bool:
  return any(w.start_ns <= t_ns <= w.end_ns for w in windows)


def pct(n: int, d: int) -> str:
  if d <= 0:
    return "0.0%"
  return f"{(100.0 * n / d):.1f}%"


def print_stats(name: str, s: Stats) -> None:
  print(f"\n[{name}]")
  print(f"frames={s.frames}, enabled={s.enabled_frames}, latActive={s.lat_active_frames}")
  print(f"op_command_frames={s.command_frames}")
  print(f"controls_blocked_enabled_frames={s.blocked_enabled_frames} ({pct(s.blocked_enabled_frames, s.enabled_frames)})")
  print(f"command_blocked_frames={s.blocked_command_frames} ({pct(s.blocked_command_frames, s.command_frames)})")
  print(f"effective_lateral_frames={s.effective_frames} ({pct(s.effective_frames, s.command_frames)})")
  print(f"interruptRateCan2_fault_frames={s.interrupt_rate_can2_fault_frames} ({pct(s.interrupt_rate_can2_fault_frames, s.frames)})")


def effective_ratio(s: Stats) -> float:
  return (s.effective_frames / s.command_frames) if s.command_frames > 0 else 0.0


def print_ranked_summary(results: list[tuple[str, Stats, Stats]], top_n: int) -> None:
  with_commands = [(path, all_stats, post_stats) for path, all_stats, post_stats in results if all_stats.command_frames > 0]
  if not with_commands:
    print("\n[Route Summary]")
    print("- No segments with OP lateral command frames were found.")
    return

  worst_all = sorted(with_commands, key=lambda x: effective_ratio(x[1]))[:top_n]
  print(f"\n[Route Summary] worst_overall_effective_top_{len(worst_all)}")
  for path, all_stats, _ in worst_all:
    print(f"- {path}: effective={pct(all_stats.effective_frames, all_stats.command_frames)}, "
          f"cmd_blocked={pct(all_stats.blocked_command_frames, all_stats.command_frames)}, "
          f"commands={all_stats.command_frames}")

  with_post_cancel = [(path, post_stats) for path, _, post_stats in with_commands if post_stats.command_frames > 0]
  if with_post_cancel:
    worst_post = sorted(with_post_cancel, key=lambda x: effective_ratio(x[1]))[:top_n]
    print(f"\n[Route Summary] worst_post_cancel_effective_top_{len(worst_post)}")
    for path, post_stats in worst_post:
      print(f"- {path}: effective={pct(post_stats.effective_frames, post_stats.command_frames)}, "
            f"cmd_blocked={pct(post_stats.blocked_command_frames, post_stats.command_frames)}, "
            f"commands={post_stats.command_frames}")


def verdict_text(s: Stats) -> str:
  if s.command_frames == 0:
    return "No OP lateral commands observed (likely stock ADAS/manual for this speed bin)."

  eff = effective_ratio(s)
  if eff >= 0.8:
    return "OP lateral intervention is likely active for most commanded frames."
  if eff <= 0.2:
    return "OP lateral commands are mostly blocked (stock ADAS/safety gate likely dominates)."
  return "Mixed behavior: OP commands are partially applied and partially blocked."


def has_lateral_command(cc) -> bool:
  if not cc.latActive:
    return False

  actuators = cc.actuators
  torque = abs(getattr(actuators, "torque", 0.0)) > 1e-3
  angle = abs(getattr(actuators, "steeringAngleDeg", 0.0)) > 0.02
  curvature = abs(getattr(actuators, "curvature", 0.0)) > 1e-6
  return torque or angle or curvature


def controls_blocked(ps_list: list[log.PandaState]) -> tuple[bool, bool]:
  active = [ps for ps in ps_list if ps.safetyModel not in IGNORED_SAFETY_MODELS]
  if not active:
    return False, False

  blocked = any(not ps.controlsAllowed for ps in active)
  interrupt = False
  fault_enum = getattr(log.PandaState.FaultType, "interruptRateCan2", None)
  if fault_enum is not None:
    interrupt = any(any(f == fault_enum for f in ps.faults) for ps in active)
  return blocked, interrupt


def analyze(rlog_path: str,
            low_speed_ms: float,
            high_speed_ms: float,
            window_s: float,
            low_bin_ms: float,
            high_bin_ms: float) -> tuple[Stats, Stats]:
  low_windows: list[Window] = []
  high_windows: list[Window] = []
  post_cancel_windows: list[Window] = []

  stats_all = Stats()
  stats_low = Stats()
  stats_high = Stats()
  stats_post_cancel = Stats()
  stats_speed_bin_low = Stats()
  stats_speed_bin_high = Stats()

  latest_enabled = False
  prev_enabled = False
  latest_v_ego = 0.0
  latest_panda_states: list[log.PandaState] = []

  cancel_button_names = {"cancel", "cancelCruise"}
  window_ns = int(window_s * 1e9)

  for msg in LogReader(rlog_path):
    which = msg.which()
    t_ns = msg.logMonoTime

    if which == "selfdriveState":
      latest_enabled = bool(msg.selfdriveState.enabled)
      if latest_enabled and not prev_enabled:
        w = Window(t_ns, t_ns + window_ns)
        if latest_v_ego < low_speed_ms:
          low_windows.append(w)
        if latest_v_ego >= high_speed_ms:
          high_windows.append(w)
      prev_enabled = latest_enabled

    elif which == "carState":
      latest_v_ego = float(msg.carState.vEgo)
      for be in msg.carState.buttonEvents:
        be_name = str(be.type)
        if be_name in cancel_button_names:
          post_cancel_windows.append(Window(t_ns, t_ns + window_ns))

    elif which == "pandaStates":
      latest_panda_states = list(msg.pandaStates)

    elif which == "carControl":
      cc = msg.carControl
      blocked, interrupt = controls_blocked(latest_panda_states)
      enabled = latest_enabled
      lat_active = bool(cc.latActive)
      has_command = has_lateral_command(cc)

      stats_all.add(enabled, lat_active, has_command, enabled and blocked, interrupt)

      if in_any_window(t_ns, low_windows):
        stats_low.add(enabled, lat_active, has_command, enabled and blocked, interrupt)
      if in_any_window(t_ns, high_windows):
        stats_high.add(enabled, lat_active, has_command, enabled and blocked, interrupt)
      if in_any_window(t_ns, post_cancel_windows):
        stats_post_cancel.add(enabled, lat_active, has_command, enabled and blocked, interrupt)
      if latest_v_ego <= low_bin_ms:
        stats_speed_bin_low.add(enabled, lat_active, has_command, enabled and blocked, interrupt)
      if latest_v_ego >= high_bin_ms:
        stats_speed_bin_high.add(enabled, lat_active, has_command, enabled and blocked, interrupt)

  print(f"rlog={rlog_path}")
  print(f"low_speed_engage_windows={len(low_windows)}, high_speed_engage_windows={len(high_windows)}, post_cancel_windows={len(post_cancel_windows)}")

  print_stats("ALL", stats_all)
  print_stats(f"LOW_ENGAGE_FIRST_{int(window_s)}S", stats_low)
  print_stats(f"HIGH_ENGAGE_FIRST_{int(window_s)}S", stats_high)
  print_stats(f"POST_CANCEL_{int(window_s)}S", stats_post_cancel)
  print_stats(f"SPEED_LE_{int(low_bin_ms * 3.6)}KPH", stats_speed_bin_low)
  print_stats(f"SPEED_GE_{int(high_bin_ms * 3.6)}KPH", stats_speed_bin_high)

  print("\n[Speed Bin Verdict]")
  print(f"- <= {int(low_bin_ms * 3.6)} km/h: {verdict_text(stats_speed_bin_low)}")
  print(f"- >= {int(high_bin_ms * 3.6)} km/h: {verdict_text(stats_speed_bin_high)}")

  if stats_all.enabled_frames == 0:
    print("\n[Note]")
    print("- No enabled frames found in this rlog. This segment cannot be used to judge OP steering intervention.")
    print("- Analyze all segments in the same route with --rlog-glob '/data/media/0/realdata/<route>--*/rlog.zst'.")

  print("\n[Interpretation Guide]")
  print("- If op_command_frames > 0 and effective_lateral_frames is very low, OP generated steering commands but they were rarely applied.")
  print("- If controls_blocked_frames is high with interruptRateCan2_fault_frames, panda safety gate blocking is likely.")
  print("- If POST_CANCEL effective_lateral_frames drops sharply versus LOW/HIGH windows, inspect post-cancel lateral re-entry path.")

  return stats_all, stats_post_cancel


def route_glob_from_rlog_path(rlog_path: str) -> str:
  seg_dir = os.path.basename(os.path.dirname(rlog_path))
  if "--" not in seg_dir:
    return ""

  route_prefix = seg_dir.rsplit("--", 1)[0]
  parent = os.path.dirname(os.path.dirname(rlog_path))
  rlog_name = os.path.basename(rlog_path)
  return os.path.join(parent, f"{route_prefix}--*", rlog_name)


def route_id_from_rlog_path(rlog_path: str) -> str:
  seg_dir = os.path.basename(os.path.dirname(rlog_path))
  if "--" not in seg_dir:
    return seg_dir
  return seg_dir.rsplit("--", 1)[0]


def collect_recent_route_patterns(realdata_root: str,
                                  recent_routes: int,
                                  min_route_hours: float) -> list[tuple[str, str, int, float]]:
  rlogs = sorted(glob.glob(os.path.join(realdata_root, "*--*", "rlog.zst")))
  if not rlogs:
    rlogs = sorted(glob.glob(os.path.join(realdata_root, "*--*", "rlog.bz2")))
  if not rlogs:
    rlogs = sorted(glob.glob(os.path.join(realdata_root, "*--*", "rlog")))

  route_map: dict[str, list[str]] = {}
  for rp in rlogs:
    rid = route_id_from_rlog_path(rp)
    route_map.setdefault(rid, []).append(rp)

  min_segments = int(min_route_hours * 60.0)
  route_meta: list[tuple[float, str, int, str]] = []
  for rid, segs in route_map.items():
    seg_count = len(segs)
    if min_segments > 0 and seg_count < min_segments:
      continue
    latest_mtime = max(os.path.getmtime(p) for p in segs)
    route_meta.append((latest_mtime, rid, seg_count, os.path.basename(segs[0])))

  route_meta.sort(reverse=True)
  selected = route_meta[:recent_routes]

  results: list[tuple[str, str, int, float]] = []
  for _, rid, seg_count, rlog_name in selected:
    pattern = os.path.join(realdata_root, f"{rid}--*", rlog_name)
    est_hours = seg_count / 60.0
    results.append((rid, pattern, seg_count, est_hours))
  return results


def main() -> None:
  parser = argparse.ArgumentParser(description="Analyze whether openpilot lateral commands were accepted")
  parser.add_argument("--rlog", type=str, default="", help="Path to rlog(.zst/.bz2/.rlog)")
  parser.add_argument("--rlog-glob", type=str, default="", help="Glob pattern for multiple rlogs (e.g. /data/media/0/realdata/<route>--*/rlog.zst)")
  parser.add_argument("--realdata-root", type=str, default="/data/media/0/realdata", help="Root to auto-pick latest rlog")
  parser.add_argument("--low-speed-ms", type=float, default=12.0, help="Engage speed threshold for low-speed window")
  parser.add_argument("--high-speed-ms", type=float, default=20.0, help="Engage speed threshold for high-speed window")
  parser.add_argument("--window-sec", type=float, default=10.0, help="Duration of each analysis window")
  parser.add_argument("--low-bin-kph", type=float, default=30.0, help="Speed-bin lower range upper bound in kph")
  parser.add_argument("--high-bin-kph", type=float, default=80.0, help="Speed-bin high range lower bound in kph")
  parser.add_argument("--summary-top", type=int, default=8, help="How many worst segments to show in route summary")
  parser.add_argument("--recent-routes", type=int, default=0, help="Analyze N most recent routes from --realdata-root")
  parser.add_argument("--min-route-hours", type=float, default=0.0, help="Only include routes with at least this estimated duration in hours")
  args = parser.parse_args()

  low_bin_ms = args.low_bin_kph / 3.6
  high_bin_ms = args.high_bin_kph / 3.6

  if args.recent_routes > 0:
    route_patterns = collect_recent_route_patterns(args.realdata_root, args.recent_routes, args.min_route_hours)
    if not route_patterns:
      raise FileNotFoundError("No routes matched recent/duration filters")

    print(f"selected_routes={len(route_patterns)}")
    for idx, (rid, pattern, seg_count, est_hours) in enumerate(route_patterns, start=1):
      print(f"\n===== [route {idx}/{len(route_patterns)}] {rid} | segments={seg_count} | est_hours={est_hours:.2f} =====")
      rlogs = sorted(glob.glob(pattern))
      if not rlogs:
        print(f"no rlogs matched route pattern: {pattern}")
        continue
      print(f"matched_rlogs={len(rlogs)}")
      results: list[tuple[str, Stats, Stats]] = []
      for i, rlog_path in enumerate(rlogs, start=1):
        print(f"\n=== [{i}/{len(rlogs)}] ===")
        stats_all, stats_post_cancel = analyze(rlog_path, args.low_speed_ms, args.high_speed_ms, args.window_sec,
                                               low_bin_ms, high_bin_ms)
        results.append((rlog_path, stats_all, stats_post_cancel))
      print_ranked_summary(results, args.summary_top)

  elif args.rlog_glob:
    rlogs = sorted(glob.glob(args.rlog_glob))
    if not rlogs:
      raise FileNotFoundError(f"No rlog files matched --rlog-glob: {args.rlog_glob}")
    print(f"matched_rlogs={len(rlogs)}")
    results: list[tuple[str, Stats, Stats]] = []
    for i, rlog_path in enumerate(rlogs, start=1):
      print(f"\n=== [{i}/{len(rlogs)}] ===")
      stats_all, stats_post_cancel = analyze(rlog_path, args.low_speed_ms, args.high_speed_ms, args.window_sec,
                                             low_bin_ms, high_bin_ms)
      results.append((rlog_path, stats_all, stats_post_cancel))
    print_ranked_summary(results, args.summary_top)
  else:
    rlog_path = args.rlog.strip() if args.rlog else pick_latest_rlog(args.realdata_root)
    stats_all, _ = analyze(rlog_path, args.low_speed_ms, args.high_speed_ms, args.window_sec,
                           low_bin_ms, high_bin_ms)

    if stats_all.enabled_frames == 0:
      inferred_glob = route_glob_from_rlog_path(rlog_path)
      if inferred_glob:
        rlogs = sorted(glob.glob(inferred_glob))
        if len(rlogs) > 1:
          print(f"\n[AutoFallback] matched_rlogs={len(rlogs)} from route glob: {inferred_glob}")
          results: list[tuple[str, Stats, Stats]] = []
          for i, rp in enumerate(rlogs, start=1):
            print(f"\n=== [auto {i}/{len(rlogs)}] ===")
            auto_stats_all, auto_stats_post_cancel = analyze(rp, args.low_speed_ms, args.high_speed_ms, args.window_sec,
                                                              low_bin_ms, high_bin_ms)
            results.append((rp, auto_stats_all, auto_stats_post_cancel))
          print_ranked_summary(results, args.summary_top)


if __name__ == "__main__":
  main()
