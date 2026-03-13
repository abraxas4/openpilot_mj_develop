#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import glob
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
  blocked_frames: int = 0
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
      self.blocked_frames += 1
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
  print(f"controls_blocked_frames={s.blocked_frames} ({pct(s.blocked_frames, s.command_frames)})")
  print(f"effective_lateral_frames={s.effective_frames} ({pct(s.effective_frames, s.command_frames)})")
  print(f"interruptRateCan2_fault_frames={s.interrupt_rate_can2_fault_frames} ({pct(s.interrupt_rate_can2_fault_frames, s.frames)})")


def has_lateral_command(cc: log.Event.carControl) -> bool:
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


def analyze(rlog_path: str, low_speed_ms: float, high_speed_ms: float, window_s: float) -> None:
  low_windows: list[Window] = []
  high_windows: list[Window] = []
  post_cancel_windows: list[Window] = []

  stats_all = Stats()
  stats_low = Stats()
  stats_high = Stats()
  stats_post_cancel = Stats()

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

  print(f"rlog={rlog_path}")
  print(f"low_speed_engage_windows={len(low_windows)}, high_speed_engage_windows={len(high_windows)}, post_cancel_windows={len(post_cancel_windows)}")

  print_stats("ALL", stats_all)
  print_stats(f"LOW_ENGAGE_FIRST_{int(window_s)}S", stats_low)
  print_stats(f"HIGH_ENGAGE_FIRST_{int(window_s)}S", stats_high)
  print_stats(f"POST_CANCEL_{int(window_s)}S", stats_post_cancel)

  print("\n[Interpretation Guide]")
  print("- If op_command_frames > 0 and effective_lateral_frames is very low, OP generated steering commands but they were rarely applied.")
  print("- If controls_blocked_frames is high with interruptRateCan2_fault_frames, panda safety gate blocking is likely.")
  print("- If POST_CANCEL effective_lateral_frames drops sharply versus LOW/HIGH windows, inspect post-cancel lateral re-entry path.")


def main() -> None:
  parser = argparse.ArgumentParser(description="Analyze whether openpilot lateral commands were accepted")
  parser.add_argument("--rlog", type=str, default="", help="Path to rlog(.zst/.bz2/.rlog)")
  parser.add_argument("--realdata-root", type=str, default="/data/media/0/realdata", help="Root to auto-pick latest rlog")
  parser.add_argument("--low-speed-ms", type=float, default=12.0, help="Engage speed threshold for low-speed window")
  parser.add_argument("--high-speed-ms", type=float, default=20.0, help="Engage speed threshold for high-speed window")
  parser.add_argument("--window-sec", type=float, default=10.0, help="Duration of each analysis window")
  args = parser.parse_args()

  rlog_path = args.rlog.strip() if args.rlog else pick_latest_rlog(args.realdata_root)
  analyze(rlog_path, args.low_speed_ms, args.high_speed_ms, args.window_sec)


if __name__ == "__main__":
  main()
