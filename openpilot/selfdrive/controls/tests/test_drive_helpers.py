import numpy as np

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.drive_helpers import (
  HIGH_SPEED_LIMIT_SCALE, LOW_SPEED_MAX_CURVATURE, LOW_SPEED_MAX_LAT_ACCEL,
  MAX_CURVATURE, MAX_LATERAL_ACCEL_NO_ROLL, PATH_STABLE_SAMPLES, PathSteerHelper,
  V_LIMIT_HIGH, V_LIMIT_LOW, clip_curvature, curvature_from_path_turn,
  curvature_from_path_xy, path_follow_weight, scheduled_lat_accel_limit,
  scheduled_max_curvature, speed_blend,
)


class _Pos:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class _Model:
  def __init__(self, x, y):
    self.position = _Pos(x, y)


def _circle_path(radius=8.0, n=33):
  # Left turn of constant radius, vehicle frame, heading +x.
  s = np.linspace(0.0, min(40.0, abs(radius) * np.pi / 2), n)
  th = s / radius
  xs = radius * np.sin(th)
  ys = radius * (1.0 - np.cos(th))
  return xs, ys


class TestSpeedSchedule:
  def test_blend_plateaus(self):
    assert speed_blend(5.0 * CV.KPH_TO_MS) == 0.0
    assert speed_blend(V_LIMIT_LOW) == 0.0
    assert speed_blend(V_LIMIT_HIGH) == 1.0
    assert speed_blend(120.0 * CV.KPH_TO_MS) == 1.0

  def test_blend_midpoint_linear(self):
    v_mid = 0.5 * (V_LIMIT_LOW + V_LIMIT_HIGH)
    assert abs(speed_blend(v_mid) - 0.5) < 1e-6

  def test_path_weight_inverse_of_blend(self):
    v = 55.0 * CV.KPH_TO_MS
    assert abs(path_follow_weight(v) + speed_blend(v) - 1.0) < 1e-9
    assert path_follow_weight(V_LIMIT_LOW) == 1.0
    assert path_follow_weight(V_LIMIT_HIGH) == 0.0

  def test_lat_accel_low_high_and_mid(self):
    low = scheduled_lat_accel_limit(V_LIMIT_LOW)
    high = scheduled_lat_accel_limit(V_LIMIT_HIGH)
    mid = scheduled_lat_accel_limit(0.5 * (V_LIMIT_LOW + V_LIMIT_HIGH))
    assert abs(low - LOW_SPEED_MAX_LAT_ACCEL) < 1e-9
    assert abs(high - MAX_LATERAL_ACCEL_NO_ROLL * HIGH_SPEED_LIMIT_SCALE) < 1e-9
    assert abs(mid - 0.5 * (low + high)) < 1e-9
    assert high > MAX_LATERAL_ACCEL_NO_ROLL
    assert abs(high / MAX_LATERAL_ACCEL_NO_ROLL - 1.20) < 1e-9


class TestClipCurvature:
  def _wind_to(self, v, target, roll=0.0):
    curv = 0.0
    limited = False
    for _ in range(500):
      curv, limited = clip_curvature(v, curv, target, roll)
    return curv, limited

  def test_high_speed_allows_20_percent_more(self):
    v = 100.0 * CV.KPH_TO_MS
    stock_max = MAX_LATERAL_ACCEL_NO_ROLL / (v ** 2)
    new_max = (MAX_LATERAL_ACCEL_NO_ROLL * HIGH_SPEED_LIMIT_SCALE) / (v ** 2)
    got, limited = self._wind_to(v, 1.0)
    assert limited
    assert got == np.clip(got, -MAX_CURVATURE, MAX_CURVATURE)
    assert abs(got - new_max) / new_max < 0.02
    assert got > stock_max * 1.10

  def test_low_speed_allows_tighter_than_stock_iso(self):
    v = 20.0 * CV.KPH_TO_MS
    stock_max = MAX_LATERAL_ACCEL_NO_ROLL / (v ** 2)
    got, _ = self._wind_to(v, 1.0)
    assert got > stock_max
    assert got <= scheduled_max_curvature(v) + 1e-9

  def test_low_speed_max_curvature_above_highway_cap(self):
    v = 8.0 * CV.KPH_TO_MS
    got, _ = self._wind_to(v, 1.0)
    assert got > MAX_CURVATURE
    assert abs(got - LOW_SPEED_MAX_CURVATURE) / LOW_SPEED_MAX_CURVATURE < 0.02

  def test_high_speed_max_curvature_unchanged(self):
    v = 100.0 * CV.KPH_TO_MS
    assert abs(scheduled_max_curvature(v) - MAX_CURVATURE) < 1e-9


class TestPathSteer:
  def test_circle_path_curvature(self):
    radius = 8.0
    xs, ys = _circle_path(radius)
    kappa = curvature_from_path_xy(xs, ys, 10.0)
    assert kappa is not None
    assert abs(kappa - 1.0 / radius) / (1.0 / radius) < 0.05

  def test_stable_low_speed_follows_path_not_action(self):
    xs, ys = _circle_path(8.0)
    helper = PathSteerHelper()
    model = _Model(xs, ys)
    action = 0.01
    v = 15.0 * CV.KPH_TO_MS
    out = action
    for _ in range(PATH_STABLE_SAMPLES):
      out = helper.blend(model, v, action, model_updated=True)
    path_kappa = curvature_from_path_turn(xs, ys, v)
    assert abs(out - path_kappa) < 1e-6
    assert abs(out) > abs(action)

  def test_shaking_path_keeps_action(self):
    helper = PathSteerHelper()
    action = 0.01
    v = 15.0 * CV.KPH_TO_MS
    out = action
    xs = np.linspace(0.0, 30.0, 33)
    for i in range(PATH_STABLE_SAMPLES):
      ys = np.full_like(xs, 3.0 if i % 2 == 0 else -3.0)
      out = helper.blend(_Model(xs, ys), v, action, model_updated=True)
    assert abs(out - action) < 1e-9

  def test_approaching_corner_low_speed_follows_path(self):
    # y at 15 m grows as a 90deg corner approaches — not L/R shake.
    helper = PathSteerHelper()
    action = 0.005
    v = 12.0 * CV.KPH_TO_MS
    out = action
    for i in range(PATH_STABLE_SAMPLES):
      xs, ys = _circle_path(8.0)
      ys = ys + 2.0 * i  # same side, growing offset
      out = helper.blend(_Model(xs, ys), v, action, model_updated=True)
    assert abs(out) > abs(action) * 3

  def test_passing_corner_same_side_follows_path(self):
    # y grows then shrinks on one side while passing a corner — not L/R shake.
    helper = PathSteerHelper()
    action = 0.005
    v = 12.0 * CV.KPH_TO_MS
    out = action
    offsets = [2.0, 6.0, 12.0, 20.0, 25.0, 22.0, 16.0, 10.0, 6.0, 3.0]
    for off in offsets:
      xs, ys = _circle_path(8.0)
      out = helper.blend(_Model(xs, ys + off), v, action, model_updated=True)
    assert abs(out) > abs(action) * 3

  def test_growing_corner_mid_speed_keeps_action(self):
    helper = PathSteerHelper()
    action = 0.01
    v = 55.0 * CV.KPH_TO_MS
    out = action
    for i in range(PATH_STABLE_SAMPLES):
      xs, ys = _circle_path(8.0)
      ys = ys + 2.0 * i
      out = helper.blend(_Model(xs, ys), v, action, model_updated=True)
    assert abs(out - action) < 1e-9

  def test_high_speed_ignores_path(self):
    xs, ys = _circle_path(8.0)
    helper = PathSteerHelper()
    model = _Model(xs, ys)
    action = 0.01
    v = 100.0 * CV.KPH_TO_MS
    out = action
    for _ in range(PATH_STABLE_SAMPLES):
      out = helper.blend(model, v, action, model_updated=True)
    assert abs(out - action) < 1e-9
