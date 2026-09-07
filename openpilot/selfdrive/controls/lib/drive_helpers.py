from collections import deque

import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
from openpilot.common.realtime import DT_CTRL, DT_MDL

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# Highway / high-speed curvature cap. Low speed uses LOW_SPEED_MAX_CURVATURE.
MAX_CURVATURE = 0.2
LOW_SPEED_MAX_CURVATURE = 0.35  # ~2.9 m radius; parking-lot / tight urban
MAX_VEL_ERR = 5.0  # m/s
MIN_STABLE_DELAY = 0.3

# EU guidelines (high-speed plateau is 20% above these)
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2

# Speed schedule: low-speed path follow / relaxed limits, high-speed +20% allow, linear in between.
V_LIMIT_LOW = 30.0 * CV.KPH_TO_MS
V_LIMIT_HIGH = 80.0 * CV.KPH_TO_MS
HIGH_SPEED_LIMIT_SCALE = 1.20
LOW_SPEED_MAX_LAT_ACCEL = 5.0  # m/s^2
LOW_SPEED_MAX_LAT_JERK = 8.0   # m/s^3
PATH_STABLE_X = 15.0           # m, UI point used for L/R shake check
PATH_STABLE_STD = 0.8          # m, mid-speed only
PATH_STABLE_SAMPLES = 10       # model frames (~0.5 s at 20 Hz)
PATH_OSCILLATION_Y = 1.0       # m, ignore |y| below this when counting L/R
PATH_OSCILLATION_FLIPS = 2     # y sign groups L-R-L => shaking, keep action
PATH_ONE_SIDED_Y = 2.0         # m, mean |y| of a real corner (not a shake)
PATH_LOOKAHEAD_S = 2.5         # s, path distance to steer toward at low speed
PATH_CURV_X_MIN = 4.0          # m
PATH_CURV_X_MAX = 16.0         # m


def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def speed_blend(v_ego: float, v_low: float = V_LIMIT_LOW, v_high: float = V_LIMIT_HIGH) -> float:
  """0 at v_low and below, 1 at v_high and above. Linear in between."""
  if v_high <= v_low:
    return 1.0 if v_ego >= v_high else 0.0
  return float(np.clip((v_ego - v_low) / (v_high - v_low), 0.0, 1.0))

def scheduled_lat_accel_limit(v_ego: float) -> float:
  t = speed_blend(v_ego)
  high = MAX_LATERAL_ACCEL_NO_ROLL * HIGH_SPEED_LIMIT_SCALE
  return float(LOW_SPEED_MAX_LAT_ACCEL * (1.0 - t) + high * t)

def scheduled_lat_jerk_limit(v_ego: float) -> float:
  t = speed_blend(v_ego)
  high = MAX_LATERAL_JERK * HIGH_SPEED_LIMIT_SCALE
  return float(LOW_SPEED_MAX_LAT_JERK * (1.0 - t) + high * t)

def scheduled_max_curvature(v_ego: float) -> float:
  t = speed_blend(v_ego)
  return float(LOW_SPEED_MAX_CURVATURE * (1.0 - t) + MAX_CURVATURE * t)

def path_follow_weight(v_ego: float) -> float:
  """Fully follow a stable UI path at/under 30 km/h, none at/above 80 km/h."""
  return 1.0 - speed_blend(v_ego)

def path_lookahead_x(v_ego: float) -> float:
  return float(np.clip(v_ego * PATH_LOOKAHEAD_S, PATH_CURV_X_MIN, PATH_CURV_X_MAX))

def curvature_from_path_xy(xs, ys, lookahead_x: float) -> float | None:
  """Constant-curvature arc through a path point at lookahead_x. Same geometry the UI draws."""
  if xs is None or ys is None or len(xs) < 3 or len(ys) < 3:
    return None
  xs_a = np.asarray(xs, dtype=float)
  ys_a = np.asarray(ys, dtype=float)
  if xs_a[-1] < 3.0:
    return None
  x = float(np.clip(lookahead_x, xs_a[0], xs_a[-1]))
  y = float(np.interp(x, xs_a, ys_a))
  denom = x * x + y * y
  if denom < 1.0:
    return 0.0
  return float(2.0 * y / denom)


def curvature_from_path_turn(xs, ys, v_ego: float) -> float | None:
  """Curvature of the UI path. Near and far looks; same-sign takes the tighter one."""
  far_x = path_lookahead_x(v_ego)
  near_x = float(np.clip(v_ego * 1.5, PATH_CURV_X_MIN, far_x))
  k_near = curvature_from_path_xy(xs, ys, near_x)
  k_far = curvature_from_path_xy(xs, ys, far_x)
  if k_near is None:
    return k_far
  if k_far is None:
    return k_near
  if k_near * k_far >= 0.0:
    return k_far if abs(k_far) >= abs(k_near) else k_near
  return k_near


class PathSteerHelper:
  """Blend steering toward the UI path at low speed.

  Reject only left/right oscillation of the path. A large turn that grows
  then shrinks as it is passed is not shake. Mid-speed still uses the std cap.
  """

  def __init__(self):
    self._y_hist: deque[float] = deque(maxlen=PATH_STABLE_SAMPLES)

  def reset(self):
    self._y_hist.clear()

  def _ready(self) -> bool:
    return len(self._y_hist) >= PATH_STABLE_SAMPLES

  def _oscillating(self) -> bool:
    """True if the 15 m point flips left/right. Grow-then-shrink on one side is not."""
    if not self._ready():
      return True
    prev = 0
    flips = 0
    for y in self._y_hist:
      if abs(y) <= PATH_OSCILLATION_Y:
        continue
      si = 1 if y > 0 else -1
      if prev != 0 and si != prev:
        flips += 1
      prev = si
    return flips >= PATH_OSCILLATION_FLIPS

  def _one_sided_turn(self) -> bool:
    if not self._ready():
      return False
    return abs(float(np.mean(self._y_hist))) >= PATH_ONE_SIDED_Y

  def _tight_stable(self) -> bool:
    if not self._ready():
      return False
    return float(np.std(self._y_hist)) <= PATH_STABLE_STD

  def update(self, model_v2, model_updated: bool) -> None:
    if not model_updated:
      return
    xs = model_v2.position.x
    ys = model_v2.position.y
    if xs is None or ys is None or len(xs) < 3 or len(ys) < 3:
      return
    xs_a = np.asarray(xs, dtype=float)
    ys_a = np.asarray(ys, dtype=float)
    x = float(np.clip(PATH_STABLE_X, xs_a[0], xs_a[-1]))
    self._y_hist.append(float(np.interp(x, xs_a, ys_a)))

  def blend(self, model_v2, v_ego: float, action_curvature: float, model_updated: bool) -> float:
    self.update(model_v2, model_updated)
    weight = path_follow_weight(v_ego)
    if weight <= 0.0 or not self._ready():
      return float(action_curvature)
    if self._oscillating() and not self._one_sided_turn():
      return float(action_curvature)
    # Below 30 km/h, a growing corner is allowed. Mid-speed still needs small std.
    if weight < 1.0 and not self._tight_stable():
      return float(action_curvature)
    path_curv = curvature_from_path_turn(model_v2.position.x, model_v2.position.y, v_ego)
    if path_curv is None:
      return float(action_curvature)
    return float(weight * path_curv + (1.0 - weight) * action_curvature)


def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # Speed-scheduled ISO-style jerk/accel limits + a max curvature.
  # Low speed allows tighter urban corners; high speed is 20% above stock ISO.
  v_clip = max(v_ego, MIN_SPEED)
  max_lat_jerk = scheduled_lat_jerk_limit(v_ego)
  max_curvature_rate = max_lat_jerk / (v_clip ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = scheduled_lat_accel_limit(v_ego) + roll_compensation
  min_lat_accel = -scheduled_lat_accel_limit(v_ego) + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_clip ** 2, max_lat_accel / v_clip ** 2)

  max_curv = scheduled_max_curvature(v_ego)
  new_curvature, limited_max_curv = clamp(new_curvature, -max_curv, max_curv)
  return float(new_curvature), limited_accel or limited_max_curv


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.3):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    if action_t < MIN_STABLE_DELAY:
      v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
    else:
      v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
  else:
    v_now = 0.0
    v_target = 0.0
    a_target = 0.0
  should_stop = (v_now < vEgoStopping and a_target < 0.1)
  return a_target, should_stop

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  if action_t < MIN_STABLE_DELAY:
    psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
  else:
    psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)
