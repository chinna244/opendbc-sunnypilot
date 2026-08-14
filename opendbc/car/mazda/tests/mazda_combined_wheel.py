"""Combined physical-wheel + synthetic CRZ_BTNS bus model.

CRZ_BTNS is same-bus (check_relay=false). Panda cannot delete wheel frames.
A logical MRCC press is a 0→1 edge on the ECU-visible combined stream.
Same-timestamp frames: physical wheel first, then synthetic (latest-wins).
"""
from __future__ import annotations

WHEEL_PERIOD_MS = 100
CTRL_DT_MS = 10
RESTORE_PHASES_MS = (0, 1, 2, 3, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 99)


def count_logical_mrcc_presses(events):
  """Rising 0→1 edges. Wheel before synth at equal time."""
  ordered = sorted(events, key=lambda e: (e["t_ms"], 0 if e["src"] == "wheel" else 1))
  n = 0
  prev = 0
  for e in ordered:
    mrcc = int(e["mrcc"])
    if mrcc and not prev:
      n += 1
    prev = mrcc
  return n, ordered


def combined_restore_events(ctrl, step_fn, lat, *, start_ctr=3, phase_ms=0,
                            duration_ms=350, available=True, enabled=False,
                            tja=0, scheduling_delay_ms=0, drop_cs_frames=0,
                            duplicate_wheel=False, jitter_ms=0, **kw):
  """Run controller against a 10 Hz wheel idle stream.

  phase_ms is milliseconds since the last physical wheel frame at t=0.
  CS starts at start_ctr. The next wheel is at (100 - phase_ms) ms with CTR+1.
  """
  events = []
  cs_ctr = int(start_ctr) & 0xF
  next_wheel_t = 0 if phase_ms == 0 else (WHEEL_PERIOD_MS - int(phase_ms))
  next_wheel_ctr = cs_ctr if phase_ms == 0 else ((cs_ctr + 1) & 0xF)
  if phase_ms != 0:
    events.append({"t_ms": -int(phase_ms), "src": "wheel", "ctr": cs_ctr, "mrcc": 0, "raw": "idle"})

  dropped = 0
  t = 0
  first_ctrl = True
  while t <= duration_ms:
    while next_wheel_t <= t:
      wheel_ctr = next_wheel_ctr
      events.append({"t_ms": next_wheel_t, "src": "wheel", "ctr": wheel_ctr, "mrcc": 0, "raw": "idle"})
      if duplicate_wheel:
        events.append({"t_ms": next_wheel_t, "src": "wheel", "ctr": wheel_ctr, "mrcc": 0, "raw": "dup"})
      if dropped < drop_cs_frames:
        dropped += 1
      else:
        cs_ctr = wheel_ctr
      next_wheel_t += WHEEL_PERIOD_MS
      next_wheel_ctr = (next_wheel_ctr + 1) & 0xF

    delay = int(scheduling_delay_ms if first_ctrl else 0)
    jitter = int(jitter_ms)
    crz, _ = step_fn(ctrl, lat, tja=tja, available=available, enabled=enabled,
                     crz_btns_counter=cs_ctr, **kw)
    first_ctrl = False
    for f in crz:
      events.append({
        "t_ms": t + delay + jitter, "src": "synth", "ctr": f["CTR"],
        "mrcc": int(f["MRCC"]), "raw": f,
      })
    t += CTRL_DT_MS
  presses, ordered = count_logical_mrcc_presses(events)
  return presses, ordered, events


def replay_tja_combined(ctrl, step_fn, tja_press_fn, lat, *, start_ctr=3, phase_ms=0,
                        pre_available=False, pre_enabled=False, hist_available=False,
                        hist_enabled=False, cam_laneinfo=None, tja_hold_ms=80,
                        post_duration_ms=250):
  """Physical TJA hold on the 10 Hz wheel, then restore against continuing idles."""
  events = []
  cs_ctr = int(start_ctr) & 0xF
  t = 0
  next_wheel_t = 0 if phase_ms == 0 else (WHEEL_PERIOD_MS - int(phase_ms))
  next_wheel_ctr = cs_ctr if phase_ms == 0 else ((cs_ctr + 1) & 0xF)
  if phase_ms != 0:
    events.append({"t_ms": -int(phase_ms), "src": "wheel", "ctr": cs_ctr, "mrcc": 0})

  toggled = False
  released = False
  hold_end = tja_hold_ms

  def emit_wheel(wt, ctr, tja_bit):
    events.append({"t_ms": wt, "src": "wheel", "ctr": ctr, "mrcc": 0, "tja": tja_bit})
    return ctr

  while t <= (hold_end + post_duration_ms):
    tja_bit = 1 if t <= hold_end else 0
    while next_wheel_t <= t:
      cs_ctr = emit_wheel(next_wheel_t, next_wheel_ctr, tja_bit)
      next_wheel_t += WHEEL_PERIOD_MS
      next_wheel_ctr = (next_wheel_ctr + 1) & 0xF

    kw = dict(available=hist_available, enabled=hist_enabled,
              pre_available=pre_available, pre_enabled=pre_enabled,
              crz_btns_counter=cs_ctr, cam_laneinfo=cam_laneinfo,
              cc_enabled=hist_enabled, long_active=hist_enabled)
    if (not toggled) and tja_bit:
      crz, _ = tja_press_fn(ctrl, lat, tja=1, **kw)
      toggled = True
    elif (not released) and (t > hold_end):
      crz, _ = step_fn(ctrl, lat, tja=0, **kw)
      released = True
    else:
      crz, _ = step_fn(ctrl, lat, tja=tja_bit, toggles=0, **kw)
    for f in crz:
      events.append({"t_ms": t, "src": "synth", "ctr": f["CTR"], "mrcc": int(f["MRCC"])})
    t += CTRL_DT_MS

  presses, ordered = count_logical_mrcc_presses(events)
  return presses, ordered, events
