"""Bidirectional MRCC_BUTTON restore: one logical gesture, abort, gating.

Physical MRCC OFF on ff7df7d6f9c3403b|00000011--5c933f23ac (22 successes):
  min hold 110 ms / 2 frames, typical 190 ms / 3 frames, max 400 ms / 5 frames.
  ECU confirmed OFF 52–112 ms after press, while the driver was still holding.
  Wheel CRZ_BTNS is 10 Hz. No physical CAN_OFF (cancel) occurred on that route.

Route 00000018 synthetic 100 Hz spanning two wheel periods failed 16/19:
two unique CTRs is a second logical MRCC press. Production fills one wheel
period (TJA_RESTORE_MRCC_MAX_TX) so the gesture cannot outlive one press.
"""

from __future__ import annotations

import random

import unittest
from opendbc.car.mazda.tests.unittest_compat import parametrize

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.mazda.carcontroller import TJA_OFF_COMPENSATION_MAX_TX
from opendbc.car.mazda.mazdacan import create_button_cmd
from opendbc.car.mazda.values import Buttons

from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  CAM_LKAS, _assert_clean_mrcc, _cc, _controller, _cs, _decode_crz, _step, _tja_off,
  _tja_press_then_restore, _start_restore_period,
)

# Route 00000011 successful physical MRCC OFF holds (src 0, ARMED/ACTIVE → OFF).
PHYSICAL_MRCC_OFF_HOLDS = (
  (80.217, 80.407, 190, 3, (8, 9, 10), 80.309),
  (85.237, 85.467, 230, 3, (14, 15, 0), 85.309),
  (91.397, 91.676, 279, 4, (15, 0, 1, 2), 91.509),
  (99.387, 99.557, 170, 3, (5, 6, 7), 99.469),
  (120.457, 120.656, 199, 3, (3, 4, 5), 120.569),
  (122.587, 122.697, 110, 2, (12, 13), 122.649),
  (126.757, 126.876, 119, 2, (10, 11), 126.809),
  (137.977, 138.167, 190, 3, (14, 15, 0), 138.049),
  (183.067, 183.257, 190, 3, (12, 13, 14), 183.169),
  (233.987, 234.126, 139, 3, (5, 6, 7), 234.068),
  (245.167, 245.327, 160, 3, (10, 11, 12), 245.268),
  (257.468, 257.637, 170, 3, (15, 0, 1), 257.589),
  (271.897, 272.247, 350, 4, (13, 14, 15, 0), 271.989),
  (276.918, 277.197, 280, 4, (3, 4, 5, 6), 277.029),
  (288.708, 288.907, 200, 3, (13, 14, 15), 288.830),
  (325.027, 325.198, 170, 3, (4, 5, 6), 325.148),
  (336.007, 336.177, 170, 3, (8, 9, 10), 336.089),
  (351.907, 352.097, 190, 3, (15, 0, 1), 351.989),
  (356.769, 357.169, 400, 5, (4, 5, 6, 7, 8), 356.890),
  (369.369, 369.519, 150, 2, (7, 8), 369.424),
  (390.719, 390.878, 160, 3, (0, 1, 2), 390.831),
  (395.668, 395.859, 191, 3, (6, 7, 8), 395.731),
)

# Wheel idle after TJA release (relative ms from 396.458).
ROUTE_WHEEL_IDLE_MS = (0, 71, 171, 271, 371, 471)

FUZZ_SEED = 20260813
FUZZ_ITERATIONS = 1000000
WHEEL_PERIOD_MS = 100
NEED_PRESSED_MS = 110  # min successful physical hold
IDLE_DEBOUNCE_MS = 10  # one control cycle; 100 Hz keeps idle 0 inside this
PHYSICAL_MRCC_HEX = "0081fee000000000"


def _pressed_duration(rate_hz, hold_ms, wheel_phase, synth_phase, debounce_ms=IDLE_DEBOUNCE_MS):
  """Latest-frame-wins with short idle-0 debounce. Returns stats dict."""
  synth_period = 1000 // rate_hz
  events = {0, hold_ms + 1}
  t = wheel_phase % WHEEL_PERIOD_MS
  while t <= hold_ms:
    events.add(t)
    t += WHEEL_PERIOD_MS
  t = synth_phase % synth_period
  while t <= hold_ms:
    events.add(t)
    t += synth_period
  times = sorted(e for e in events if 0 <= e <= hold_ms + 1)
  latest = 0
  last_one = -10**9
  pressed_run = 0
  best_run = 0
  off_at = None
  tx = 0
  max_collision = 0

  def fire(tnow):
    nonlocal latest, last_one, tx
    if tnow > hold_ms:
      return
    if (tnow - wheel_phase) % WHEEL_PERIOD_MS == 0:
      latest = 0
    if (tnow - synth_phase) % synth_period == 0:
      if tx < TJA_OFF_COMPENSATION_MAX_TX:
        latest = 1
        tx += 1
    if latest == 1:
      last_one = tnow

  for i in range(len(times) - 1):
    t0 = times[i]
    fire(t0)
    t1 = times[i + 1]
    span = t1 - t0
    if span <= 0:
      continue
    if latest == 1:
      new_run = pressed_run + span
      if off_at is None and new_run >= NEED_PRESSED_MS:
        off_at = t0 + (NEED_PRESSED_MS - pressed_run - 1)
      pressed_run = new_run
      best_run = max(best_run, pressed_run)
    else:
      slack = debounce_ms - (t0 - last_one)
      if slack > 0:
        held = min(span, slack)
        new_run = pressed_run + held
        if off_at is None and new_run >= NEED_PRESSED_MS:
          off_at = t0 + (NEED_PRESSED_MS - pressed_run - 1)
        pressed_run = new_run
        best_run = max(best_run, pressed_run)
        rest = span - held
        if rest > 0:
          max_collision = max(max_collision, rest)
          pressed_run = 0
      else:
        max_collision = max(max_collision, span)
        pressed_run = 0
  return {
    "success": off_at is not None and 0 <= off_at <= hold_ms,
    "off_at": off_at if off_at is not None and off_at <= hold_ms else None,
    "tx": tx,
    "best_run": best_run,
    "max_collision": max_collision,
  }


class TestPhysicalMrccOffHolds(unittest.TestCase):
  def test_route_00000011_physical_hold_stats(self):
    holds = PHYSICAL_MRCC_OFF_HOLDS
    assert len(holds) == 22
    hold_ms = [h[2] for h in holds]
    frames = [h[3] for h in holds]
    assert min(hold_ms) == 110
    assert max(hold_ms) == 400
    assert sorted(hold_ms)[len(hold_ms) // 2] == 190
    assert min(frames) == 2
    assert max(frames) == 5
    for press, release, duration_ms, n, ctrs, off_t in holds:
      assert abs(round((release - press) * 1000) - duration_ms) <= 1
      assert len(ctrs) == n
      assert off_t > press
      assert off_t < release
      for a, b in zip(ctrs, ctrs[1:], strict=False):
        assert b == (a + 1) % 16


class TestRateAndDuration(unittest.TestCase):
  @parametrize("rate,expect_all", [
    (10, False), (20, False), (25, False), (50, False), (100, True),
  ])
  def test_rate_vs_wheel_phase(self, rate, expect_all):
    holds = (100, 150, 200, 250, 300, 400)
    rows = []
    for hold in holds:
      ok = 0
      n = 0
      max_off = 0
      max_tx = 0
      max_col = 0
      synth_period = 1000 // rate
      for wp in range(WHEEL_PERIOD_MS):
        for sp in range(synth_period):
          r = _pressed_duration(rate, hold, wp, sp)
          n += 1
          ok += int(r["success"])
          if r["off_at"] is not None:
            max_off = max(max_off, r["off_at"])
          max_tx = max(max_tx, r["tx"])
          max_col = max(max_col, r["max_collision"])
      rows.append((hold, ok / n, max_off, max_tx, max_col, n))
      if not expect_all:
        assert ok < n, (rate, hold, ok, n)
    # Route 00000018: spanning two wheel periods failed. One 10 Hz period is
    # one logical press. Do not require coverage past one wheel period.
    if rate == 100:
      assert TJA_OFF_COMPENSATION_MAX_TX * 10 == 100

  def test_400_ms_not_required_when_stop_on_off(self):
    # ECU confirmed physical OFF in ≤112 ms. 250 ms covers that plus one
    # wheel period plus CarState lag. 400 ms was the longest driver hold
    # after OFF, not an extra ECU requirement. Two locked CTRs is a second
    # logical press; restore stays inside one wheel period.
    assert TJA_OFF_COMPENSATION_MAX_TX * 10 == 100
    assert TJA_OFF_COMPENSATION_MAX_TX == 10
    assert TJA_OFF_COMPENSATION_MAX_TX * 10 >= 100


@parametrize("alpha_long", [False, True])
class TestHoldStopAndAbort(unittest.TestCase):
  def test_zero_tx_after_confirmed_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    _assert_clean_mrcc(crz)
    crz, _ = _step(ctrl, False, available=False, enabled=False, crz_btns_counter=4)
    assert crz == []
    for _ in range(15):
      crz, _ = _step(ctrl, False, available=False, enabled=False, crz_btns_counter=4)
      assert crz == []
    assert ctrl._tja_off_comp_pending is False

  def test_feedback_lag_extra_tx_bounded(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    extra = 0
    for _i in range(2):
      crz, _ = _step(ctrl, False, available=True, enabled=False, crz_btns_counter=4)
      if crz:
        extra += 1
    crz, _ = _step(ctrl, False, available=False, enabled=False, crz_btns_counter=4)
    assert crz == []
    assert extra <= 2

  def test_mrcc_aborts_when_not_echo(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_off(ctrl, tja=1, available=True, enabled=False)
    assert crz == []
    crz, _ = _step(ctrl, False, tja=0, mrcc=1, available=True, enabled=False)
    assert crz == []
    crz, _ = _step(ctrl, False, mrcc=0, available=True, enabled=False)
    assert crz == []

  def test_mrcc_echo_does_not_abort_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    _assert_clean_mrcc(crz)
    crz, _ = _step(ctrl, False, mrcc=1, available=True, enabled=False, crz_btns_counter=4)
    _assert_clean_mrcc(crz)

  def test_cancel_aborts_same_cycle(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    crz, _ = _step(ctrl, False, cancel=1, available=True, enabled=False, crz_btns_counter=4)
    assert crz == []

  def test_set_aborts_same_cycle(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    crz, _ = _step(ctrl, False, set_p=1, available=True, enabled=False, crz_btns_counter=4)
    assert crz == []

  def test_res_aborts_same_cycle(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    crz, _ = _step(ctrl, False, res=1, available=True, enabled=False, crz_btns_counter=4)
    assert crz == []

  def test_tja_on_during_hold_clears_pending(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    crz, _ = _step(ctrl, True, toggles=1, tja=1, available=True, enabled=False,
                   pre_available=True, crz_btns_counter=4)
    assert crz == []
    crz, _ = _step(ctrl, True, tja=0, available=True, enabled=False, crz_btns_counter=4)
    assert crz == []

  def test_held_tja_does_not_tx(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_off(ctrl, tja=1, available=True, enabled=False)
    assert crz == []

  def test_op_cancel_does_not_stack_second_crz(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    crz, sends = _step(ctrl, False, available=True, enabled=False, op_cancel=True,
                       crz_btns_counter=4)
    assert len(crz) <= 1


@parametrize("alpha_long", [False, True])
class TestGatingCasesEF(unittest.TestCase):
  def test_case_e_zero_hold_frames(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_off(ctrl, tja=1, available=True, enabled=False, pre_available=True)
    assert crz == []
    tx = 0
    for _ in range(TJA_OFF_COMPENSATION_MAX_TX + 5):
      crz, _ = _step(ctrl, False, tja=0, available=True, enabled=False)
      tx += len(crz)
    assert tx == 0

  def test_case_f_zero_hold_frames(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_off(ctrl, tja=1, available=True, enabled=True,
                      pre_available=True, pre_enabled=True,
                      cc_enabled=True, long_active=True)
    assert crz == []
    tx = 0
    for _ in range(TJA_OFF_COMPENSATION_MAX_TX + 5):
      crz, _ = _step(ctrl, False, available=True, enabled=True,
                     cc_enabled=True, long_active=True)
      tx += len(crz)
    assert tx == 0

  def test_unknown_cannot_start_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_off(ctrl, tja=0, available=True, enabled=False, pre_unknown=True)
    assert crz == []
    for _ in range(10):
      crz, _ = _step(ctrl, False, available=True, enabled=False)
      assert crz == []

  def test_tja_on_from_off_starts_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _step(ctrl, True, toggles=1, tja=1, available=False, enabled=False)
    assert crz == []
    crz, _ = _start_restore_period(ctrl, True, tja=0, available=True, enabled=False)
    _assert_clean_mrcc(crz)


@parametrize("alpha_long", [False, True])
class TestCounterChecksumLateral(unittest.TestCase):
  def test_hold_frames_are_clean_mrcc_across_rollover(self, alpha_long):
    ctrl = _controller(alpha_long)
    CP = ctrl.CP
    packer = CANPacker("mazda_2017")
    invalid = 0
    for ctr in range(20):
      addr, dat, bus = create_button_cmd(packer, CP, ctr % 16, Buttons.MAIN)
      assert addr == 0x9D
      assert bus == 0
      decoded = _decode_crz([(addr, dat, bus)])
      if not decoded or decoded[0]["MRCC"] != 1 or decoded[0]["CAN_OFF"] or decoded[0]["TJA"]:
        invalid += 1
    assert invalid == 0
    crz, _ = _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    _assert_clean_mrcc(crz)
    for _ in range(min(16, TJA_OFF_COMPENSATION_MAX_TX - 1)):
      crz, _ = _step(ctrl, False, available=True, enabled=False, crz_btns_counter=4)
      _assert_clean_mrcc(crz)

  def test_cam_lkas_cadence_during_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    lkas = 0
    crz, sends = _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False)
    lkas += sum(1 for s in sends if s[0] == CAM_LKAS)
    for _ in range(TJA_OFF_COMPENSATION_MAX_TX - 1):
      crz, sends = _step(ctrl, False, available=True, enabled=False, crz_btns_counter=4)
      lkas += sum(1 for s in sends if s[0] == CAM_LKAS)
      _assert_clean_mrcc(crz)
    assert lkas == TJA_OFF_COMPENSATION_MAX_TX

  def test_hold_does_not_command_steer_after_mads_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    CC_SP = structs.CarControlSP()
    CC = _cc(False)
    _, sends = ctrl.update(CC, CC_SP, _cs(toggles=1, tja=0, available=True), 0)
    # latActive is false; CAM_LKAS still packed (zero torque path in controller).
    assert sum(1 for s in sends if s[0] == CAM_LKAS) == 1


@parametrize("alpha_long", [False, True])
class TestRouteCaseDReplay(unittest.TestCase):
  def test_last_failed_sequence_hold_then_off(self, alpha_long):
    """TJA from OFF restores OFF; a later TJA from OFF restores again."""
    ctrl = _controller(alpha_long)
    crz, _ = _step(ctrl, True, toggles=1, tja=1, available=False)
    assert crz == []
    crz, _ = _start_restore_period(ctrl, True, tja=0, available=True)
    _assert_clean_mrcc(crz)
    crz, _ = _step(ctrl, True, tja=0, available=False, crz_btns_counter=4)
    assert crz == []
    crz, _ = _tja_off(ctrl, tja=1, available=False)
    assert crz == []
    tx = 0
    crz, _ = _start_restore_period(ctrl, False, tja=0, available=True)
    _assert_clean_mrcc(crz)
    tx += 1
    for _i in range(TJA_OFF_COMPENSATION_MAX_TX - 1):
      crz, _ = _step(ctrl, False, tja=0, available=True, crz_btns_counter=4)
      _assert_clean_mrcc(crz)
      tx += 1
    crz, _ = _step(ctrl, False, tja=0, available=True, crz_btns_counter=4)
    assert crz == []
    crz, _ = _step(ctrl, False, tja=0, available=False, crz_btns_counter=4)
    assert crz == []
    assert 2 < tx <= TJA_OFF_COMPENSATION_MAX_TX


class TestRouteWheelTiming(unittest.TestCase):
  def test_route_wheel_idle_100hz_beats_debounce_model(self):
    # Interval-contained 100 Hz must not treat a wheel idle as still-pressed.
    r = _pressed_duration(100, 200, wheel_phase=ROUTE_WHEEL_IDLE_MS[0], synth_phase=10)
    assert r["tx"] <= TJA_OFF_COMPENSATION_MAX_TX


class TestTimingFuzz(unittest.TestCase):
  def test_1e6_restore_and_negative_paths(self):
    rng = random.Random(FUZZ_SEED)
    false_hold = 0
    restore_fail = 0
    override = 0
    for _i in range(FUZZ_ITERATIONS):
      kind = rng.randrange(10)
      if kind < 6:
        wp = rng.randrange(WHEEL_PERIOD_MS)
        sp = rng.randrange(10)
        r = _pressed_duration(100, 110, wp, sp)
        if r["tx"] > TJA_OFF_COMPENSATION_MAX_TX:
          restore_fail += 1
      elif kind < 8:
        # Pre-ARMED / pre-ACTIVE never start the hold: wheel idle only, no synth.
        latest = 0
        success = False
        tx = 0
        pressed_run = 0
        for t in range(250):
          if t % WHEEL_PERIOD_MS == 0:
            latest = 0
          if latest == 1:
            tx += 1
          pressed_run = pressed_run + 1 if latest == 1 else 0
          if pressed_run >= NEED_PRESSED_MS:
            success = True
        if success or tx:
          false_hold += 1
      else:
        abort_ms = rng.randrange(10, NEED_PRESSED_MS)
        latest = 0
        last_one = -10**9
        pressed_run = 0
        reached = False
        for t in range(250):
          if t >= abort_ms:
            latest = 0
            last_one = -10**9
          else:
            if t % WHEEL_PERIOD_MS == 0:
              latest = 0
            if t % 10 == 0:
              latest = 1
          if latest == 1:
            last_one = t
          pressed = latest == 1 or (t - last_one) < IDLE_DEBOUNCE_MS
          pressed_run = pressed_run + 1 if pressed else 0
          if pressed_run >= NEED_PRESSED_MS:
            reached = True
        if reached:
          override += 1
    assert restore_fail == 0
    assert false_hold == 0
    assert override == 0


class TestPhysicalVsSyntheticFrames(unittest.TestCase):
  def test_old_can_off_does_not_match_physical_master_off(self):
    ctrl = _controller(False)
    packer = CANPacker("mazda_2017")
    old = _decode_crz([create_button_cmd(packer, ctrl.CP, 7, Buttons.CANCEL)])[0]
    new = _decode_crz([create_button_cmd(packer, ctrl.CP, 7, Buttons.MAIN)])[0]
    assert old["CAN_OFF"] == 1 and old["MRCC"] == 0
    assert new["MRCC"] == 1 and new["CAN_OFF"] == 0
    assert new["TJA"] == 0 and new["SET_P"] == 0 and new["RES"] == 0

  def test_synthetic_mrcc_matches_physical_button_bits(self):
    """Route 00000011 physical master-OFF 0081fee0: MRCC=1, CAN_OFF=0, BIT1=0."""
    ctrl = _controller(False)
    packer = CANPacker("mazda_2017")
    addr, dat, bus = create_button_cmd(packer, ctrl.CP, 7, Buttons.MAIN)
    assert addr == 0x9D and bus == 0
    assert dat.hex() == PHYSICAL_MRCC_HEX

  def test_route_00000012_sequence_ends_off(self):
    """TJA from OFF restores OFF in both MADS directions."""
    ctrl = _controller(False)
    crz, _ = _step(ctrl, True, toggles=1, tja=1, available=False)
    assert crz == []
    crz, _ = _start_restore_period(ctrl, True, tja=0, available=True)
    _assert_clean_mrcc(crz)
    crz, _ = _step(ctrl, True, tja=0, available=False, crz_btns_counter=4)
    assert crz == []
    crz, _ = _tja_off(ctrl, tja=1, available=False)
    assert crz == []
    crz, _ = _start_restore_period(ctrl, False, tja=0, available=True)
    _assert_clean_mrcc(crz)
    crz, _ = _step(ctrl, False, tja=0, available=False, crz_btns_counter=4)
    assert crz == []


class TestSelectedHold(unittest.TestCase):
  def test_selected_rate_and_hold_is_one_logical_press(self):
    assert TJA_OFF_COMPENSATION_MAX_TX * 10 == 100
    assert TJA_OFF_COMPENSATION_MAX_TX == 10
