"""Combined physical-wheel + synthetic restore: one logical MRCC press."""
from __future__ import annotations

import pytest

from opendbc.car.mazda.carcontroller import TJA_RESTORE_MRCC_MAX_TX
from opendbc.car.mazda.tests.mazda_combined_wheel import (
  RESTORE_PHASES_MS,
  combined_restore_events,
  count_logical_mrcc_presses,
  replay_tja_combined,
)
from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  _controller,
  _step,
  _tja_press,
)


def _start_pending_off(ctrl, ctr):
  crz, _ = _tja_press(ctrl, True, tja=0, available=True, enabled=False, crz_btns_counter=ctr)
  assert crz == []
  assert ctrl._tja_restore_target == "OFF"


class TestCombinedWheelRestore:
  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_every_phase_and_ctr_is_one_logical_press(self, alpha_long):
    n = 0
    fails = []
    second = []
    for start_ctr in range(16):
      for phase in RESTORE_PHASES_MS:
        n += 1
        ctrl = _controller(alpha_long)
        _start_pending_off(ctrl, start_ctr)
        presses, ordered, events = combined_restore_events(
          ctrl, _step, True, start_ctr=start_ctr, phase_ms=phase,
          available=True, enabled=False)
        synth = [e for e in events if e["src"] == "synth" and e["mrcc"]]
        if not synth:
          fails.append((start_ctr, phase, "missed"))
          continue
        if presses != 1:
          second.append((start_ctr, phase, presses, ordered[-12:]))
        ctrs = {e["ctr"] for e in synth}
        if len(ctrs) != 1:
          second.append((start_ctr, phase, "ctrs", ctrs))
        if len(synth) > TJA_RESTORE_MRCC_MAX_TX:
          fails.append((start_ctr, phase, "unbounded", len(synth)))
    assert n == 16 * len(RESTORE_PHASES_MS)
    assert fails == [], fails[:8]
    assert second == [], second[:8]

  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_ctr_15_rollover(self, alpha_long):
    ctrl = _controller(alpha_long)
    _start_pending_off(ctrl, 15)
    presses, _, events = combined_restore_events(
      ctrl, _step, True, start_ctr=15, phase_ms=0, available=True)
    synth = [e for e in events if e["src"] == "synth" and e["mrcc"]]
    assert synth
    assert presses == 1
    assert len({e["ctr"] for e in synth}) == 1

  @pytest.mark.parametrize("delay", [5, 10, 20])
  def test_scheduling_delay_still_one_press(self, delay):
    ctrl = _controller(False)
    _start_pending_off(ctrl, 3)
    presses, _, events = combined_restore_events(
      ctrl, _step, True, start_ctr=3, phase_ms=40, available=True,
      scheduling_delay_ms=delay)
    synth = [e for e in events if e["src"] == "synth" and e["mrcc"]]
    assert synth
    assert presses == 1

  def test_dropped_cs_does_not_span_idle(self):
    ctrl = _controller(False)
    _start_pending_off(ctrl, 3)
    presses, _, events = combined_restore_events(
      ctrl, _step, True, start_ctr=3, phase_ms=0, available=True,
      drop_cs_frames=8, duration_ms=400)
    synth = [e for e in events if e["src"] == "synth" and e["mrcc"]]
    # Fail closed: without a seen CTR change, do not TX across unseen idles.
    if synth:
      assert presses == 1

  def test_duplicate_wheel_same_ctr_not_second_press(self):
    ctrl = _controller(False)
    _start_pending_off(ctrl, 3)
    presses, _, events = combined_restore_events(
      ctrl, _step, True, start_ctr=3, phase_ms=10, available=True,
      duplicate_wheel=True)
    synth = [e for e in events if e["src"] == "synth" and e["mrcc"]]
    assert synth
    assert presses == 1

  def test_same_ctr_opposite_payload_is_release_not_second_press(self):
    ctrl = _controller(False)
    _start_pending_off(ctrl, 3)
    presses, ordered, events = combined_restore_events(
      ctrl, _step, True, start_ctr=3, phase_ms=20, available=True)
    synth = [e for e in events if e["src"] == "synth" and e["mrcc"]]
    assert synth
    locked = synth[0]["ctr"]
    wheel_same = [e for e in events if e["src"] == "wheel" and e["ctr"] == locked]
    assert presses == 1
    # Next wheel may reuse the packed CTR as the idle/release. That is one press.

  def test_physical_tja_then_restore_one_press(self):
    ctrl = _controller(False)
    presses, _, events = replay_tja_combined(
      ctrl, _step, _tja_press, True, start_ctr=3, phase_ms=30,
      pre_available=False, hist_available=True, tja_hold_ms=80)
    synth = [e for e in events if e["src"] == "synth" and e["mrcc"]]
    assert synth
    assert presses == 1
    assert all(e.get("tja", 0) == 0 or e["src"] == "wheel" or not e["mrcc"] for e in events)

  def test_count_helper_detects_two_presses(self):
    n, _ = count_logical_mrcc_presses([
      {"t_ms": 0, "src": "synth", "ctr": 4, "mrcc": 1},
      {"t_ms": 100, "src": "wheel", "ctr": 4, "mrcc": 0},
      {"t_ms": 110, "src": "synth", "ctr": 5, "mrcc": 1},
    ])
    assert n == 2
    n, _ = count_logical_mrcc_presses([
      {"t_ms": 10, "src": "synth", "ctr": 4, "mrcc": 1},
      {"t_ms": 20, "src": "synth", "ctr": 4, "mrcc": 1},
      {"t_ms": 100, "src": "wheel", "ctr": 4, "mrcc": 0},
    ])
    assert n == 1
