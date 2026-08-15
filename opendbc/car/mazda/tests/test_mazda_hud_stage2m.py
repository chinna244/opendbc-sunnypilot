"""Stage 2M final repair: HUD scheduler, FSC fail-safe, bounded restore retry.

Does not mask a genuine FSC/camera latch. Does not suppress genuine
steerRequired / steerSaturated. Exactly two HUD compromises:
  #1 TJA-off while MADS ON + MRCC ACTIVE may FULL-OFF MRCC.
  #2 MADS ON + MRCC ARMED packs TJA=0 so physical MRCC ARMED→OFF works.
"""
from __future__ import annotations

import random
import unittest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.mazda.carcontroller import (
  HUD_MIN_INTERVAL_FRAMES,
  TJA_RESTORE_MAX_ATTEMPTS,
  TJA_RESTORE_MRCC_MAX_TX,
  TJA_RESTORE_MRCC_MAX_TX_TOTAL,
)
from opendbc.car.mazda.tests.test_mazda_hud_stage2j import (
  ROUTE21_MRCC_OFF_IGNORED,
  ROUTE22_TJA_ON,
  _expected,
  _model_off_tja_on,
  _packed_tja,
)
from opendbc.car.mazda.tests.test_mazda_startup_lkas import _decode_cam_lkas
from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  CAM_LKAS_STOCK,
  _controller,
  _decode_laneinfo,
  _hud_step,
  _laneinfo_tja,
  _next_wheel_ctr,
  _start_restore_period,
  _step,
  _tja_press,
  _tja_press_then_restore,
)
from opendbc.car.mazda.tests.unittest_compat import parametrize

VisualAlert = structs.CarControl.HUDControl.VisualAlert

FSC_ERR = {"BIT_1": 0, "ERR_BIT_1": 1, "ERR_BIT_2": 1}


def _decode_hod(sends):
  frames = [(addr, dat, bus) for addr, dat, bus in sends if addr == 0x440]
  if not frames:
    return []
  cp = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
  for st in cp.message_states.values():
    st.ignore_checksum = True
    st.ignore_counter = True
    st.ignore_alive = True
  out = []
  for addr, dat, bus in frames:
    cp.update([(0, [(addr, dat, bus)])])
    vl = cp.vl["CAM_LANEINFO"]
    out.append({
      "TJA": int(vl["TJA"]),
      "HANDS_ON_STEER_WARN": int(vl["HANDS_ON_STEER_WARN"]),
      "HANDS_WARN_3_BITS": int(vl["HANDS_WARN_3_BITS"]),
    })
  return out


def _laneinfo_counts(sends):
  return sum(1 for a, _, _ in sends if a == 0x440)


class TestMazdaHudStage2mPolicy(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_independence_matrix(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    rows = (
      (False, "OFF", False, False),
      (True, "OFF", False, False),
      (False, "ARMED", True, False),
      (True, "ARMED", True, False),
      (False, "ACTIVE", True, True),
      (True, "ACTIVE", True, True),
    )
    for mads, mrcc, available, enabled in rows:
      crz, sends = _hud_step(ctrl, mads, mads_enabled=mads, available=available,
                             enabled=enabled, cc_enabled=enabled, long_active=enabled,
                             cam_laneinfo=cam)
      assert crz == []
      assert _packed_tja(sends) == _expected(mads, mrcc)
      assert _packed_tja(sends) != 4

  @parametrize("alpha_long", [False, True])
  def test_transition_a_off_off_tja_on_white(self, alpha_long):
    ctrl = _controller(alpha_long)
    modeled = _model_off_tja_on(ctrl, cam_tja=0, transition=2)
    assert modeled["FINAL_MRCC"] == "OFF"
    assert modeled["FIRST_TJA2"] is not None
    assert all(t == 0 for t in modeled["tja_while_unresolved"])
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=_laneinfo_tja(0, transition=2))
    assert crz == []
    assert _packed_tja(sends) == 2

  @parametrize("alpha_long", [False, True])
  def test_transition_b_mads_on_mrcc_arms_no_icon(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False, cam_laneinfo=cam)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_transition_c_armed_physical_mrcc_off_white(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False, cam_laneinfo=cam)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           mrcc=1, cam_laneinfo=cam)
    assert crz == []
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 2
    assert ctrl._tja_restore_target is None

  @parametrize("alpha_long", [False, True])
  def test_transition_d_tja_off_while_armed_keeps_armed(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False, cam_laneinfo=cam)
    crz, sends = _tja_press(ctrl, False, tja=1, available=True, enabled=False,
                            pre_available=True, pre_enabled=False, cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target == "ARMED"
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_transition_e_tja_off_while_active_compromise_1(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(3, transition=2)
    _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
              cc_enabled=True, long_active=True, cam_laneinfo=cam)
    crz, _ = _tja_press(ctrl, False, tja=1, available=True, enabled=True,
                        pre_available=True, pre_enabled=True, cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target is None
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_never_tja4_or_mads_off_nonzero(self, alpha_long):
    ctrl = _controller(alpha_long)
    for mrcc in ("OFF", "ARMED", "ACTIVE"):
      av = mrcc != "OFF"
      en = mrcc == "ACTIVE"
      for tja in (0, 1, 2, 3, 4):
        cam = _laneinfo_tja(tja, transition=2)
        _, sends = _hud_step(ctrl, False, mads_enabled=False, available=av, enabled=en,
                             cc_enabled=en, long_active=en, cam_laneinfo=cam)
        packed = _packed_tja(sends)
        assert packed == 0
        assert packed != 4


class TestMazdaHudStage2mScheduler(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_no_duplicate_immediate_440(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                         cam_laneinfo=cam)
    assert _laneinfo_counts(sends) == 1
    ctrl.frame = 51
    _, sends = _step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                     cam_laneinfo=cam)
    assert _laneinfo_counts(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_min_interval_50ms_on_dirty_tja(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                         cam_laneinfo=cam)
    assert _packed_tja(sends) == 2
    extras = 0
    for fr in range(51, 55):
      ctrl.frame = fr
      _, sends = _step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                       cam_laneinfo=cam)
      extras += _laneinfo_counts(sends)
    assert extras == 0
    ctrl.frame = 55
    _, sends = _step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                     cam_laneinfo=cam)
    assert _packed_tja(sends) == 0
    assert _laneinfo_counts(sends) == 1

  @parametrize("alpha_long", [False, True])
  def test_hold_flag_without_tja_change_no_extra(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False, cam_laneinfo=cam)
    crz, sends = _tja_press(ctrl, True, tja=1, available=False, enabled=False, cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target == "OFF"
    ctrl.frame = 51
    _, sends = _step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                     cam_laneinfo=cam)
    assert _laneinfo_counts(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_armed_first_edge_before_rapid_mrcc(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False, cam_laneinfo=cam)
    ctrl.frame = 50 + HUD_MIN_INTERVAL_FRAMES
    _, sends = _step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                     cam_laneinfo=cam)
    assert _packed_tja(sends) == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           mrcc=1, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_genuine_steer_required_not_suppressed(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(2, transition=2)
    _, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                         cam_laneinfo=cam, visual_alert=VisualAlert.steerRequired)
    hod = _decode_hod(sends)
    assert hod and hod[0]["HANDS_ON_STEER_WARN"] == 1
    assert hod[0]["TJA"] == 2

  @parametrize("alpha_long", [False, True])
  def test_restore_pending_never_tja2(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _tja_press(ctrl, True, tja=1, available=False, enabled=False, cam_laneinfo=cam)
    assert ctrl._tja_restore_target == "OFF"
    for _ in range(8):
      _, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
      tja = _packed_tja(sends)
      if tja is not None:
        assert tja == 0


class TestMazdaHudStage2mFscFailSafe(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_fsc_err_zeros_torque_while_lat_active(self, alpha_long):
    ctrl = _controller(alpha_long)
    _, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                         cam_lkas=FSC_ERR)
    cam = _decode_cam_lkas(sends)
    assert cam
    assert cam[0]["LKAS_REQUEST"] == 0
    assert cam[0]["ERR_BIT_1"] == 1
    assert cam[0]["ERR_BIT_2"] == 1

  @parametrize("mads,available,enabled", [
    (False, False, False),
    (True, False, False),
    (True, True, False),
    (True, True, True),
  ])
  def test_fsc_err_does_not_start_restore(self, mads, available, enabled):
    ctrl = _controller(False)
    crz, sends = _hud_step(ctrl, mads, mads_enabled=mads, available=available,
                           enabled=enabled, cc_enabled=enabled, long_active=enabled,
                           cam_lkas=FSC_ERR)
    assert crz == []
    assert ctrl._tja_restore_target is None
    assert _laneinfo_counts(sends) <= 1

  @parametrize("alpha_long", [False, True])
  def test_fsc_lane_lines_invalid_no_hud_flood(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    cam["LANE_LINES"] = 0
    n = 0
    for i in range(20):
      ctrl.frame = i
      _, sends = _step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                       cam_laneinfo=cam, cam_lkas=FSC_ERR)
      n += _laneinfo_counts(sends)
    assert n <= 1

  @parametrize("alpha_long", [False, True])
  def test_healthy_lkas_still_requests_when_lat_active(self, alpha_long):
    ctrl = _controller(alpha_long)
    _, sends = _step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                     cam_lkas=CAM_LKAS_STOCK)
    cam = _decode_cam_lkas(sends)
    assert cam
    assert cam[0]["ERR_BIT_1"] == 0
    assert abs(cam[0]["LKAS_REQUEST"]) > 0


class TestMazdaHudStage2mRestoreRetry(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_second_period_then_stop(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                     crz_btns_counter=3)
    assert crz
    period = 4
    tx = 1
    for _ in range(TJA_RESTORE_MRCC_MAX_TX):
      crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=period)
      if not crz:
        break
      tx += 1
    assert tx <= TJA_RESTORE_MRCC_MAX_TX
    crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=5)
    assert crz
    tx2 = 1
    for _ in range(TJA_RESTORE_MRCC_MAX_TX):
      crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=5)
      if not crz:
        break
      tx2 += 1
    assert tx2 <= TJA_RESTORE_MRCC_MAX_TX
    crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=6)
    assert crz == []
    assert tx + tx2 <= TJA_RESTORE_MRCC_MAX_TX_TOTAL
    assert ctrl._tja_restore_attempts <= TJA_RESTORE_MAX_ATTEMPTS

  @parametrize("alpha_long", [False, True])
  def test_late_ack_cancels_retry(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                     crz_btns_counter=3)
    assert crz
    crz, _ = _step(ctrl, True, available=False, enabled=False, crz_btns_counter=5)
    assert crz == []
    assert ctrl._tja_restore_target is None

  @parametrize("alpha_long", [False, True])
  def test_driver_set_cancels_retry(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                            crz_btns_counter=3)
    crz, _ = _step(ctrl, True, available=True, enabled=False, set_p=1, crz_btns_counter=5)
    assert crz == []
    crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=5)
    assert crz == []

  @parametrize("alpha_long", [False, True])
  def test_hold_tja0_between_attempts(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _tja_press(ctrl, True, tja=1, available=False, enabled=False, cam_laneinfo=cam)
    _start_restore_period(ctrl, True, tja=0, available=True, enabled=False,
                          cam_laneinfo=cam, crz_btns_counter=3)
    _, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                         cam_laneinfo=cam, crz_btns_counter=4)
    tja = _packed_tja(sends)
    if tja is not None:
      assert tja == 0


class TestMazdaHudStage2mRouteModels(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_route22_and_route21_independence(self, alpha_long):
    for ev in ROUTE22_TJA_ON:
      ctrl = _controller(alpha_long)
      modeled = _model_off_tja_on(
        ctrl, brake=bool(ev.get("brake")), v_ego=float(ev.get("v") or 0.0),
        cam_tja=0, transition=2)
      assert modeled["FINAL_MRCC"] == "OFF"
      assert all(t == 0 for t in modeled["tja_while_unresolved"])
    assert ROUTE21_MRCC_OFF_IGNORED == 14

  @parametrize("alpha_long", [False, True])
  def test_route19_active_no_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, sends = _tja_press(ctrl, True, tja=1, available=True, enabled=True,
                            pre_available=True, pre_enabled=True,
                            cc_enabled=True, long_active=True)
    assert crz == []
    assert ctrl._tja_restore_target is None
    crz, _ = _step(ctrl, True, available=True, enabled=True, cc_enabled=True,
                   long_active=True)
    assert crz == []

  @parametrize("alpha_long", [False, True])
  def test_route20_fsc_err_zeros_torque(self, alpha_long):
    ctrl = _controller(alpha_long)
    _, sends = _step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                     cc_enabled=True, long_active=True, cam_lkas=FSC_ERR)
    cam = _decode_cam_lkas(sends)
    assert cam[0]["LKAS_REQUEST"] == 0

  @parametrize("alpha_long", [False, True])
  def test_route26_fsc_err_mads_off_no_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                           cam_lkas=FSC_ERR)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert ctrl._tja_restore_target is None


class TestMazdaHudStage2mFuzz(unittest.TestCase):
  def test_hud_no_oscillation_without_state_change(self):
    rng = random.Random(20260815)
    ctrl = _controller(False)
    cam = _laneinfo_tja(0, transition=2)
    last = None
    flips = 0
    mads, av, en = True, False, False
    for i in range(200):
      if rng.random() < 0.08:
        mads, av, en = rng.choice([
          (True, False, False), (True, True, False), (True, True, True),
          (False, False, False), (False, True, False), (False, True, True),
        ])
      ctrl.frame = 50 if i % 7 == 0 else ctrl.frame + 1
      _, sends = _step(ctrl, mads, mads_enabled=mads, available=av, enabled=en,
                       cc_enabled=en, long_active=en, cam_laneinfo=cam)
      tja = _packed_tja(sends)
      if tja is not None:
        if last is not None and tja != last:
          flips += 1
        last = tja
        assert tja in (0, 2, 3)
    assert flips <= 40

  def test_restore_fuzz_bounds(self):
    rng = random.Random(20260815)
    unbounded = 0
    third = 0
    for _ in range(80):
      ctrl = _controller(False)
      ctr = rng.randrange(16)
      crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                       crz_btns_counter=ctr)
      if not crz:
        continue
      total = 1
      attempts = 1
      period = _next_wheel_ctr(ctr)
      for step_i in range(40):
        if rng.random() < 0.15:
          period = _next_wheel_ctr(period)
          attempts += 1
        crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=period)
        total += int(bool(crz))
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        if total > TJA_RESTORE_MRCC_MAX_TX_TOTAL:
          unbounded += 1
      crz, _ = _step(ctrl, True, available=True, enabled=False,
                     crz_btns_counter=_next_wheel_ctr(_next_wheel_ctr(period)))
      if crz and attempts >= TJA_RESTORE_MAX_ATTEMPTS:
        third += 1
    assert unbounded == 0
    assert third == 0
