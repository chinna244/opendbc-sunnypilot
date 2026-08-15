"""Stage 2I-A restore HUD-ordering, ported onto Stage 2J policy.

Hold bus0 TJA=0 until OFF-preservation restore resolves. Steady ARMED is
TJA=0 (Stage 2J compromise #2), not Stage 2G TJA=2.

Does not reintroduce Stage 2H post-edge MRCC blanking. Does not change
restore TX, CRZ_BTNS contents, panda, or TJA_TRANSITION. Modeled OEM
acceptance is not vehicle proof.
"""
from __future__ import annotations

import unittest

from opendbc.can import CANPacker
from opendbc.can import CANParser
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests import test_mazda_hud_stage2g as _s2g
from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  CAM_LKAS, CRZ_BTNS, CRZ_EVENTS, _assert_clean_mrcc, _controller, _decode_laneinfo,
  _hud_step, _laneinfo_tja, _next_wheel_ctr, _restore_armed_after_tja,
  _start_restore_period, _step, _tja_press)
from opendbc.car.mazda.tests.unittest_compat import parametrize


def _parser():
  parser = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 0)
  for st in parser.message_states.values():
    st.ignore_checksum = True
    st.ignore_counter = True
    st.ignore_alive = True
  return parser


def _cam(tja, transition=2, lane_lines=3):
  return {
    "LINE_VISIBLE": 1, "LINE_NOT_VISIBLE": 0, "LANE_LINES": lane_lines,
    "BIT1": 1, "BIT2": 0, "BIT3": 0, "NO_ERR_BIT": 0, "S1": 0, "S1_HBEAM": 0,
    "TJA": tja, "TJA_TRANSITION": transition,
  }


def _packed_tja(sends):
  li = _decode_laneinfo(sends)
  return None if not li else li[0]["TJA"]


def _packed_tr(sends):
  li = _decode_laneinfo(sends)
  return None if not li else li[0]["TJA_TRANSITION"]


# Route 00000022--cc26bb0e65 physical TJA-on from OFF/OFF (route-relative s).
ROUTE22_TJA_ON = (
  {"EVENT": "R22#1", "TIME": 19.126, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 0.0, "brake": True},
  {"EVENT": "R22#2", "TIME": 76.495, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 1.997, "brake": False},
  {"EVENT": "R22#3", "TIME": 88.446, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 4.915, "brake": False},
  {"EVENT": "R22#4", "TIME": 100.276, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 1.289, "brake": True},
  {"EVENT": "R22#5", "TIME": 124.836, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 0.0, "brake": True},
)

ROUTE21_TJA_ON = (
  {"EVENT": "R21#1", "TIME": 38.500, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True},
  {"EVENT": "R21#2", "TIME": 133.591, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True},
  {"EVENT": "R21#3", "TIME": 225.521, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True},
  {"EVENT": "R21#4", "TIME": 267.801, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True},
  {"EVENT": "R21#5", "TIME": 276.381, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True},
  {"EVENT": "R21#6", "TIME": 282.811, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True},
)


def _model_off_tja_on(ctrl, *, brake=False, v_ego=0.0, cam_tja=0, transition=2):
  """MADS OFF + MRCC OFF → physical TJA → OEM ARM → restore TX → OEM OFF.

  Returns HUD/restore timestamps in controller steps, not vehicle seconds.
  """
  cam = _laneinfo_tja(cam_tja, transition=transition)
  ctr = 3
  out = {
    "PRE_MRCC": "OFF",
    "OEM_TEMP_ARM": False,
    "RESTORE_TX_START": None,
    "FIRST_TJA2": None,
    "RESTORE_ACCEPTED": None,
    "RESTORE_RESOLVED": None,
    "tja_while_unresolved": [],
    "hud": [],
  }
  step = 0

  def rec(sends, crz, tag, available):
    tja = _packed_tja(sends)
    tr = _packed_tr(sends)
    out["hud"].append({
      "step": step, "tag": tag, "tja": tja, "tr": tr,
      "target": ctrl._tja_restore_target, "tx": bool(crz), "av": available,
    })
    if tja == 2 and out["FIRST_TJA2"] is None:
      out["FIRST_TJA2"] = step
    if crz and out["RESTORE_TX_START"] is None:
      out["RESTORE_TX_START"] = step
    if tja is not None and ctrl._tja_restore_target == "OFF":
      out["tja_while_unresolved"].append(tja)
    if tja is not None and crz:
      out["tja_while_unresolved"].append(tja)

  crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                         brake=brake, v_ego=v_ego, cam_laneinfo=cam, crz_btns_counter=ctr)
  rec(sends, crz, "pre", False)
  assert _packed_tja(sends) == 0
  assert ctrl._tja_restore_target is None
  step += 1

  crz, sends = _tja_press(ctrl, True, tja=1, available=False, enabled=False, brake=brake,
                          v_ego=v_ego, cam_laneinfo=cam, crz_btns_counter=ctr)
  rec(sends, crz, "tja_edge", False)
  assert crz == []
  assert ctrl._tja_restore_target == "OFF"
  assert _packed_tja(sends) == 0
  assert _packed_tr(sends) == transition
  step += 1

  crz, sends = _step(ctrl, True, tja=0, available=False, enabled=False, brake=brake,
                     v_ego=v_ego, cam_laneinfo=cam, crz_btns_counter=ctr)
  rec(sends, crz, "released_still_off", False)
  assert crz == []
  step += 1

  crz, sends = _start_restore_period(ctrl, True, tja=0, available=True, enabled=False,
                                     brake=brake, v_ego=v_ego, cam_laneinfo=cam,
                                     crz_btns_counter=ctr)
  rec(sends, crz, "restore_tx", True)
  _assert_clean_mrcc(crz)
  out["OEM_TEMP_ARM"] = True
  tja_tx = _packed_tja(sends)
  if tja_tx is not None:
    assert tja_tx == 0
  step += 1

  crz, sends = _hud_step(ctrl, True, tja=0, available=False, enabled=False, brake=brake,
                         v_ego=v_ego, cam_laneinfo=cam,
                         crz_btns_counter=_next_wheel_ctr(ctr))
  rec(sends, crz, "oem_off", False)
  assert crz == []
  out["RESTORE_ACCEPTED"] = step
  out["RESTORE_RESOLVED"] = step
  assert ctrl._tja_restore_target is None
  assert _packed_tja(sends) == 2
  assert _packed_tr(sends) == transition
  out["FINAL_MRCC"] = "OFF"
  out["RESULT"] = "modeled_tja0_until_resolved_then_tja2"
  return out


class TestMazdaHudStage2iaPacker(unittest.TestCase):
  def test_force_tja0_holds_off_and_armed_not_active(self):
    packer = CANPacker("mazda_2017")
    parser = _parser()
    cam = _cam(4, transition=1)
    msg = mazdacan.create_alert_command(
      packer, cam, False, False, mrcc_active=False, mads_enabled=True,
      mrcc_armed=False, force_tja0=True)
    parser.update([(0, [msg])])
    assert parser.vl["CAM_LANEINFO"]["TJA"] == 0
    assert parser.vl["CAM_LANEINFO"]["TJA_TRANSITION"] == 1
    msg = mazdacan.create_alert_command(
      packer, cam, False, False, mrcc_active=True, mads_enabled=True,
      mrcc_armed=False, force_tja0=True)
    parser.update([(0, [msg])])
    assert parser.vl["CAM_LANEINFO"]["TJA"] == 3
    assert parser.vl["CAM_LANEINFO"]["TJA"] != 4

  def test_force_tja0_false_is_stage2g_off(self):
    packer = CANPacker("mazda_2017")
    parser = _parser()
    cam = _cam(0, transition=2)
    msg = mazdacan.create_alert_command(
      packer, cam, False, False, mrcc_active=False, mads_enabled=True,
      mrcc_armed=False, force_tja0=False)
    parser.update([(0, [msg])])
    assert parser.vl["CAM_LANEINFO"]["TJA"] == 2


class TestMazdaHudStage2iaSteadyMatrix(_s2g.TestMazdaHudStage2gMatrix):
  """No pending restore: Stage 2J steady matrix (ARMED TJA=0)."""


class TestMazdaHudStage2iaNegatives(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_steady_off_no_restore_still_tja2(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target is None
    assert _packed_tja(sends) == 2
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_armed_not_held(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_active_not_held(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(4, transition=1)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 3
    assert _packed_tr(sends) == 1

  @parametrize("alpha_long", [False, True])
  def test_non_tja_mads_on_no_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert _packed_tja(sends) == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target is None
    assert _packed_tja(sends) == 2

  @parametrize("alpha_long", [False, True])
  def test_init_without_restore_no_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    assert ctrl._tja_restore_target is None
    assert ctrl._hud_off_restore_tja0_hold is False
    cam = _laneinfo_tja(2, transition=2)
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_set_res_cancel_abort_clears_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _tja_press(ctrl, True, tja=1, available=False, enabled=False, cam_laneinfo=cam)
    assert ctrl._tja_restore_target == "OFF"
    assert _packed_tja(sends) == 0
    for btn in ("set_p", "res", "cancel"):
      ctrl = _controller(alpha_long)
      _tja_press(ctrl, True, tja=1, available=False, enabled=False, cam_laneinfo=cam)
      kw = {btn: 1, "available": True, "enabled": False, "cam_laneinfo": cam, "tja": 0}
      crz, sends = _hud_step(ctrl, True, **kw)
      assert ctrl._tja_restore_target is None
      assert crz == []
      assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_physical_mrcc_abort_clears_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _tja_press(ctrl, True, tja=1, available=False, enabled=False, cam_laneinfo=cam)
    crz, sends = _hud_step(ctrl, True, tja=0, available=True, enabled=False, mrcc=1,
                           cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target is None
    assert _packed_tja(sends) == 0

  @parametrize("alpha_long", [False, True])
  def test_armed_restore_does_not_hold_tja0_via_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _restore_armed_after_tja(ctrl, True, cam_laneinfo=cam)
    crz, sends = _hud_step(ctrl, True, available=True, enabled=False, cam_laneinfo=cam)
    assert crz == []
    # ARMED restore is not the OFF-restore hold. Stage 2J packs TJA=0 while ARMED.
    assert _packed_tja(sends) == 0


class TestMazdaHudStage2iaRouteReplay(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_route22_all_five_hold_until_resolved(self, alpha_long):
    results = []
    for ev in ROUTE22_TJA_ON:
      ctrl = _controller(alpha_long)
      modeled = _model_off_tja_on(
        ctrl, brake=bool(ev.get("brake")), v_ego=float(ev.get("v") or 0.0),
        cam_tja=0, transition=2)
      modeled["EVENT"] = ev["EVENT"]
      assert modeled["OEM_TEMP_ARM"] is True
      assert modeled["RESTORE_TX_START"] is not None
      assert modeled["FIRST_TJA2"] is not None
      assert modeled["RESTORE_ACCEPTED"] is not None
      assert modeled["FIRST_TJA2"] >= modeled["RESTORE_TX_START"]
      assert modeled["FIRST_TJA2"] >= modeled["RESTORE_ACCEPTED"]
      assert all(t == 0 for t in modeled["tja_while_unresolved"]), modeled
      assert modeled["FINAL_MRCC"] == "OFF"
      results.append(modeled)
    assert len(results) == 5
    assert sum(1 for r in results if r["FINAL_MRCC"] == "OFF") == 5
    assert sum(1 for r in results if r["FINAL_MRCC"] == "ARMED") == 0

  @parametrize("alpha_long", [False, True])
  def test_route21_tja_on_not_stuck_at_tja0(self, alpha_long):
    for ev in ROUTE21_TJA_ON:
      ctrl = _controller(alpha_long)
      modeled = _model_off_tja_on(ctrl, cam_tja=0, transition=2)
      assert modeled["FIRST_TJA2"] is not None
      assert modeled["FINAL_MRCC"] == "OFF"
      assert ctrl._tja_restore_target is None
      crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                             cam_laneinfo=_laneinfo_tja(0, transition=2))
      assert crz == []
      assert _packed_tja(sends) == 2
      _ = ev

  @parametrize("alpha_long", [False, True])
  def test_no_second_mrcc_pulse(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    _model_off_tja_on(ctrl, cam_tja=0)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert not any(a == CRZ_BTNS for a, _, _ in sends)
    assert any(a == CAM_LKAS for a, _, _ in sends)


class TestMazdaHudStage2iaNoProblem1Pretend(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_armed_physical_mrcc_does_not_use_stage2h_interlock(self, alpha_long):
    """Stage 2H interlock is not present. Stage 2J already packs TJA=0 while ARMED."""
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert _packed_tja(sends) == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           mrcc=1, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert ctrl._tja_restore_target is None
    assert not hasattr(ctrl, "_hud_mrcc_off_interlock")
