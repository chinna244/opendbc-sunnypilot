"""Stage 2J final stock-hardware HUD.

Two accepted compromises only:
  #1 TJA-off while MADS ON + MRCC ACTIVE may FULL-OFF MRCC.
  #2 MADS ON + MRCC ARMED packs TJA=0 so physical MRCC ARMED→OFF works.

Does not reintroduce Stage 2H post-edge MRCC blanking. Does not change
restore TX, CRZ_BTNS contents, panda, CAM_LKAS, or TJA_TRANSITION.
Modeled OEM acceptance is not vehicle proof.
"""
from __future__ import annotations

import unittest

from opendbc.can import CANPacker
from opendbc.can import CANParser
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  CAM_LKAS, CRZ_BTNS, CRZ_EVENTS, _assert_clean_mrcc, _controller, _decode_laneinfo,
  _hud_step, _laneinfo_tja, _next_wheel_ctr, _start_restore_period, _step, _tja_press)
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


def _expected(mads, mrcc):
  if not mads:
    return 0
  if mrcc == "ACTIVE":
    return 3
  if mrcc == "ARMED":
    return 0
  return 2


# Route 00000022--cc26bb0e65 physical TJA-on from OFF/OFF (route-relative s).
ROUTE22_TJA_ON = (
  {"EVENT": "R22#1", "TIME": 19.126, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 0.0, "brake": True},
  {"EVENT": "R22#2", "TIME": 76.495, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 1.997, "brake": False},
  {"EVENT": "R22#3", "TIME": 88.446, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 4.915, "brake": False},
  {"EVENT": "R22#4", "TIME": 100.276, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 1.289, "brake": True},
  {"EVENT": "R22#5", "TIME": 124.836, "PRE_MRCC": "OFF", "OEM_TEMP_ARM": True, "v": 0.0, "brake": True},
)

ROUTE21_MRCC_OFF_IGNORED = 14


def _model_off_tja_on(ctrl, *, brake=False, v_ego=0.0, cam_tja=0, transition=2):
  """MADS OFF + MRCC OFF → physical TJA → OEM ARM → restore TX → OEM OFF."""
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
  tja_edge = _packed_tja(sends)
  assert tja_edge in (None, 0)
  if tja_edge is not None:
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


class TestMazdaHudStage2jPacker(unittest.TestCase):
  def test_steady_matrix_never_tja4(self):
    packer = CANPacker("mazda_2017")
    parser = _parser()
    rows = (
      (False, "OFF", 0),
      (True, "OFF", 2),
      (False, "ARMED", 0),
      (True, "ARMED", 0),
      (False, "ACTIVE", 0),
      (True, "ACTIVE", 3),
    )
    armed_tja2 = 0
    for mads, mrcc, exp in rows:
      for fsc in (0, 2, 3, 4):
        cam = _cam(fsc, transition=1)
        msg = mazdacan.create_alert_command(
          packer, cam, False, False,
          mrcc_active=(mrcc == "ACTIVE"), mads_enabled=mads, mrcc_armed=(mrcc == "ARMED"))
        parser.update([(0, [msg])])
        vl = parser.vl["CAM_LANEINFO"]
        assert vl["TJA"] == exp, (mads, mrcc, fsc, vl["TJA"])
        assert vl["TJA"] != 4
        assert vl["TJA_TRANSITION"] == 1
        if mads and mrcc == "ARMED" and vl["TJA"] == 2:
          armed_tja2 += 1
    assert armed_tja2 == 0

  def test_force_tja0_holds_off_not_active(self):
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


class TestMazdaHudStage2jSteady(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_controller_six_rows(self, alpha_long):
    armed_tja2 = 0
    rows = (
      (False, False, False, 0),
      (True, False, False, 2),
      (False, True, False, 0),
      (True, True, False, 0),
      (False, True, True, 0),
      (True, True, True, 3),
    )
    for mads, available, enabled, exp in rows:
      ctrl = _controller(alpha_long)
      cam = _laneinfo_tja(4, transition=2)
      kw = dict(mads_enabled=mads, available=available, enabled=enabled, cam_laneinfo=cam)
      if enabled:
        kw.update(cc_enabled=True, long_active=True)
      crz, sends = _hud_step(ctrl, mads, **kw)
      assert crz == []
      tja = _packed_tja(sends)
      assert tja == exp, (mads, available, enabled, tja)
      assert tja != 4
      assert _packed_tr(sends) == 2
      if mads and available and not enabled and tja == 2:
        armed_tja2 += 1
      assert not hasattr(ctrl, "_hud_mrcc_off_interlock")
    assert armed_tja2 == 0


class TestMazdaHudStage2jOffRestore(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_pending_off_restore_holds_tja0(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _tja_press(ctrl, True, tja=1, available=False, enabled=False, cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target == "OFF"
    tja_edge = _packed_tja(sends)
    assert tja_edge in (None, 0)
    crz, sends = _start_restore_period(ctrl, True, tja=0, available=True, enabled=False,
                                       cam_laneinfo=cam, crz_btns_counter=3)
    _assert_clean_mrcc(crz)
    tja_tx = _packed_tja(sends)
    if tja_tx is not None:
      assert tja_tx == 0
    crz, sends = _hud_step(ctrl, True, tja=0, available=False, enabled=False,
                           cam_laneinfo=cam, crz_btns_counter=_next_wheel_ctr(3))
    assert crz == []
    assert ctrl._tja_restore_target is None
    assert _packed_tja(sends) == 2

  @parametrize("alpha_long", [False, True])
  def test_no_stuck_hold_after_resolve(self, alpha_long):
    ctrl = _controller(alpha_long)
    _model_off_tja_on(ctrl)
    assert ctrl._tja_restore_target is None
    assert ctrl._hud_off_restore_tja0_hold is False
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 2


class TestMazdaHudStage2jTransitions(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_mrcc_arm_while_mads_on_packs_tja0(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 2
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           mrcc=1, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert ctrl._tja_restore_target is None
    assert not hasattr(ctrl, "_hud_mrcc_off_interlock")

  @parametrize("alpha_long", [False, True])
  def test_armed_tja0_before_physical_mrcc_then_off_tja2(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           mrcc=1, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert not any(a == CRZ_BTNS for a, _, _ in sends)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 2
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_set_engage_armed_to_active_tja3(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert _packed_tja(sends) == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, set_p=1, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 3
    assert _packed_tja(sends) != 4

  @parametrize("alpha_long", [False, True])
  def test_oem_active_to_armed_tja0(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 3
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_tja_off_while_armed_preserves_armed(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert _packed_tja(sends) == 0
    crz, sends = _tja_press(ctrl, False, tja=1, available=True, enabled=False,
                            pre_available=True, pre_enabled=False, cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target == "ARMED"
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_tja_off_while_active_no_restore_fight(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(3, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert _packed_tja(sends) == 3
    crz, sends = _tja_press(ctrl, False, tja=1, available=True, enabled=True,
                            pre_available=True, pre_enabled=True, cam_laneinfo=cam)
    assert crz == []
    assert ctrl._tja_restore_target is None
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert not any(a == CRZ_BTNS for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_policy_change_extra_packs_before_next_slot(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert _packed_tja(sends) == 2
    ctrl.frame = 55
    crz, sends = _step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                       cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0


class TestMazdaHudStage2jRoute22(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_route22_all_five_modeled_off(self, alpha_long):
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


class TestMazdaHudStage2jRoute21(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_route21_modeled_mrcc_off_from_tja0(self, alpha_long):
    modeled_off = 0
    armed_tja2 = 0
    for _ in range(ROUTE21_MRCC_OFF_IGNORED):
      ctrl = _controller(alpha_long)
      cam = _laneinfo_tja(0, transition=2)
      crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                             cam_laneinfo=cam)
      assert crz == []
      tja = _packed_tja(sends)
      if tja == 2:
        armed_tja2 += 1
      assert tja == 0
      crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                             mrcc=1, cam_laneinfo=cam)
      assert crz == []
      assert _packed_tja(sends) == 0
      assert not any(a == CRZ_BTNS for a, _, _ in sends)
      crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                             cam_laneinfo=cam)
      assert crz == []
      assert _packed_tja(sends) == 2
      modeled_off += 1
    assert modeled_off == 14
    assert armed_tja2 == 0


class TestMazdaHudStage2jRoute19(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_active_stable_tja3_no_synth(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(3, transition=2)
    for _ in range(8):
      crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                             cc_enabled=True, long_active=True, cam_laneinfo=cam)
      assert crz == []
      assert _packed_tja(sends) == 3
      assert _packed_tja(sends) != 4
      assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)
      assert any(a == CAM_LKAS for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_no_spontaneous_active_to_armed_from_hud(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(3, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 3
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 3


class TestMazdaHudStage2jNoStage2h(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_no_post_edge_interlock(self, alpha_long):
    ctrl = _controller(alpha_long)
    assert not hasattr(ctrl, "_hud_mrcc_off_interlock")
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           mrcc=1, cam_laneinfo=cam)
    assert crz == []
    assert _packed_tja(sends) == 0
    assert not hasattr(ctrl, "_hud_mrcc_off_interlock")
    assert not any(a == CRZ_BTNS for a, _, _ in sends)


class TestMazdaHudStage2jInvariants(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_cruise_buttons_do_not_synth_and_do_not_hold_tja2_armed(self, alpha_long):
    cam = _laneinfo_tja(0, transition=2)
    armed_tja2 = 0
    for btn in ("mrcc", "set_p", "set_m", "res", "cancel"):
      ctrl = _controller(alpha_long)
      kw = {btn: 1, "mads_enabled": True, "available": True, "enabled": False,
            "cam_laneinfo": cam}
      crz, sends = _hud_step(ctrl, True, **kw)
      assert crz == []
      tja = _packed_tja(sends)
      if tja == 2:
        armed_tja2 += 1
      assert tja == 0
      assert ctrl._tja_restore_target is None
    assert armed_tja2 == 0

  def test_expected_helper_matches_packer(self):
    for mads in (False, True):
      for mrcc in ("OFF", "ARMED", "ACTIVE"):
        assert _expected(mads, mrcc) != 4
        if mads and mrcc == "ARMED":
          assert _expected(mads, mrcc) == 0
