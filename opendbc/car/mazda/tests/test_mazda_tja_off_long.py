"""TJA toggles MADS only; final MRCC matches the per-edge pre-TJA snapshot.

Physical TJA is a MADS/lateral toggle. One rising edge:

  1. snapshot confident MRCC immediately before that edge (OFF/ARMED/ACTIVE/UNKNOWN)
  2. toggle MADS
  3. after OEM TJA_BUTTON settles, if post-TJA state differs from OFF or ARMED,
     restore with one bounded clean MRCC_BUTTON gesture contained in one 10 Hz
     wheel period (≤10 TX / 1 locked CTR; stop on the next wheel CTR)
  4. if the snapshot already matches, send nothing
  5. if ACTIVE, never synthesize (do not invent RES); never pack CAM_LANEINFO.TJA in {2,3,4}
  6. if UNKNOWN, fail closed

Driver MRCC/SET+/SET-/RES/CANCEL never toggle MADS and abort a pending restore.
Stock TJA is TJA_BUTTON only (route 00000017). Restoration is clean MRCC_BUTTON.
CAN_OFF is not the 2025 CX-5 master cruise control.
"""

from types import SimpleNamespace
import random

import unittest
from opendbc.car.mazda.tests.unittest_compat import parametrize

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.mazda.carcontroller import (TJA_OFF_COMPENSATION_MAX_TX,
                                             TJA_OFF_COMPENSATION_TIMEOUT_FRAMES,
                                             TJA_RESTORE_MRCC_MAX_TX,
                                             TJA_RESTORE_MRCC_MAX_UNIQUE_CTR,
                                             CarController)
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

CRZ_BTNS = 0x09D
CAM_LKAS = 0x243
CAM_LANEINFO_ADDR = 0x440

CAM_LANEINFO = {
  "LINE_VISIBLE": 0, "LINE_NOT_VISIBLE": 0, "LANE_LINES": 0,
  "BIT1": 0, "BIT2": 0, "BIT3": 0, "NO_ERR_BIT": 0, "S1": 0, "S1_HBEAM": 0,
}
CAM_LKAS_STOCK = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0}


def _controller(alpha_long):
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [],
                               alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [],
                                     alpha_long, False, False)
  cc = CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)
  cc.frame = 1
  return cc


def _cc(lat_active, cancel=False, resume=False, enabled=False, long_active=False):
  CC = structs.CarControl.new_message()
  CC.latActive = lat_active
  CC.enabled = enabled
  CC.longActive = long_active
  CC.actuators.torque = 0.2 if lat_active else 0.0
  CC.cruiseControl.cancel = cancel
  CC.cruiseControl.resume = resume
  return CC.as_reader()


def _cs(toggles=0, tja=0, available=False, enabled=False, standstill=False, brake=False,
        v_ego=10.0, set_p=0, set_m=0, res=0, cancel=0, mrcc=0,
        pre_available=False, pre_enabled=False, pre_unknown=False, tja_edge_reset=False,
        crz_btns_counter=3, cam_laneinfo=None):
  out = SimpleNamespace(
    vEgoRaw=v_ego, steeringTorque=0, brakePressed=brake, standstill=standstill,
    gasPressed=False,
    cruiseState=SimpleNamespace(available=available, enabled=enabled),
  )
  return SimpleNamespace(
    out=out, tja_toggles_this_update=toggles, tja_button=tja, crz_btns_counter=crz_btns_counter,
    cancel_button=cancel, accel_button=set_p, decel_button=set_m, resume_button=res,
    main_button=mrcc, lkas_allowed_speed=True, cam_lkas=CAM_LKAS_STOCK,
    cam_laneinfo=CAM_LANEINFO if cam_laneinfo is None else cam_laneinfo,
    stock_radar_alive=False, fsc_settled=True,
    tja_pre_cruise_available=pre_available, tja_pre_cruise_enabled=pre_enabled,
    tja_pre_cruise_unknown=pre_unknown, tja_edge_reset=tja_edge_reset,
  )


def _decode_crz(sends):
  frames = [(addr, dat, bus) for addr, dat, bus in sends if addr == CRZ_BTNS]
  if not frames:
    return []
  cp = CANParser("mazda_2017", [("CRZ_BTNS", float("nan"))], 0)
  for st in cp.message_states.values():
    st.ignore_checksum = True
    st.ignore_counter = True
    st.ignore_alive = True
  decoded = []
  for addr, dat, bus in frames:
    cp.update([(0, [(addr, dat, bus)])])
    vl = cp.vl["CRZ_BTNS"]
    decoded.append({
      "CAN_OFF": int(vl["CAN_OFF"]), "TJA": int(vl["TJA_BUTTON"]),
      "MRCC": int(vl["MRCC_BUTTON"]), "SET_P": int(vl["SET_P"]),
      "SET_M": int(vl["SET_M"]), "RES": int(vl["RES"]),
      "CTR": int(vl["CTR"]),
    })
  return decoded


def _decode_laneinfo(sends):
  frames = [(addr, dat, bus) for addr, dat, bus in sends if addr == CAM_LANEINFO_ADDR]
  if not frames:
    return []
  cp = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
  for st in cp.message_states.values():
    st.ignore_checksum = True
    st.ignore_counter = True
    st.ignore_alive = True
  decoded = []
  for addr, dat, bus in frames:
    cp.update([(0, [(addr, dat, bus)])])
    vl = cp.vl["CAM_LANEINFO"]
    decoded.append({
      "TJA": int(vl["TJA"]), "TJA_TRANSITION": int(vl["TJA_TRANSITION"]),
      "LANE_LINES": int(vl["LANE_LINES"]), "ERR_BIT": int(vl["ERR_BIT"]),
    })
  return decoded


def _hud_step(ctrl, lat, **kw):
  """Force the 2 Hz CAM_LANEINFO slot (frame % 50 == 0)."""
  ctrl.frame = 50
  return _step(ctrl, lat, **kw)


def _laneinfo_tja(tja, transition=0):
  d = dict(CAM_LANEINFO)
  d["TJA"] = tja
  d["TJA_TRANSITION"] = transition
  return d


def _step(ctrl, lat, **kw):
  CC_SP = structs.CarControlSP()
  _, sends = ctrl.update(_cc(lat, cancel=kw.pop("op_cancel", False),
                             resume=kw.pop("resume", False),
                             enabled=kw.pop("cc_enabled", False),
                             long_active=kw.pop("long_active", False)),
                         CC_SP, _cs(**kw), 0)
  return _decode_crz(sends), sends


def _tja_press(ctrl, lat_after, **kw):
  return _step(ctrl, lat_after, toggles=1, **kw)


def _tja_off(ctrl, **kw):
  return _tja_press(ctrl, False, **kw)


def _next_wheel_ctr(ctr):
  return (int(ctr) + 1) & 0xF


def _start_restore_period(ctrl, lat, **kw):
  """Sample mismatch on the current wheel CTR, then TX on the next 10 Hz period."""
  ctr = int(kw.pop("crz_btns_counter", 3)) & 0xF
  crz, sends = _step(ctrl, lat, crz_btns_counter=ctr, **kw)
  assert crz == [], "restore must not TX in the mismatch-sample wheel period"
  return _step(ctrl, lat, crz_btns_counter=_next_wheel_ctr(ctr), **kw)


def _tja_press_then_restore(ctrl, lat, **kw):
  """Physical TJA (not held) + next wheel CTR so the one-period burst may TX."""
  ctr = int(kw.get("crz_btns_counter", 3)) & 0xF
  crz, _ = _tja_press(ctrl, lat, **kw)
  assert crz == [], "restore must not TX in the mismatch-sample wheel period"
  rest = dict(kw)
  rest["tja"] = 0
  rest["crz_btns_counter"] = _next_wheel_ctr(ctr)
  return _step(ctrl, lat, **rest)


def _assert_clean_mrcc(crz):
  assert len(crz) == 1
  assert crz[0]["MRCC"] == 1
  assert crz[0]["CAN_OFF"] == 0
  assert crz[0]["TJA"] == 0
  assert crz[0]["SET_P"] == 0
  assert crz[0]["SET_M"] == 0
  assert crz[0]["RES"] == 0


def _assert_clean_can_off(crz):
  # Compatibility alias: restoration is MRCC_BUTTON, not CAN_OFF.
  _assert_clean_mrcc(crz)


def _restore_off_after_tja(ctrl, lat_after, **kw):
  """Physical TJA from confident OFF, wait while held, then restore after OEM ARM."""
  ctr = int(kw.pop("crz_btns_counter", 3)) & 0xF
  crz, _ = _tja_press(ctrl, lat_after, tja=1, available=False, enabled=False,
                      crz_btns_counter=ctr, **kw)
  assert crz == []
  crz, _ = _step(ctrl, lat_after, tja=0, available=False, enabled=False, crz_btns_counter=ctr)
  assert crz == []
  crz, _ = _start_restore_period(ctrl, lat_after, tja=0, available=True, enabled=False,
                                 crz_btns_counter=ctr)
  _assert_clean_mrcc(crz)
  crz, _ = _step(ctrl, lat_after, tja=0, available=False, enabled=False,
                 crz_btns_counter=_next_wheel_ctr(ctr))
  assert crz == []
  return ctrl


def _restore_armed_after_tja(ctrl, lat_after, **kw):
  """Physical TJA from confident ARMED; OEM disarms; restore ARMED."""
  ctr = int(kw.pop("crz_btns_counter", 3)) & 0xF
  crz, _ = _tja_press(ctrl, lat_after, tja=1, available=True, enabled=False,
                      pre_available=True, crz_btns_counter=ctr, **kw)
  assert crz == []
  crz, _ = _step(ctrl, lat_after, tja=0, available=True, enabled=False, crz_btns_counter=ctr)
  assert crz == []
  crz, _ = _start_restore_period(ctrl, lat_after, tja=0, available=False, enabled=False,
                                 crz_btns_counter=ctr)
  _assert_clean_mrcc(crz)
  crz, _ = _step(ctrl, lat_after, tja=0, available=True, enabled=False,
                 crz_btns_counter=_next_wheel_ctr(ctr))
  assert crz == []
  return ctrl


@parametrize("alpha_long", [False, True])
class TestTjaMadsOnlyIndependence(unittest.TestCase):
  def test_sequence_1_mads_on_from_mrcc_off_restores_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    _restore_off_after_tja(ctrl, True)
    assert ctrl._tja_restore_mrcc_off_pending is False

  def test_sequence_2_mads_off_from_mrcc_off_restores_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    _step(ctrl, True, toggles=1, tja=0, available=False)
    _restore_off_after_tja(ctrl, False)

  def test_sequence_3_pre_armed_stays_armed_never_synth(self, alpha_long):
    ctrl = _controller(alpha_long)
    for lat_after in (True, False):
      ctrl = _controller(alpha_long)
      if not lat_after:
        ctrl._lat_active_prev = True
      crz, _ = _tja_press(ctrl, lat_after, tja=1, available=True, enabled=False,
                          pre_available=True)
      assert crz == []
      crz, _ = _step(ctrl, lat_after, tja=0, available=True, enabled=False)
      assert crz == []
      for _ in range(20):
        crz, _ = _step(ctrl, lat_after, available=True, enabled=False)
        assert crz == []

  def test_sequence_3b_pre_armed_oem_disarm_restores_armed(self, alpha_long):
    for lat_after in (True, False):
      ctrl = _controller(alpha_long)
      if not lat_after:
        ctrl._lat_active_prev = True
      _restore_armed_after_tja(ctrl, lat_after)
      assert ctrl._tja_restore_mrcc_off_pending is False

  def test_sequence_4_pre_active_never_synth(self, alpha_long):
    for lat_after in (True, False):
      ctrl = _controller(alpha_long)
      crz, _ = _tja_press(ctrl, lat_after, tja=1, available=True, enabled=True,
                          pre_available=True, pre_enabled=True,
                          cc_enabled=True, long_active=True)
      assert crz == []
      crz, _ = _step(ctrl, lat_after, tja=0, available=True, enabled=True,
                     cc_enabled=True, long_active=True)
      assert crz == []
      for _ in range(20):
        crz, _ = _step(ctrl, lat_after, available=True, enabled=True,
                       cc_enabled=True, long_active=True)
        assert crz == []

  def test_sequence_4b_pre_active_oem_off_fail_closed_no_resume(self, alpha_long):
    """Historical routes: some physical TJA events cancel ACTIVE to OFF.

    Do not invent RES/SET. Fail closed: zero synthetic.
    """
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=1, available=True, enabled=True,
                        pre_available=True, pre_enabled=True,
                        cc_enabled=True, long_active=True)
    assert crz == []
    crz, _ = _step(ctrl, True, tja=0, available=False, enabled=False)
    assert crz == []
    for _ in range(20):
      crz, _ = _step(ctrl, True, available=False, enabled=False)
      assert crz == []

  def test_sequence_5_driver_set_aborts_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=1, available=False)
    _step(ctrl, True, tja=0, available=True)
    crz, _ = _step(ctrl, True, tja=0, set_p=1, available=True, enabled=True,
                   cc_enabled=True, long_active=True)
    assert crz == []
    assert ctrl._tja_restore_mrcc_off_pending is False
    crz, _ = _tja_press(ctrl, False, tja=1, available=True, enabled=True,
                        pre_available=True, pre_enabled=True,
                        cc_enabled=True, long_active=True)
    assert crz == []
    for _ in range(20):
      crz, _ = _step(ctrl, False, available=True, enabled=True,
                     cc_enabled=True, long_active=True)
      assert crz == []

  def test_sequence_6_driver_mrcc_aborts_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=1, available=False)
    assert crz == []
    crz, _ = _step(ctrl, True, tja=0, mrcc=1, available=True, enabled=False)
    assert crz == []
    assert ctrl._tja_restore_mrcc_off_pending is False
    crz, _ = _step(ctrl, True, mrcc=0, available=True, enabled=False)
    assert crz == []

  def test_sequence_7_later_long_buttons_do_not_change_mads(self, alpha_long):
    ctrl = _controller(alpha_long)
    _restore_off_after_tja(ctrl, True)
    for kwargs in (
      {"mrcc": 1, "available": False},
      {"set_p": 1, "available": True, "enabled": True},
      {"set_m": 1, "available": True, "enabled": True},
      {"res": 1, "available": True, "enabled": True},
      {"cancel": 1, "available": True, "enabled": True},
    ):
      crz, _ = _step(ctrl, True, toggles=0, **kwargs)
      assert crz == []

  def test_sequence_8_pre_armed_both_tja_presses(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=1, available=True, pre_available=True)
    assert crz == []
    crz, _ = _tja_press(ctrl, False, tja=1, available=True, pre_available=True)
    assert crz == []
    for _ in range(20):
      crz, _ = _step(ctrl, False, available=True)
      assert crz == []

  def test_sequence_9_pre_active_both_tja_presses(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=1, available=True, enabled=True,
                        pre_available=True, pre_enabled=True,
                        cc_enabled=True, long_active=True)
    assert crz == []
    crz, _ = _tja_press(ctrl, False, tja=1, available=True, enabled=True,
                        pre_available=True, pre_enabled=True,
                        cc_enabled=True, long_active=True)
    assert crz == []
    for _ in range(20):
      crz, _ = _step(ctrl, False, available=True, enabled=True,
                     cc_enabled=True, long_active=True)
      assert crz == []

  def test_held_tja_does_not_tx_until_release(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=1, available=True, enabled=False)
    assert crz == []
    crz, _ = _step(ctrl, True, tja=0, available=True, enabled=False)
    assert crz == []
    crz, _ = _step(ctrl, True, tja=0, available=True, enabled=False, crz_btns_counter=4)
    _assert_clean_mrcc(crz)

  def test_unknown_pre_state_does_not_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=0, available=True, enabled=False, pre_unknown=True)
    assert crz == []
    assert ctrl._tja_restore_mrcc_off_pending is False
    for _ in range(20):
      crz, _ = _step(ctrl, True, available=True, enabled=False)
      assert crz == []

  def test_rapid_second_tja_unknown_aborts_stale_restore(self, alpha_long):
    for gap_frames in (5, 10, 15, 20, 30):  # 50/100/150/200/300 ms at 100 Hz
      ctrl = _controller(alpha_long)
      _tja_press(ctrl, True, tja=1, available=False)
      for _ in range(gap_frames):
        _step(ctrl, True, tja=0, available=False)
      crz, _ = _tja_press(ctrl, False, tja=1, available=False, pre_unknown=True)
      assert crz == []
      assert ctrl._tja_restore_mrcc_off_pending is False
      crz, _ = _step(ctrl, False, tja=0, available=True)
      assert crz == []

  def test_driver_overrides_abort_pending_and_hold(self, alpha_long):
    for btn in ("set_p", "set_m", "res", "cancel", "mrcc"):
      ctrl = _controller(alpha_long)
      _tja_press(ctrl, True, tja=1, available=False)
      crz, _ = _step(ctrl, True, tja=0, **{btn: 1}, available=True, enabled=False)
      assert crz == []
      assert ctrl._tja_restore_mrcc_off_pending is False
      crz, _ = _step(ctrl, True, available=True, enabled=False)
      assert crz == []

  def test_new_tja_aborts_previous_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=0, available=True)
    assert ctrl._tja_restore_mrcc_off_pending is True
    crz, _ = _tja_press(ctrl, False, tja=1, available=True, pre_available=True)
    assert crz == []
    # New edge snapshots ARMED and stays ARMED: pending exists, but zero TX.
    assert ctrl._tja_restore_target == "ARMED"
    crz, _ = _step(ctrl, False, tja=0, available=True)
    assert crz == []

  def test_timeout_without_arm_emits_nothing(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=0, available=False, enabled=False)
    for _ in range(TJA_OFF_COMPENSATION_TIMEOUT_FRAMES + 2):
      crz, _ = _step(ctrl, True, available=False, enabled=False)
      assert crz == []
    crz, _ = _step(ctrl, True, available=True, enabled=False)
    assert crz == []

  def test_hold_every_frame_then_stops(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                     crz_btns_counter=3)
    _assert_clean_mrcc(crz)
    for _ in range(TJA_OFF_COMPENSATION_MAX_TX - 1):
      crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=4)
      _assert_clean_mrcc(crz)
    for _ in range(20):
      crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=4)
      assert crz == []
    assert ctrl._tja_off_comp_tx == TJA_RESTORE_MRCC_MAX_TX

  def test_own_mrcc_echo_does_not_abort_hold(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                     crz_btns_counter=3)
    _assert_clean_mrcc(crz)
    crz, _ = _step(ctrl, True, mrcc=1, available=True, enabled=False, crz_btns_counter=4)
    _assert_clean_mrcc(crz)

  def test_op_cancel_does_not_stack_second_talker(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                            crz_btns_counter=3)
    crz, _ = _step(ctrl, True, tja=0, available=True, enabled=False, op_cancel=True,
                   crz_btns_counter=4)
    assert len(crz) <= 1
    if crz:
      assert crz[0]["MRCC"] == 1
      assert crz[0]["CAN_OFF"] == 0

  def test_parser_batch_two_toggles_fail_closed(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _step(ctrl, True, toggles=2, tja=0, available=False)
    assert crz == []
    assert ctrl._tja_restore_mrcc_off_pending is False

  def test_standstill_moving_brake_pre_off_still_restores(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                     standstill=True, brake=True, v_ego=0.0)
    _assert_clean_mrcc(crz)
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, False, tja=0, available=True, enabled=False, v_ego=20.0)
    _assert_clean_mrcc(crz)

  def test_brake_rise_aborts_pending_restore(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=1, available=False, enabled=False, brake=False)
    assert crz == []
    crz, _ = _step(ctrl, True, tja=0, brake=True, available=True, enabled=False)
    assert crz == []
    assert ctrl._tja_restore_target is None
    crz, _ = _step(ctrl, True, brake=True, available=True, enabled=False)
    assert crz == []

  def test_cam_lkas_still_sent_on_tja(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, False, tja=1, available=True)
    _, sends = _step(ctrl, False, tja=0, available=True)
    assert sum(1 for s in sends if s[0] == CAM_LKAS) == 1

  def test_controller_reinit_drops_pending(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=1, available=True)
    ctrl2 = _controller(alpha_long)
    crz, _ = _step(ctrl2, True, tja=0, available=True)
    assert crz == []

  def test_carstate_reset_aborts_pending(self, alpha_long):
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=0, available=True)
    assert ctrl._tja_restore_mrcc_off_pending is True
    crz, _ = _step(ctrl, True, tja=0, available=True, tja_edge_reset=True)
    assert crz == []
    assert ctrl._tja_restore_mrcc_off_pending is False
    crz, _ = _step(ctrl, True, tja=0, available=True)
    assert crz == []

  def test_alpha_long_active_not_cancelled_by_tja(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, False, tja=0, available=True, enabled=True,
                        pre_available=True, pre_enabled=True,
                        cc_enabled=True, long_active=True)
    assert crz == []
    crz, _ = _step(ctrl, False, available=True, enabled=True,
                   cc_enabled=True, long_active=True, op_cancel=False)
    assert crz == []

  def test_route_00000011_tja_from_off_restores_with_hold(self, alpha_long):
    """ff7df7d6f9c3403b|00000011: TJA from OFF, OEM ARMED, restore with MRCC_BUTTON hold."""
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press(ctrl, True, tja=1, available=False, enabled=False)
    assert crz == []
    crz, _ = _start_restore_period(ctrl, True, tja=0, available=True, enabled=False)
    _assert_clean_mrcc(crz)
    for _ in range(TJA_OFF_COMPENSATION_MAX_TX - 1):
      crz, _ = _step(ctrl, True, tja=0, available=True, enabled=False, crz_btns_counter=4)
      _assert_clean_mrcc(crz)
    crz, _ = _step(ctrl, True, tja=0, available=False, enabled=False, crz_btns_counter=4)
    assert crz == []
    # Later TJA OFF from OFF (after driver or restore already OFF).
    crz, _ = _tja_press(ctrl, False, tja=1, available=False, enabled=False)
    assert crz == []
    crz, _ = _start_restore_period(ctrl, False, tja=0, available=True, enabled=False)
    _assert_clean_mrcc(crz)

  def test_route_00000012_same_independence(self, alpha_long):
    ctrl = _controller(alpha_long)
    _restore_off_after_tja(ctrl, True)
    _restore_off_after_tja(ctrl, False)

  def test_route_00000016_armed_tja_does_not_restore(self, alpha_long):
    """00000016: TJA ON from OFF restores OFF. A later TJA while ARMED stays ARMED."""
    ctrl = _controller(alpha_long)
    _restore_off_after_tja(ctrl, True)
    crz, _ = _tja_press(ctrl, False, tja=0, available=True, pre_available=True)
    assert crz == []
    for _ in range(20):
      crz, _ = _step(ctrl, False, available=True)
      assert crz == []

  def test_route_00000017_stock_tja_is_tja_button_only(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True)
    _assert_clean_mrcc(crz)
    assert crz[0]["CAN_OFF"] == 0
    assert crz[0]["TJA"] == 0

  def test_final_state_invariants(self, alpha_long):
    ctrl = _controller(alpha_long)
    _restore_off_after_tja(ctrl, True)
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=0, available=True, pre_available=True)
    crz, _ = _step(ctrl, True, available=True)
    assert crz == []
    ctrl = _controller(alpha_long)
    _restore_armed_after_tja(ctrl, True)
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=0, available=True, enabled=True,
               pre_available=True, pre_enabled=True,
               cc_enabled=True, long_active=True)
    crz, _ = _step(ctrl, True, available=True, enabled=True,
                   cc_enabled=True, long_active=True)
    assert crz == []

  def test_interval_contained_ctr_is_one_logical_press(self, alpha_long):
    """Route 00000019: spanning a wheel idle is a second press. Stay in one period."""
    ctrl = _controller(alpha_long)
    crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                     crz_btns_counter=3)
    _assert_clean_mrcc(crz)
    packed = {crz[0]["CTR"]}
    for _ in range(8):
      crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=4)
      if not crz:
        break
      _assert_clean_mrcc(crz)
      packed.add(crz[0]["CTR"])
    assert len(packed) == TJA_RESTORE_MRCC_MAX_UNIQUE_CTR
    crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=5)
    assert crz == []

  def test_route_00000018_all_32_tja_final_state(self, alpha_long):
    """Replay every physical TJA on ff7df7d6f9c3403b|00000018--f0206780a5."""
    # (id, pre, post_physical_TJA). Restore if and only if those differ and pre is OFF/ARMED.
    events = (
      (1, "OFF", "ARMED"), (2, "OFF", "ARMED"), (3, "ARMED", "OFF"),
      (4, "OFF", "ARMED"), (5, "OFF", "ARMED"), (6, "ARMED", "OFF"),
      (7, "OFF", "ARMED"), (8, "ARMED", "OFF"), (9, "OFF", "ARMED"),
      (10, "OFF", "ARMED"), (11, "ARMED", "OFF"), (12, "OFF", "ARMED"),
      (13, "OFF", "ARMED"), (14, "OFF", "ARMED"), (15, "ARMED", "OFF"),
      (16, "OFF", "ARMED"), (17, "ARMED", "OFF"), (18, "OFF", "ARMED"),
      (19, "OFF", "ARMED"), (20, "OFF", "ARMED"), (21, "ARMED", "OFF"),
      (22, "OFF", "ARMED"), (23, "ARMED", "OFF"), (24, "OFF", "ARMED"),
      (25, "ARMED", "OFF"), (26, "OFF", "ARMED"), (27, "ARMED", "ARMED"),
      (28, "ARMED", "ARMED"), (29, "ARMED", "OFF"), (30, "OFF", "ARMED"),
      (31, "OFF", "ARMED"), (32, "ARMED", "OFF"),
    )
    assert len(events) == 32
    violations = 0
    for tid, pre, post in events:
      ctrl = _controller(alpha_long)
      pre_av = pre in ("ARMED", "ACTIVE")
      pre_en = pre == "ACTIVE"
      post_av = post in ("ARMED", "ACTIVE")
      post_en = post == "ACTIVE"
      crz, _ = _tja_press(ctrl, True, tja=1, available=post_av, enabled=post_en,
                          pre_available=pre_av, pre_enabled=pre_en)
      assert crz == [], tid
      crz, _ = _step(ctrl, True, tja=0, available=post_av, enabled=post_en)
      need = pre in ("OFF", "ARMED") and post != pre
      if not need:
        assert crz == [], tid
        for _ in range(5):
          crz, _ = _step(ctrl, True, available=post_av, enabled=post_en)
          assert crz == [], tid
        continue
      assert crz == [], tid
      crz, _ = _step(ctrl, True, tja=0, available=post_av, enabled=post_en,
                     crz_btns_counter=4)
      _assert_clean_mrcc(crz)
      # Gesture takes; cruise returns to snapshot.
      final_av = pre in ("ARMED", "ACTIVE")
      final_en = pre == "ACTIVE"
      crz, _ = _step(ctrl, True, tja=0, available=final_av, enabled=final_en,
                     crz_btns_counter=4)
      assert crz == [], tid
      if ctrl._tja_restore_target is not None and pre != "UNKNOWN":
        violations += 1
    assert violations == 0

  def test_hud_tja2_clamped_while_mrcc_active(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(2, transition=2)
    cam["LANE_LINES"] = 3
    crz, sends = _hud_step(ctrl, True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    li = _decode_laneinfo(sends)
    assert len(li) == 1
    assert li[0]["TJA"] == 0
    assert li[0]["TJA_TRANSITION"] == 2
    assert li[0]["LANE_LINES"] == 3

  def test_hud_tja2_passthrough_when_mrcc_not_active(self, alpha_long):
    for available, enabled in ((False, False), (True, False)):
      ctrl = _controller(alpha_long)
      crz, sends = _hud_step(ctrl, True, available=available, enabled=enabled,
                             cam_laneinfo=_laneinfo_tja(2, transition=2))
      assert crz == []
      li = _decode_laneinfo(sends)
      assert len(li) == 1
      assert li[0]["TJA"] == 2
      assert li[0]["TJA_TRANSITION"] == 2

  @parametrize("tja", [0, 1])
  def test_hud_non_engaged_tja_passthrough_while_active(self, alpha_long, tja):
    ctrl = _controller(alpha_long)
    crz, sends = _hud_step(ctrl, True, available=True, enabled=True,
                           cc_enabled=True, long_active=True,
                           cam_laneinfo=_laneinfo_tja(tja, transition=0))
    assert crz == []
    li = _decode_laneinfo(sends)
    assert len(li) == 1
    assert li[0]["TJA"] == tja

  @parametrize("tja", [2, 3, 4])
  def test_hud_engaged_tja_clamped_while_mrcc_active(self, alpha_long, tja):
    """Functional MRCC preservation: TJA 2/3/4 while ACTIVE canceled cruise.

    Route 00000019 events 24/40/43/46 packed TJA=3, not TJA=2. Event 28 kept
    ACTIVE with TJA=0. Do not rewrite TJA_TRANSITION.
    """
    ctrl = _controller(alpha_long)
    crz, sends = _hud_step(ctrl, True, available=True, enabled=True,
                           cc_enabled=True, long_active=True,
                           cam_laneinfo=_laneinfo_tja(tja, transition=2))
    assert crz == []
    li = _decode_laneinfo(sends)
    assert len(li) == 1
    assert li[0]["TJA"] == 0
    assert li[0]["TJA_TRANSITION"] == 2

  def test_pre_active_12_events_never_tx_tja2(self, alpha_long):
    """Replay the 12 proven comma/mazda-safety pre-ACTIVE TJA events.

    Failures historically packed CAM_LANEINFO.TJA=2 while CRZ_ACTIVE, which
    latched CRZ_CTRL 08 20. Successes packed TJA=0. After the clamp, every
    event must pack non-2 while ACTIVE and emit no synthetic SET/RES/MRCC.
    """
    events = (
      ("00000000", 213.607, 0), ("00000000", 219.067, 2),
      ("00000007", 171.630, 0), ("00000007", 171.820, 0),
      ("00000007", 178.771, 2), ("00000007", 199.710, 2),
      ("00000008", 128.263, 2), ("00000008", 202.172, 2),
      ("00000008", 509.473, 2),
      ("00000011", 107.912, 0), ("00000011", 308.061, 0),
      ("00000011", 323.501, 0),
    )
    assert len(events) == 12
    tja2_tx = latch_0820 = synth = set_speed = 0
    pass_count = 0
    for route, t_s, cam_tja in events:
      ctrl = _controller(alpha_long)
      cam = _laneinfo_tja(cam_tja)
      event_synth = 0
      crz, sends = _hud_step(ctrl, True, available=True, enabled=True,
                             pre_available=True, pre_enabled=True,
                             cc_enabled=True, long_active=True, cam_laneinfo=cam)
      assert crz == [], (route, t_s)
      li = _decode_laneinfo(sends)
      assert len(li) == 1, (route, t_s)
      packed = li[0]["TJA"]
      if packed in (2, 3, 4):
        tja2_tx += 1
        latch_0820 += 1
      crz, sends = _tja_press(ctrl, False, tja=1, available=True, enabled=True,
                              pre_available=True, pre_enabled=True,
                              cc_enabled=True, long_active=True, cam_laneinfo=cam)
      if crz:
        event_synth += 1
      for _ in range(20):
        crz, sends = _step(ctrl, False, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
        if crz:
          event_synth += 1
      synth += event_synth
      if packed not in (2, 3, 4) and event_synth == 0:
        pass_count += 1
    assert tja2_tx == 0
    assert latch_0820 == 0
    assert synth == 0
    assert set_speed == 0
    assert pass_count == 12

  def test_route_00000019_all_53_physical_events(self, alpha_long):
    """Replay every physical button on 00000019--de4affe83c against the user matrix."""
    # (id, button, pre_mads, pre_mrcc, cam_tja, oem_hist_post)
    events = (
      (1, "TJA", False, "OFF", 2, "ARMED"),
      (2, "MRCC", True, "OFF", 0, "ARMED"),
      (3, "TJA", True, "ARMED", 2, "ARMED"),
      (4, "MRCC", False, "ARMED", 0, "ARMED"),
      (5, "MRCC", False, "ARMED", 0, "OFF"),
      (6, "MRCC", False, "OFF", 0, "ARMED"),
      (7, "MRCC", False, "ARMED", 0, "ARMED"),
      (8, "TJA", False, "OFF", 2, "ARMED"),
      (9, "TJA", True, "ARMED", 0, "OFF"),
      (10, "TJA", False, "OFF", 2, "ARMED"),
      (11, "MRCC", True, "ARMED", 0, "ARMED"),
      (12, "MRCC", True, "ARMED", 0, "OFF"),
      (13, "TJA", True, "OFF", 2, "ARMED"),
      (14, "TJA", False, "ARMED", 0, "OFF"),
      (15, "TJA", True, "OFF", 0, "ARMED"),
      (16, "TJA", False, "ARMED", 0, "OFF"),
      (17, "MRCC", True, "OFF", 0, "ARMED"),
      (18, "TJA", True, "ARMED", 2, "ARMED"),
      (19, "SET+", False, "ARMED", 2, "ARMED"),
      (20, "SET+", False, "ARMED", 3, "ACTIVE"),
      (21, "SET+", False, "ACTIVE", 3, "ACTIVE"),
      (22, "RES", False, "ACTIVE", 3, "ACTIVE"),
      (23, "SET+", False, "ACTIVE", 3, "ACTIVE"),
      (24, "TJA", False, "ACTIVE", 3, "ACTIVE"),
      (25, "MRCC", True, "OFF", 0, "ARMED"),
      (26, "SET+", True, "ARMED", 0, "ACTIVE"),
      (27, "SET+", True, "ACTIVE", 0, "ACTIVE"),
      (28, "TJA", True, "ACTIVE", 0, "ACTIVE"),
      (29, "TJA", False, "ACTIVE", 3, "ACTIVE"),
      (30, "MRCC", True, "OFF", 0, "ARMED"),
      (31, "TJA", True, "ARMED", 2, "ARMED"),
      (32, "TJA", False, "ARMED", 0, "OFF"),
      (33, "TJA", True, "OFF", 2, "ARMED"),
      (34, "TJA", False, "ARMED", 2, "OFF"),
      (35, "TJA", True, "OFF", 2, "ARMED"),
      (36, "MRCC", False, "ARMED", 2, "ARMED"),
      (37, "SET+", False, "ARMED", 0, "ACTIVE"),
      (38, "RES", False, "ARMED", 0, "ACTIVE"),
      (39, "TJA", False, "ACTIVE", 0, "ACTIVE"),
      (40, "TJA", True, "ACTIVE", 3, "ACTIVE"),
      (41, "TJA", False, "OFF", 2, "ARMED"),
      (42, "SET+", True, "ARMED", 3, "ACTIVE"),
      (43, "TJA", True, "ACTIVE", 3, "ACTIVE"),
      (44, "TJA", False, "OFF", 0, "ARMED"),
      (45, "SET+", True, "ARMED", 3, "ACTIVE"),
      (46, "TJA", True, "ACTIVE", 3, "ACTIVE"),
      (47, "TJA", False, "OFF", 2, "ARMED"),
      (48, "TJA", True, "ARMED", 0, "OFF"),
      (49, "MRCC", False, "OFF", 0, "ARMED"),
      (50, "MRCC", False, "ARMED", 0, "OFF"),
      (51, "MRCC", False, "OFF", 0, "ARMED"),
      (52, "TJA", False, "ARMED", 2, "ARMED"),
      (53, "TJA", True, "ARMED", 0, "OFF"),
    )
    assert len(events) == 53
    fails = []
    for eid, button, pre_mads, pre_mrcc, cam_tja, post_mrcc in events:
      ctrl = _controller(alpha_long)
      cam = _laneinfo_tja(cam_tja, 2 if cam_tja else 0)
      expected_mads = (not pre_mads) if button == "TJA" else pre_mads
      if button == "TJA":
        expected_mrcc = pre_mrcc
        pre_av = pre_mrcc in ("ARMED", "ACTIVE")
        pre_en = pre_mrcc == "ACTIVE"
        hist_av = post_mrcc in ("ARMED", "ACTIVE")
        hist_en = post_mrcc == "ACTIVE"
        crz, sends = _hud_step(ctrl, expected_mads, available=pre_av, enabled=pre_en,
                               pre_available=pre_av, pre_enabled=pre_en,
                               cc_enabled=pre_en, long_active=pre_en, cam_laneinfo=cam)
        if pre_mrcc == "ACTIVE":
          for li in _decode_laneinfo(sends):
            if li["TJA"] in (2, 3, 4):
              fails.append((eid, "tja_engaged_packed", li["TJA"]))
        crz, _ = _tja_press(ctrl, expected_mads, tja=1, available=hist_av, enabled=hist_en,
                            pre_available=pre_av, pre_enabled=pre_en, cam_laneinfo=cam)
        if crz:
          fails.append((eid, "synth_while_held", crz))
        crz, _ = _step(ctrl, expected_mads, tja=0, available=hist_av, enabled=hist_en,
                       cam_laneinfo=cam, crz_btns_counter=3)
        need = pre_mrcc in ("OFF", "ARMED") and post_mrcc != pre_mrcc
        if pre_mrcc == "ACTIVE":
          if crz:
            fails.append((eid, "active_synth", crz))
          actual = "ACTIVE"
        elif not need:
          if crz:
            fails.append((eid, "false_restore", crz))
          actual = pre_mrcc
        else:
          if crz:
            fails.append((eid, "tx_in_sample_period", crz))
          crz, _ = _step(ctrl, expected_mads, tja=0, available=hist_av, enabled=hist_en,
                         cam_laneinfo=cam, crz_btns_counter=4)
          if not crz:
            fails.append((eid, "missed_restore"))
          else:
            _assert_clean_mrcc(crz)
            packed_one = {crz[0]["CTR"]}
            for _ in range(8):
              crz2, _ = _step(ctrl, expected_mads, available=hist_av, enabled=hist_en,
                              crz_btns_counter=4)
              if not crz2:
                break
              packed_one.add(crz2[0]["CTR"])
            if len(packed_one) > 1:
              fails.append((eid, "multigesture", packed_one))
            crz2, _ = _step(ctrl, expected_mads, available=hist_av, enabled=hist_en,
                            crz_btns_counter=5)
            if crz2:
              fails.append((eid, "second_logical_press", crz2))
          actual = pre_mrcc
          crz, _ = _step(ctrl, expected_mads,
                         available=(pre_mrcc in ("ARMED", "ACTIVE")),
                         enabled=(pre_mrcc == "ACTIVE"), crz_btns_counter=5)
          if crz:
            fails.append((eid, "tx_after_match", crz))
        if actual != expected_mrcc:
          fails.append((eid, "mrcc", actual, expected_mrcc))
      else:
        extra = {}
        if button == "MRCC":
          extra["mrcc"] = 1
        elif button == "SET+":
          extra["set_p"] = 1
        elif button == "SET-":
          extra["set_m"] = 1
        elif button == "RES":
          extra["res"] = 1
        elif button == "CANCEL":
          extra["cancel"] = 1
        kw = {
          "available": pre_mrcc in ("ARMED", "ACTIVE"),
          "enabled": pre_mrcc == "ACTIVE",
          "pre_available": pre_mrcc in ("ARMED", "ACTIVE"),
          "pre_enabled": pre_mrcc == "ACTIVE",
          "cc_enabled": pre_mrcc == "ACTIVE",
          "long_active": pre_mrcc == "ACTIVE",
          "cam_laneinfo": cam,
        }
        crz, sends = _step(ctrl, pre_mads, **kw, **extra)
        if any(f["TJA"] or f["MRCC"] or f["SET_P"] or f["SET_M"] or f["RES"] for f in crz):
          fails.append((eid, "long_synth", crz))
        if pre_mrcc == "ACTIVE":
          _, sends = _hud_step(ctrl, pre_mads, **kw)
          for li in _decode_laneinfo(sends):
            if li["TJA"] in (2, 3, 4):
              fails.append((eid, "tja_engaged_packed", li["TJA"]))
    assert fails == [], fails[:8]

  def test_matrix_row_1_no_tja_stays_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    crz, _ = _step(ctrl, False, available=False, enabled=False)
    assert crz == []
    assert ctrl._tja_restore_target is None


RESTORE_FUZZ_SEED = 20260813
RESTORE_FUZZ_ITERATIONS = 1_000_000


def _crz_tja_count(crz):
  return sum(1 for f in crz if f["TJA"])


class TestMazdaTjaOffLong(unittest.TestCase):
  def test_tja_restore_timing_fuzz(self):
    rng = random.Random(RESTORE_FUZZ_SEED)
    false_off = unknown_restore = pre_armed_cancel = pre_active_cancel = 0
    driver_owned_cancel = mrcc_mads = set_plus_mads = set_minus_mads = 0
    res_mads = cancel_mads = unbounded = panda_reject = 0
    stale_after_restart = synth_after_reinit = 0
    active_tja2_tx = active_0820_latch = 0

    def do_step(ctrl, lat, **kw):
      nonlocal active_tja2_tx, active_0820_latch
      cam = dict(CAM_LANEINFO)
      cam["TJA"] = rng.choice([0, 1, 2, 3])
      cam["TJA_TRANSITION"] = rng.choice([0, 2])
      kw.setdefault("cam_laneinfo", cam)
      if rng.randrange(8) == 0:
        ctrl.frame = 50
      crz, sends = _step(ctrl, lat, **kw)
      if kw.get("enabled"):
        for li in _decode_laneinfo(sends):
          if li["TJA"] in (2, 3, 4):
            active_tja2_tx += 1
            active_0820_latch += 1
      return crz

    def do_tja_press(ctrl, lat_after, **kw):
      return do_step(ctrl, lat_after, toggles=1, **kw)

    ctrl = _controller(False)
    mads = False
    mrcc = "OFF"  # OFF / ARMED / ACTIVE / UNKNOWN
    tja_held = False
    pending_arm_delay = -1
    pending_disarm_delay = -1
    last_tja_unconfirmed = False
    driver_took = False
    wheel_ctr = 3

    def snapshot_kw():
      if mrcc == "UNKNOWN":
        return {"pre_unknown": True, "pre_available": False, "pre_enabled": False,
                "available": False, "enabled": False}
      return {
        "pre_unknown": last_tja_unconfirmed,
        "pre_available": mrcc in ("ARMED", "ACTIVE"),
        "pre_enabled": mrcc == "ACTIVE",
        "available": mrcc in ("ARMED", "ACTIVE"),
        "enabled": mrcc == "ACTIVE",
      }

    for _ in range(RESTORE_FUZZ_ITERATIONS):
      ev = rng.randrange(16)
      if ev == 15:
        ctrl = _controller(False)
        crz = do_step(ctrl, mads, available=(mrcc in ("ARMED", "ACTIVE")),
                      enabled=(mrcc == "ACTIVE"))
        if crz:
          stale_after_restart += 1
          synth_after_reinit += 1
        mads = False
        mrcc = "OFF"
        tja_held = False
        pending_arm_delay = -1
        pending_disarm_delay = -1
        last_tja_unconfirmed = False
        driver_took = False
        continue

      kw = snapshot_kw()
      long_kw = {**kw, "cc_enabled": mrcc == "ACTIVE", "long_active": mrcc == "ACTIVE",
                 "crz_btns_counter": wheel_ctr}
      if _ % 10 == 0:
        wheel_ctr = (wheel_ctr + 1) & 0xF

      if ev == 0:
        classified_off = (not kw["pre_unknown"] and not kw["pre_available"] and not kw["pre_enabled"])
        actual_off = mrcc == "OFF"
        if classified_off and not actual_off and not last_tja_unconfirmed:
          false_off += 1
        mads = not mads
        tja_held = True
        snap = mrcc
        crz = do_tja_press(ctrl, mads, tja=1, **long_kw)
        if kw["pre_unknown"] and (crz or ctrl._tja_restore_mrcc_off_pending):
          unknown_restore += 1
        if snap == "ARMED" and crz:
          pre_armed_cancel += 1
        if snap == "ACTIVE" and crz:
          pre_active_cancel += 1
        if driver_took and snap in ("ARMED", "ACTIVE") and crz:
          driver_owned_cancel += 1
        if mrcc == "OFF" and not kw["pre_unknown"]:
          pending_arm_delay = rng.randrange(4)  # 0–3 cycle delay
          pending_disarm_delay = -1
        elif mrcc == "ARMED" and not kw["pre_unknown"]:
          pending_disarm_delay = rng.randrange(4)
          pending_arm_delay = -1
        else:
          pending_arm_delay = -1
          pending_disarm_delay = -1
        last_tja_unconfirmed = False
        driver_took = False
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        for f in crz:
          if f["TJA"] or f["CAN_OFF"] or f["SET_P"] or f["SET_M"] or f["RES"]:
            panda_reject += 1
      elif ev == 1:
        tja_held = False
        crz = do_step(ctrl, mads, tja=0, **long_kw)
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        for f in crz:
          if f["TJA"] or f["CAN_OFF"] or f["SET_P"] or f["SET_M"] or f["RES"]:
            panda_reject += 1
      elif ev == 2:
        crz = do_step(ctrl, mads, mrcc=1, **long_kw)
        mrcc_mads += _crz_tja_count(crz)
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        mrcc = "OFF" if mrcc != "OFF" else "ARMED"
        last_tja_unconfirmed = False
        driver_took = True
        pending_arm_delay = -1
        pending_disarm_delay = -1
      elif ev in (3, 4):
        extra = {"set_p": 1} if ev == 3 else {"set_m": 1}
        crz = do_step(ctrl, mads, **long_kw, **extra)
        if ev == 3:
          set_plus_mads += _crz_tja_count(crz)
        else:
          set_minus_mads += _crz_tja_count(crz)
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        if mrcc == "ARMED":
          mrcc = "ACTIVE"
        driver_took = True
        last_tja_unconfirmed = False
        pending_arm_delay = -1
        pending_disarm_delay = -1
      elif ev == 5:
        crz = do_step(ctrl, mads, res=1, **long_kw)
        res_mads += _crz_tja_count(crz)
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        driver_took = True
        last_tja_unconfirmed = False
        pending_arm_delay = -1
        pending_disarm_delay = -1
      elif ev == 6:
        crz = do_step(ctrl, mads, cancel=1, **long_kw)
        cancel_mads += _crz_tja_count(crz)
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        driver_took = True
        last_tja_unconfirmed = False
        pending_arm_delay = -1
        pending_disarm_delay = -1
      elif ev == 7:
        mrcc = "UNKNOWN"
        last_tja_unconfirmed = True
        crz = do_step(ctrl, mads, pre_unknown=True, available=False, enabled=False)
        # In-flight restore from a prior confident TJA is not an UNKNOWN-origin restore.
      elif ev == 8 and tja_held:
        # Rapid second TJA while previous restore may be pending.
        mads = not mads
        crz = do_tja_press(ctrl, mads, tja=1, pre_unknown=True, available=False, enabled=False)
        if crz or ctrl._tja_restore_mrcc_off_pending:
          unknown_restore += 1
        last_tja_unconfirmed = True
      else:
        if pending_arm_delay == 0 and mrcc == "OFF" and not driver_took:
          mrcc = "ARMED"
          last_tja_unconfirmed = False
          pending_arm_delay = -1
        elif pending_disarm_delay == 0 and mrcc == "ARMED" and not driver_took:
          mrcc = "OFF"
          last_tja_unconfirmed = False
          pending_disarm_delay = -1
        elif pending_arm_delay > 0:
          pending_arm_delay -= 1
        elif pending_disarm_delay > 0:
          pending_disarm_delay -= 1
        crz = do_step(ctrl, mads, tja=int(tja_held),
                      available=(mrcc in ("ARMED", "ACTIVE")),
                      enabled=(mrcc == "ACTIVE"),
                      cc_enabled=(mrcc == "ACTIVE"),
                      long_active=(mrcc == "ACTIVE"),
                      pre_unknown=(mrcc == "UNKNOWN"))
        if crz and not tja_held and not driver_took and not last_tja_unconfirmed:
          if mrcc == "OFF":
            mrcc = "ARMED"
          elif mrcc == "ARMED":
            mrcc = "OFF"
        if crz and mrcc == "ARMED" and driver_took:
          pre_armed_cancel += 1
        if crz and mrcc == "ACTIVE":
          pre_active_cancel += 1
        if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
          unbounded += 1
        for f in crz:
          if f["TJA"] or f["CAN_OFF"] or f["SET_P"] or f["SET_M"] or f["RES"]:
            panda_reject += 1

    assert false_off == 0
    assert unknown_restore == 0
    assert pre_armed_cancel == 0
    assert pre_active_cancel == 0
    assert driver_owned_cancel == 0
    assert mrcc_mads == 0
    assert set_plus_mads == 0
    assert set_minus_mads == 0
    assert res_mads == 0
    assert cancel_mads == 0
    assert unbounded == 0
    assert panda_reject == 0
    assert stale_after_restart == 0
    assert synth_after_reinit == 0
    assert active_tja2_tx == 0
    assert active_0820_latch == 0
