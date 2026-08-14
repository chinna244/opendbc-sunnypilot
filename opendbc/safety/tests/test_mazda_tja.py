#!/usr/bin/env python3
"""Panda single-TJA lateral latch, independence matrix, restart, and parity fuzz."""

import random
import unittest

from opendbc.car.mazda.tja_edge import ARMED, HELD, MazdaTjaEdge
from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

PARITY_SEED = 20260814
PARITY_ITERATIONS = 20000
STRESS_SEED = 20260813
STRESS_ITERATIONS = 100000
# Must match MAZDA_RESTORE_MRCC_MAX_TX in opendbc/safety/modes/mazda.h
RESTORE_MRCC_MAX_TX = 10


class TestMazdaTjaSafety(unittest.TestCase):
  """TJA flag set. Not a CarSafetyTest: ACC-main must not own lateral."""

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self._configure_tja_mads()

  def _configure_tja_mads(self, tja_flag=True, enable_mads=True, longitudinal=False):
    param = (MazdaSafetyFlags.TJA if tja_flag else 0) | (MazdaSafetyFlags.LONG if longitudinal else 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, param)
    self.safety.init_tests()
    self.safety.set_mads_params(enable_mads, False, False)
    self.safety.set_heartbeat_engaged_mads(True)

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return bool(self.safety.safety_tx_hook(msg))

  def _torque_cmd_msg(self, torque):
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": torque})

  def _tja_button_msg(self, tja=False, mrcc=False, set_p=False, set_m=False, res=False, cancel=False, ctr=0):
    values = {
      "TJA_BUTTON": tja,
      "MRCC_BUTTON": mrcc,
      "SET_P": set_p,
      "SET_M": set_m,
      "RES": res,
      "CAN_OFF": cancel,
      "CTR": ctr,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def _acc_main_msg(self, enable):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_AVAILABLE": enable})

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, {"CRZ_ACTIVE": enable})

  def _long_acc_state_msg(self, armed, active):
    return self.packer.make_can_msg_safety("PEDALS", 0, {
      "ACC_OFF": armed and not active,
      "ACC_ACTIVE": active,
      "BRAKE_ON": 0,
    })

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("PEDALS", 0, {"BRAKE_ON": brake})

  def _speed_msg(self, speed):
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, {"SPEED": speed})

  def _torque_driver_msg(self, torque):
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, {"STEER_TORQUE_SENSOR": torque})

  def _rx_single_tja(self, expect_lateral=None):
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertTrue(self._rx(self._tja_button_msg(True)))
    if expect_lateral is not None:
      self.assertEqual(expect_lateral, self.safety.get_controls_allowed_lateral())

  def test_single_tja_toggles_lateral_once(self):
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))
    # Held frames do not repeat-toggle.
    for _ in range(8):
      self.assertTrue(self._rx(self._tja_button_msg(True)))
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._rx_single_tja(expect_lateral=False)
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))

  def test_boot_held_tja_does_not_toggle(self):
    self.assertTrue(self._rx(self._tja_button_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))
    for _ in range(5):
      self.assertTrue(self._rx(self._tja_button_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._tja_button_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_mrcc_set_res_cancel_do_not_toggle_lateral(self):
    self._rx_single_tja(expect_lateral=True)
    for kwargs in (
      {"mrcc": True},
      {"set_p": True},
      {"set_m": True},
      {"res": True},
      {"cancel": True},
    ):
      self.assertTrue(self._rx(self._tja_button_msg(False, **kwargs)))
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_tja_while_other_buttons_still_toggles(self):
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertTrue(self._rx(self._tja_button_msg(True, mrcc=True, set_p=True, cancel=True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_acc_main_toggles_do_not_change_lateral(self):
    self._rx_single_tja(expect_lateral=True)
    for available in (True, False, True, False):
      self.assertTrue(self._rx(self._acc_main_msg(available)))
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))

  def test_mrcc_only_cannot_authorize_nonzero_torque(self):
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))

  def test_startup_init_is_restore_inert(self):
    # mazda_init must leave the MRCC restore window closed: idle MRCC rejected, lateral off,
    # zero-torque CAM_LKAS accepted, nonzero CAM_LKAS rejected.
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._tja_button_msg(True)))  # boot-held TJA
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))

  def test_crz_btns_tx_truth_table_startup_and_engaged(self):
    # Startup / controls not allowed: only cancel. Restore MRCC is not a general grant.
    self.assertFalse(self.safety.get_controls_allowed())
    table = (
      ("idle", {}, False),
      ("tja", {"tja": True}, False),
      ("mrcc", {"mrcc": True}, False),
      ("set", {"set_p": True}, False),
      ("res", {"res": True}, False),
      ("cancel", {"cancel": True}, True),
    )
    for name, kwargs, allowed in table:
      self.assertEqual(allowed, self._tx(self._tja_button_msg(**kwargs)), name)

    # Long engaged: resume/set/cancel/idle allowed; clean MRCC still restore-window-only.
    # Synthetic TJA is never allowed (physical TJA only).
    self.safety.set_controls_allowed(1)
    engaged = (
      ("idle", {}, True),
      ("tja", {"tja": True}, False),
      ("mrcc", {"mrcc": True}, False),
      ("set", {"set_p": True}, True),
      ("res", {"res": True}, True),
      ("cancel", {"cancel": True}, True),
    )
    for name, kwargs, allowed in engaged:
      self.assertEqual(allowed, self._tx(self._tja_button_msg(**kwargs)), f"engaged-{name}")
    self.safety.set_controls_allowed(0)

    # Restore window: only clean MRCC. Existing TJA/SET/RES/CANCEL/idle rules unchanged.
    self._open_restore_window()
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._tx(self._tja_button_msg(cancel=True)))
    self.assertFalse(self._tx(self._tja_button_msg(set_p=True)))
    self.assertFalse(self._tx(self._tja_button_msg(res=True)))

  def test_tja_off_allows_only_cancel_tx(self):
    # After TJA OFF, lateral auth is false. Panda must still allow cancel
    # (Alpha Long) and must reject MRCC/SET/RES unless the MRCC restore window is open.
    self._rx_single_tja(expect_lateral=True)
    self._rx_single_tja(expect_lateral=False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._tja_button_msg(cancel=True)))
    for ctr in range(32):
      self.assertTrue(self._tx(self._tja_button_msg(cancel=True, ctr=ctr % 16)), ctr)
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self._tx(self._tja_button_msg(set_p=True)))
    self.assertFalse(self._tx(self._tja_button_msg(res=True)))
    self.assertFalse(self._tx(self._tja_button_msg()))

  def _open_restore_window(self, *, mads_on=False, longitudinal=False):
    """TJA from confident MRCC OFF, release, OEM ARMED. Window ignores MADS/lateral."""
    if longitudinal:
      self._configure_tja_mads(longitudinal=True)
      self.assertTrue(self._rx(self._long_acc_state_msg(False, False)))
    else:
      self.assertTrue(self._rx(self._acc_main_msg(False)))
    self._rx_single_tja(expect_lateral=True)
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))  # still held
    if mads_on:
      self.assertTrue(self._rx(self._tja_button_msg(False)))
      self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
      if longitudinal:
        self.assertTrue(self._rx(self._long_acc_state_msg(True, False)))
      else:
        self.assertTrue(self._rx(self._acc_main_msg(True)))
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    else:
      if longitudinal:
        self.assertTrue(self._rx(self._long_acc_state_msg(False, False)))
      else:
        self.assertTrue(self._rx(self._acc_main_msg(False)))
      self.assertTrue(self._rx(self._tja_button_msg(False)))
      self.assertTrue(self._rx(self._tja_button_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed_lateral())
      self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
      self.assertTrue(self._rx(self._tja_button_msg(False)))
      if longitudinal:
        self.assertTrue(self._rx(self._long_acc_state_msg(True, False)))
      else:
        self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_restore_window_allows_only_clean_mrcc(self):
    self._open_restore_window()
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True, set_p=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True, res=True)))
    # data[0]==0x01 is still the stock cancel path; extra MRCC does not open a general MRCC grant.
    self.assertTrue(self._tx(self._tja_button_msg(cancel=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True, tja=True)))
    self.assertFalse(self._tx(self._tja_button_msg(set_p=True)))
    self.assertFalse(self._tx(self._tja_button_msg(res=True)))
    self.assertTrue(self._tx(self._tja_button_msg(cancel=True)))

  def test_restore_rejects_mrcc_outside_window_and_pre_armed_active(self):
    out_of_window = 0
    pre_armed = 0
    pre_active = 0
    # TJA ON from OFF while held: no TX yet.
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self._rx_single_tja(expect_lateral=True)
    if self._tx(self._tja_button_msg(mrcc=True)):
      out_of_window += 1
    # Independently ARMED before TJA: both MADS directions reject.
    self._configure_tja_mads()
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    if self._tx(self._tja_button_msg(mrcc=True)):
      pre_armed += 1
    self._rx_single_tja(expect_lateral=False)
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    if self._tx(self._tja_button_msg(mrcc=True)):
      pre_armed += 1
    # Independently ACTIVE before TJA.
    self._configure_tja_mads()
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    if self._tx(self._tja_button_msg(mrcc=True)):
      pre_active += 1
    # Re-init: stale window closed.
    self._configure_tja_mads()
    if self._tx(self._tja_button_msg(mrcc=True)):
      out_of_window += 1
    self.assertEqual(0, out_of_window)
    self.assertEqual(0, pre_armed)
    self.assertEqual(0, pre_active)

  def test_mads_on_from_off_opens_restore_window(self):
    self._open_restore_window(mads_on=True)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))

  def test_mads_off_from_off_opens_restore_window(self):
    self._open_restore_window(mads_on=False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))

  def _open_armed_restore_window(self, *, mads_on=False, longitudinal=False):
    """TJA from confident MRCC ARMED, release, OEM OFF. Re-ARM window."""
    if longitudinal:
      self._configure_tja_mads(longitudinal=True)
      self.assertTrue(self._rx(self._long_acc_state_msg(True, False)))
    else:
      self.assertTrue(self._rx(self._acc_main_msg(True)))
    self._rx_single_tja(expect_lateral=True)
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    if not mads_on:
      self.assertTrue(self._rx(self._tja_button_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed_lateral())
      self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
      self.assertTrue(self._rx(self._tja_button_msg(False)))
    if longitudinal:
      self.assertTrue(self._rx(self._long_acc_state_msg(False, False)))
    else:
      self.assertTrue(self._rx(self._acc_main_msg(False)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pre_armed_stays_armed_rejects_mrcc(self):
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_pre_armed_oem_off_allows_one_gesture(self):
    self._open_armed_restore_window(mads_on=True)
    for _ in range(RESTORE_MRCC_MAX_TX):
      self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_pre_armed_restore_closes_when_rearmed(self):
    self._open_armed_restore_window(mads_on=True)
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_pre_active_never_allows_restore_mrcc(self):
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_tja_off_while_armed_does_not_open_window(self):
    # Pre-TJA ARMED: TJA toggles MADS only. No synthetic MRCC.
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertTrue(self._rx(self._tja_button_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_driver_set_aborts_restore_window(self):
    self._open_restore_window(mads_on=True)
    self.assertTrue(self._rx(self._tja_button_msg(set_p=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_restore_timeout_and_max_tx(self):
    self._open_restore_window()
    for _ in range(RESTORE_MRCC_MAX_TX):
      self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

    self._open_restore_window()
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.safety.set_timer(500001)
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_late_synthetic_mrcc_2s_after_release_rejected(self):
    late = 0
    self._open_restore_window()
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.safety.set_timer(400000)
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.safety.set_timer(2000000)
    if self._tx(self._tja_button_msg(mrcc=True)):
      late += 1
    self.assertEqual(0, late)

  def test_restore_closes_on_physical_mrcc(self):
    self._open_restore_window()
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_restore_closes_on_driver_set_res_cancel_and_success(self):
    self._open_restore_window()
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._tja_button_msg(set_p=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

    self._open_restore_window()
    self.assertTrue(self._rx(self._tja_button_msg(set_m=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

    self._open_restore_window()
    self.assertTrue(self._rx(self._tja_button_msg(res=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

    self._open_restore_window()
    self.assertTrue(self._rx(self._tja_button_msg(cancel=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

    self._open_restore_window()
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

    self._open_restore_window()
    self.assertTrue(self._rx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_restore_rejects_while_tja_held_and_no_recent_tja(self):
    self.assertTrue(self._rx(self._acc_main_msg(False)))
    if self._tx(self._tja_button_msg(mrcc=True)):
      raise AssertionError("no-recent-tja accepted")
    self._open_restore_window()
    # New TJA press after release closes the window.
    self.assertTrue(self._rx(self._tja_button_msg(True)))
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_restore_longitudinal_window_from_pedals(self):
    self._open_restore_window(longitudinal=True)
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self._tx(self._tja_button_msg(set_p=True)))

  def test_active_cruise_never_allows_restore_mrcc(self):
    self._open_restore_window()
    self.assertTrue(self._rx(self.packer.make_can_msg_safety(
      "CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 1})))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))

  def test_non_tja_mazda_still_allows_torque_from_controls_allowed(self):
    self._configure_tja_mads(tja_flag=False, longitudinal=False)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))

  def test_longitudinal_independence_matrix(self):
    self._configure_tja_mads(longitudinal=True)
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertTrue(self._rx(self._long_acc_state_msg(False, False)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())

    self._rx_single_tja(expect_lateral=True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))

    self.assertTrue(self._rx(self._long_acc_state_msg(True, True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self.safety.get_controls_allowed())

    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))

    self.assertTrue(self._rx(self._long_acc_state_msg(True, True)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self.safety.get_controls_allowed())

    self.safety.set_heartbeat_engaged_mads(True)
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))

    self.assertTrue(self._rx(self._long_acc_state_msg(False, False)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))

  def test_safety_reinit_clears_stale_state(self):
    self._rx_single_tja(expect_lateral=True)
    self.safety.set_mads_params(True, False, False)
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.TJA)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertEqual(-1, self.safety.get_mads_button_press())

  def test_safety_reinit_closes_restore_without_new_tja(self):
    self._open_restore_window(mads_on=True)
    self.assertTrue(self._tx(self._tja_button_msg(mrcc=True)))
    self._configure_tja_mads()
    self.assertFalse(self._tx(self._tja_button_msg(mrcc=True)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_held_tja_across_reinit_does_not_synthesize_auth(self):
    self._rx_single_tja(expect_lateral=True)
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.TJA)
    self.safety.set_mads_params(True, False, False)
    self.safety.set_heartbeat_engaged_mads(True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    for _ in range(3):
      self.assertTrue(self._rx(self._tja_button_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._rx(self._tja_button_msg(False)))
    self.assertTrue(self._rx(self._tja_button_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_duplicate_frames_do_not_extra_toggle(self):
    self.assertTrue(self._rx(self._tja_button_msg(False, ctr=1)))
    self.assertTrue(self._rx(self._tja_button_msg(True, ctr=1)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    for ctr in (1, 1, 2, 2, 15, 0, 0):  # includes counter rollover
      self.assertTrue(self._rx(self._tja_button_msg(True, ctr=ctr)))
      self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_rx_lag_clears_lateral_and_does_not_queue_toggle(self):
    self._rx_single_tja(expect_lateral=True)
    self.safety.set_timer(int(3e6))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.safety_config_valid())
    self.assertTrue(self._rx(self._tja_button_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))

    self.assertTrue(self._rx(self._acc_main_msg(False)))
    self.assertTrue(self._rx(self._torque_driver_msg(0)))
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._rx_single_tja(expect_lateral=True)
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))

  def test_driver_torque_limits_not_weakened(self):
    self._rx_single_tja(expect_lateral=True)
    self.safety.set_torque_driver(-80, -80)
    # Stock Mazda DriverTorqueLimited: large opposing driver torque must still block.
    self.assertFalse(self._tx(self._torque_cmd_msg(200)))


class TestMazdaTjaParityFuzz(unittest.TestCase):
  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.TJA)
    self.safety.init_tests()
    self.safety.set_mads_params(True, False, False)
    self.safety.set_heartbeat_engaged_mads(True)

  def _tja_msg(self, tja, ctr=0):
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": tja, "CTR": ctr})

  def test_python_panda_edge_parity(self):
    rng = random.Random(PARITY_SEED)
    edge = MazdaTjaEdge()
    expected_lateral = False
    divergences = 0
    for i in range(PARITY_ITERATIONS):
      tja = bool(rng.randrange(2))
      if rng.random() < 0.1:
        tja = True  # extra held stretches
      toggle = edge.update(tja)
      if toggle:
        expected_lateral = not expected_lateral
      self.assertTrue(self.safety.safety_rx_hook(self._tja_msg(tja, ctr=i % 16)))
      panda_lat = bool(self.safety.get_controls_allowed_lateral())
      if panda_lat != expected_lateral:
        divergences += 1
      self.assertEqual(expected_lateral, panda_lat, f"iter={i} tja={tja} state={edge.state}")
    self.assertEqual(0, divergences)

  def test_python_edge_stress_no_false_positive(self):
    rng = random.Random(STRESS_SEED)
    edge = MazdaTjaEdge()
    false_positives = 0
    for _ in range(STRESS_ITERATIONS):
      tja = bool(rng.randrange(2))
      before = edge.state
      toggle = edge.update(tja)
      if toggle and before != ARMED:
        false_positives += 1
      if before == HELD and tja and toggle:
        false_positives += 1
    self.assertEqual(0, false_positives)

  def test_long_buttons_never_change_lateral_fuzz(self):
    rng = random.Random(STRESS_SEED)
    self.assertTrue(self.safety.safety_rx_hook(self._tja_msg(False)))
    changed = 0
    for i in range(1_000_000):
      if i == 250000:
        self.assertTrue(self.safety.safety_rx_hook(self._tja_msg(True, ctr=i % 16)))
      lat = bool(self.safety.get_controls_allowed_lateral())
      kind = rng.randrange(5)
      values = {"TJA_BUTTON": 0, "CTR": i % 16}
      if kind == 0:
        values["MRCC_BUTTON"] = 1
      elif kind == 1:
        values["SET_P"] = 1
      elif kind == 2:
        values["SET_M"] = 1
      elif kind == 3:
        values["RES"] = 1
      else:
        values["CAN_OFF"] = 1
      self.assertTrue(self.safety.safety_rx_hook(
        self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)))
      if bool(self.safety.get_controls_allowed_lateral()) != lat:
        changed += 1
    self.assertEqual(0, changed)


if __name__ == "__main__":
  unittest.main()
