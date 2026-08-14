import unittest
from opendbc.car.mazda.tests.unittest_compat import parametrize

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

from opendbc.car.mazda.tests.test_mazda_tja_pre_state_race import (
  ARMED, OFF, _arm_tja_sm, _ci, _confident_off, _snapshot_label, _update,
)


class TestMazdaCarstate_test_carstate_runs_with_real_parsers(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_carstate_runs_with_real_parsers(self, alpha_long):
    # vl_all, unlike vl, has no lazy message registration: every message read through it
    # must be listed in get_can_parsers. The op-long FSC settle gate crashed card on its
    # first update when CAM_LANEINFO was missing from the cam parser (KeyError, 2026-07-29).
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
    assert CP.openpilotLongitudinalControl == alpha_long

    CI = CarInterface(CP, CP_SP)
    for _ in range(10):
      CI.update([])


class TestMazdaCarstate_test_startup_held_tja_is_not_a_rising_edge(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_startup_held_tja_is_not_a_rising_edge(self, alpha_long):
    CI, _ = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    assert CI.CS.tja_toggles_this_update == 0
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    assert CI.CS.tja_toggles_this_update == 0
    _update(CI, p, alpha_long, {}, OFF)
    assert CI.CS.tja_toggles_this_update == 0
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    assert CI.CS.tja_toggles_this_update == 1


class TestMazdaCarstate_test_per_edge_snapshot_classification(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_per_edge_snapshot_classification(self, alpha_long):
    CI, _ = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    assert _snapshot_label(CI.CS) == OFF
    assert _confident_off(CI.CS)
    _update(CI, p, alpha_long, {}, ARMED)
    _update(CI, p, alpha_long, {}, ARMED)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)
    # Consecutive TJA must keep a confident cruise snapshot. Route 00000019
    # event 53 failed because TJA itself was marked unconfirmed.
    assert _snapshot_label(CI.CS) == ARMED
    assert not CI.CS.tja_pre_cruise_unknown


class TestMazdaCarstate_test_rapid_second_tja_keeps_confident_snapshot(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_rapid_second_tja_keeps_confident_snapshot(self, alpha_long):
    CI, _ = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    _update(CI, p, alpha_long, {}, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    assert CI.CS.tja_toggles_this_update == 1
    assert not CI.CS.tja_pre_cruise_unknown
    assert _confident_off(CI.CS)


class TestMazdaCarstate_test_reset_tja_fail_closed(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_reset_tja_fail_closed(self, alpha_long):
    CI, _ = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    CI.CS.reset_tja()
    assert CI.CS.tja_toggles_this_update == 0
    assert CI.CS.tja_pre_cruise_unknown
    assert CI.CS.tja_edge_reset
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    assert CI.CS.tja_toggles_this_update == 0  # boot-held after reset
