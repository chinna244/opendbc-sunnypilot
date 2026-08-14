"""Single-TJA edge SM, carstate vl_all consumption, and whole-drive stress."""

import random

import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.tja_edge import ARMED, HELD, UNINITIALIZED, WAIT_FOR_RELEASE, MazdaTjaEdge
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type

STRESS_SEED = 20260813
STRESS_TRANSITIONS = 100000
PARITY_SEED = 20260814


def _interface(car=CAR.MAZDA_CX5_2022, alpha_long=False):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(car, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, car, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def _send_crz(CI, packer, values, extra_msgs=None):
  msgs = [packer.make_can_msg("CRZ_BTNS", 0, values)]
  if extra_msgs:
    msgs.extend(extra_msgs)
  ret, _ = CI.update([(0, msgs)])
  return ret


class TestMazdaTjaEdgeUnit:
  def test_boot_held_does_not_toggle(self):
    edge = MazdaTjaEdge()
    assert edge.state == UNINITIALIZED
    assert edge.update(True) is False
    assert edge.state == WAIT_FOR_RELEASE
    assert edge.update(True) is False
    assert edge.update(False) is False
    assert edge.state == ARMED
    assert edge.update(True) is True
    assert edge.state == HELD

  def test_fresh_rise_toggles_once_hold_does_not(self):
    edge = MazdaTjaEdge()
    assert edge.update(False) is False
    assert edge.update(True) is True
    for _ in range(50):
      assert edge.update(True) is False
    assert edge.update(False) is False
    assert edge.update(True) is True

  def test_release_does_not_toggle(self):
    edge = MazdaTjaEdge()
    edge.update(False)
    edge.update(True)
    assert edge.update(False) is False

  def test_duplicate_ones_do_not_extra_toggle(self):
    edge = MazdaTjaEdge()
    edge.update(False)
    assert edge.update(True) is True
    assert edge.update(True) is False
    assert edge.update(True) is False

  def test_missing_release_does_not_extra_toggle(self):
    edge = MazdaTjaEdge()
    edge.update(False)
    edge.update(True)
    # lost the 0 sample; next 1 must still not toggle
    assert edge.update(True) is False

  def test_other_buttons_do_not_exist_in_sm(self):
    # The SM takes only TJA; callers must not pass brake/MRCC as conflicts.
    edge = MazdaTjaEdge()
    edge.update(False)
    assert edge.update(True) is True


class TestMazdaTjaCarstate:
  def test_cx5_2022_has_tja_flag(self):
    CI = _interface()
    assert CI.CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.TJA
    assert CI.CS._tja_edge is not None

  def test_non_tja_platform_has_no_edge_sm(self):
    CI = _interface(car=CAR.MAZDA_CX5)
    assert not (CI.CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.TJA)
    assert CI.CS._tja_edge is None

  def test_single_tja_emits_one_lkas_press(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    events = _send_crz(CI, packer, {"TJA_BUTTON": 1}).buttonEvents
    assert [(e.type, e.pressed) for e in events if e.type == ButtonType.lkas] == [(ButtonType.lkas, True)]
    held = _send_crz(CI, packer, {"TJA_BUTTON": 1}).buttonEvents
    assert all(e.type != ButtonType.lkas for e in held)
    released = _send_crz(CI, packer, {}).buttonEvents
    assert [(e.type, e.pressed) for e in released if e.type == ButtonType.lkas] == [(ButtonType.lkas, False)]

  def test_boot_held_tja_does_not_emit_lkas(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    events = _send_crz(CI, packer, {"TJA_BUTTON": 1}).buttonEvents
    assert all(e.type != ButtonType.lkas for e in events)
    _send_crz(CI, packer, {})
    events = _send_crz(CI, packer, {"TJA_BUTTON": 1}).buttonEvents
    assert any(e.type == ButtonType.lkas and e.pressed for e in events)

  def test_vl_all_batch_toggles_once_for_rise_then_hold(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    idle = packer.make_can_msg("CRZ_BTNS", 0, {})
    held = packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1})
    ret, _ = CI.update([(0, [idle, held, held, held])])
    lkas = [e for e in ret.buttonEvents if e.type == ButtonType.lkas]
    assert [(e.type, e.pressed) for e in lkas] == [(ButtonType.lkas, True)]

  def test_vl_all_even_toggles_emit_two_lkas_presses(self):
    # press-release-press is two physical rising edges. Emit both so MADS can
    # apply two sequential toggles (net latch unchanged).
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    press = packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1})
    release = packer.make_can_msg("CRZ_BTNS", 0, {})
    ret, _ = CI.update([(0, [press, release, press])])
    lkas = [e for e in ret.buttonEvents if e.type == ButtonType.lkas and e.pressed]
    assert len(lkas) == 2
    assert CI.CS.tja_toggles_this_update == 2

  def test_vl_all_press_release_emits_one_press(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    press = packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1})
    release = packer.make_can_msg("CRZ_BTNS", 0, {})
    ret, _ = CI.update([(0, [press, release])])
    lkas = [e for e in ret.buttonEvents if e.type == ButtonType.lkas]
    assert [(e.type, e.pressed) for e in lkas] == [(ButtonType.lkas, True)]

  def test_mrcc_set_res_cancel_do_not_emit_lkas(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    for values in ({"MRCC_BUTTON": 1}, {"SET_P": 1}, {"SET_M": 1}, {"RES": 1}, {"CAN_OFF": 1}):
      events = _send_crz(CI, packer, values).buttonEvents
      assert all(e.type != ButtonType.lkas for e in events)

  def test_tja_while_brake_and_mrcc_still_toggles(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    events = _send_crz(CI, packer, {"TJA_BUTTON": 1, "MRCC_BUTTON": 1, "CAN_OFF": 1, "SET_P": 1}).buttonEvents
    assert any(e.type == ButtonType.lkas and e.pressed for e in events)

  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_carstate_runs_with_tja_parsers(self, alpha_long):
    CI = _interface(alpha_long=alpha_long)
    packer = CANPacker("mazda_2017")
    for _ in range(5):
      _send_crz(CI, packer, {})


class TestMazdaTjaStress:
  def test_randomized_transitions_no_false_positive(self):
    rng = random.Random(STRESS_SEED)
    edge = MazdaTjaEdge()
    prev = None
    toggles = 0
    armed_rises = 0
    for i in range(STRESS_TRANSITIONS):
      tja = bool(rng.randrange(2)) if rng.random() > 0.15 else (prev if prev is not None else False)
      # Occasional duplicates / drops are just the same or skipped sample.
      if rng.random() < 0.05:
        continue
      before = edge.state
      toggle = edge.update(tja)
      if toggle:
        toggles += 1
        armed_rises += 1
        assert before == ARMED
        assert tja is True
        assert edge.state == HELD
      else:
        # Held ones must never toggle.
        if before == HELD and tja:
          assert toggle is False
      prev = tja
    assert toggles == armed_rises
    assert toggles > 100


MULTI_EDGE_CASES = [
  # samples, expected_rising_after_init_arm, notes
  ([0, 1], 1),
  ([0, 1, 1, 1], 1),
  ([0, 1, 0], 1),
  ([0, 1, 0, 1], 2),
  ([0, 1, 1, 0, 1], 2),
  ([0, 1, 0, 1, 0, 1], 3),
]


class TestMazdaTjaMultiEdgeExactCount:
  def test_physical_edges_match_userspace_toggle_count(self):
    packer = CANPacker("mazda_2017")
    for samples, expected in MULTI_EDGE_CASES:
      CI = _interface()
      msgs = [packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": int(s)}) for s in samples]
      ret, _ = CI.update([(0, msgs)])
      presses = sum(1 for e in ret.buttonEvents if e.type == ButtonType.lkas and e.pressed)
      assert CI.CS.tja_toggles_this_update == expected, samples
      assert presses == expected, samples

  def test_boot_held_initial_ones_are_zero_then_one_real_rise(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    msgs = [packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": s}) for s in (1, 1, 1, 0, 1)]
    ret, _ = CI.update([(0, msgs)])
    presses = sum(1 for e in ret.buttonEvents if e.type == ButtonType.lkas and e.pressed)
    assert CI.CS.tja_toggles_this_update == 1
    assert presses == 1

  def test_bus2_copy_does_not_create_a_second_edge(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    physical = packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1})
    echo = packer.make_can_msg("CRZ_BTNS", 2, {"TJA_BUTTON": 1})
    ret, _ = CI.update([(0, [physical, echo])])
    presses = sum(1 for e in ret.buttonEvents if e.type == ButtonType.lkas and e.pressed)
    assert presses == 1
    assert CI.CS.tja_toggles_this_update == 1

  def test_tja_and_mrcc_same_frame_still_one_tja_edge(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    ret = _send_crz(CI, packer, {"TJA_BUTTON": 1, "MRCC_BUTTON": 1, "SET_P": 1, "RES": 1, "CAN_OFF": 1})
    presses = sum(1 for e in ret.buttonEvents if e.type == ButtonType.lkas and e.pressed)
    assert presses == 1
    mains = [e for e in ret.buttonEvents if e.type == ButtonType.mainCruise]
    assert any(e.pressed for e in mains)


class TestTjaPreCruiseCapture:
  """Pre-TJA MRCC state is the previous CarState cycle, not this cycle's OEM reaction."""

  def test_same_batch_oem_arm_does_not_look_like_pre_armed(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {}),
      packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 0, "CRZ_ACTIVE": 0}),
    ])])
    ret, _ = CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1}),
      packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 0}),
    ])])
    assert CI.CS.tja_toggles_this_update == 1
    assert CI.CS.tja_pre_cruise_available is False
    assert CI.CS.tja_pre_cruise_enabled is False
    assert CI.CS.tja_pre_cruise_unknown is False
    assert ret.cruiseState.available is True
    assert ret.cruiseState.enabled is False

  def test_pre_armed_when_previous_cycle_was_armed(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {}),
      packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 0}),
    ])])
    CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1}),
      packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 0}),
    ])])
    assert CI.CS.tja_toggles_this_update == 1
    assert CI.CS.tja_pre_cruise_available is True
    assert CI.CS.tja_pre_cruise_enabled is False

  def test_pre_active_when_previous_cycle_was_active(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {}),
      packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 1}),
    ])])
    CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1}),
      packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 1}),
    ])])
    assert CI.CS.tja_toggles_this_update == 1
    assert CI.CS.tja_pre_cruise_available is True
    assert CI.CS.tja_pre_cruise_enabled is True

  def test_alpha_long_pre_state_from_pedals(self):
    CI = _interface(alpha_long=True)
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {}),
      packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 0, "ACC_ACTIVE": 0, "BRAKE_ON": 0}),
    ])])
    CI.update([(0, [
      packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1}),
      packer.make_can_msg("PEDALS", 0, {"ACC_OFF": 1, "ACC_ACTIVE": 0, "BRAKE_ON": 0}),
    ])])
    assert CI.CS.tja_toggles_this_update == 1
    assert CI.CS.tja_pre_cruise_available is False
    assert CI.CS.tja_pre_cruise_enabled is False


class TestMazdaTjaDbcBits:
  def test_tja_and_mrcc_bits_match_panda_get_bit_numbering(self):
    packer = CANPacker("mazda_2017")
    tja = packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1})[1]
    mrcc = packer.make_can_msg("CRZ_BTNS", 0, {"MRCC_BUTTON": 1})[1]
    idle = packer.make_can_msg("CRZ_BTNS", 0, {})[1]
    assert (tja[11 // 8] >> (11 % 8)) & 1
    assert not ((idle[11 // 8] >> (11 % 8)) & 1)
    assert (mrcc[15 // 8] >> (15 % 8)) & 1
    assert not ((idle[15 // 8] >> (15 % 8)) & 1)
    # no overlap
    assert not ((tja[15 // 8] >> (15 % 8)) & 1)
    assert not ((mrcc[11 // 8] >> (11 % 8)) & 1)

  def test_set_res_cancel_bits_unchanged(self):
    packer = CANPacker("mazda_2017")
    cancel = packer.make_can_msg("CRZ_BTNS", 0, {"CAN_OFF": 1})[1]
    res = packer.make_can_msg("CRZ_BTNS", 0, {"RES": 1})[1]
    set_p = packer.make_can_msg("CRZ_BTNS", 0, {"SET_P": 1})[1]
    assert (cancel[0] >> 0) & 1
    assert (res[0] >> 2) & 1  # DBC RES start 2
    assert (set_p[0] >> 4) & 1

  def test_panda_reconnect_resets_boot_held(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    _send_crz(CI, packer, {"TJA_BUTTON": 1})
    assert CI.CS.tja_toggles_this_update == 1
    CI.CS.on_panda_alive(False)
    CI.CS.on_panda_alive(True)
    # held TJA after reconnect is not a press
    ret = _send_crz(CI, packer, {"TJA_BUTTON": 1})
    assert CI.CS.tja_toggles_this_update == 0
    assert all(not (e.type == ButtonType.lkas and e.pressed) for e in ret.buttonEvents)
    _send_crz(CI, packer, {})
    ret = _send_crz(CI, packer, {"TJA_BUTTON": 1})
    assert CI.CS.tja_toggles_this_update == 1
