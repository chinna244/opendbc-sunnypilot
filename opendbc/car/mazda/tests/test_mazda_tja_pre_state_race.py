"""Pre-TJA MRCC snapshot races against real CarState.update() ordering.

Previous-cycle cruiseState must never be treated as confident OFF when a
driver MRCC/SET/RES/CANCEL has not yet been confirmed by CRZ_CTRL/PEDALS.
UNKNOWN forbids MRCC_BUTTON compensation.
"""

import random

import unittest
from opendbc.car.mazda.tests.unittest_compat import parametrize

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

from opendbc.car.mazda.tests.test_mazda_tja_off_long import _assert_clean_can_off, _cc, _decode_crz

FUZZ_SEED_A = 20260816
FUZZ_SEED_B = 20260813
FUZZ_TRANSITIONS = 100000

OFF, ARMED, ACTIVE = "OFF", "ARMED", "ACTIVE"


def _ci(alpha_long=False):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [],
                               alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long, False, False)
  return CarInterface(CP, CP_SP), CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


def _cruise_msgs(p, alpha_long, available, enabled):
  if alpha_long:
    return [p.make_can_msg("PEDALS", 0, {
      "ACC_OFF": int(available and not enabled),
      "ACC_ACTIVE": int(enabled),
      "BRAKE_ON": 0,
    })]
  return [p.make_can_msg("CRZ_CTRL", 0, {
    "CRZ_AVAILABLE": int(available),
    "CRZ_ACTIVE": int(enabled),
  })]


def _state_bits(state):
  if state == ACTIVE:
    return True, True
  if state == ARMED:
    return True, False
  return False, False


def _label(available, enabled):
  if enabled:
    return ACTIVE
  if available:
    return ARMED
  return OFF


def _confident_off(CS):
  return (not CS.tja_pre_cruise_available and not CS.tja_pre_cruise_enabled
          and not CS.tja_pre_cruise_unknown)


def _snapshot_label(CS):
  if CS.tja_pre_cruise_unknown:
    return "UNKNOWN"
  return _label(CS.tja_pre_cruise_available, CS.tja_pre_cruise_enabled)


def _update(CI, p, alpha_long, buttons, cruise_state, extra_crz=None):
  avail, en = _state_bits(cruise_state)
  btns = dict(buttons)
  if "CTR" not in btns:
    btns["CTR"] = int(getattr(CI, "_wheel_ctr", 0)) & 0xF
    CI._wheel_ctr = (int(getattr(CI, "_wheel_ctr", 0)) + 1) & 0xF
  msgs = [p.make_can_msg("CRZ_BTNS", 0, btns)]
  if extra_crz:
    msgs.extend(extra_crz)
  msgs.extend(_cruise_msgs(p, alpha_long, avail, en))
  ret, _ = CI.update([(0, msgs)])
  return ret


def _arm_tja_sm(CI, p, alpha_long, cruise_state=OFF):
  _update(CI, p, alpha_long, {}, cruise_state)


def _step_ctrl(ctrl, CS, lat_active, lat_prev=None):
  if lat_prev is not None:
    ctrl._lat_active_prev = lat_prev
  CC_SP = structs.CarControlSP()
  _, sends = ctrl.update(_cc(lat_active), CC_SP, CS, 0)
  return _decode_crz(sends)


@parametrize("alpha_long", [False, True])
class TestAdjacentCycleButtonThenTja(unittest.TestCase):
  def test_a_off_to_armed_immediate_tja_not_off(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, {"MRCC_BUTTON": 1}, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)
    assert CI.CS.tja_toggles_this_update == 1
    assert _snapshot_label(CI.CS) != OFF
    assert not _confident_off(CI.CS)
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ARMED)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []

  def test_b_off_to_active_immediate_tja_not_off(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, {"SET_P": 1}, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ACTIVE)
    assert CI.CS.tja_toggles_this_update == 1
    assert _snapshot_label(CI.CS) != OFF
    assert not _confident_off(CI.CS)
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ACTIVE)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []

  def test_c_armed_to_active_immediate_tja(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, ARMED)
    _update(CI, p, alpha_long, {}, ARMED)
    _update(CI, p, alpha_long, {"SET_P": 1}, ARMED)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ACTIVE)
    assert CI.CS.tja_toggles_this_update == 1
    assert _snapshot_label(CI.CS) in (ARMED, ACTIVE, "UNKNOWN")
    assert _snapshot_label(CI.CS) != OFF
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ACTIVE)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []

  def test_d_active_to_armed_via_cancel_immediate_tja(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, ACTIVE)
    _update(CI, p, alpha_long, {}, ACTIVE)
    _update(CI, p, alpha_long, {"CAN_OFF": 1}, ACTIVE)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)
    assert CI.CS.tja_toggles_this_update == 1
    assert _snapshot_label(CI.CS) != OFF
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ARMED)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []

  def test_e_armed_to_off_immediate_tja_may_snapshot_off(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, ARMED)
    _update(CI, p, alpha_long, {}, ARMED)
    _update(CI, p, alpha_long, {"MRCC_BUTTON": 1}, ARMED)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)
    assert CI.CS.tja_toggles_this_update == 1
    # Unconfirmed MRCC: must not guess OFF even if this cycle's CRZ_CTRL is OFF.
    assert not _confident_off(CI.CS)
    assert _snapshot_label(CI.CS) in (ARMED, "UNKNOWN")
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ARMED)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []

  def test_e_confirmed_off_then_tja_may_compensate(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, ARMED)
    _update(CI, p, alpha_long, {}, ARMED)
    _update(CI, p, alpha_long, {"MRCC_BUTTON": 1}, ARMED)
    _update(CI, p, alpha_long, {}, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)
    assert CI.CS.tja_toggles_this_update == 1
    assert _snapshot_label(CI.CS) == OFF
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    assert crz == []
    _update(CI, p, alpha_long, {}, ARMED)
    crz = _step_ctrl(ctrl, CI.CS, False)
    assert crz == []
    _update(CI, p, alpha_long, {}, ARMED)
    crz = _step_ctrl(ctrl, CI.CS, False)
    _assert_clean_can_off(crz)


@parametrize("alpha_long", [False, True])
class TestSameParserCycleOrdering(unittest.TestCase):
  def test_state_before_tja_reaction_is_prev_cycle(self, alpha_long):
    CI, _ = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, ARMED)
    _update(CI, p, alpha_long, {}, ARMED)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)
    assert _snapshot_label(CI.CS) == ARMED
    assert not CI.CS.tja_pre_cruise_unknown

  def test_state_after_tja_oem_arm_still_prev_off(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)
    assert _snapshot_label(CI.CS) == OFF
    assert not CI.CS.tja_pre_cruise_unknown
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    assert crz == []  # TJA still held
    _update(CI, p, alpha_long, {}, ARMED)
    crz = _step_ctrl(ctrl, CI.CS, False)
    assert crz == []
    _update(CI, p, alpha_long, {}, ARMED)
    crz = _step_ctrl(ctrl, CI.CS, False)
    _assert_clean_can_off(crz)

  def test_tja_on_arm_delayed_then_tja_off_restores_off(self, alpha_long):
    """ROW 3: TJA off while still OFF must not leave OEM ARMED from the first TJA.

    Route 00000019 event 53: marking TJA itself unconfirmed made the next
    edge UNKNOWN and skipped ARMED restore. Do not do that. Prev-cycle OFF
    is a confident OFF snapshot; restore the OEM ARM side-effect.
    """
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, OFF)  # TJA ON, OEM arm not yet visible
    _step_ctrl(ctrl, CI.CS, True, lat_prev=False)
    _update(CI, p, alpha_long, {}, OFF)
    _step_ctrl(ctrl, CI.CS, True)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)  # TJA OFF, CRZ_CTRL now ARMED
    assert CI.CS.tja_toggles_this_update == 1
    assert _confident_off(CI.CS)
    assert _snapshot_label(CI.CS) == OFF
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    assert crz == []  # TJA still held
    _update(CI, p, alpha_long, {}, ARMED)
    crz = _step_ctrl(ctrl, CI.CS, False)
    assert crz == []
    _update(CI, p, alpha_long, {}, ARMED)
    crz = _step_ctrl(ctrl, CI.CS, False)
    _assert_clean_can_off(crz)

  def test_delayed_cancel_then_mrcc_then_tja_not_confident_off(self, alpha_long):
    """CANCEL's delayed OFF must not clear a newer unconfirmed MRCC before TJA."""
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, ARMED)
    _update(CI, p, alpha_long, {}, ARMED)
    _update(CI, p, alpha_long, {"CAN_OFF": 1}, ARMED)
    _update(CI, p, alpha_long, {"MRCC_BUTTON": 1}, OFF)
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, ARMED)
    assert CI.CS.tja_toggles_this_update == 1
    assert not _confident_off(CI.CS)
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ARMED)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []

  def test_tja_then_mrcc_same_vl_all_is_unknown(self, alpha_long):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    tja = p.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1})
    mrcc = p.make_can_msg("CRZ_BTNS", 0, {"MRCC_BUTTON": 1})
    msgs = [tja, mrcc] + _cruise_msgs(p, alpha_long, True, False)
    CI.update([(0, msgs)])
    assert CI.CS.tja_toggles_this_update == 1
    assert CI.CS.tja_pre_cruise_unknown
    assert not _confident_off(CI.CS)
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ARMED)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []


@parametrize("alpha_long", [False, True])
@parametrize("delay", [0, 1, 2, 3])
class TestDelayedCruiseState(unittest.TestCase):
  def _delayed_then_tja(self, alpha_long, delay, start, end, button):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, start)
    _update(CI, p, alpha_long, {}, start)
    _update(CI, p, alpha_long, button, start)
    for _ in range(delay):
      _update(CI, p, alpha_long, {}, start)
    actual = end
    _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, end)
    snap = _snapshot_label(CI.CS)
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, end)
    crz += _step_ctrl(ctrl, CI.CS, False)
    compensated = any(f["MRCC"] == 1 or f["CAN_OFF"] == 1 for f in crz)
    if actual in (ARMED, ACTIVE):
      assert not compensated, (actual, snap, delay, alpha_long)
      assert snap != OFF or CI.CS.tja_pre_cruise_unknown
    return actual, snap, compensated

  def test_off_to_armed(self, alpha_long, delay):
    actual, snap, comp = self._delayed_then_tja(
      alpha_long, delay, OFF, ARMED, {"MRCC_BUTTON": 1})
    assert actual == ARMED
    assert not comp
    assert snap in (ARMED, "UNKNOWN")

  def test_armed_to_active(self, alpha_long, delay):
    actual, snap, comp = self._delayed_then_tja(
      alpha_long, delay, ARMED, ACTIVE, {"SET_P": 1})
    assert actual == ACTIVE
    assert not comp
    assert snap != OFF

  def test_active_to_armed(self, alpha_long, delay):
    actual, snap, comp = self._delayed_then_tja(
      alpha_long, delay, ACTIVE, ARMED, {"CAN_OFF": 1})
    assert actual == ARMED
    assert not comp
    assert snap != OFF

  def test_armed_to_off(self, alpha_long, delay):
    actual, snap, comp = self._delayed_then_tja(
      alpha_long, delay, ARMED, OFF, {"MRCC_BUTTON": 1})
    assert actual == OFF
    assert not comp
    # Prev-cycle still ARMED, and MRCC is unconfirmed: never confident OFF.
    assert snap in (ARMED, "UNKNOWN")


@parametrize("alpha_long", [False, True])
class TestSameFrameLongPlusTja(unittest.TestCase):
  @parametrize("btn,key", [
    ({"MRCC_BUTTON": 1, "TJA_BUTTON": 1}, "MRCC"),
    ({"SET_P": 1, "TJA_BUTTON": 1}, "SET"),
    ({"RES": 1, "TJA_BUTTON": 1}, "RES"),
    ({"CAN_OFF": 1, "TJA_BUTTON": 1}, "CANCEL"),
  ])
  def test_same_frame_does_not_false_compensate(self, alpha_long, btn, key):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, OFF)
    _update(CI, p, alpha_long, btn, ARMED)
    assert CI.CS.tja_toggles_this_update == 1
    assert not _confident_off(CI.CS)
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, ARMED)
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == [], key


@parametrize("alpha_long", [False, True])
class TestRapidDriverSequences(unittest.TestCase):
  @parametrize("gap_ms", [0, 50, 100, 150, 200, 300])
  @parametrize("btn,start,mid", [
    ({"MRCC_BUTTON": 1}, OFF, ARMED),
    ({"SET_P": 1}, ARMED, ACTIVE),
    ({"CAN_OFF": 1}, ACTIVE, ARMED),
  ])
  def test_press_release_tja_gap(self, alpha_long, gap_ms, btn, start, mid):
    CI, ctrl = _ci(alpha_long)
    p = CANPacker("mazda_2017")
    _arm_tja_sm(CI, p, alpha_long, start)
    _update(CI, p, alpha_long, {}, start)
    _update(CI, p, alpha_long, btn, start)
    idle_cycles = 0 if gap_ms == 0 else max(1, gap_ms // 10)
    # 0 ms = same batch as TJA; otherwise CRZ_BTNS is 10 Hz (~100 ms/frame)
    if gap_ms == 0:
      tja = p.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1})
      press = p.make_can_msg("CRZ_BTNS", 0, btn)
      msgs = [press, tja] + _cruise_msgs(p, alpha_long, *_state_bits(mid))
      CI.update([(0, msgs)])
    else:
      _update(CI, p, alpha_long, {}, mid if idle_cycles >= 2 else start)
      for i in range(idle_cycles - 1):
        st = mid if i >= 0 else start
        _update(CI, p, alpha_long, {}, st)
      _update(CI, p, alpha_long, {"TJA_BUTTON": 1}, mid)
    assert CI.CS.tja_toggles_this_update >= 1
    if mid in (ARMED, ACTIVE):
      assert not _confident_off(CI.CS)
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    _update(CI, p, alpha_long, {}, mid)
    crz += _step_ctrl(ctrl, CI.CS, False)
    if mid in (ARMED, ACTIVE):
      assert crz == []


def _fuzz_one(seed, n, alpha_long):
  rng = random.Random(seed)
  CI, ctrl = _ci(alpha_long)
  p = CANPacker("mazda_2017")
  oem = OFF
  parsed = OFF
  delay_left = None
  delay_state = None
  tja_held = False
  prev_btns = {k: 0 for k in ("MRCC_BUTTON", "SET_P", "RES", "CAN_OFF", "SET_M")}
  mads = False
  episode_true_pre = None
  false_off = 0
  false_can_off = 0
  unknown_comp = 0
  _arm_tja_sm(CI, p, alpha_long, OFF)

  def apply_btn_to_oem(name):
    nonlocal oem
    if name == "MRCC_BUTTON":
      oem = OFF if oem != OFF else ARMED
    elif name in ("SET_P", "RES"):
      if oem != OFF:
        oem = ACTIVE
    elif name == "CAN_OFF":
      if oem == ACTIVE:
        oem = ARMED
      elif oem == ARMED:
        oem = OFF
    elif name == "SET_M":
      if oem == ACTIVE:
        oem = ARMED

  def enqueue_parsed(st, delay):
    nonlocal delay_left, delay_state, parsed
    if delay <= 0:
      parsed = st
      delay_left = None
    else:
      delay_left = delay
      delay_state = st

  for _ in range(n):
    if delay_left is not None:
      delay_left -= 1
      if delay_left <= 0:
        parsed = delay_state
        delay_left = None

    kind = rng.randrange(14)
    buttons = {}
    extra = None
    tja_edge = False
    true_pre = oem

    if kind <= 3:
      name = ("MRCC_BUTTON", "SET_P", "RES", "CAN_OFF")[kind]
      buttons = {name: 1}
      if not prev_btns[name]:
        apply_btn_to_oem(name)
        enqueue_parsed(oem, rng.choice([0, 1, 2, 3]))
    elif kind == 4 and not tja_held:
      buttons = {"TJA_BUTTON": 1}
      tja_edge = True
      tja_held = True
      true_pre = oem
      if oem == OFF:
        oem = ARMED
        enqueue_parsed(oem, rng.choice([0, 1]))
    elif kind == 5 and tja_held:
      tja_held = False
    elif kind == 6:
      name = rng.choice(["MRCC_BUTTON", "SET_P", "RES", "CAN_OFF", "SET_M"])
      buttons = {name: 1}
      if not prev_btns[name]:
        apply_btn_to_oem(name)
        enqueue_parsed(oem, rng.choice([0, 1, 2]))
      if not tja_held:
        buttons["TJA_BUTTON"] = 1
        tja_edge = True
        tja_held = True
        true_pre = oem
        if oem == OFF:
          oem = ARMED
          enqueue_parsed(oem, 0)
    elif kind == 7:
      extra = p.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": int(tja_held)})
    elif kind == 8:
      extra = p.make_can_msg("CRZ_BTNS", 0, buttons)
    # 9-13: idle / missing cruise frame

    if tja_held:
      buttons.setdefault("TJA_BUTTON", 1)

    avail, en = _state_bits(parsed)
    msgs = [p.make_can_msg("CRZ_BTNS", 0, buttons)]
    if extra is not None:
      msgs.append(extra)
    send_cruise = kind != 9
    if send_cruise:
      msgs.extend(_cruise_msgs(p, alpha_long, avail, en))
    CI.update([(0, msgs)])
    prev_btns = {k: int(buttons.get(k, 0)) for k in prev_btns}

    crz = []
    if tja_edge and CI.CS.tja_toggles_this_update:
      if true_pre in (ARMED, ACTIVE) and _confident_off(CI.CS):
        false_off += 1
      mads = not mads
      episode_true_pre = true_pre
      crz = _step_ctrl(ctrl, CI.CS, mads)
      if CI.CS.tja_pre_cruise_unknown and (crz or ctrl._tja_restore_mrcc_off_pending):
        unknown_comp += 1
    elif CI.CS.tja_toggles_this_update:
      if _confident_off(CI.CS) is False and CI.CS.tja_pre_cruise_unknown:
        crz = _step_ctrl(ctrl, CI.CS, mads)
        if crz or ctrl._tja_restore_mrcc_off_pending:
          unknown_comp += 1
      else:
        if _confident_off(CI.CS) and true_pre in (ARMED, ACTIVE):
          false_off += 1
        mads = not mads
        episode_true_pre = parsed
        crz = _step_ctrl(ctrl, CI.CS, mads)
    else:
      crz = _step_ctrl(ctrl, CI.CS, mads)

    if any(f["MRCC"] == 1 or f["CAN_OFF"] == 1 for f in crz):
      if episode_true_pre in (ARMED, ACTIVE):
        false_can_off += 1

  return false_off, false_can_off, unknown_comp, n


class TestOrderingFuzz(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  @parametrize("seed", [FUZZ_SEED_A, FUZZ_SEED_B])
  def test_fuzz_no_false_off_compensation(self, alpha_long, seed):
    n = FUZZ_TRANSITIONS
    false_off, false_can_off, unknown_comp, nrun = _fuzz_one(seed, n, alpha_long)
    assert nrun == n
    assert false_off == 0
    assert false_can_off == 0
    assert unknown_comp == 0


class TestMazdaTjaPreStateRace(unittest.TestCase):
  def test_mrcc_then_delayed_crz_ctrl_then_tja_must_not_snapshot_off(self):
    """Cycle N: MRCC press, CRZ_CTRL still OFF. Cycle N+1: CRZ_CTRL ARMED + TJA."""
    CI, ctrl = _ci()
    p = CANPacker("mazda_2017")
    CI.update([(0, [p.make_can_msg("CRZ_BTNS", 0, {}),
                    p.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 0, "CRZ_ACTIVE": 0})])])
    CI.update([(0, [p.make_can_msg("CRZ_BTNS", 0, {"MRCC_BUTTON": 1}),
                    p.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 0, "CRZ_ACTIVE": 0})])])
    CI.update([(0, [p.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": 1}),
                    p.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 0})])])
    assert CI.CS.tja_toggles_this_update == 1
    assert not _confident_off(CI.CS)
    assert CI.CS.tja_pre_cruise_unknown
    crz = _step_ctrl(ctrl, CI.CS, False, lat_prev=True)
    CI.update([(0, [p.make_can_msg("CRZ_BTNS", 0, {}),
                    p.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": 1, "CRZ_ACTIVE": 0})])])
    crz += _step_ctrl(ctrl, CI.CS, False)
    assert crz == []
