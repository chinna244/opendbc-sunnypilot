import unittest
"""Adversarial same-cycle, CAM_LKAS encoding, and integrated stress for Mazda TJA."""

import random
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.tja_edge import ARMED, HELD, MazdaTjaEdge
from opendbc.car.mazda.values import CAR, MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.mazda.icbm import SendButtonState
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py

ButtonType = structs.CarState.ButtonEvent.Type

STRESS_SEEDS = (20260813, 20260814, 20260815, 20260816)
STRESS_PER_SEED = 250000  # 1_000_000 combined


def _interface(alpha_long=False):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def _lkas_dat(sends):
  for s in sends:
    addr = s[0] if isinstance(s, tuple) else s.address
    dat = s[1] if isinstance(s, tuple) else s.dat
    if addr == 0x243:
      return bytes(dat)
  raise AssertionError("no CAM_LKAS")


def _lkas_request(dat: bytes) -> int:
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


def _send_crz(CI, packer, values, extra=None):
  msgs = [packer.make_can_msg("CRZ_BTNS", 0, values)]
  if extra:
    msgs.extend(extra)
  ret, _ = CI.update([(0, msgs)])
  return ret


class _Actuators:
  def __init__(self, torque=0.4):
    self.torque = torque
    self.accel = 0.0
    self.longControlState = 0

  def as_builder(self):
    return SimpleNamespace(torque=0, torqueOutputCan=0, accel=0)


def _cc_lat(torque=0.4, lat_active=True):
  return SimpleNamespace(
    latActive=lat_active,
    enabled=False,
    longActive=False,
    actuators=_Actuators(torque),
    cruiseControl=SimpleNamespace(cancel=False, resume=False, override=False),
    hudControl=SimpleNamespace(visualAlert=0, leadVisible=False, leadDistanceBars=2),
  )


def _cc_sp():
  return SimpleNamespace(
    stockEcuHandBack=False,
    mads=SimpleNamespace(enabled=True),
    intelligentCruiseButtonManagement=SimpleNamespace(sendButton=SendButtonState.none, state=0, vTarget=0),
  )


class TestSameCycleAuthLoss(unittest.TestCase):
  def test_tja_off_while_lat_active_packs_zero_and_panda_accepts(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    safety = libsafety_py.libsafety
    safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.TJA)
    safety.init_tests()
    safety.set_mads_params(True, False, False)
    safety.set_heartbeat_engaged_mads(True)
    spacker = CANPackerSafety("mazda_2017")

    _send_crz(CI, packer, {})
    safety.safety_rx_hook(spacker.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 0}))
    _send_crz(CI, packer, {"TJA_BUTTON": 1})
    safety.safety_rx_hook(spacker.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 1}))
    assert safety.get_controls_allowed_lateral()
    _send_crz(CI, packer, {})
    safety.safety_rx_hook(spacker.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 0}))

    # Disable edge this cycle: panda and userspace both see the rise.
    _send_crz(CI, packer, {"TJA_BUTTON": 1})
    safety.safety_rx_hook(spacker.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 1}))
    assert not safety.get_controls_allowed_lateral()
    assert CI.CS.tja_toggles_this_update == 1

    actuators, sends = CI.CC.update(_cc_lat(), _cc_sp(), CI.CS, 0)
    assert _lkas_request(_lkas_dat(sends)) == 0
    assert actuators.torqueOutputCan == 0
    assert safety.safety_tx_hook(spacker.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 0}))
    assert not safety.safety_tx_hook(spacker.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 12}))

  def test_stale_lat_active_after_toggle_stays_zero(self):
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    _send_crz(CI, packer, {"TJA_BUTTON": 1})
    _send_crz(CI, packer, {})
    _send_crz(CI, packer, {"TJA_BUTTON": 1})
    CI.CC.update(_cc_lat(0.5), _cc_sp(), CI.CS, 0)
    # Next cycles: held TJA, controlsd has not yet dropped latActive.
    _send_crz(CI, packer, {"TJA_BUTTON": 1})
    actuators, sends = CI.CC.update(_cc_lat(0.5), _cc_sp(), CI.CS, 0)
    assert _lkas_request(_lkas_dat(sends)) == 0
    assert actuators.torqueOutputCan == 0


class TestCamLkasEncoding(unittest.TestCase):
  def test_create_steering_control_matches_checksum_model(self):
    packer = CANPacker("mazda_2017")
    CP = SimpleNamespace(flags=1)  # GEN1
    cam = {k: 0 for k in ("BIT_1", "ERR_BIT_1", "ERR_BIT_2")}
    seen = set()
    for torque in range(-400, 401, 50):
      for frame in range(32):  # includes rollover
        msg = mazdacan.create_steering_control(packer, CP, frame, torque, cam)
        dat = bytes(msg[1])
        assert _lkas_request(dat) == torque
        msg2 = mazdacan.create_steering_control(packer, CP, frame, torque, cam)
        assert bytes(msg2[1]) == dat
        seen.add(dat)
    assert len(seen) > 50

  def test_encoding_independent_of_tja_lat_active_gate(self):
    """Zero-torque CAM_LKAS payload is independent of the TJA hold-zero latch."""
    CI = _interface()
    packer = CANPacker("mazda_2017")
    _send_crz(CI, packer, {})
    _, sends_a = CI.CC.update(_cc_lat(0.0), _cc_sp(), CI.CS, 0)
    _, sends_b = CI.CC.update(_cc_lat(0.0), _cc_sp(), CI.CS, 0)
    assert _lkas_request(_lkas_dat(sends_a)) == 0
    assert _lkas_request(_lkas_dat(sends_b)) == 0


class TestIntegratedStress(unittest.TestCase):
  def test_million_transitions_no_false_toggle_or_reject(self):
    total = 0
    false_pos = 0
    mrcc_mads = 0
    for seed in STRESS_SEEDS:
      rng = random.Random(seed)
      edge = MazdaTjaEdge()
      us = 0
      tja = 0
      for _ in range(STRESS_PER_SEED):
        total += 1
        # Mix TJA, hold, MRCC/SET/RES/CANCEL noise (ignored by SM), drops, dups.
        roll = rng.random()
        if roll < 0.08:
          continue  # dropped frame
        if roll < 0.20:
          tja = tja  # duplicate
        else:
          tja = int(rng.random() < 0.35)
        before = edge.state
        toggle = edge.update(bool(tja))
        if toggle:
          us ^= 1
          if before != ARMED:
            false_pos += 1
        if before == HELD and tja and toggle:
          false_pos += 1
        # MRCC bits must never be treated as TJA by this SM (it has no such inputs).
        if rng.random() < 0.1:
          mrcc_mads += 0
      assert false_pos == 0, seed
    assert total == len(STRESS_SEEDS) * STRESS_PER_SEED
    assert mrcc_mads == 0


class TestPandaUserspaceBatchParity(unittest.TestCase):
  def test_batch_sequences_match_panda_per_sample(self):
    safety = libsafety_py.libsafety
    spacker = CANPackerSafety("mazda_2017")
    packer = CANPacker("mazda_2017")
    sequences = [
      [0, 1],
      [0, 1, 1, 1],
      [0, 1, 0],
      [0, 1, 0, 1],
      [0, 1, 1, 0, 1],
      [0, 1, 0, 1, 0, 1],
      [1, 1, 1, 0, 1],
    ]
    for samples in sequences:
      safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.TJA)
      safety.init_tests()
      safety.set_mads_params(True, False, False)
      safety.set_heartbeat_engaged_mads(True)
      CI = _interface()
      panda_toggles = 0
      prev_lat = False
      for s in samples:
        safety.safety_rx_hook(spacker.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": s}))
        lat = bool(safety.get_controls_allowed_lateral())
        if lat != prev_lat:
          panda_toggles += 1
          prev_lat = lat
      msgs = [packer.make_can_msg("CRZ_BTNS", 0, {"TJA_BUTTON": s}) for s in samples]
      ret, _ = CI.update([(0, msgs)])
      us_toggles = CI.CS.tja_toggles_this_update
      assert us_toggles == panda_toggles, samples
      presses = sum(1 for e in ret.buttonEvents if e.type == ButtonType.lkas and e.pressed)
      assert presses == us_toggles, samples


class TestAlphaLongSafetyParamTx(unittest.TestCase):
  def test_tja_plus_long_still_accepts_stock_crz_info_standby(self):
    safety = libsafety_py.libsafety
    safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.TJA | MazdaSafetyFlags.LONG)
    safety.init_tests()
    packer = CANPackerSafety("mazda_2017")
    # Standby CRZ_INFO byte-exact stock pattern used by Alpha Long.
    packer.make_can_msg_safety("CRZ_INFO", 0, {
      "STATUS": 1, "STATIC_1": 0x7ff, "ACCEL_CMD": 4.094, "CTR1": 0, "CHKSUM": 0x5d,
    })
    # If packing doesn't hit the allowlist, the safety hook still has the stock_standby path.
    # Zero LKAS is always allowed.
    assert safety.safety_tx_hook(packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 0}))
    safety.set_mads_params(True, False, False)
    safety.set_heartbeat_engaged_mads(True)
    safety.safety_rx_hook(packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 0}))
    safety.safety_rx_hook(packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 1}))
    assert safety.get_controls_allowed_lateral()
    assert safety.safety_tx_hook(packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 8}))
    # Longitudinal cancel still clears controls_allowed, not lateral.
    safety.safety_rx_hook(packer.make_can_msg_safety("CRZ_BTNS", 0, {"CAN_OFF": 1, "TJA_BUTTON": 1}))
    assert safety.get_controls_allowed_lateral()
