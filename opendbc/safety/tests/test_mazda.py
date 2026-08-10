#!/usr/bin/env python3
import unittest

from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x09d, 2], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  # Block stock LKAS/HUD camera->car, and CRZ_BTNS car->camera (TJA FSC isolation).
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440], 0: [0x09d]}

  MAX_RATE_UP = 12
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [1200]

  MAX_RT_DELTA = 384

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 15

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))

  def test_crz_btns_fsc_isolation(self):
    # Stage 1: block all CRZ_BTNS bus 0 -> bus 2; preserve other Mazda forwarding.
    representative_addr = 0x202  # ENGINE_DATA

    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x09d))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, representative_addr))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x09d))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, representative_addr))

  @staticmethod
  def _fsc_crz_btns(dat: bytes) -> bytes:
    out = bytearray(dat)
    tja_pressed = bool(out[1] & 0x08)
    out[1] &= ~0x08  # TJA_BUTTON bit 11
    if tja_pressed:
      out[1] |= 0x80  # MRCC_BUTTON bit 15
      out[2] &= ~0x01  # captured MRCC companion bit 16
    return bytes(out)

  def _rx_crz_btns(self, dat: bytes):
    return self._rx(common.make_msg(0, 0x09d, 8, dat))

  def _tx_crz_btns_cam(self, dat: bytes):
    return self._tx(common.make_msg(2, 0x09d, 8, dat))

  def test_crz_btns_sanitized_clone_contract(self):
    # Representative physical frames with undefined trailing bytes preserved.
    idle = bytes.fromhex("0001fff000000000")
    tja = bytes.fromhex("0009fff400000000")
    mrcc = bytes.fromhex("0081fef400000000")
    cancel = bytes.fromhex("0101ffffffffff06")
    resume = bytes.fromhex("0401ffffffffff07")
    set_p = bytes.fromhex("1001ffffffffff08")
    set_m = bytes.fromhex("2001ffffffffff09")
    distance = bytes.fromhex("8001ffffffffff0a")

    # 1. Reject before any physical source frame.
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(idle)))

    # 2. Exact sanitized clone accepted.
    self.assertTrue(self._rx_crz_btns(idle))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(idle)))

    # 3. Physical TJA must match the exact same-counter stock MRCC capture. The old
    # Stage-2 representation, unchanged TJA, and incomplete Stage-5B shape are rejected
    # without consuming the source authorization.
    self.assertTrue(self._rx_crz_btns(tja))
    old_stage2 = bytearray(tja)
    old_stage2[1] &= ~0x08
    self.assertFalse(self._tx_crz_btns_cam(bytes(old_stage2)))
    self.assertFalse(self._tx_crz_btns_cam(tja))
    incomplete_stage5b = bytearray(tja)
    incomplete_stage5b[1] = (incomplete_stage5b[1] & ~0x08) | 0x80
    self.assertEqual(bytes.fromhex("0081fff400000000"), bytes(incomplete_stage5b))
    self.assertFalse(self._tx_crz_btns_cam(bytes(incomplete_stage5b)))
    coherent_tja = self._fsc_crz_btns(tja)
    self.assertEqual(mrcc, coherent_tja)
    self.assertTrue(self._tx_crz_btns_cam(coherent_tja))

    # 4. Neither an arbitrary bit-16 clear nor MRCC injection is authorized by idle.
    self.assertTrue(self._rx_crz_btns(idle))
    cleared_companion = bytearray(idle)
    cleared_companion[2] &= ~0x01
    self.assertFalse(self._tx_crz_btns_cam(bytes(cleared_companion)))
    injected_mrcc = bytearray(idle)
    injected_mrcc[1] |= 0x80
    self.assertFalse(self._tx_crz_btns_cam(bytes(injected_mrcc)))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(idle)))

    # 5. A stock-shaped physical MRCC remains byte-exact, and physical TJA+MRCC
    # clears only TJA and the companion bit while retaining MRCC.
    self.assertEqual(mrcc, self._fsc_crz_btns(mrcc))
    self.assertTrue(self._rx_crz_btns(mrcc))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(mrcc)))
    tja_mrcc = bytes.fromhex("0089fff40000000b")
    self.assertTrue(self._rx_crz_btns(tja_mrcc))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(tja_mrcc)))

    def assert_altered_rejected(src: bytes, mutator):
      self.assertTrue(self._rx_crz_btns(src))
      bad = bytearray(self._fsc_crz_btns(src))
      mutator(bad)
      self.assertFalse(self._tx_crz_btns_cam(bytes(bad)))
      self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(src)))

    # 6-12. Altered button / CTR bits rejected.
    assert_altered_rejected(cancel, lambda d: d.__setitem__(0, d[0] ^ 0x01))
    assert_altered_rejected(resume, lambda d: d.__setitem__(0, d[0] ^ 0x04))
    assert_altered_rejected(set_p, lambda d: d.__setitem__(0, d[0] ^ 0x10))
    assert_altered_rejected(set_m, lambda d: d.__setitem__(0, d[0] ^ 0x20))
    assert_altered_rejected(mrcc, lambda d: d.__setitem__(1, d[1] ^ 0x80))
    assert_altered_rejected(distance, lambda d: d.__setitem__(0, d[0] ^ 0x80))
    assert_altered_rejected(idle, lambda d: d.__setitem__(3, (d[3] & 0xC0) | ((((d[3] >> 2) + 1) & 0x0F) << 2)))

    # 13. Alteration of an otherwise-unrelated / undefined payload bit rejected.
    assert_altered_rejected(idle, lambda d: d.__setitem__(5, d[5] ^ 0x01))

    # 14. Duplicate replacement for same physical source rejected.
    self.assertTrue(self._rx_crz_btns(idle))
    good = self._fsc_crz_btns(idle)
    self.assertTrue(self._tx_crz_btns_cam(good))
    self.assertFalse(self._tx_crz_btns_cam(good))

    # 15. Stale replacement rejected.
    self.safety.set_timer(0)
    self.assertTrue(self._rx_crz_btns(tja))
    self.safety.set_timer(100000 + 1)  # MAZDA_CRZ_BTNS_CLONE_TIMEOUT_US
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(tja)))

    # 16. Next physical source permits exactly one next replacement.
    self.safety.set_timer(200000)
    self.assertTrue(self._rx_crz_btns(mrcc))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(mrcc)))
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(mrcc)))

  def test_bus0_button_commands_isolated_from_clone_permission(self):
    # 15. Existing Mazda bus-0 synthesized button commands retain existing behavior and
    # do not satisfy / bypass the bus-2 clone validator.
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(bytes.fromhex("0101ffffffffff00"))))

    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=True)))
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(bytes.fromhex("0401ffffffffff00"))))

    # Physical RX still required for a valid bus-2 clone.
    src = bytes.fromhex("0001ffffffffff0b")
    self.assertTrue(self._rx_crz_btns(src))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(src)))

  @staticmethod
  def _crz_btns_with_ctr(base: bytes, ctr: int) -> bytes:
    # CTR is the 4-bit motorola field at bits 29..26 (byte3 bits 5..2).
    dat = bytearray(base)
    dat[3] = (dat[3] & 0xC3) | ((ctr & 0x0F) << 2)
    return bytes(dat)

  def test_sanitized_clone_allowed_when_controls_disallowed(self):
    # CRZ_BTNS must keep flowing to the FSC while openpilot/MADS is disengaged.
    self.safety.set_controls_allowed(False)
    src = bytes.fromhex("0009ffffffffff0c")
    self.assertTrue(self._rx_crz_btns(src))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(src)))

    # Existing bus-0 synthesized button rules remain unchanged.
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

  def test_safety_reinit_clears_clone_authorization(self):
    src = bytes.fromhex("0001ffffffffff0d")
    self.assertTrue(self._rx_crz_btns(src))
    self._reset_safety_hooks()
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(src)))

  def test_clone_queue_overflow_and_stale_recovery(self):
    # Fill beyond the 8-entry queue with no clones emitted, then let them go stale.
    self.safety.set_timer(0)
    overflow_srcs = []
    for i in range(10):
      src = self._crz_btns_with_ctr(bytes.fromhex("0001ffffffffff00"), i)
      overflow_srcs.append(src)
      self.assertTrue(self._rx_crz_btns(src))

    self.safety.set_timer(100000 + 1)
    for src in overflow_srcs:
      self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(src)))

    # One fresh physical source after the delay must authorize exactly its clone.
    fresh = self._crz_btns_with_ctr(bytes.fromhex("0009ffffffffff00"), 11)
    self.assertTrue(self._rx_crz_btns(fresh))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(fresh)))
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(fresh)))

  def test_stale_head_does_not_block_fresh_clone(self):
    self.safety.set_timer(0)
    stale = bytes.fromhex("0001ffffffffff01")
    self.assertTrue(self._rx_crz_btns(stale))

    self.safety.set_timer(100000 + 1)
    fresh = bytes.fromhex("0009ffffffffff02")
    self.assertTrue(self._rx_crz_btns(fresh))

    # Stale head must be discarded so the fresh clone can pass.
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(stale)))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(fresh)))

  def test_consumed_source_not_reused_after_ctr_wrap_payload(self):
    # Consumed authorization must not revive just because CTR wrapped to the same payload.
    self.safety.set_timer(0)
    src = self._crz_btns_with_ctr(bytes.fromhex("0001ffffffffff00"), 0)
    self.assertTrue(self._rx_crz_btns(src))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(src)))

    # Same bytes again with no new physical RX: reject even inside the freshness window.
    self.safety.set_timer(1000)
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(src)))

    # A later physical frame that reuses CTR=0 is a new source and authorizes once.
    wrapped = self._crz_btns_with_ctr(bytes.fromhex("0001ffffffffff00"), 0)
    self.assertTrue(self._rx_crz_btns(wrapped))
    self.assertTrue(self._tx_crz_btns_cam(self._fsc_crz_btns(wrapped)))
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(wrapped)))

  def test_host_bus0_tx_cannot_authorize_bus2_clone(self):
    # Only mazda_rx_hook bus-0 RX may enqueue clone authorization; host TX must not.
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._button_msg(resume=True)))

    # Same synthesized resume payload on bus 2 must still be rejected without physical RX.
    resume_dat = self.packer.make_can_msg("CRZ_BTNS", 0, {
      "CAN_OFF": 0,
      "CAN_OFF_INV": 1,
      "RES": 1,
      "RES_INV": 0,
    })[1]
    self.assertFalse(self._tx_crz_btns_cam(self._fsc_crz_btns(bytes(resume_dat))))


class TestMazdaLongitudinalSafety(TestMazdaSafety, common.LongitudinalAccelSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x09d, 2], [0x440, 0], [0x21b, 0], [0x21c, 0], [0x499, 0],
             [0x361, 0], [0x362, 0], [0x363, 0], [0x364, 0], [0x365, 0], [0x366, 0], [0x764, 0],
             [0x21b, 2], [0x21c, 2], [0x499, 2], [0x361, 2], [0x362, 2], [0x363, 2], [0x364, 2], [0x365, 2], [0x366, 2]]

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.LONG)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable, "BRAKE_ON": 0}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _accel_msg(self, accel: float, bus: int = 0):
    values = {"ACCEL_CMD": accel}
    return self.packer.make_can_msg_safety("CRZ_INFO", bus, values)

  def _crz_ctrl_cmd_msg(self, active: bool, bus: int = 0):
    values = {"CRZ_ACTIVE": active}
    return self.packer.make_can_msg_safety("CRZ_CTRL", bus, values)

  def test_camera_bus_accel_actuation_limits(self):
    # the synthetic radar frames are duplicated onto the camera bus; same limits apply there
    for accel in (self.MIN_ACCEL - 1, self.MIN_ACCEL, self.INACTIVE_ACCEL, self.MAX_ACCEL, self.MAX_ACCEL + 1):
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        should_tx = controls_allowed and self.MIN_ACCEL <= accel <= self.MAX_ACCEL
        should_tx = should_tx or accel == self.INACTIVE_ACCEL
        self.assertEqual(should_tx, self._tx(self._accel_msg(accel, bus=2)))

  def test_stock_crz_info_standby_allowed(self):
    # stock standby pegs the command field high; it must pass byte-exactly, checksum included,
    # instead of being decoded as a huge accel command
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        for counter in range(16):
          checksum = (0x5d - counter) & 0xff
          dat = bytes.fromhex(f"01ffe3ffc000{counter:02x}{checksum:02x}")
          self.assertTrue(self._tx(common.make_msg(bus, 0x21b, 8, dat)))

        bad_checksum = bytes.fromhex("01ffe3ffc0000000")
        self.assertFalse(self._tx(common.make_msg(bus, 0x21b, 8, bad_checksum)))

  def test_empty_radar_tracks_allowed(self):
    radar_messages = {
      0x499: bytes.fromhex("0008c00000000000"),
      0x361: bytes.fromhex("fff7fefe1fc00080"),
      0x362: bytes.fromhex("fff7fefe1fc78c80"),
      0x363: bytes.fromhex("fff7fefe1fc00000"),
      0x364: bytes.fromhex("fff7fefe1fc00000"),
      0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
      0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
    }

    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for bus in (0, 2):
        for addr, dat in radar_messages.items():
          self.assertTrue(self._tx(common.make_msg(bus, addr, 8, dat)))

  def test_synthetic_lead_radar_track_gated_on_controls(self):
    for bus in (0, 2):
      for counter in range(16):
        dat = bytes.fromhex(f"0a4000001dc0000{counter:x}")
        self.safety.set_controls_allowed(False)
        self.assertFalse(self._tx(common.make_msg(bus, 0x364, 8, dat)))
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._tx(common.make_msg(bus, 0x364, 8, dat)))

  def test_unexpected_radar_tracks_blocked(self):
    bad_messages = {
      0x499: bytes.fromhex("0008c00100000000"),
      0x361: bytes.fromhex("fff7fefe1fc00180"),
      0x362: bytes.fromhex("fff7fefe1fc00080"),
      0x363: bytes.fromhex("fff7fefe1fc00080"),
      0x364: bytes.fromhex("fff7fefe1fc00080"),
      0x365: bytes.fromhex("fff7fe7ffbff3f80"),
      0x366: bytes.fromhex("fff7fe7ffbff3f80"),
    }

    self.safety.set_controls_allowed(True)
    for bus in (0, 2):
      for addr, dat in bad_messages.items():
        self.assertFalse(self._tx(common.make_msg(bus, addr, 8, dat)))

  def test_radar_uds_allowlist(self):
    # tester present and session control only, main bus only
    self.assertTrue(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("023e800000000000"))))
    self.assertTrue(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0210020000000000"))))
    self.assertFalse(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0210030000000000"))))
    self.assertFalse(self._tx(common.make_msg(0, 0x764, 8, bytes.fromhex("0227010000000000"))))
    self.assertFalse(self._tx(common.make_msg(2, 0x764, 8, bytes.fromhex("023e800000000000"))))

  def test_crz_ctrl_active_gated_on_controls(self):
    for bus in (0, 2):
      self.safety.set_controls_allowed(False)
      self.assertFalse(self._tx(self._crz_ctrl_cmd_msg(True, bus)))
      self.assertTrue(self._tx(self._crz_ctrl_cmd_msg(False, bus)))

      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._crz_ctrl_cmd_msg(True, bus)))


class TestMazdaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
