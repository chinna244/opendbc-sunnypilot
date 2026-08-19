#!/usr/bin/env python3
"""CAM_LKAS packing and CarController handshake.

Packer default (no tx_lnv) still copies FSC LINE_NOT_VISIBLE. Once latActive
has completed the internal ~50 ms request=0 settle, the controller transmits
CAM_LKAS LNV=0 with steering allowed, independent of FSC painted-lane bits and
independent of FSC CAM_LANEINFO TJA. FSC ERR_BIT_1/2 still zeros the request.
Panda/MADS auth is CC.latActive. 0x440 remains an FSC copy at ~2 Hz.
"""

import random

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

# bus2 FSC CAM_LKAS captured on ff7df7d6f9c3403b|00000036--dca9bb4a70
# just before the FSC ERR_BIT rise (LNV=1, ERR=0, request=0)
ROUTE36_FSC_PREFAULT = bytes.fromhex("48000820020000c3")
# same route, first FSC ERR_BIT_1/2 frame
ROUTE36_FSC_FAULT = bytes.fromhex("68000960020000b0")
# public FSC-origin LNV=1 + nonzero request (checksum-valid)
PUBLIC_FSC_LNV1_NZ = bytes.fromhex("17a5082002000022")


def _cp():
  fingerprint = {0: {}, 1: {}, 2: {}}
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long=False, is_release_sp=False, docs=False)
  return CP, CP_SP


def _lkas(*, lnv=0, err1=0, err2=0, bit1=1):
  return {
    "BIT_1": bit1,
    "ERR_BIT_1": err1,
    "ERR_BIT_2": err2,
    "LINE_NOT_VISIBLE": lnv,
  }


def _decode_cam_lkas(dat):
  cp = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)
  cp.update([(0, [(0x243, bytes(dat), 0)])])
  return cp.vl["CAM_LKAS"]


def _pack(CP, frame, torque, lkas):
  packer = CANPacker("mazda_2017")
  return mazdacan.create_steering_control(packer, CP, frame, torque, lkas)


def _checksum(ctr, torque, lnv, er1, er2, b1):
  tmp = torque + 2048
  lo = tmp & 0xFF
  hi = tmp >> 8
  steering_angle = 0
  b2 = 0
  ldw = 0
  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)
  csum = csum - ahi - amd - alo - b2
  if ahi == 1:
    csum = csum + 15
  if csum < 0:
    csum = csum + 512 if csum < -256 else csum + 256
  return csum % 256


class TestFscCamLkasAllowsSteer:
  def test_lnv0_no_err_allows(self):
    assert mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=0))

  def test_lnv1_does_not_block(self):
    assert mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=1))

  def test_err_bits_block(self):
    assert not mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=0, err1=1))
    assert not mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=0, err2=1))
    assert not mazdacan.fsc_cam_lkas_allows_steer(_lkas(lnv=1, err1=1, err2=1))

  def test_missing_lnv_does_not_block(self):
    assert mazdacan.fsc_cam_lkas_allows_steer({"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 0})


class TestCreateSteeringControlCoherence:
  def setup_method(self):
    self.CP, _ = _cp()

  def test_lnv0_passes_through_torque(self):
    addr, dat, bus = _pack(self.CP, 4, 90, _lkas(lnv=0))
    assert addr == 0x243 and bus == 0
    v = _decode_cam_lkas(dat)
    assert int(v["LINE_NOT_VISIBLE"]) == 0
    assert int(v["LKAS_REQUEST"]) == 90
    assert int(v["CTR"]) == 4
    assert int(v["ERR_BIT_1"]) == 0
    assert int(v["CHKSUM"]) == _checksum(4, 90, 0, 0, 0, 1)

  def test_lnv1_copies_fsc_and_allows_request(self):
    addr, dat, bus = _pack(self.CP, 4, 90, _lkas(lnv=1))
    v = _decode_cam_lkas(dat)
    assert int(v["LINE_NOT_VISIBLE"]) == 1
    assert int(v["LKAS_REQUEST"]) == 90
    assert int(v["CTR"]) == 4
    assert int(v["CHKSUM"]) == _checksum(4, 90, 1, 0, 0, 1)

  def test_err_bit_zeros_request_and_copies_err(self):
    v = _decode_cam_lkas(_pack(self.CP, 6, 114, _lkas(lnv=1, err1=1, err2=1))[1])
    assert int(v["LINE_NOT_VISIBLE"]) == 1
    assert int(v["ERR_BIT_1"]) == 1
    assert int(v["ERR_BIT_2"]) == 1
    assert int(v["LKAS_REQUEST"]) == 0

  def test_lnv_1_to_0_keeps_request(self):
    lnv1 = _decode_cam_lkas(_pack(self.CP, 1, 50, _lkas(lnv=1))[1])
    lnv0 = _decode_cam_lkas(_pack(self.CP, 2, 50, _lkas(lnv=0))[1])
    assert int(lnv1["LKAS_REQUEST"]) == 50
    assert int(lnv1["LINE_NOT_VISIBLE"]) == 1
    assert int(lnv0["LKAS_REQUEST"]) == 50
    assert int(lnv0["LINE_NOT_VISIBLE"]) == 0

  def test_lnv_0_to_1_keeps_request_and_copies_lnv(self):
    lnv0 = _decode_cam_lkas(_pack(self.CP, 1, 50, _lkas(lnv=0))[1])
    lnv1 = _decode_cam_lkas(_pack(self.CP, 2, 50, _lkas(lnv=1))[1])
    assert int(lnv0["LKAS_REQUEST"]) == 50
    assert int(lnv1["LKAS_REQUEST"]) == 50
    assert int(lnv1["LINE_NOT_VISIBLE"]) == 1

  def test_counter_still_frame_mod_16(self):
    for frame in range(40):
      v = _decode_cam_lkas(_pack(self.CP, frame, 0, _lkas(lnv=0))[1])
      assert int(v["CTR"]) == frame % 16

  def test_checksum_tracks_lnv_and_torque(self):
    for lnv, torque, expect_torque in ((0, 12, 12), (1, 12, 12), (0, 0, 0)):
      v = _decode_cam_lkas(_pack(self.CP, 9, torque, _lkas(lnv=lnv))[1])
      assert int(v["LKAS_REQUEST"]) == expect_torque
      assert int(v["CHKSUM"]) == _checksum(9, expect_torque, lnv, 0, 0, 1)


class TestRoutePrefaultFrames:
  def setup_method(self):
    self.CP, _ = _cp()
    self.cp = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)

  def _fsc(self, dat):
    self.cp.update([(0, [(0x243, dat, 0)])])
    return dict(self.cp.vl["CAM_LKAS"])

  def test_route36_prefault_copies_lnv1_and_may_request(self):
    fsc = self._fsc(ROUTE36_FSC_PREFAULT)
    assert int(fsc["LINE_NOT_VISIBLE"]) == 1
    assert int(fsc["LKAS_REQUEST"]) == 0
    assert int(fsc["ERR_BIT_1"]) == 0
    packed = _decode_cam_lkas(_pack(self.CP, 13, 90, fsc)[1])
    assert int(packed["LINE_NOT_VISIBLE"]) == 1
    assert int(packed["LKAS_REQUEST"]) == 90
    assert int(packed["ERR_BIT_1"]) == 0

  def test_route36_fault_frame_does_not_request_steer(self):
    fsc = self._fsc(ROUTE36_FSC_FAULT)
    assert int(fsc["ERR_BIT_1"]) == 1
    packed = _decode_cam_lkas(_pack(self.CP, 2, 118, fsc)[1])
    assert int(packed["LINE_NOT_VISIBLE"]) == 1
    assert int(packed["ERR_BIT_1"]) == 1
    assert int(packed["LKAS_REQUEST"]) == 0

  def test_public_fsc_lnv1_nonzero_is_valid_encoding(self):
    fsc = self._fsc(PUBLIC_FSC_LNV1_NZ)
    assert int(fsc["LINE_NOT_VISIBLE"]) == 1
    assert int(fsc["LKAS_REQUEST"]) == -91
    assert int(fsc["ERR_BIT_1"]) == 0
    packed = _decode_cam_lkas(_pack(self.CP, 1, -91, fsc)[1])
    assert int(packed["LINE_NOT_VISIBLE"]) == 1
    assert int(packed["LKAS_REQUEST"]) == -91


class TestCarControllerLnvCoherence:
  def setup_method(self):
    self.CP, self.CP_SP = _cp()
    self.CI = CarInterface(self.CP, self.CP_SP)
    self.packer = CANPacker("mazda_2017")
    self.t = 0

  def _step_fsc(self, *, lnv, err1=0, torque=0.2, lat_active=True, settle_handshake=True):
    packed = mazdacan.create_steering_control(self.packer, self.CP, 0, 0,
                                              _lkas(lnv=lnv, err1=err1, err2=err1))
    cam = CanData(packed[0], packed[1], 2)
    CC = structs.CarControl()
    CC.latActive = lat_active
    CC.actuators.torque = torque
    CC_SP = structs.CarControlSP()

    def _apply():
      self.t += 10_000_000
      self.CI.update([(self.t, [cam])])
      return self.CI.apply(CC.as_reader(), CC_SP, self.t)

    actuators, sends = _apply()
    if lat_active and err1 == 0 and settle_handshake:
      self.t += mazdacan.TJA3_ACTIVATION_HOLD_NS
      actuators, sends = _apply()
    lkas = next(s for s in sends if s[0] == 0x243)
    return actuators, _decode_cam_lkas(lkas[1])

  def test_controller_lnv0_can_request_steer(self):
    actuators, v = self._step_fsc(lnv=0)
    assert int(v["LINE_NOT_VISIBLE"]) == 0
    assert abs(int(v["LKAS_REQUEST"])) > 0
    assert abs(actuators.torqueOutputCan) > 0

  def test_controller_lnv1_enters_active_command_lnv0(self):
    actuators, v = self._step_fsc(lnv=1)
    assert int(v["LINE_NOT_VISIBLE"]) == 0
    assert abs(int(v["LKAS_REQUEST"])) > 0
    assert abs(actuators.torqueOutputCan) > 0

  def test_controller_handshake_zeros_request_for_50ms(self):
    _, v = self._step_fsc(lnv=1, settle_handshake=False)
    assert int(v["LINE_NOT_VISIBLE"]) == 0
    assert int(v["LKAS_REQUEST"]) == 0

  def test_controller_err_bit_zeros_wire(self):
    _, v = self._step_fsc(lnv=0, err1=1)
    assert int(v["ERR_BIT_1"]) == 1
    assert int(v["LINE_NOT_VISIBLE"]) == 0
    assert int(v["LKAS_REQUEST"]) == 0

  def test_controller_err_bit_lnv1_zeros_wire(self):
    _, v = self._step_fsc(lnv=1, err1=1)
    assert int(v["ERR_BIT_1"]) == 1
    assert int(v["LINE_NOT_VISIBLE"]) == 1
    assert int(v["LKAS_REQUEST"]) == 0

  def test_controller_lat_inactive_zeros_request_and_copies_lnv(self):
    _, v = self._step_fsc(lnv=1, lat_active=False)
    assert int(v["LINE_NOT_VISIBLE"]) == 1
    assert int(v["LKAS_REQUEST"]) == 0

  def test_cam_lkas_truth_table_fuzz(self):
    rng = random.Random(36)
    for _ in range(80):
      lnv = rng.choice((0, 1))
      err = rng.choice((0, 1))
      lat = rng.choice((True, False))
      torque = rng.choice((0.0, 0.2, -0.2, 1.0))
      _, v = self._step_fsc(lnv=lnv, err1=err, torque=torque, lat_active=lat)
      if lat and not err:
        assert int(v["LINE_NOT_VISIBLE"]) == 0
      else:
        assert int(v["LINE_NOT_VISIBLE"]) == lnv
      if err or not lat or torque == 0.0:
        assert int(v["LKAS_REQUEST"]) == 0
      else:
        assert abs(int(v["LKAS_REQUEST"])) > 0

  def test_controller_hud_stays_at_2hz_while_mads_active(self):
    # Route 3F: no 100 Hz CAM_LANEINFO while ACTIVE. frame%50 @ 100 Hz ≈ 2 Hz.
    # The FSC 0x440 has to keep arriving for the HUD to be sent at all, so feed it
    # alongside CAM_LKAS the way the camera does.
    packed = mazdacan.create_steering_control(self.packer, self.CP, 0, 0, _lkas(lnv=1))
    cam = CanData(packed[0], packed[1], 2)
    laneinfo = CanData(0x440, mazdacan.OEM_LL1_HUD_OFF, 2)
    CC = structs.CarControl()
    CC.latActive = True
    CC.actuators.torque = 0.2
    CC_SP = structs.CarControlSP()
    hud_count = 0
    for _ in range(100):
      self.t += 10_000_000
      self.CI.update([(self.t, [cam, laneinfo])])
      _, sends = self.CI.apply(CC.as_reader(), CC_SP, self.t)
      hud_count += sum(1 for s in sends if s[0] == 0x440)
    assert hud_count == 2
