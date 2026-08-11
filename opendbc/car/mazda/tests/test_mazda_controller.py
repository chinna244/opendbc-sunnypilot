#!/usr/bin/env python3
"""Tests for the Mazda CX-5 2022+ EPS steering parameters (gated on the EPS, not the model)
and the longitudinal message builders and stop-and-go state machine."""

from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.longitudinal import (HOLD_CTRL_LATCH_FRAMES, HOLD_LATCH_FRAMES, HOLD_PASSIVE_FRAMES,
                                            RESUME_REACTIVATE_FRAMES, RESUME_RELEASE_FRAMES, RESUME_UNLATCH_FRAMES,
                                            StopAndGoStateMachine, StopGoState)
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaSafetyFlags
from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface


class TestCarControllerParams:

  @pytest.fixture
  def cx5_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5_2022
      minSteerSpeed = 0.0   # steer_to_zero -> CX-5 2022+ EPS present
    return CarControllerParams(FakeCP())

  @pytest.fixture
  def eps_swap_params(self):
    # A CX-5 2022+ EPS swapped into (or shared by) another Mazda: different model, same EPS.
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX9_2021
      minSteerSpeed = 0.0
    return CarControllerParams(FakeCP())

  @pytest.fixture
  def pre_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5
      minSteerSpeed = 12.5   # no CX-5 EPS -> low-speed lockout, minSteerSpeed > 0
    return CarControllerParams(FakeCP())

  def test_cx5_2022_has_lookup(self, cx5_2022_params):
    assert hasattr(cx5_2022_params, 'STEER_MAX_LOOKUP')
    assert cx5_2022_params.STEER_MAX == 1200

  def test_cx5_2022_low_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [0.0, 5.0, 10.0, 14.2]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 1200

  def test_cx5_2022_high_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [14.5, 20.0, 30.0]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 800

  def test_cx5_2022_rate_limits(self, cx5_2022_params):
    assert cx5_2022_params.STEER_DELTA_UP == 12
    assert cx5_2022_params.STEER_DELTA_DOWN == 25

  def test_cx5_eps_driver_multiplier(self, cx5_2022_params):
    # 15 is the CX-5-EPS tune (upstream stock is 1)
    assert cx5_2022_params.STEER_DRIVER_MULTIPLIER == 15

  def test_eps_swap_gets_cx5_tune(self, eps_swap_params):
    # EPS present (minSteerSpeed == 0) on a non-CX-5 model still gets the higher-authority tune
    assert eps_swap_params.STEER_MAX == 1200
    assert eps_swap_params.STEER_DRIVER_MULTIPLIER == 15
    assert hasattr(eps_swap_params, 'STEER_MAX_LOOKUP')

  def test_no_eps_no_lookup(self, pre_2022_params):
    assert not hasattr(pre_2022_params, 'STEER_MAX_LOOKUP')
    assert pre_2022_params.STEER_MAX == 800
    assert pre_2022_params.STEER_DRIVER_MULTIPLIER == 1


class TestMazdaLateralAuthorization:

  @pytest.fixture
  def controller(self):
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=False,
                                 is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    assert not CP.openpilotLongitudinalControl
    controller = CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)
    controller.frame = 1  # keep this steering-only fixture off the periodic HUD update frame
    assert controller.CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.TJA
    return controller

  @pytest.fixture
  def legacy_controller(self):
    CP = CarInterface.get_params(CAR.MAZDA_CX9_2021, {0: {}, 1: {}, 2: {}}, [], alpha_long=False,
                                 is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX9_2021, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    assert not (CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.TJA)
    controller = CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)
    controller.frame = 1
    return controller

  @pytest.fixture(autouse=True)
  def disable_icbm(self, monkeypatch):
    monkeypatch.setattr(IntelligentCruiseButtonManagementInterface, "update", lambda *args: [])

  @staticmethod
  def controls(lat_active=True):
    control = structs.CarControl()
    control.latActive = lat_active
    control.actuators.torque = 1.0
    return control.as_reader(), structs.CarControlSP()

  @staticmethod
  def carstate(available):
    out = structs.CarState()
    out.vEgoRaw = 10.0
    out.steeringTorque = 0.0
    out.cruiseState.available = available
    return SimpleNamespace(out=out, crz_btns_counter=0,
                           cam_laneinfo={}, cam_lkas={"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
                           lkas_allowed_speed=True, cancel_button=0)

  @staticmethod
  def lkas_request(sends):
    dat = next(dat for addr, dat, bus in sends if addr == 0x243 and bus == 0)
    return (((dat[0] & 0x0f) << 8) | dat[1]) - 2048

  def test_tja_lateral_does_not_depend_on_acc_main(self, controller):
    control, control_sp = self.controls()
    carstate = self.carstate(available=False)

    emitted = []
    for _ in range(4):
      actuators, sends = controller.update(control, control_sp, carstate, 0)
      emitted.append(self.lkas_request(sends))
      assert actuators.torqueOutputCan == emitted[-1]
    assert emitted == [12, 24, 36, 48]

  def test_non_tja_lateral_remains_gated_by_acc_main(self, legacy_controller):
    control, control_sp = self.controls()
    carstate = self.carstate(available=False)

    for _ in range(20):
      actuators, sends = legacy_controller.update(control, control_sp, carstate, 0)
      assert self.lkas_request(sends) == 0
      assert actuators.torqueOutputCan == 0
      assert legacy_controller.apply_torque_last == 0

    carstate.out.cruiseState.available = True
    _, sends = legacy_controller.update(control, control_sp, carstate, 0)
    assert self.lkas_request(sends) == legacy_controller.params.STEER_DELTA_UP

  def test_lat_inactive_always_commands_zero(self, controller):
    control, control_sp = self.controls(lat_active=False)
    carstate = self.carstate(available=False)
    for _ in range(4):
      actuators, sends = controller.update(control, control_sp, carstate, 0)
      assert self.lkas_request(sends) == 0
      assert actuators.torqueOutputCan == 0
      assert controller.apply_torque_last == 0

  def test_mrcc_available_path_retains_normal_ramp(self, controller):
    control, control_sp = self.controls()
    carstate = self.carstate(available=True)

    emitted = []
    for _ in range(4):
      _, sends = controller.update(control, control_sp, carstate, 0)
      emitted.append(self.lkas_request(sends))
    assert emitted == [12, 24, 36, 48]

  @pytest.mark.parametrize(("lat_active", "available", "camera_raw", "expected_tja", "expected_torque"), [
    (False, False, "4201000000001040", (0, 0), 0),
    (True, False, "4201000000001040", (2, 2), 12),
    (False, True, "4201000a20001040", (0, 0), 0),
    (True, True, "4201000a20001040", (2, 2), 12),
  ])
  def test_tja_hud_matrix_does_not_change_steering_or_follow_mrcc(self, controller, lat_active, available,
                                                                  camera_raw, expected_tja, expected_torque):
    control, control_sp = self.controls(lat_active=lat_active)
    carstate = self.carstate(available=available)
    carstate.cam_laneinfo = decode_laneinfo(camera_raw, bus=2)
    controller.frame = 50

    actuators, sends = controller.update(control, control_sp, carstate, 0)
    assert actuators.torqueOutputCan == expected_torque
    assert self.lkas_request(sends) == expected_torque
    hud = next(dat for addr, dat, bus in sends if addr == 0x440 and bus == 0)
    decoded = decode_laneinfo(hud.hex(), bus=0)
    assert (decoded["TJA"], decoded["TJA_TRANSITION"]) == expected_tja


def decode_laneinfo(raw, bus):
  parser = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], bus)
  parser.update([(0, [(0x440, bytes.fromhex(raw), bus)])])
  return dict(parser.vl["CAM_LANEINFO"])


class TestMazdaHudMessages:

  @pytest.fixture
  def packer(self):
    return CANPacker("mazda_2017")

  @pytest.mark.parametrize(("raw", "tja_active", "expected_raw", "expected_tja"), [
    ("4201000000001040", False, "4201000000001040", (0, 0)),
    ("4201000a20001040", False, "4201000000001040", (0, 0)),  # MADS off, MRCC on
    ("4201000000001040", True, "4201000820001040", (2, 2)),   # MADS on, MRCC off
    ("4201000a20001040", True, "4201000820001040", (2, 2)),  # MADS on, MRCC on
  ])
  def test_tja_platform_display_matrix(self, packer, raw, tja_active, expected_raw, expected_tja):
    camera = decode_laneinfo(raw, bus=2)
    _, generated, bus = mazdacan.create_alert_command(packer, camera, False, False,
                                                      tja_lateral=True, tja_active=tja_active)
    decoded = decode_laneinfo(generated.hex(), bus=0)
    assert bus == 0
    assert generated.hex() == expected_raw
    assert (decoded["TJA"], decoded["TJA_TRANSITION"]) == expected_tja

  def test_tja_on_off_reenable_does_not_latch_state(self, packer):
    camera = decode_laneinfo("4201000a20001040", bus=2)  # MRCC remains on throughout
    for tja_active, expected in ((True, (2, 2)), (False, (0, 0)), (True, (2, 2))):
      _, generated, _ = mazdacan.create_alert_command(packer, camera, False, False,
                                                      tja_lateral=True, tja_active=tja_active)
      decoded = decode_laneinfo(generated.hex(), bus=0)
      assert (decoded["TJA"], decoded["TJA_TRANSITION"]) == expected

  def test_mrcc_changes_do_not_toggle_active_tja_hud(self, packer):
    for raw in ("4201000a20001040", "4201000000001040", "4201000a20001040"):
      camera = decode_laneinfo(raw, bus=2)
      _, generated, _ = mazdacan.create_alert_command(packer, camera, False, False,
                                                      tja_lateral=True, tja_active=True)
      decoded = decode_laneinfo(generated.hex(), bus=0)
      assert (decoded["TJA"], decoded["TJA_TRANSITION"]) == (2, 2)

  def test_non_tja_stage5c_payload_is_unchanged(self, packer):
    camera = decode_laneinfo("4201000a20001040", bus=2)
    _, generated, _ = mazdacan.create_alert_command(packer, camera, False, False,
                                                    tja_lateral=False, tja_active=True)
    assert generated.hex() == "4201000000001040"

  @pytest.mark.parametrize(("steer_required", "expected", "raw"), [
    (False, (0, 0, 0), "4201000820001040"),
    (True, (7, 1, 1), "4201000820001e49"),
  ])
  def test_hands_warning_and_tja_state_are_independent(self, packer, steer_required, expected, raw):
    camera = decode_laneinfo("4201000a20001040", bus=2)
    _, generated, _ = mazdacan.create_alert_command(packer, camera, False, steer_required,
                                                    tja_lateral=True, tja_active=True)
    decoded = decode_laneinfo(generated.hex(), bus=0)
    assert generated.hex() == raw
    assert (decoded["HANDS_WARN_3_BITS"], decoded["HANDS_ON_STEER_WARN"], decoded["HANDS_ON_STEER_WARN_2"]) == expected
    assert (decoded["TJA"], decoded["TJA_TRANSITION"]) == (2, 2)

  def test_lane_fields_remain_camera_pass_through(self, packer):
    camera = decode_laneinfo("4201000a20001040", bus=2)
    _, generated, _ = mazdacan.create_alert_command(packer, camera, False, False,
                                                    tja_lateral=True, tja_active=True)
    decoded = decode_laneinfo(generated.hex(), bus=0)
    for signal in ("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM"):
      assert decoded[signal] == camera[signal]


def crz_info_reference_checksum(dat):
  # independent reimplementation of the CRZ_INFO checksum, validated against 1.94M stock
  # frames including all 10,350 stop-bit frames
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04)) & 0xFF)) & 0xFF


def decode_accel_cmd_raw(dat):
  return (((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096


class TestMazdaLongitudinalMessages:
  """The synthetic CRZ_INFO/CRZ_CTRL/radar frames must reproduce stock captures byte for
  byte; the hex values below come from real radar traffic."""

  @pytest.fixture
  def packer(self):
    return CANPacker("mazda_2017")

  def test_crz_info_standby_matches_stock(self, packer):
    for counter in range(16):
      checksum = (0x5d - counter) & 0xff
      expected = f"01ffe3ffc000{counter:02x}{checksum:02x}"
      dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, False, False, False, False)[1]
      assert dat.hex() == expected

  def test_crz_info_available_matches_stock(self, packer):
    for counter in range(16):
      checksum = (0x99 - counter) & 0xff
      expected = f"01ffe2000480{counter:02x}{checksum:02x}"
      dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, False, True, False, False)[1]
      assert dat.hex() == expected

  @pytest.mark.parametrize(("accel", "stopping", "unlatching", "counter", "expected"), [
    (0.0, False, False, 0, "01ffe20006800097"),     # engaged, zero command
    (2.0, False, False, 3, "01ffe2fa0680039a"),     # ISO max accel, raw 2000
    (-3.5, False, False, 7, "01ffe04a868007c8"),    # ISO max brake, raw -3500
    (-1.024, True, False, 5, "01ffe18006841503"),   # standstill hold, raw -1024 + stop bits
    (-0.001, False, False, 9, "01ffe1ffe68009b0"),  # latched hold, raw -1
    (0.0, False, True, 11, "01ffe20006804b4c"),     # resume unlatch pulse
  ])
  def test_crz_info_engaged_golden_bytes(self, packer, accel, stopping, unlatching, counter, expected):
    dat = mazdacan.create_acc_command(packer, 0, counter, accel, True, False, stopping, unlatching)[1]
    assert dat.hex() == expected

  def test_crz_info_accel_encoding_and_checksum(self, packer):
    # the packed command must round-trip at the 0.001 factor and carry a valid masked-bit
    # checksum over the whole command window, stop bits set or not
    for raw in range(-3500, 2001, 137):
      for stopping in (False, True):
        dat = mazdacan.create_acc_command(packer, 0, raw % 16, raw / 1000.0, True, False, stopping, False)[1]
        assert decode_accel_cmd_raw(dat) == raw
        assert dat[7] == crz_info_reference_checksum(dat)
        assert bool(dat[5] & 0x04) == stopping
        assert bool(dat[6] & 0x10) == stopping

  @pytest.mark.parametrize(("long_active", "acc_available", "gap", "has_lead", "phase", "acc_active_2", "expected"), [
    (False, False, 0, False, 0, False, "0201010000000000"),  # standby
    (False, True, 2, False, 0, False, "02010b0000000000"),   # MRCC armed, SET allowed
    (True, True, 2, True, 1, True, "0a018b2000001000"),      # engaged, cruise, no lead
    (True, True, 2, True, 2, True, "0a018b4000001000"),      # engaged, following a lead
    (True, True, 2, True, 3, True, "0a018b6000001000"),      # stop-and-go hold (near phase)
    (True, True, 2, True, 4, True, "0a018b8000001000"),      # stop-and-go hold (far phase)
    (True, True, 2, True, 3, False, "0a018b6000000000"),     # relaxed hold, ACC_ACTIVE_2 drops
    (True, True, 1, True, 2, True, "0a01874000001000"),      # driver gap 1 mirrored to the dash
  ])
  def test_crz_ctrl_golden_bytes(self, packer, long_active, acc_available, gap, has_lead, phase, acc_active_2, expected):
    dat = mazdacan.create_crz_ctrl(packer, 0, long_active, acc_available, gap, has_lead, phase, acc_active_2)[1]
    assert dat.hex() == expected

  def test_radar_frames_match_stock(self):
    expected = [
      (0x499, "0008c00000000000"),
      (0x361, "fff7fefe1fc00080"),
      (0x362, "fff7fefe1fc78c80"),
      (0x363, "fff7fefe1fc00000"),
      (0x364, "fff7fefe1fc00000"),
      (0x365, "fff7fe7ffbff3fc0"),
      (0x366, "fff7fe7ffbff3fc0"),
    ]
    frames = mazdacan.create_radar_frames(0, 0, synthetic_lead=False)
    assert [(f.address, f.dat.hex()) for f in frames] == expected

  def test_radar_frames_counter_and_synthetic_lead(self):
    frames = mazdacan.create_radar_frames(2, 15, synthetic_lead=True)
    assert all(f.src == 2 for f in frames)
    # counter stamps the low nibble of the last byte on every track
    assert [f.dat[7] & 0x0f for f in frames[1:]] == [15] * 6
    tracks = {f.address: f.dat.hex() for f in frames}
    assert tracks[0x364] == "0a4000001dc0000f"


class TestStopAndGoStateMachine:

  @pytest.fixture
  def sm(self):
    return StopAndGoStateMachine()

  @staticmethod
  def run(sm, frames, **kwargs):
    defaults = dict(long_active=True, stopping=False, standstill=False,
                    resume_pressed=False, virtual_resume=False, gas_override=False)
    defaults.update(kwargs)
    for _ in range(frames):
      state = sm.update(**defaults)
    return state

  def test_full_stop_cycle_virtual_resume(self, sm):
    assert self.run(sm, 1) == StopGoState.CRUISING
    assert self.run(sm, 1, stopping=True) == StopGoState.STOPPING
    assert self.run(sm, 1, stopping=True, standstill=True) == StopGoState.HOLD
    assert sm.stop_bits

    # a virtual resume cannot release the strong hold phase
    assert self.run(sm, HOLD_LATCH_FRAMES - 2, stopping=True, standstill=True, virtual_resume=True) == StopGoState.HOLD

    assert self.run(sm, 2, stopping=True, standstill=True) == StopGoState.HOLD_LATCHED
    assert not sm.stop_bits
    assert self.run(sm, HOLD_PASSIVE_FRAMES, stopping=True, standstill=True) == StopGoState.HOLD_PASSIVE
    assert not sm.acc_active_2

    # resume out of the passive hold: latched-profile blip, then the unlatch pulse
    assert self.run(sm, 1, stopping=True, standstill=True, virtual_resume=True) == StopGoState.RESUMING
    assert not sm.resume_unlatching
    self.run(sm, RESUME_REACTIVATE_FRAMES, stopping=True, standstill=True, virtual_resume=True)
    assert sm.resume_unlatching
    self.run(sm, RESUME_UNLATCH_FRAMES, stopping=True, standstill=True, virtual_resume=True)
    assert not sm.resume_unlatching

    # car creeps off the hold, request clears, release window runs out
    assert self.run(sm, RESUME_RELEASE_FRAMES, stopping=False, standstill=False) == StopGoState.CRUISING

  def test_hold_command_relaxes_at_latch(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 1, stopping=True, standstill=True)
    # strong hold with stop bits and ACC_ACTIVE_2 set, near stop phase
    assert sm.state == StopGoState.HOLD
    assert sm.stop_bits and sm.acc_active_2
    assert sm.ctrl_phase(lead_visible=True) == 3
    # after the measured 3.8 s the command relaxes: stop bits and ACC_ACTIVE_2 clear together
    self.run(sm, HOLD_LATCH_FRAMES, stopping=True, standstill=True)
    assert sm.state == StopGoState.HOLD_LATCHED
    assert not sm.stop_bits and not sm.acc_active_2
    assert sm.ctrl_phase(lead_visible=True) == 3

  def test_physical_res_waits_for_ctrl_latch(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 1, stopping=True, standstill=True)
    # earlier than any stock-observed release: RES is ignored
    assert self.run(sm, 10, stopping=True, standstill=True, resume_pressed=True) == StopGoState.HOLD
    self.run(sm, HOLD_CTRL_LATCH_FRAMES, stopping=True, standstill=True)
    assert self.run(sm, 1, stopping=True, standstill=True, resume_pressed=True) == StopGoState.RESUMING

  def test_gas_releases_hold_immediately(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 1, stopping=True, standstill=True)
    assert self.run(sm, 1, stopping=True, standstill=True, gas_override=True) == StopGoState.RESUMING

  def test_rehold_when_car_does_not_move(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, HOLD_LATCH_FRAMES + 2, stopping=True, standstill=True)
    self.run(sm, 1, stopping=True, standstill=True, virtual_resume=True)
    # request disappears, car never moved: fall back into a fresh hold
    assert self.run(sm, RESUME_RELEASE_FRAMES, stopping=True, standstill=True) == StopGoState.HOLD
    assert sm.hold_frames == 0
    assert sm.stop_bits

  def test_long_disengage_resets(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, HOLD_LATCH_FRAMES + 2, stopping=True, standstill=True)
    assert self.run(sm, 1, long_active=False) == StopGoState.CRUISING
    assert sm.hold_frames == 0

  def test_stop_abort_returns_to_cruising(self, sm):
    self.run(sm, 1, stopping=True)
    # lead speeds up again before the car reaches standstill
    assert self.run(sm, 1, stopping=False) == StopGoState.CRUISING


def _mock_cc(long_active=True, accel=0.5, long_state=None, standstill=False, gas=False, override=False,
             resume=False, lead_visible=True, gap=2, available=True,
             stock_radar_alive=False, fsc_settled=True, handback=False, cruise_engaged=False,
             enabled=None):
  # openpilot is enabled whenever it is longitudinally active; a gas override is the case
  # where it stays enabled with longActive low
  enabled = long_active if enabled is None else enabled
  out = SimpleNamespace(standstill=standstill, gasPressed=gas,
                        cruiseState=SimpleNamespace(available=available, enabled=cruise_engaged))
  actuators = SimpleNamespace(accel=accel, longControlState=long_state)
  cruise = SimpleNamespace(resume=resume, override=override, cancel=False)
  hud = SimpleNamespace(leadVisible=lead_visible, leadDistanceBars=gap)
  cc = SimpleNamespace(enabled=enabled, longActive=long_active, actuators=actuators,
                       cruiseControl=cruise, hudControl=hud)
  cc_sp = SimpleNamespace(stockEcuHandBack=handback)
  cs = SimpleNamespace(out=out, resume_button=0,
                       stock_radar_alive=stock_radar_alive, fsc_settled=fsc_settled)
  return cc, cc_sp, cs


@pytest.fixture
def cc():
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=True,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], True, False, False)
  assert CP.openpilotLongitudinalControl
  return CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


def _long_frames(sends):
  """(ACCEL_CMD raw, CRZ_INFO.ACC_ACTIVE, CRZ_CTRL.CRZ_ACTIVE) from a bus 0 emission, or None."""
  info = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
  ctrl = next((d for a, d, b in sends if a == 0x21c and b == 0), None)
  if info is None:
    return None
  cp = CANParser("mazda_2017", [("CRZ_INFO", float("nan")), ("CRZ_CTRL", float("nan"))], 0)
  cp.update([(0, [(0x21b, info, 0), (0x21c, ctrl, 0)])])
  return decode_accel_cmd_raw(info), cp.vl["CRZ_INFO"]["ACC_ACTIVE"], cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"]


def _step(cc, **kw):
  kw.setdefault("long_state", structs.CarControl.Actuators.LongControlState.pid)
  control, control_sp, carstate = _mock_cc(**kw)
  sends = cc.update_longitudinal(control, control_sp, carstate, virtual_resume_sent=False)
  cc.frame += 1
  return sends


class TestLongitudinalIntegration:
  """Drives the real CarController.update_longitudinal through an engage -> cruise -> stop ->
  hold -> resume timeline and checks the emitted CAN, not just the state machine in isolation."""

  def test_engaged_frame_rates_and_counters(self, cc):
    long = structs.CarControl.Actuators.LongControlState
    crz_info = crz_ctrl = radar_static = tester = 0
    for _ in range(100):  # 1 s at 100 Hz
      sends = _step(cc, long_state=long.pid, accel=1.0, gap=2)
      addrs = [a for a, _, _ in sends]
      buses = {a: [] for a, _, _ in sends}
      for a, _, b in sends:
        buses[a].append(b)
      crz_info += addrs.count(0x21b)
      crz_ctrl += addrs.count(0x21c)
      radar_static += addrs.count(0x499)
      tester += sum(1 for a, _, _ in sends if a == 0x764)
      # CRZ_INFO/CRZ_CTRL, when emitted, always go to both bus 0 and bus 2
      if 0x21b in buses:
        assert sorted(buses[0x21b]) == [0, 2]
        assert sorted(buses[0x21c]) == [0, 2]

    # 100 Hz loop: long msgs at 50 Hz (x2 buses), radar at 10 Hz (x2), tester at 2 Hz
    assert crz_info == crz_ctrl == 100    # 50 frames x 2 buses
    assert radar_static == 20             # 10 frames x 2 buses
    assert tester == 2                    # 2 Hz, single bus
    assert cc.long_counter == 50 and cc.radar_counter == 10

  def test_gap_setting_mirrors_driver(self, cc):
    for gap in (1, 2, 3):
      cc.frame = 0  # force emission on the first step
      sends = _step(cc, gap=gap, long_state=structs.CarControl.Actuators.LongControlState.pid)
      ctrl = next(dat for a, dat, b in sends if a == 0x21c and b == 0)
      cp = CANParser("mazda_2017", [("CRZ_CTRL", float("nan"))], 0)
      cp.update([(0, [(0x21c, ctrl, 0)])])
      assert cp.vl["CRZ_CTRL"]["DISTANCE_SETTING"] == gap

  def test_stop_emits_hold_then_relaxes(self, cc):
    long = structs.CarControl.Actuators.LongControlState

    def accel_cmd(sends):
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      return None if dat is None else decode_accel_cmd_raw(dat)

    # approach the stop
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=False)
    # reach standstill: expect the strong hold command
    hold_seen = latched_seen = False
    for _ in range(int(8.0 / 0.01)):
      sends = _step(cc, long_state=long.stopping, accel=-1.5, standstill=True)
      cmd = accel_cmd(sends)
      if cmd is None:
        continue
      if cmd == round(CarControllerParams.ACCEL_HOLD * 1000):
        hold_seen = True
      if hold_seen and cmd == round(CarControllerParams.ACCEL_HOLD_LATCHED * 1000):
        latched_seen = True
    assert hold_seen, "strong -1024 hold command never emitted at standstill"
    assert latched_seen, "hold never relaxed to the latched -1 command"

  def test_gas_override_stays_engaged(self, cc):
    """A gas press is an override, not a disengagement. The command goes to zero as on every
    other port, but the engaged bits stay set the way Honda drives CONTROL_ON off CC.enabled.
    Clearing them mid-decel takes the PCM out of ACC mode (docs/mazda-gas-override.md)."""
    long = structs.CarControl.Actuators.LongControlState

    # braking hard, then the driver taps the gas
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=-2.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-2.0)

    cmds = []
    for _ in range(100):  # 1 s of override
      sends = _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0.,
                    gas=True, override=True, cruise_engaged=True)
      frame = _long_frames(sends)
      if frame is not None:
        cmds.append(frame)

    raw, acc_active, crz_active = zip(*cmds, strict=True)
    assert all(acc_active), "ACC_ACTIVE dropped during a gas override"
    assert all(crz_active), "CRZ_ACTIVE dropped during a gas override"
    assert set(raw) == {0}, f"command should be zero through the override, got {sorted(set(raw))}"

  def test_command_slew_is_rate_limited(self, cc):
    """The plan can step; the wire should not. Windup is limited tightly because dumping the
    brake in one frame is what the driver feels, winddown loosely so braking is never delayed."""
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=-2.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-2.0)

    # plan jumps straight to +1.0: the command must ramp, not step
    prev = cc.accel_last
    for _ in range(5):
      _step(cc, long_state=long.pid, accel=1.0, cruise_engaged=True)
      assert cc.accel_last - prev == pytest.approx(CarControllerParams.ACCEL_WINDUP_LIMIT, abs=1e-6)
      prev = cc.accel_last

    # and the other way, at the looser winddown limit
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=1.0, cruise_engaged=True)
    prev = cc.accel_last
    for _ in range(5):
      _step(cc, long_state=long.pid, accel=-3.0, cruise_engaged=True)
      assert cc.accel_last - prev == pytest.approx(CarControllerParams.ACCEL_WINDDOWN_LIMIT, abs=1e-6)
      prev = cc.accel_last

  def test_accel_last_tracks_the_wire_not_the_plan(self, cc):
    # update() reports accel_last as actuatorsOutput.accel, the way Toyota, Ford and Honda
    # report the value they sent. It must be the wire value, clip and hold included.
    long = structs.CarControl.Actuators.LongControlState

    # a plan beyond the envelope is reported clipped, not as asked
    for _ in range(400):
      sends = _step(cc, long_state=long.pid, accel=-9.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(CarControllerParams.ACCEL_MIN)
    frame = _long_frames(sends)
    if frame is not None:
      assert frame[0] == round(cc.accel_last * 1000)

    # the standstill hold is a fixed stock replay, and that is what gets reported
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=True, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(CarControllerParams.ACCEL_HOLD)

    # through a gas override we report the zero we actually send
    for _ in range(10):
      _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0., gas=True,
            override=True, cruise_engaged=True)
    assert cc.accel_last == 0.

  def test_gas_from_standstill_hold_releases_the_brake(self, cc):
    # gas out of a hold is a resume, not a slow release: the hold command must go straight to
    # zero rather than ramping off at the cruising override rate
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(int(3.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=True, cruise_engaged=True)
    assert cc.accel_last < -0.5, "never reached the standstill hold"

    for _ in range(20):
      _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0., gas=True,
            override=True, standstill=True, cruise_engaged=True)
    assert cc.accel_last == 0., f"hold not released for the driver's gas: {cc.accel_last}"

  def test_gas_pedal_without_cruise_stays_disengaged(self, cc):
    # gas pressed while openpilot is not enabled must not advertise an engaged ACC
    off = structs.CarControl.Actuators.LongControlState.off
    cc.frame = 0
    sends = _step(cc, long_active=False, enabled=False, long_state=off, gas=True, available=True)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe2000480")  # armed-but-idle pattern, zero command

  def test_disengaged_emits_stock_patterns(self, cc):
    off = structs.CarControl.Actuators.LongControlState.off
    # main off, not available: the exact standby pattern the panda allowlists byte-for-byte
    cc.frame = 0
    sends = _step(cc, long_active=False, long_state=off, available=False)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe3ffc000")
    # MRCC armed but not engaged: stock advertises ACC_SET_ALLOWED with a zero command
    cc.frame = 0
    sends = _step(cc, long_active=False, long_state=off, available=True)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe2000480")


SESSION_PROG_DAT = bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0])
SESSION_DFLT_DAT = bytes([0x02, 0x10, 0x01, 0, 0, 0, 0, 0])
TESTER_PRESENT_DAT = bytes([0x02, 0x3e, 0x80, 0, 0, 0, 0, 0])


class TestRadarSessionSequencing:
  """Boot teardown deferral and the ordered hand-back: what goes on the bus in each
  radar session state, driven through the real CarController.update_longitudinal."""

  def _step(self, cc, stock_radar_alive, fsc_settled, handback=False, cruise_engaged=False):
    off = structs.CarControl.Actuators.LongControlState.off
    return _step(cc, long_active=False, accel=0., long_state=off, lead_visible=False, available=False,
                 stock_radar_alive=stock_radar_alive, fsc_settled=fsc_settled,
                 handback=handback, cruise_engaged=cruise_engaged)

  @staticmethod
  def _uds(sends):
    return [dat for a, dat, b in sends if a == 0x764]

  @staticmethod
  def _synthetic(sends):
    return [a for a, _, _ in sends if a in (0x21b, 0x21c, 0x499)]

  def test_stock_state_is_silent(self, cc):
    # radar alive, gate not yet passed: nothing at all goes on the bus
    for _ in range(200):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=False)
      assert sends == []

  def test_boot_teardown_sequence(self, cc):
    # gate passes with the stock radar alive: programming-session requests at 2 Hz,
    # still no synthetic frames and no tester present
    for i in range(100):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True)
      if i % CarControllerParams.RADAR_UDS_STEP == 0:
        assert self._uds(sends) == [SESSION_PROG_DAT]
      else:
        assert self._uds(sends) == []
      assert self._synthetic(sends) == []
    # radar goes quiet: synthetic frames + tester present take over, session requests stop
    saw_tester = False
    for _ in range(100):
      frame = cc.frame
      sends = self._step(cc, stock_radar_alive=False, fsc_settled=True)
      assert SESSION_PROG_DAT not in self._uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(self._synthetic(sends)) > 0
      saw_tester |= TESTER_PRESENT_DAT in self._uds(sends)
    assert saw_tester

  def test_handback_sequence(self, cc):
    # reach SILENCED
    self._step(cc, stock_radar_alive=False, fsc_settled=True)
    # hand-back requested: default-session requests at 2 Hz, tester present stops,
    # synthetic frames continue while the radar is still quiet
    saw_default = False
    for _ in range(100):
      frame = cc.frame
      sends = self._step(cc, stock_radar_alive=False, fsc_settled=True, handback=True)
      assert TESTER_PRESENT_DAT not in self._uds(sends)
      saw_default |= SESSION_DFLT_DAT in self._uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(self._synthetic(sends)) > 0
    assert saw_default
    # stock radar returns: everything stops
    for _ in range(200):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, handback=True)
      assert sends == []

  def test_handback_before_teardown_stops_everything(self, cc):
    # toggle-off while still waiting on the gate: no session ever entered, so no
    # hand-back traffic either
    self._step(cc, stock_radar_alive=True, fsc_settled=False)
    for _ in range(120):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=False, handback=True)
      assert sends == []

  def test_teardown_waits_for_stock_cruise_disengage(self, cc):
    # driver engaged stock MRCC before the gate passed (warm boot): hold the teardown
    for _ in range(120):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, cruise_engaged=True)
      assert sends == []
    # driver disengages: teardown proceeds
    cc.frame = 0
    sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, cruise_engaged=False)
    assert SESSION_PROG_DAT in self._uds(sends)

  def test_s3_recovery_resilences(self, cc):
    # radar reappears mid-drive (dropped tester present, S3 timeout): re-request the session
    self._step(cc, stock_radar_alive=False, fsc_settled=True)
    cc.frame = CarControllerParams.RADAR_UDS_STEP  # align to a session-request frame
    sends = self._step(cc, stock_radar_alive=True, fsc_settled=True)
    assert SESSION_PROG_DAT in self._uds(sends)
    # and settles back to silenced once quiet again
    sends = self._step(cc, stock_radar_alive=False, fsc_settled=True)
    assert SESSION_PROG_DAT not in self._uds(sends)
