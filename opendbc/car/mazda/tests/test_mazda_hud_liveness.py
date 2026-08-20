#!/usr/bin/env python3
"""0x440 is sent only while the FSC is actually talking.

Panda marks 0x440 check_relay, so once the relay opens openpilot is the cluster's only
source of that frame. Before the first FSC payload, and after the ~2 Hz stream goes stale,
nothing is transmitted at all: no synthesized frame, no replay of the last one. A stale
latch would otherwise show a healthy HUD indefinitely after the camera died.

These run through the real CarInterface so the raw latch (interface), the liveness counter
(carstate) and the send gate (carcontroller) are exercised together.
"""

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carstate import CAM_LANEINFO_STALE_FRAMES
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

LANEINFO_ADDR = 0x440
WARN_PAYLOAD = bytes.fromhex("4202000640001040")  # route-3C OEM LDW overlay: must pass through
BOOT_PAYLOAD = bytes.fromhex("4261000000001040")  # NO_ERR_BIT boot marker: must pass through


def _lkas_frame(packer, CP):
  lkas = {"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 0, "LINE_NOT_VISIBLE": 0}
  addr, dat = mazdacan.create_steering_control(packer, CP, 0, 0, lkas)[:2]
  return CanData(addr, dat, 2)


class _Rig:
  """Drives CarInterface at 100 Hz with independent control of the 0x243 and 0x440 streams."""

  def __init__(self):
    fingerprint = {0: {}, 1: {}, 2: {}}
    self.CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False,
                                      is_release=False, docs=False)
    self.CP_SP = CarInterface.get_params_sp(self.CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                            False, False, False)
    self.CI = CarInterface(self.CP, self.CP_SP)
    self.packer = CANPacker("mazda_2017")
    self.t = 0
    self.lkas = _lkas_frame(self.packer, self.CP)

  def step(self, *, laneinfo: bytes | None = None, lkas: bool = True,
           lat_active: bool = False, mads_enabled: bool = False, n: int = 1):
    """Advance n control frames. Returns every 0x440 emitted across them."""
    CC = structs.CarControl()
    CC.latActive = lat_active
    CC_SP = structs.CarControlSP()
    CC_SP.mads.available = True
    CC_SP.mads.enabled = mads_enabled
    out = []
    for _ in range(n):
      self.t += 10_000_000
      frames = []
      if lkas:
        frames.append(self.lkas)
      if laneinfo is not None:
        frames.append(CanData(LANEINFO_ADDR, laneinfo, 2))
      self.CI.update([(self.t, frames)])
      _, sends = self.CI.apply(CC.as_reader(), CC_SP, self.t)
      out.extend([bytes(s[1]) for s in sends if s[0] == LANEINFO_ADDR])
    return out

  @property
  def live(self):
    return self.CI.CS.cam_laneinfo_live

  @property
  def raw(self):
    return self.CI.CS.cam_laneinfo_raw


class TestStartup:
  def test_no_hud_before_first_laneinfo(self):
    """Startup: 0x243 healthy, no 0x440 yet -> nothing is transmitted."""
    rig = _Rig()
    sent = rig.step(n=300)
    assert sent == []
    assert rig.raw is None
    assert not rig.live

  def test_no_all_zero_payload_is_ever_emitted(self):
    """The removed reconstruction path used to emit 0000000000000000 here."""
    rig = _Rig()
    sent = rig.step(n=300)
    assert bytes(8) not in sent
    sent += rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=200)
    assert sent, "expected HUD once the FSC stream started"
    assert all(s != bytes(8) for s in sent)

  def test_fresh_payload_starts_transmission(self):
    rig = _Rig()
    assert rig.step(n=100) == []
    sent = rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=100)
    assert len(sent) == 2, "2 Hz while live"
    assert rig.live


class TestMalformed:
  @pytest.mark.parametrize("bad", [b"", b"\x42\x01", b"\x42\x01\x00\x00\x00\x00\x10",
                                   b"\x42\x01\x00\x00\x00\x00\x10\x40\x00"])
  def test_malformed_is_not_latched_and_not_sent(self, bad):
    rig = _Rig()
    sent = rig.step(laneinfo=bad, n=300)
    assert rig.raw is None, "malformed payload must not be latched"
    assert sent == []


class TestStaleTimeout:
  def test_stale_laneinfo_stops_hud_while_lkas_healthy(self):
    """The core bug: 0x243 alive keeps cam_lkas_live true, but 0x440 must still stop."""
    rig = _Rig()
    rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=10)
    assert rig.live

    # 0x243 keeps flowing; 0x440 goes quiet.
    rig.step(n=CAM_LANEINFO_STALE_FRAMES + 5)
    assert rig.CI.CS.cam_lkas_live, "0x243 must still be healthy for this to be meaningful"
    assert not rig.live

    sent = rig.step(n=200)
    assert sent == [], "must not replay the stale latch"
    assert rig.raw == mazdacan.OEM_LL1_HUD_OFF, "latch itself is retained, just not transmitted"

  def test_timeout_boundary_is_exact(self):
    rig = _Rig()
    rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=1)
    assert rig.CI.CS.cam_laneinfo_stale_frames == 0

    # One frame short of the timeout is still live.
    rig.step(n=CAM_LANEINFO_STALE_FRAMES)
    assert rig.CI.CS.cam_laneinfo_stale_frames == CAM_LANEINFO_STALE_FRAMES
    assert rig.live

    # One more frame crosses it.
    rig.step(n=1)
    assert rig.CI.CS.cam_laneinfo_stale_frames == CAM_LANEINFO_STALE_FRAMES + 1
    assert not rig.live

  def test_timeout_tolerates_two_missed_frames_at_2hz(self):
    """~2 Hz stream: 1.5 s must not trip on two consecutive misses (~0.60 s gaps)."""
    rig = _Rig()
    rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=1)
    rig.step(n=120)  # 1.2 s == two missed 0.6 s intervals
    assert rig.live


class TestRecovery:
  def test_first_returning_frame_restores_hud_immediately(self):
    rig = _Rig()
    rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=10)
    rig.step(n=CAM_LANEINFO_STALE_FRAMES + 50)
    assert not rig.live

    # Liveness must recover on the very frame the FSC returns, not a frame later.
    rig.step(laneinfo=mazdacan.OEM_LL1_HUD_WHITE, n=1)
    assert rig.live
    assert rig.raw == mazdacan.OEM_LL1_HUD_WHITE

    sent = rig.step(laneinfo=mazdacan.OEM_LL1_HUD_WHITE, n=100)
    assert len(sent) == 2

  def test_recovery_after_dropout_does_not_emit_stale_bytes(self):
    rig = _Rig()
    rig.step(laneinfo=BOOT_PAYLOAD, n=10)
    rig.step(n=CAM_LANEINFO_STALE_FRAMES + 50)
    sent = rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=100)
    assert sent, "expected transmission to resume"
    assert BOOT_PAYLOAD not in sent, "must not emit the pre-dropout payload"


class TestPassthroughUnchanged:
  @pytest.mark.parametrize("payload", [WARN_PAYLOAD, BOOT_PAYLOAD])
  def test_warning_and_fault_payloads_pass_through_byte_exact_while_fresh(self, payload):
    rig = _Rig()
    sent = rig.step(laneinfo=payload, n=200)
    assert sent, "fresh stream must still transmit"
    assert all(s == payload for s in sent), f"expected byte-exact passthrough of {payload.hex()}"

  def test_cadence_is_2hz_while_live(self):
    rig = _Rig()
    sent = rig.step(laneinfo=mazdacan.OEM_LL1_HUD_OFF, n=500)
    assert len(sent) == 10, "500 frames @ 100 Hz == 5 s == 10 HUD ticks"

  @pytest.mark.parametrize("payload", [bytes.fromhex("4102000000001040"),
                                        bytes.fromhex("4102000600001040"),
                                        bytes.fromhex("4102000700001040"),
                                        WARN_PAYLOAD])
  def test_mads_steering_cannot_replace_real_fsc_lanes_or_warning(self, payload):
    rig = _Rig()
    sent = rig.step(laneinfo=payload, lat_active=True, mads_enabled=True, n=200)
    assert sent
    assert all(dat == payload for dat in sent)
