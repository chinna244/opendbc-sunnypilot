#!/usr/bin/env python3
"""Mazda HUD contract: FSC owns every displayed lane and warning bit."""

import itertools

import pytest

from opendbc.can import CANPacker
from opendbc.car.mazda import mazdacan


# Exact FSC frames observed across the HUD investigation. These intentionally include
# idle, transition, real lane-line, OEM TJA, LDW/warning, and boot/status payloads.
FSC_CAPTURES = (
  bytes.fromhex("4201000000001040"),
  bytes.fromhex("4201000020001040"),
  bytes.fromhex("4201000a00001040"),
  bytes.fromhex("4201000c00001040"),
  bytes.fromhex("4102000000001040"),
  bytes.fromhex("4102000600001040"),
  bytes.fromhex("4102000700001040"),
  bytes.fromhex("4202000040001040"),
  bytes.fromhex("4202000640001040"),
  bytes.fromhex("4261000000001040"),
  bytes.fromhex("4361000000000040"),
)


@pytest.fixture
def packer():
  return CANPacker("mazda_2017")


class TestPureFscHud:
  @pytest.mark.parametrize("raw", FSC_CAPTURES)
  @pytest.mark.parametrize(
    "mads_available,mads_enabled,green_hud_enabled,green_allowed",
    tuple(itertools.product((False, True), repeat=4)),
  )
  def test_every_valid_fsc_payload_is_byte_exact_in_every_state(
      self, packer, raw, mads_available, mads_enabled, green_hud_enabled, green_allowed):
    hud = mazdacan.create_alert_command(
      packer, {}, False, False,
      mads_available=mads_available,
      mads_enabled=mads_enabled,
      green_hud_enabled=green_hud_enabled,
      green_allowed=green_allowed,
      fsc_raw=raw,
    )
    assert hud == (0x440, raw, 0, mazdacan.HUD_PASSTHROUGH)

  @pytest.mark.parametrize("ldw,steer_required,apply_torque",
                           tuple(itertools.product((False, True), (False, True), (-800, 0, 800))))
  def test_openpilot_alerts_and_steering_cannot_change_fsc(self, packer, ldw, steer_required, apply_torque):
    raw = bytes.fromhex("4102000700001040")
    hud = mazdacan.create_alert_command(packer, {}, ldw, steer_required, apply_torque=apply_torque,
                                        fsc_raw=raw)
    assert hud[1] == raw
    assert hud[3] == mazdacan.HUD_PASSTHROUGH

  @pytest.mark.parametrize("raw", [None, b"", b"\x42\x01", bytes(7), bytes(9)])
  def test_absent_or_malformed_fsc_is_not_sent(self, packer, raw):
    assert mazdacan.create_alert_command(packer, {}, False, False, fsc_raw=raw) is None

  def test_payload_is_not_reconstructed_from_dbc_signals(self, packer):
    signals = {"TJA": 4, "LANE_LINES": 2, "LDW_WARN_LL": 1}
    assert mazdacan.create_alert_command(packer, signals, True, True, fsc_raw=None) is None
