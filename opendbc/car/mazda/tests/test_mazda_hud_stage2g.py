"""Stage 2G candidate HUD matrix. Isolated candidate; not merged policy.

Packed TJA depends only on MADS and MRCC. FSC TJA is not leaked.
TJA_TRANSITION is copied. TJA=4 is never generated. HUD packing does not
emit cruise buttons.
"""
from __future__ import annotations

import unittest

from opendbc.can import CANPacker
from opendbc.can import CANParser
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  CAM_LKAS, CRZ_BTNS, CRZ_EVENTS, _controller, _decode_laneinfo, _hud_step,
  _laneinfo_tja, _tja_press)
from opendbc.car.mazda.tests.unittest_compat import parametrize


def _parser():
  parser = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 0)
  for st in parser.message_states.values():
    st.ignore_checksum = True
    st.ignore_counter = True
    st.ignore_alive = True
  return parser


def _cam(tja, transition=2, lane_lines=3):
  cam = {
    "LINE_VISIBLE": 1, "LINE_NOT_VISIBLE": 0, "LANE_LINES": lane_lines,
    "BIT1": 1, "BIT2": 0, "BIT3": 0, "NO_ERR_BIT": 0, "S1": 0, "S1_HBEAM": 0,
    "TJA": tja, "TJA_TRANSITION": transition,
  }
  return cam


def _expected(mads, mrcc):
  if not mads:
    return 0
  if mrcc == "ACTIVE":
    return 3
  return 2


class TestMazdaHudStage2gMatrix(unittest.TestCase):
  COPIED = (
    "LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES",
    "BIT1", "BIT2", "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM",
  )

  @parametrize("mads,mrcc,fsc_tja", [
    (mads, mrcc, tja)
    for mads in (False, True)
    for mrcc in ("OFF", "ARMED", "ACTIVE")
    for tja in (0, 2, 3, 4)
  ])
  def test_static_matrix_independent_of_fsc(self, mads, mrcc, fsc_tja):
    packer = CANPacker("mazda_2017")
    cam = _cam(fsc_tja, transition=2, lane_lines=3)
    msg = mazdacan.create_alert_command(
      packer, cam, False, False,
      mrcc_active=(mrcc == "ACTIVE"), mads_enabled=mads, mrcc_armed=(mrcc == "ARMED"))
    parser = _parser()
    parser.update([(0, [msg])])
    vl = parser.vl["CAM_LANEINFO"]
    assert vl["TJA"] == _expected(mads, mrcc), (mads, mrcc, fsc_tja, vl["TJA"])
    assert vl["TJA"] != 4
    assert vl["TJA_TRANSITION"] == 2
    for s in self.COPIED:
      assert vl[s] == cam[s], s

  def test_explicit_six_rows(self):
    packer = CANPacker("mazda_2017")
    parser = _parser()
    rows = (
      (False, "OFF", 0),
      (True, "OFF", 2),
      (False, "ARMED", 0),
      (True, "ARMED", 2),
      (False, "ACTIVE", 0),
      (True, "ACTIVE", 3),
    )
    for mads, mrcc, exp in rows:
      for fsc in (0, 2, 3, 4):
        cam = _cam(fsc, transition=1, lane_lines=3)
        msg = mazdacan.create_alert_command(
          packer, cam, False, False,
          mrcc_active=(mrcc == "ACTIVE"), mads_enabled=mads, mrcc_armed=(mrcc == "ARMED"))
        parser.update([(0, [msg])])
        vl = parser.vl["CAM_LANEINFO"]
        assert vl["TJA"] == exp, (mads, mrcc, fsc)
        assert vl["TJA_TRANSITION"] == 1


class TestMazdaHudStage2gTransitions(unittest.TestCase):
  @parametrize("alpha_long", [False, True])
  def test_off_mads_on_tja_0_to_2_mrcc_stays_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    li = _decode_laneinfo(sends)
    assert li[0]["TJA"] == 2
    assert li[0]["TJA_TRANSITION"] == 2
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_off_mads_off_tja_2_to_0_mrcc_stays_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(3, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 2
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=False, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 0
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_armed_mads_on_off_preserves_armed(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 2
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=True, enabled=False,
                           cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 0
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_active_mads_on_packs_tja3_preserves_active(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(0, transition=2)
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 0
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    li = _decode_laneinfo(sends)
    assert li[0]["TJA"] == 3
    assert li[0]["TJA_TRANSITION"] == 2
    assert not any(a in (CRZ_BTNS, CRZ_EVENTS) for a, _, _ in sends)
    assert any(a == CAM_LKAS for a, _, _ in sends)

  @parametrize("alpha_long", [False, True])
  def test_active_tja3_stable_across_lat_active(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(4, transition=1)
    for lat in (True, False, True, False):
      crz, sends = _hud_step(ctrl, lat, mads_enabled=True, available=True, enabled=True,
                             cc_enabled=True, long_active=True, cam_laneinfo=cam)
      assert crz == []
      li = _decode_laneinfo(sends)
      assert li[0]["TJA"] == 3
      assert li[0]["TJA_TRANSITION"] == 1
      assert li[0]["TJA"] != 4

  @parametrize("alpha_long", [False, True])
  def test_active_physical_tja_off_hud_tja0_no_software_mrcc_off(self, alpha_long):
    ctrl = _controller(alpha_long)
    cam = _laneinfo_tja(3, transition=2)
    crz, sends = _hud_step(ctrl, True, mads_enabled=True, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 3
    crz, _ = _tja_press(ctrl, False, tja=1, available=True, enabled=True,
                        pre_available=True, pre_enabled=True, cam_laneinfo=cam)
    # Software must not command MRCC OFF. Restore may TX clean MRCC to keep ACTIVE.
    assert not any(f.get("CAN_OFF") for f in crz)
    crz, sends = _hud_step(ctrl, False, mads_enabled=False, available=True, enabled=True,
                           cc_enabled=True, long_active=True, cam_laneinfo=cam)
    assert crz == []
    assert _decode_laneinfo(sends)[0]["TJA"] == 0
    assert not any(a == CRZ_EVENTS for a, _, _ in sends)
