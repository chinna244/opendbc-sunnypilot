"""Startup CAM_LKAS must flow; MRCC restore must stay inert at ignition.

Vehicle evidence: ff7df7d6f9c3403b|00000014--19759b82aa
panda leftover mazda relay blocked stock 0x243 while card waited ~9 s for
selfdriveInitializing. FSC latched ERR_BIT_1/2 at ~3.83 s until power cycle.

This file does not weaken CAM_LKAS or MRCC fail-close. It proves:
- cold CarController emits CAM_LKAS every 10 ms with torque 0 and ERR bits 0
- no synthetic MRCC/TJA at startup
- panda mazda/TJA accepts those frames and does not invent an MRCC restore
"""

import pytest

from opendbc.can import CANParser
from opendbc.car.mazda.values import MazdaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py

from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  CAM_LKAS, CRZ_BTNS, _controller, _cs, _decode_crz, _step,
)

# Route 00000014: FSC ERR at 3.831 s. Keepalive must cover that window.
STARTUP_FRAMES = 400  # 4.0 s at 100 Hz
FSC_LATCH_FRAMES = 383  # 3.83 s


def _decode_cam_lkas(sends):
  frames = [(addr, dat, bus) for addr, dat, bus in sends if addr == CAM_LKAS]
  if not frames:
    return []
  cp = CANParser("mazda_2017", [("CAM_LKAS", float("nan"))], 0)
  for st in cp.message_states.values():
    st.ignore_checksum = True
    st.ignore_counter = True
    st.ignore_alive = True
  out = []
  for addr, dat, bus in frames:
    cp.update([(0, [(addr, dat, bus)])])
    vl = cp.vl["CAM_LKAS"]
    out.append({
      "ERR_BIT_1": int(vl["ERR_BIT_1"]),
      "ERR_BIT_2": int(vl["ERR_BIT_2"]),
      "LKAS_REQUEST": int(vl["LKAS_REQUEST"]),
      "bus": bus,
      "dat": bytes(dat).hex(),
    })
  return out


def test_unfixed_card_gap_exceeds_fsc_latch():
  # card.py before keepalive: apply starts at ~9.165 s, card was running at 0.56 s.
  unfixed_gap_ms = (9.165 - 0.56) * 1000
  assert unfixed_gap_ms > FSC_LATCH_FRAMES * 10


@pytest.mark.parametrize("alpha_long", [False, True])
def test_cold_controller_emits_cam_lkas_every_frame(alpha_long):
  ctrl = _controller(alpha_long)
  mrcc = 0
  for i in range(STARTUP_FRAMES):
    crz, sends = _step(ctrl, False)
    cam = _decode_cam_lkas(sends)
    assert len(cam) == 1, i
    assert cam[0]["ERR_BIT_1"] == 0
    assert cam[0]["ERR_BIT_2"] == 0
    assert cam[0]["LKAS_REQUEST"] == 0
    assert cam[0]["bus"] == 0
    mrcc += sum(1 for f in crz if f["MRCC"] == 1)
    assert crz == []
  assert mrcc == 0
  assert STARTUP_FRAMES > FSC_LATCH_FRAMES


def test_startup_panda_accepts_zero_torque_cam_lkas_no_restore():
  packer = CANPackerSafety("mazda_2017")
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(CarParams.SafetyModel.mazda, MazdaSafetyFlags.TJA)
  safety.init_tests()
  safety.set_mads_params(True, False, False)
  safety.set_heartbeat_engaged_mads(True)

  assert not safety.get_controls_allowed()
  assert not safety.get_controls_allowed_lateral()

  idle = packer.make_can_msg_safety("CRZ_BTNS", 0, {"TJA_BUTTON": 0, "CTR": 0})
  cam0 = packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 0})
  mrcc = packer.make_can_msg_safety("CRZ_BTNS", 0, {"MRCC_BUTTON": 1})

  rejects = 0
  tja_on = False
  for i in range(STARTUP_FRAMES):
    assert safety.safety_rx_hook(idle)
    if safety.get_controls_allowed_lateral():
      tja_on = True
    if not safety.safety_tx_hook(cam0):
      rejects += 1
    if safety.safety_tx_hook(mrcc):
      raise AssertionError("synthetic MRCC accepted at startup")
  assert rejects == 0
  assert tja_on is False
  assert not safety.get_controls_allowed_lateral()
  assert not safety.get_controls_allowed()
  # Nonzero torque still fail-closed without lateral auth.
  assert not safety.safety_tx_hook(packer.make_can_msg_safety("CAM_LKAS", 0, {"LKAS_REQUEST": 12}))


CAM_LANEINFO_ADDR = 0x440
CRZ_INFO = 0x21B
CRZ_CTRL = 0x21C
RADAR_STATIC = 0x499
RADAR_UDS = 0x764
RADAR_TRACKS = tuple(range(0x361, 0x367))
FORBIDDEN_EARLY_INIT = {CRZ_BTNS, CAM_LANEINFO_ADDR, CRZ_INFO, CRZ_CTRL, RADAR_STATIC, RADAR_UDS, *RADAR_TRACKS}

# Route 00000014 (rlog relative).
ROUTE_00000014_ONROAD_MS = 295
ROUTE_00000014_CARD_RUNNING_MS = 560
ROUTE_00000014_FSC_ERR_MS = 3831
KEEPALIVE_PERIOD_MS = 10


def _addrs(sends):
  return [(int(a), int(b)) for a, _, b in sends]


@pytest.mark.parametrize("alpha_long", [False, True])
def test_early_init_keepalive_emits_only_inactive_cam_lkas(alpha_long):
  ctrl = _controller(alpha_long)
  ctrl.frame = 0
  # Stale MRCC restore / TJA state must not leak onto this path.
  ctrl._tja_off_comp_pending = True
  ctrl._lat_active_prev = True
  cs = _cs(toggles=1, tja=0, available=True, enabled=False,
           pre_available=False, pre_enabled=False, set_p=1, res=1, cancel=1, mrcc=1)
  cs.cam_lkas = {"BIT_1": 0, "ERR_BIT_1": 1, "ERR_BIT_2": 1}
  seen = set()
  err1 = err2 = torque = crz = 0
  for i in range(STARTUP_FRAMES):
    sends = ctrl.early_init_cam_lkas_keepalive(cs)
    addrs = _addrs(sends)
    assert addrs == [(CAM_LKAS, 0)], (i, addrs)
    seen.update(a for a, _ in addrs)
    cam = _decode_cam_lkas(sends)
    assert len(cam) == 1
    assert cam[0]["LKAS_REQUEST"] == 0
    err1 += cam[0]["ERR_BIT_1"]
    err2 += cam[0]["ERR_BIT_2"]
    torque += abs(cam[0]["LKAS_REQUEST"])
    crz += len(_decode_crz(sends))
  assert seen == {CAM_LKAS}
  assert err1 == 0
  assert err2 == 0
  assert torque == 0
  assert crz == 0
  assert ctrl._tja_off_comp_pending is True  # restore state not consumed


@pytest.mark.parametrize("alpha_long", [False, True])
def test_full_apply_is_not_the_early_init_path(alpha_long):
  # First CI.apply frame (frame % 50 == 0) also packs CAM_LANEINFO HUD.
  ctrl = _controller(alpha_long)
  ctrl.frame = 0
  _, sends = _step(ctrl, False)
  addrs = {a for a, _, _ in sends}
  assert CAM_LKAS in addrs
  assert CAM_LANEINFO_ADDR in addrs


def test_route_00000014_keepalive_beats_fsc_window():
  first_after_onroad_ms = ROUTE_00000014_CARD_RUNNING_MS - ROUTE_00000014_ONROAD_MS
  assert first_after_onroad_ms == 265
  assert first_after_onroad_ms < ROUTE_00000014_FSC_ERR_MS
  assert KEEPALIVE_PERIOD_MS < ROUTE_00000014_FSC_ERR_MS
  # Unfixed gap from card-running to first OP CAM_LKAS.
  unfixed_gap_ms = 9165 - ROUTE_00000014_CARD_RUNNING_MS
  assert unfixed_gap_ms > ROUTE_00000014_FSC_ERR_MS

  ctrl = _controller(False)
  ctrl.frame = 0
  cs = _cs()
  t_ms = ROUTE_00000014_CARD_RUNNING_MS
  last_tx = None
  max_gap = first_after_onroad_ms
  err1 = err2 = 0
  unexpected = set()
  while t_ms <= ROUTE_00000014_FSC_ERR_MS + 200:
    sends = ctrl.early_init_cam_lkas_keepalive(cs)
    for a, _, b in sends:
      if (a, b) != (CAM_LKAS, 0):
        unexpected.add((a, b))
    cam = _decode_cam_lkas(sends)
    assert cam and cam[0]["LKAS_REQUEST"] == 0
    err1 += cam[0]["ERR_BIT_1"]
    err2 += cam[0]["ERR_BIT_2"]
    if last_tx is not None:
      max_gap = max(max_gap, t_ms - last_tx)
    last_tx = t_ms
    t_ms += KEEPALIVE_PERIOD_MS
  assert unexpected == set()
  assert err1 == 0
  assert err2 == 0
  assert max_gap == KEEPALIVE_PERIOD_MS or max_gap == first_after_onroad_ms
  assert max_gap < ROUTE_00000014_FSC_ERR_MS
