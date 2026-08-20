#!/usr/bin/env python3
"""OEM HUD ownership + MADS CAM_LKAS steering independence.

Route 3F proved MADS/EPS steering works. Forced ACTIVE TJA=4 + LANE_LINES=1 at
~100 Hz did not. Production copies FSC CAM_LANEINFO intact at ~2 Hz and keeps
an internal ~50 ms CAM_LKAS request=0 settle before ACTIVE LNV=0 steering.
"""

from opendbc.can import CANPacker, CANParser
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.mazdacan import (
  FSC_LNV1_LKAS_REQUEST_MAX, LKAS_TX_ACTIVE, LKAS_TX_AUTH_PAUSED, LKAS_TX_FAULT,
  LKAS_TX_IDLE, LKAS_TX_TRANSITION_TO_ACTIVE, STEER_ACTIVATION_HOLD_NS,
  TJA3_ACTIVATION_HOLD_NS,
)
from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.car.structs import CarParams


def _lkas(lnv=1, err1=0, err2=0, bit1=1):
  return {
    "LINE_NOT_VISIBLE": lnv,
    "ERR_BIT_1": err1,
    "ERR_BIT_2": err2,
    "BIT_1": bit1,
  }


def _lane(tja=0, ll=1, lnv=1, vis=0, tr=0):
  return {
    "TJA": tja,
    "TJA_TRANSITION": tr,
    "LANE_LINES": ll,
    "LINE_NOT_VISIBLE": lnv,
    "LINE_VISIBLE": vis,
    "BIT1": 1,
    "BIT2": 0,
    "BIT3": 1,
    "NO_ERR_BIT": 0,
    "S1": 1,
    "S1_HBEAM": 0,
  }


def _lane_raw(lane: dict) -> bytes:
  packer = CANPacker("mazda_2017")
  values = {s: int(lane.get(s, 0)) for s in mazdacan.CAM_LANEINFO_SIGNALS}
  return bytes(packer.make_can_msg("CAM_LANEINFO", 0, values)[1])


def _hud(lane, mads_enabled=True, mads_available=True, fsc_raw=None):
  packer = CANPacker("mazda_2017")
  raw = _lane_raw(lane) if fsc_raw is None else bytes(fsc_raw)
  return bytes(mazdacan.create_alert_command(
    packer, lane, False, False, mads_enabled=mads_enabled,
    mads_available=mads_available, fsc_raw=raw)[1])


def _cp():
  CP = CarParams()
  CP.flags = int(MazdaFlags.GEN1)
  CP.carFingerprint = str(CAR.MAZDA_CX5_2022)
  return CP


def _decode_243(dat):
  cp = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)
  cp.update([(0, [(0x243, dat, 0)])])
  return dict(cp.vl["CAM_LKAS"])


def _decode_440(dat):
  cp = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 0)
  cp.update([(0, [(0x440, dat, 0)])])
  return dict(cp.vl["CAM_LANEINFO"])


def _step(lat_active, now_ns, start_ns=None, prev=LKAS_TX_IDLE, desired=163,
          fsc_lnv=1, fsc_tja=0, err1=0, fsc_ll=1, fsc_hud_lnv=1, fsc_vis=0, fsc_tr=0):
  return mazdacan.lkas_tx_step(
    lat_active=lat_active, fsc_ok=err1 == 0, now_ns=now_ns,
    handshake_start_ns=start_ns, prev_state=prev, desired_torque=desired,
    fsc_lkas=_lkas(lnv=fsc_lnv, err1=err1),
    fsc_lane=_lane(tja=fsc_tja, ll=fsc_ll, lnv=fsc_hud_lnv, vis=fsc_vis, tr=fsc_tr),
  )


def _pack(tx, fsc_lane=None, req=None):
  packer = CANPacker("mazda_2017")
  torque = tx.apply_torque if req is None else req
  lane = fsc_lane or _lane(tja=tx.tja, ll=tx.lane_lines, lnv=tx.hud_lnv,
                           vis=tx.line_visible, tr=tx.tja_transition)
  steer = mazdacan.create_steering_control(packer, _cp(), 3, torque, _lkas(lnv=1),
                                           tx_lnv=tx.cam_lkas_lnv)
  hud = mazdacan.create_alert_command(packer, lane, False, False, tx=tx, fsc_raw=_lane_raw(lane))
  return _decode_243(steer[1]), _decode_440(hud[1])


class TestHandshakeStateMachine:
  def test_idle_copies_fsc(self):
    tx = _step(False, 0, fsc_tja=2, fsc_lnv=1, desired=400, fsc_ll=0)
    assert tx.state == LKAS_TX_IDLE
    assert tx.apply_torque == 0
    assert tx.tja == 2
    assert tx.cam_lkas_lnv == 1
    assert tx.lane_lines == 0
    assert tx.send_hud_every_frame is False

  def test_no_torque_before_auth(self):
    tx = _step(False, 0, desired=1088, fsc_lnv=1)
    assert tx.apply_torque == 0
    v243, v440 = _pack(tx)
    assert v243["LKAS_REQUEST"] == 0
    assert v440["TJA"] == 0  # exact FSC HUD; MADS does not alter it

  def test_auth_enters_internal_settle_not_tja3(self):
    tx = _step(True, 0, start_ns=None, desired=400, fsc_lnv=1, fsc_tja=0)
    assert tx.state == LKAS_TX_TRANSITION_TO_ACTIVE
    assert tx.tja == 0  # FSC TJA, not forced 3
    assert tx.cam_lkas_lnv == 0
    assert tx.apply_torque == 0
    assert tx.lane_lines == 1
    assert tx.send_hud_every_frame is False
    v243, v440 = _pack(tx)
    assert v243["LKAS_REQUEST"] == 0
    assert v243["LINE_NOT_VISIBLE"] == 0
    assert v440["TJA"] == 0  # exact FSC HUD, never synthetic TJA 3/4
    assert v440["LANE_LINES"] == 1

  def test_transition_lasts_50ms_not_forever(self):
    t0 = 1_000_000_000
    tx0 = _step(True, t0, None, desired=400, fsc_lnv=1, fsc_tja=2)
    tx_mid = _step(True, t0 + STEER_ACTIVATION_HOLD_NS - 1, tx0.handshake_start_ns,
                   tx0.state, desired=400, fsc_lnv=1, fsc_tja=2)
    tx_go = _step(True, t0 + STEER_ACTIVATION_HOLD_NS, tx0.handshake_start_ns,
                  tx_mid.state, desired=400, fsc_lnv=1, fsc_tja=2)
    assert tx_mid.state == LKAS_TX_TRANSITION_TO_ACTIVE
    assert tx_mid.apply_torque == 0
    assert tx_mid.tja == 2
    assert tx_go.state == LKAS_TX_ACTIVE
    assert tx_go.tja == 2  # still FSC, not forced 4
    assert tx_go.cam_lkas_lnv == 0
    assert tx_go.apply_torque == 400
    assert tx_go.send_hud_every_frame is False

  def test_fsc_lnv1_does_not_hold_settle(self):
    t0 = 0
    tx = _step(True, t0, None, desired=90, fsc_lnv=1, fsc_tja=0)
    for dt in (50_000_000, 2_000_000_000, 10_000_000_000):
      tx = _step(True, t0 + dt, tx.handshake_start_ns, tx.state, desired=90, fsc_lnv=1, fsc_tja=0)
      assert tx.state == LKAS_TX_ACTIVE
      assert tx.tja == 0
      assert tx.cam_lkas_lnv == 0
      assert tx.apply_torque == 90

  def test_active_steers_with_fsc_hud(self):
    tx = _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=537, fsc_lnv=1, fsc_tja=0, fsc_ll=1)
    v243, v440 = _pack(tx)
    assert v243["LINE_NOT_VISIBLE"] == 0
    assert v243["LKAS_REQUEST"] == 537
    assert v243["ERR_BIT_1"] == 0
    assert v440["TJA"] == 0  # exact FSC HUD, not WHITE/GREEN synthesis
    assert v440["LANE_LINES"] == 1
    assert v440["LINE_VISIBLE"] == 0
    assert v440["LINE_NOT_VISIBLE"] == 1

  def test_active_does_not_clamp_to_200(self):
    tx = _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=1088, fsc_lnv=1)
    v243, _ = _pack(tx)
    assert v243["LKAS_REQUEST"] == 1088
    assert v243["LINE_NOT_VISIBLE"] == 0

  def test_auth_loss_zeros_request_keeps_fsc_hud(self):
    active = _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
                   desired=200, fsc_lnv=1)
    paused = _step(False, STEER_ACTIVATION_HOLD_NS + 10_000_000, active.handshake_start_ns,
                   active.state, desired=200, fsc_lnv=1, fsc_tja=2)
    assert paused.state == LKAS_TX_AUTH_PAUSED
    assert paused.apply_torque == 0
    assert paused.tja == 2
    assert paused.handshake_start_ns is None

  def test_auth_restore_reenters_settle_then_active(self):
    paused = _step(False, 100, None, LKAS_TX_ACTIVE, desired=200, fsc_tja=2)
    t1 = 200
    tr = _step(True, t1, paused.handshake_start_ns, paused.state, desired=200, fsc_lnv=1, fsc_tja=2)
    assert tr.state == LKAS_TX_TRANSITION_TO_ACTIVE
    assert tr.apply_torque == 0
    assert tr.tja == 2
    act = _step(True, t1 + STEER_ACTIVATION_HOLD_NS, tr.handshake_start_ns, tr.state,
                desired=200, fsc_lnv=1, fsc_tja=2)
    assert act.state == LKAS_TX_ACTIVE
    assert act.tja == 2
    assert act.apply_torque == 200

  def test_err_zeros_request(self):
    tx = _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_ACTIVE, desired=400, err1=1)
    assert tx.state == LKAS_TX_FAULT
    assert tx.apply_torque == 0
    packer = CANPacker("mazda_2017")
    steer = mazdacan.create_steering_control(packer, _cp(), 3, tx.apply_torque,
                                             _lkas(lnv=1, err1=1), tx_lnv=tx.cam_lkas_lnv)
    v243 = _decode_243(steer[1])
    assert v243["LKAS_REQUEST"] == 0
    assert v243["ERR_BIT_1"] == 1


class TestPackerBackstops:
  def test_err_zeros_even_if_lnv0(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 1, 400, _lkas(lnv=0, err1=1), tx_lnv=0)
    v = _decode_243(msg[1])
    assert v["LKAS_REQUEST"] == 0
    assert v["ERR_BIT_1"] == 1

  def test_lnv1_wire_still_caps(self):
    packer = CANPacker("mazda_2017")
    msg = mazdacan.create_steering_control(packer, _cp(), 13, 1088, _lkas(lnv=1), tx_lnv=1)
    v = _decode_243(msg[1])
    assert v["LINE_NOT_VISIBLE"] == 1
    assert abs(v["LKAS_REQUEST"]) == FSC_LNV1_LKAS_REQUEST_MAX


class TestRouteReplay:
  """One state machine through Routes 34/36/39. latActive includes panda auth."""

  def _replay(self, t_auth, t_fault, fsc_lnv, desired=90, fsc_tja=0):
    start = None
    prev = LKAS_TX_IDLE
    rows = []
    reached_active = False
    stayed_in_settle = False
    pre_auth_nz = False
    forced_tja_while_mads = False
    t0 = int((t_auth - 0.05) * 1e9)
    t1 = int((t_fault + 0.05) * 1e9)
    for ns in range(t0, t1, 10_000_000):
      t = ns / 1e9
      lat = t >= t_auth
      err = 1 if t >= t_fault else 0
      tx = _step(lat, ns, start, prev, desired=desired, fsc_lnv=fsc_lnv, fsc_tja=fsc_tja, err1=err)
      start, prev = tx.handshake_start_ns, tx.state
      if tx.state == LKAS_TX_ACTIVE:
        reached_active = True
      if t >= t_auth + 0.050 and t < t_fault and tx.state == LKAS_TX_TRANSITION_TO_ACTIVE:
        stayed_in_settle = True
      if t < t_auth and tx.apply_torque != 0:
        pre_auth_nz = True
      if lat and err == 0 and tx.tja != fsc_tja:
        forced_tja_while_mads = True
      rows.append((round(t, 3), tx.state, tx.tja, tx.cam_lkas_lnv, tx.lane_lines,
                   tx.apply_torque, lat, err))
    return {
      "reached_active": reached_active,
      "stayed_in_settle": stayed_in_settle,
      "pre_auth_nz": pre_auth_nz,
      "forced_tja_while_mads": forced_tja_while_mads,
      "rows": rows,
    }

  def test_route39(self):
    r = self._replay(112.021, 117.882, fsc_lnv=1, desired=1088, fsc_tja=0)
    assert r["pre_auth_nz"] is False
    assert r["forced_tja_while_mads"] is False
    assert r["stayed_in_settle"] is False
    assert r["reached_active"] is True
    active = [row for row in r["rows"] if row[1] == LKAS_TX_ACTIVE]
    assert active
    assert all(row[2] == 0 and row[3] == 0 and row[4] == 1 for row in active)
    trans = [row for row in r["rows"] if row[1] == LKAS_TX_TRANSITION_TO_ACTIVE]
    assert trans
    assert all(row[2] == 0 and row[5] == 0 for row in trans)

  def test_route36(self):
    r = self._replay(39.039, 62.199, fsc_lnv=1, desired=90, fsc_tja=0)
    assert r["pre_auth_nz"] is False
    assert r["stayed_in_settle"] is False
    assert r["reached_active"] is True
    assert r["forced_tja_while_mads"] is False
    # Route 36 EPS EFFECTIVE with TJA=0 / LANE_LINES=1: forced TJA=4 is not required.
    active = [row for row in r["rows"] if row[1] == LKAS_TX_ACTIVE]
    assert active
    assert all(row[2] == 0 and row[3] == 0 and row[4] == 1 and row[5] != 0 for row in active)

  def test_route34(self):
    r = self._replay(55.303, 61.213, fsc_lnv=1, desired=238, fsc_tja=0)
    assert r["pre_auth_nz"] is False
    assert r["stayed_in_settle"] is False
    assert r["reached_active"] is True
    assert r["forced_tja_while_mads"] is False


class TestOemHudOwnership:
  def test_fsc_tja4_passes_when_fsc_produces_it(self):
    tx = _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=163, fsc_lnv=1, fsc_tja=4, fsc_ll=2, fsc_hud_lnv=1, fsc_vis=0)
    _, v440 = _pack(tx)
    assert v440["TJA"] == 4
    assert v440["LANE_LINES"] == 2
    assert v440["LINE_NOT_VISIBLE"] == 1
    assert v440["LINE_VISIBLE"] == 0

  def test_fsc_tja3_passes_when_fsc_produces_it(self):
    tx = _step(True, 0, None, desired=163, fsc_lnv=1, fsc_tja=3, fsc_ll=2,
               fsc_hud_lnv=0, fsc_vis=1)
    _, v440 = _pack(tx)
    assert tx.state == LKAS_TX_TRANSITION_TO_ACTIVE
    assert v440["TJA"] == 3
    assert v440["LANE_LINES"] == 2
    assert v440["LINE_VISIBLE"] == 1
    assert v440["LINE_NOT_VISIBLE"] == 0

  def test_mads_active_does_not_force_tja4(self):
    tx = _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
               desired=163, fsc_lnv=1, fsc_tja=0, fsc_ll=1)
    _, v440 = _pack(tx)
    assert tx.state == LKAS_TX_ACTIVE
    assert v440["TJA"] == 0  # exact FSC HUD, not TJA=4
    assert v440["LANE_LINES"] == 1


class TestHudOemCopyCases:
  """Prompt section 9 A–E (HUD) + F–G (auth settle)."""

  def _active(self, fsc_ll, desired=400, fsc_tja=0):
    return _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
                 desired=desired, fsc_lnv=1, fsc_ll=fsc_ll, fsc_tja=fsc_tja)

  def test_a_mads_active_fsc_tja2_ll1(self):
    tx = self._active(1, fsc_tja=2)
    _, v440 = _pack(tx)
    assert tx.state == LKAS_TX_ACTIVE
    assert v440["TJA"] == 2
    assert v440["LANE_LINES"] == 1
    assert tx.apply_torque == 400
    assert tx.cam_lkas_lnv == 0

  def test_b_mads_active_fsc_tja0(self):
    tx = self._active(1, fsc_tja=0)
    _, v440 = _pack(tx)
    assert v440["TJA"] == 0  # exact FSC HUD
    assert tx.apply_torque == 400

  def test_c_fsc_lane_line_changes_follow_exactly(self):
    t0 = STEER_ACTIVATION_HOLD_NS
    start = 0
    prev = LKAS_TX_TRANSITION_TO_ACTIVE
    seen = []
    for i, ll in enumerate((0, 1, 2, 0, 2, 1)):
      tx = _step(True, t0 + i * 10_000_000, start, prev, desired=537, fsc_lnv=1,
                 fsc_ll=ll, fsc_tja=2)
      start, prev = tx.handshake_start_ns, tx.state
      _, v440 = _pack(tx)
      assert tx.state == LKAS_TX_ACTIVE
      assert v440["TJA"] == 2
      assert v440["LANE_LINES"] == ll
      assert tx.apply_torque == 537
      assert tx.cam_lkas_lnv == 0
      seen.append(ll)
    assert seen == [0, 1, 2, 0, 2, 1]

  def test_d_mads_state_changes_do_not_alter_tja(self):
    fsc_tja = 2
    idle = _step(False, 0, fsc_tja=fsc_tja, fsc_ll=1)
    settle = _step(True, 0, None, idle.state, desired=200, fsc_tja=fsc_tja, fsc_ll=1)
    active = _step(True, STEER_ACTIVATION_HOLD_NS, settle.handshake_start_ns, settle.state,
                   desired=200, fsc_tja=fsc_tja, fsc_ll=1)
    paused = _step(False, STEER_ACTIVATION_HOLD_NS + 10_000_000, active.handshake_start_ns,
                   active.state, desired=200, fsc_tja=fsc_tja, fsc_ll=1)
    for tx in (idle, settle, active, paused):
      assert tx.tja == fsc_tja
      _, v440 = _pack(tx)
      assert v440["TJA"] == fsc_tja

  def test_e_no_100hz_hud_flag(self):
    for state_fn in (
      lambda: _step(False, 0, fsc_tja=2),
      lambda: _step(True, 0, None, desired=90, fsc_tja=0),
      lambda: _step(True, STEER_ACTIVATION_HOLD_NS, 0, LKAS_TX_TRANSITION_TO_ACTIVE,
                    desired=90, fsc_tja=0),
    ):
      assert state_fn().send_hud_every_frame is False

  def test_f_panda_auth_false_request_zero(self):
    tx = _step(False, 0, desired=1088, fsc_lnv=1)
    assert tx.apply_torque == 0

  def test_g_panda_auth_true_after_settle_resumes(self):
    paused = _step(False, 100, None, LKAS_TX_ACTIVE, desired=200, fsc_tja=0)
    tr = _step(True, 200, paused.handshake_start_ns, paused.state, desired=200)
    assert tr.apply_torque == 0
    act = _step(True, 200 + STEER_ACTIVATION_HOLD_NS, tr.handshake_start_ns, tr.state, desired=200)
    assert act.state == LKAS_TX_ACTIVE
    assert act.apply_torque == 200

  def test_invalid_route3f_tuple_not_created_by_mads(self):
    # Production must not invent (TJA=4, LL=1, VIS=0, LNV=1) solely because MADS is active.
    tx = self._active(1, fsc_tja=0)
    assert not (tx.tja == 4 and tx.lane_lines == 1 and tx.line_visible == 0 and tx.hud_lnv == 1)
    # Nor MADS-created (TJA=3, LL=1, VIS=1, LNV=0) during settle.
    settle = _step(True, 0, None, desired=200, fsc_tja=0, fsc_ll=1)
    assert not (settle.tja == 3 and settle.lane_lines == 1 and settle.line_visible == 1 and settle.hud_lnv == 0)


class TestStateFuzz:
  def test_invariants(self):
    packer = CANPacker("mazda_2017")
    CP = _cp()
    for lat in (False, True):
      for fsc_lnv in (0, 1):
        for err1 in (0, 1):
          for desired in (0, 12, 177, 200, 537, 1088, -1088):
            for fsc_tja in (0, 2, 3, 4):
              for fsc_ll in (0, 1, 2):
                for elapsed in (0, STEER_ACTIVATION_HOLD_NS):
                  start = 0 if lat and err1 == 0 else None
                  tx = _step(lat, elapsed, start, LKAS_TX_IDLE if start is None else LKAS_TX_TRANSITION_TO_ACTIVE,
                             desired=desired, fsc_lnv=fsc_lnv, fsc_tja=fsc_tja, err1=err1, fsc_ll=fsc_ll)
                  steer = mazdacan.create_steering_control(packer, CP, 5, tx.apply_torque,
                                                           _lkas(lnv=fsc_lnv, err1=err1),
                                                           tx_lnv=tx.cam_lkas_lnv)
                  hud = mazdacan.create_alert_command(packer, _lane(tja=fsc_tja, ll=fsc_ll), False, False, tx=tx,
                                                      fsc_raw=_lane_raw(_lane(tja=fsc_tja, ll=fsc_ll)))
                  v243 = _decode_243(steer[1])
                  v440 = _decode_440(hud[1])
                  wire = int(v243["LKAS_REQUEST"])
                  assert tx.lane_lines == fsc_ll
                  assert tx.tja == fsc_tja
                  assert v440["LANE_LINES"] == fsc_ll
                  assert v440["TJA"] == fsc_tja
                  assert tx.send_hud_every_frame is False
                  if err1 or not lat:
                    assert wire == 0
                  if tx.state == LKAS_TX_TRANSITION_TO_ACTIVE:
                    assert wire == 0
                    assert elapsed < STEER_ACTIVATION_HOLD_NS or not lat or err1
                  if tx.state == LKAS_TX_ACTIVE:
                    assert v243["LINE_NOT_VISIBLE"] == 0
                  if err1:
                    assert tx.state == LKAS_TX_FAULT


class TestExactFscHudRelay:
  """Every valid FSC payload is relayed byte-for-byte, independent of MADS."""

  FAMILY_OFF = {
    "TJA": 0, "TJA_TRANSITION": 0, "LANE_LINES": 1,
    "LINE_VISIBLE": 0, "LINE_NOT_VISIBLE": 1,
    "BIT1": 1, "BIT2": 0, "BIT3": 1, "NO_ERR_BIT": 0, "S1": 1, "S1_HBEAM": 0,
  }
  FAMILY_WHITE = dict(FAMILY_OFF, TJA=2)
  OEM_OFF = "4201000000001040"
  OEM_WHITE = "4201000020001040"
  OEM_LL2_TJA4 = "4202000040001040"
  BOOT_S1 = "4361000000000040"
  RARE_LL1 = "4201000030001b40"
  TR_WARN = dict(FAMILY_WHITE, TJA_TRANSITION=3)

  def _raw(self, lane, mads_enabled, fsc_raw=None, mads_available=True):
    return _hud(lane, mads_enabled=mads_enabled, mads_available=mads_available,
                fsc_raw=fsc_raw).hex()

  def _pack(self, lane, mads_enabled, fsc_raw=None):
    return _decode_440(_hud(lane, mads_enabled=mads_enabled, fsc_raw=fsc_raw))

  def test_without_raw_nothing_is_sent(self):
    """No live FSC payload means no 0x440 at all: never a reconstruction, never zeros."""
    packer = CANPacker("mazda_2017")
    assert mazdacan.create_alert_command(
      packer, self.FAMILY_OFF, False, False, mads_enabled=True) is None
    assert mazdacan.create_alert_command(
      packer, self.FAMILY_WHITE, False, False, mads_enabled=False) is None
    # A malformed (non-8-byte) payload is also refused rather than padded or packed.
    assert mazdacan.create_alert_command(
      packer, self.FAMILY_OFF, False, False, mads_enabled=True, fsc_raw=b"\x42\x01\x00") is None
    assert self._raw(self.FAMILY_OFF, mads_enabled=False,
                     fsc_raw=bytes.fromhex(self.OEM_OFF)) == self.OEM_OFF

  def test_b_family_off_mads_on_stays_exact_off(self):
    assert self._raw(self.FAMILY_OFF, mads_enabled=True,
                     fsc_raw=bytes.fromhex(self.OEM_OFF)) == self.OEM_OFF

  def test_c_family_white_mads_off_stays_exact_white(self):
    assert self._raw(self.FAMILY_WHITE, mads_enabled=False,
                     fsc_raw=bytes.fromhex(self.OEM_WHITE)) == self.OEM_WHITE

  def test_d_family_white_mads_on_exact_white(self):
    assert self._raw(self.FAMILY_WHITE, mads_enabled=True,
                     fsc_raw=bytes.fromhex(self.OEM_WHITE)) == self.OEM_WHITE

  def test_e_ll2_tja4_unchanged_regardless_of_mads(self):
    lane = _lane(tja=4, ll=2, lnv=1, vis=0)
    raw = bytes.fromhex(self.OEM_LL2_TJA4)
    for mads in (False, True):
      dat = _hud(lane, mads_enabled=mads, fsc_raw=raw)
      assert dat.hex() == self.OEM_LL2_TJA4
      v = _decode_440(dat)
      assert v["TJA"] == 4
      assert v["LANE_LINES"] == 2

  def test_f_boot_s1_variant_unchanged(self):
    raw = bytes.fromhex(self.BOOT_S1)
    lane = _decode_440(raw)
    for mads in (False, True):
      assert self._raw(lane, mads_enabled=mads, fsc_raw=raw) == self.BOOT_S1

  def test_g_rare_non_family_ll1_unchanged(self):
    raw = bytes.fromhex(self.RARE_LL1)
    lane = _decode_440(raw)
    assert int(lane["TJA"]) == 3
    off = self._raw(lane, mads_enabled=False, fsc_raw=raw)
    on = self._raw(lane, mads_enabled=True, fsc_raw=raw)
    assert off == on == self.RARE_LL1
    v = _decode_440(bytes.fromhex(off))
    assert v["TJA"] == 3
    assert off not in (self.OEM_OFF, self.OEM_WHITE)

  def test_h_tja_transition_warning_variant_unchanged(self):
    baseline = _hud(self.TR_WARN, mads_enabled=True).hex()
    # TR=3 is not the proven pair; MADS must not rewrite it.
    assert baseline != self.OEM_OFF
    assert baseline != self.OEM_WHITE
    assert self._raw(self.TR_WARN, mads_enabled=False) == baseline
    assert self._raw(self.TR_WARN, mads_enabled=True) == baseline

  def test_i_no_payload_is_transformed(self):
    allowed = {self.OEM_OFF, self.OEM_WHITE}
    transformed = []
    for lane, fsc_hex in ((self.FAMILY_OFF, self.OEM_OFF), (self.FAMILY_WHITE, self.OEM_WHITE)):
      for mads in (False, True):
        out = self._raw(lane, mads_enabled=mads, fsc_raw=bytes.fromhex(fsc_hex))
        assert out in allowed
        if out != fsc_hex:
          transformed.append(out)
    assert transformed == []

  def test_no_tja3_tja4_synthesis(self):
    for tja in (3, 4):
      lane = _lane(tja=tja, ll=1)
      for mads in (False, True):
        v = self._pack(lane, mads_enabled=mads)
        assert v["TJA"] == tja

  def test_route41_is_exact_fsc_regardless_of_mads(self):
    seq = [
      (True, 0),
      (False, 2),
      (True, 0),
      (False, 2),
    ]
    for mads, fsc_tja in seq:
      fsc_hex = self.OEM_OFF if fsc_tja == 0 else self.OEM_WHITE
      lane = dict(self.FAMILY_WHITE)
      lane["TJA"] = fsc_tja
      v = self._pack(lane, mads_enabled=mads, fsc_raw=bytes.fromhex(fsc_hex))
      assert v["TJA"] == fsc_tja
      raw = self._raw(lane, mads_enabled=mads, fsc_raw=bytes.fromhex(fsc_hex))
      assert raw == fsc_hex

  def test_mads_unavailable_is_pure_fsc(self):
    cases = (
      (self.FAMILY_OFF, bytes.fromhex(self.OEM_OFF), self.OEM_OFF),
      (self.FAMILY_WHITE, bytes.fromhex(self.OEM_WHITE), self.OEM_WHITE),
      (_lane(tja=4, ll=2, lnv=1, vis=0), bytes.fromhex(self.OEM_LL2_TJA4), self.OEM_LL2_TJA4),
      (_decode_440(bytes.fromhex(self.RARE_LL1)), bytes.fromhex(self.RARE_LL1), self.RARE_LL1),
    )
    for lane, raw, expect in cases:
      for enabled in (False, True):
        dat = _hud(lane, mads_enabled=enabled, mads_available=False, fsc_raw=raw)
        assert dat.hex() == expect

  def test_err_bit_is_not_canonicalized_into_family(self):
    for base, mads, fsc_hex in (
      (self.FAMILY_OFF, False, self.OEM_OFF),
      (self.FAMILY_WHITE, True, self.OEM_WHITE),
    ):
      faulted = dict(base, ERR_BIT=1)
      raw = _lane_raw(faulted)
      assert raw.hex() != fsc_hex
      assert raw not in mazdacan.OEM_LL1_HUD_FAMILY
      assert raw not in mazdacan.OEM_LL1_HUD_GREEN_VARIANTS
      dat = _hud(faulted, mads_enabled=mads, fsc_raw=raw)
      assert dat.hex() not in (self.OEM_OFF, self.OEM_WHITE)
      assert dat == raw
      v = _decode_440(dat)
      assert v["ERR_BIT"] == 1
      assert v["TJA"] == base["TJA"]
      assert v["LANE_LINES"] == 1

  def test_unnamed_bit_mutations_are_not_family(self):
    pair = {bytes.fromhex(self.OEM_OFF), bytes.fromhex(self.OEM_WHITE)}
    mutated = []
    for base in pair:
      base_d = _decode_440(base)
      for byte_i in range(8):
        for bit_i in range(8):
          raw = bytearray(base)
          raw[byte_i] ^= 1 << bit_i
          raw_b = bytes(raw)
          if raw_b in pair:
            continue
          d = _decode_440(raw_b)
          named_same = all(int(d.get(s, 0)) == int(base_d.get(s, 0))
                           for s in mazdacan.CAM_LANEINFO_SIGNALS)
          if named_same:
            mutated.append(raw_b)
    assert len(mutated) == 80
    eligible = 0
    rewrites = 0
    for raw in mutated:
      assert raw not in pair
      for mads in (False, True):
        out = _hud(_decode_440(raw), mads_enabled=mads, fsc_raw=raw)
        if raw in mazdacan.OEM_LL1_HUD_FAMILY:
          eligible += 1
        if out in pair:
          rewrites += 1
        assert out == raw
    assert eligible == 0
    assert rewrites == 0

  def test_named_non_family_not_eligible(self):
    cases = (
      dict(self.FAMILY_OFF, TJA_TRANSITION=1),
      dict(self.FAMILY_WHITE, TJA_TRANSITION=3),
      dict(self.FAMILY_OFF, HANDS_ON_STEER_WARN=1),
      dict(self.FAMILY_OFF, S1=0),
      _lane(tja=3, ll=1),
      _lane(tja=1, ll=1),
    )
    pair = {self.OEM_OFF, self.OEM_WHITE}
    for lane in cases:
      raw = _lane_raw(lane)
      assert raw.hex() not in pair
      for mads in (False, True):
        out = _hud(lane, mads_enabled=mads, fsc_raw=raw)
        assert out == raw
        assert out.hex() not in pair

  def test_carstate_latches_exact_bus2_payload(self):
    from opendbc.car import gen_empty_fingerprint, structs
    from opendbc.car.can_definitions import CanData
    from opendbc.car.mazda.interface import CarInterface
    from opendbc.car.mazda.values import CAR

    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False,
                                 is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                       alpha_long=False, is_release_sp=False, docs=False)
    ci = CarInterface(CP, CP_SP)
    mutated = bytes.fromhex("4601000000001040")
    ci.update([(10_000_000, [CanData(0x440, mutated, 2)])])
    assert ci.CS.cam_laneinfo_raw == mutated
    cc = structs.CarControl()
    cc_sp = structs.CarControlSP()
    cc_sp.mads.available = True
    cc_sp.mads.enabled = True
    _, sends = ci.apply(cc.as_reader(), cc_sp, 10_000_000)
    hud = next(s for s in sends if s[0] == 0x440)
    assert bytes(hud[1]) == mutated


assert TJA3_ACTIVATION_HOLD_NS == STEER_ACTIVATION_HOLD_NS
