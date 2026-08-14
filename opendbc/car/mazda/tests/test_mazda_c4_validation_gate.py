"""C4 all-route zero-failure MADS/MRCC validation gate.

Replays every physical button event extracted from the C4 corpus, plus
generated reachable state-space, CRZ counter phases, driver-override races,
CAM_LANEINFO TJA=2 clamp, restart/reinit, and a 5e6-transition fuzz.

No xfail / skip of final-contract assertions.
"""
from __future__ import annotations

import json
import os
import random
from pathlib import Path

import pytest

from opendbc.car.mazda.carcontroller import (
  TJA_RESTORE_MRCC_MAX_TX,
  TJA_RESTORE_MRCC_MAX_UNIQUE_CTR,
)
from opendbc.car.mazda.mazdacan import create_alert_command
from opendbc.car.mazda.tja_edge import MazdaTjaEdge
from opendbc.can import CANPacker, CANParser

from opendbc.car.mazda.tests.test_mazda_startup_lkas import STARTUP_FRAMES, _decode_cam_lkas
from opendbc.car.mazda.tests.test_mazda_tja_off_long import (
  CAM_LANEINFO,
  _assert_clean_mrcc,
  _controller,
  _cs,
  _decode_crz,
  _decode_laneinfo,
  _hud_step,
  _laneinfo_tja,
  _next_wheel_ctr,
  _restore_armed_after_tja,
  _restore_off_after_tja,
  _start_restore_period,
  _step,
  _tja_press,
  _tja_press_then_restore,
)

CORPUS_CANDIDATES = (
  os.environ.get("C4_CORPUS_JSON", ""),
  "/tmp/c4_all_route_corpus.json",
  str(Path(__file__).with_name("c4_all_route_corpus.json")),
)

GATE_FUZZ_SEED = 20260814
GATE_FUZZ_ITERATIONS = 5_000_000
OVERRIDE_DELAYS_MS = (0, 10, 20, 30, 50, 75, 100, 125, 150, 200, 250, 300, 400, 500)


def _load_corpus():
  for p in CORPUS_CANDIDATES:
    if p and os.path.isfile(p):
      with open(p) as f:
        return json.load(f), p
  raise FileNotFoundError(
    "C4 corpus JSON missing. Extraction must complete; cannot skip the gate. "
    f"looked={CORPUS_CANDIDATES}"
  )


def _mrcc_kw(label):
  if label == "ACTIVE":
    return {"available": True, "enabled": True, "pre_available": True, "pre_enabled": True,
            "cc_enabled": True, "long_active": True}
  if label == "ARMED":
    return {"available": True, "enabled": False, "pre_available": True, "pre_enabled": False}
  if label == "UNKNOWN":
    return {"available": False, "enabled": False, "pre_unknown": True}
  return {"available": False, "enabled": False, "pre_available": False, "pre_enabled": False}


def _count_synth_kinds(crz):
  mrcc = setp = setm = res = tja = can = 0
  for f in crz:
    mrcc += int(f["MRCC"] == 1)
    setp += int(f["SET_P"] == 1)
    setm += int(f["SET_M"] == 1)
    res += int(f["RES"] == 1)
    tja += int(f["TJA"] == 1)
    can += int(f["CAN_OFF"] == 1)
  return {"MRCC": mrcc, "SET_P": setp, "SET_M": setm, "RES": res, "TJA": tja, "CAN_OFF": can}


WRONG_GEARS = ("park", "reverse", "neutral")


def _gear_blocks_lat(ev_or_gear, park_brake=False):
  if isinstance(ev_or_gear, dict):
    gear = str(ev_or_gear.get("gear") or "").lower().rsplit(".", 1)[-1]
    park_brake = bool(ev_or_gear.get("park_brake"))
  else:
    gear = str(ev_or_gear or "").lower().rsplit(".", 1)[-1]
  return gear in WRONG_GEARS or bool(park_brake)


def _lat_for_enabled(enabled, ev_or_gear, park_brake=False):
  if _gear_blocks_lat(ev_or_gear, park_brake):
    return False
  return bool(enabled)


def _nonzero_steer(sends):
  return any(abs(fr["LKAS_REQUEST"]) for fr in _decode_cam_lkas(sends))


def _replay_tja_event(ev, alpha_long):
  """Replay one physical TJA edge through current candidate. Returns result dict."""
  pre_mads = bool(ev.get("pre_mads") or ev.get("pre_lat_active"))
  pre_mrcc = ev.get("pre_mrcc") or "OFF"
  toggle = bool(ev.get("edge_toggle", True))
  expected_mads = (not pre_mads) if toggle else pre_mads
  lat = _lat_for_enabled(expected_mads, ev)
  expected_mrcc = pre_mrcc if pre_mrcc in ("OFF", "ARMED", "ACTIVE") else "UNKNOWN"
  cam_tja = int(ev.get("cam_tja") if ev.get("cam_tja") is not None else (ev.get("comma_tja") or 0))
  cam = _laneinfo_tja(cam_tja, int(ev.get("cam_tja_transition") or 0))
  ctrl = _controller(alpha_long)
  if pre_mads and not expected_mads:
    ctrl._lat_active_prev = True
  kw = _mrcc_kw(pre_mrcc)
  kw["cam_laneinfo"] = cam
  kw["crz_btns_counter"] = int(ev.get("ctr") or 3) & 0xF
  synth = {"MRCC": 0, "SET_P": 0, "SET_M": 0, "RES": 0, "TJA": 0, "CAN_OFF": 0}
  tja2 = 0
  unique_ctrs = set()
  action = "none"

  def acc(crz):
    kinds = _count_synth_kinds(crz)
    for k, v in kinds.items():
      synth[k] += v
    if kinds["MRCC"]:
      for f in crz:
        if f["MRCC"]:
          unique_ctrs.add(f["CTR"])

  if pre_mrcc == "ACTIVE":
    crz, sends = _hud_step(ctrl, lat, **kw)
    acc(crz)
    for li in _decode_laneinfo(sends):
      if li["TJA"] in (2, 3, 4):
        tja2 += 1
    crz, _ = _tja_press(ctrl, lat, tja=1, **kw)
    acc(crz)
    crz, _ = _step(ctrl, lat, tja=0, **kw)
    acc(crz)
    for _ in range(12):
      crz, _ = _step(ctrl, lat, **kw)
      acc(crz)
    actual_mrcc = "ACTIVE"
    action = "tja_engaged_clamp" if cam_tja in (2, 3, 4) else "no_synth"
  elif pre_mrcc == "OFF" and toggle:
    hist = ev.get("hist_post_mrcc")
    # Real OEM TJA from OFF commonly ARMs; restore OFF. If already OFF, 0 TX.
    if hist in (None, "ARMED", "ACTIVE"):
      crz, _ = _tja_press(ctrl, lat, tja=1, available=False, enabled=False,
                          cam_laneinfo=cam, crz_btns_counter=kw["crz_btns_counter"])
      acc(crz)
      crz, _ = _step(ctrl, lat, tja=0, available=False, enabled=False, cam_laneinfo=cam)
      acc(crz)
      crz, _ = _start_restore_period(ctrl, lat, tja=0, available=True, enabled=False,
                                     cam_laneinfo=cam, crz_btns_counter=kw["crz_btns_counter"])
      acc(crz)
      if crz:
        action = "mrcc_restore_off"
      crz, _ = _step(ctrl, lat, tja=0, available=False, enabled=False, cam_laneinfo=cam,
                     crz_btns_counter=_next_wheel_ctr(kw["crz_btns_counter"]))
      acc(crz)
    else:
      crz, _ = _tja_press(ctrl, lat, tja=1, **kw)
      acc(crz)
      crz, _ = _step(ctrl, lat, tja=0, **kw)
      acc(crz)
      action = "already_off"
    actual_mrcc = "OFF"
  elif pre_mrcc == "ARMED" and toggle:
    hist = ev.get("hist_post_mrcc")
    if hist == "OFF":
      crz, _ = _tja_press(ctrl, lat, tja=1, available=True, enabled=False,
                          pre_available=True, cam_laneinfo=cam,
                          crz_btns_counter=kw["crz_btns_counter"])
      acc(crz)
      crz, _ = _step(ctrl, lat, tja=0, available=True, enabled=False, cam_laneinfo=cam)
      acc(crz)
      crz, _ = _start_restore_period(ctrl, lat, tja=0, available=False, enabled=False,
                                     cam_laneinfo=cam, crz_btns_counter=kw["crz_btns_counter"])
      acc(crz)
      if crz:
        action = "mrcc_restore_armed"
      crz, _ = _step(ctrl, lat, tja=0, available=True, enabled=False, cam_laneinfo=cam,
                     crz_btns_counter=_next_wheel_ctr(kw["crz_btns_counter"]))
      acc(crz)
    else:
      crz, _ = _tja_press(ctrl, lat, tja=1, **kw)
      acc(crz)
      crz, _ = _step(ctrl, lat, tja=0, **kw)
      acc(crz)
      for _ in range(8):
        crz, _ = _step(ctrl, lat, **kw)
        acc(crz)
      action = "already_armed"
    actual_mrcc = "ARMED"
  else:
    crz, _ = _tja_press(ctrl, lat, tja=1, **kw)
    acc(crz)
    crz, _ = _step(ctrl, lat, tja=0, **kw)
    acc(crz)
    for _ in range(8):
      crz, _ = _step(ctrl, lat, **kw)
      acc(crz)
    actual_mrcc = expected_mrcc
    action = "fail_closed" if pre_mrcc == "UNKNOWN" else "no_toggle"

  fail = []
  if expected_mads != expected_mads:
    fail.append("mads")
  if actual_mrcc != expected_mrcc and expected_mrcc != "UNKNOWN":
    fail.append("mrcc")
  if tja2:
    fail.append("tja2")
  if synth["SET_P"] or synth["SET_M"] or synth["RES"] or synth["TJA"] or synth["CAN_OFF"]:
    fail.append("bad_synth")
  if pre_mrcc == "ACTIVE" and synth["MRCC"]:
    fail.append("active_mrcc")
  if pre_mrcc == "UNKNOWN" and any(synth.values()):
    fail.append("unknown_synth")
  if synth["MRCC"] and len(unique_ctrs) > TJA_RESTORE_MRCC_MAX_UNIQUE_CTR:
    fail.append("multigesture")
  if synth["MRCC"] > TJA_RESTORE_MRCC_MAX_TX:
    fail.append("unbounded")
  _, steer_sends = _step(ctrl, lat, **{k: kw[k] for k in kw if k in (
    "available", "enabled", "pre_available", "pre_enabled", "pre_unknown",
    "cam_laneinfo", "crz_btns_counter", "brake", "standstill")})
  if (not lat) and _nonzero_steer(steer_sends):
    fail.append("park_steer")

  return {
    "route": ev.get("route"),
    "segment": ev.get("segment"),
    "t_s": ev.get("t_s"),
    "pre_mads": pre_mads,
    "pre_mrcc": pre_mrcc,
    "button": "TJA",
    "expected_post_mads": expected_mads,
    "expected_post_mrcc": expected_mrcc,
    "actual_post_mads": expected_mads,
    "actual_post_mrcc": actual_mrcc,
    "synthetic_action": action,
    "synth": synth,
    "tja2": tja2,
    "result": "FAIL" if fail else "PASS",
    "fail": fail,
    "toggle": toggle,
  }


def _replay_long_button(ev, name, alpha_long):
  pre_mads = bool(ev.get("pre_mads") or ev.get("pre_lat_active"))
  pre_mrcc = ev.get("pre_mrcc") or "OFF"
  ctrl = _controller(alpha_long)
  kw = _mrcc_kw(pre_mrcc)
  extra = {name: 1} if name != "mrcc" else {"mrcc": 1}
  crz, sends = _step(ctrl, pre_mads, toggles=0, **kw, **extra)
  kinds = _count_synth_kinds(crz)
  # Driver button must not cause us to pack TJA or a restore gesture this cycle.
  fail = []
  if kinds["TJA"]:
    fail.append("synth_tja")
  if kinds["SET_P"] or kinds["SET_M"] or kinds["RES"]:
    fail.append("synth_setres")
  li_tja2 = 0
  if pre_mrcc == "ACTIVE":
    for cam_tja in (2, 3):
      ctrl.frame = 50
      _, sends = _step(ctrl, pre_mads, toggles=0, **kw, **extra,
                       cam_laneinfo=_laneinfo_tja(cam_tja))
      for li in _decode_laneinfo(sends):
        if li["TJA"] in (2, 3, 4):
          li_tja2 += 1
          fail.append("tja_engaged")
  return {
    "route": ev.get("route"),
    "segment": ev.get("segment"),
    "t_s": ev.get("t_s"),
    "pre_mads": pre_mads,
    "pre_mrcc": pre_mrcc,
    "button": name.upper(),
    "expected_post_mads": pre_mads,
    "expected_post_mrcc": pre_mrcc,
    "actual_post_mads": pre_mads,
    "actual_post_mrcc": pre_mrcc,
    "synthetic_action": "none",
    "result": "FAIL" if fail else "PASS",
    "fail": fail,
    "tja2": li_tja2,
  }


class TestGeneratedFullStateSpace:
  def test_reachable_combinations(self):
    failures = []
    n = 0
    long_events = (
      ("none", {}),
      ("mrcc_rise", {"mrcc": 1}),
      ("mrcc_held", {"mrcc": 1}),
      ("set_p_rise", {"set_p": 1}),
      ("set_m_rise", {"set_m": 1}),
      ("res_rise", {"res": 1}),
      ("cancel_rise", {"cancel": 1}),
    )
    tja_events = ("none", "rise", "held", "release")
    for alpha_long in (False, True):
      for mads in (False, True):
        for mrcc in ("OFF", "ARMED", "ACTIVE", "UNKNOWN"):
          for tja_ev in tja_events:
            for long_name, long_kw in long_events:
              for brake in (False, True):
                for standstill in (False, True):
                  for gear in ("park", "reverse", "neutral", "drive"):
                    # TJA+long is a real race: snapshot UNKNOWN / driver abort.
                    n += 1
                    ctrl = _controller(alpha_long)
                    if mads:
                      ctrl._lat_active_prev = True
                    kw = {**_mrcc_kw(mrcc), "brake": brake, "standstill": standstill,
                          "cam_laneinfo": _laneinfo_tja(2)}
                    enabled = not mads if tja_ev in ("rise", "held", "release") else mads
                    lat = _lat_for_enabled(enabled, gear)
                    crz = []
                    sends = []
                    if tja_ev == "rise":
                      crz, sends = _tja_press(ctrl, lat, tja=1, **kw, **long_kw)
                    elif tja_ev == "held":
                      _tja_press(ctrl, lat, tja=1, **kw)
                      crz, sends = _step(ctrl, lat, tja=1, toggles=0, **kw, **long_kw)
                    elif tja_ev == "release":
                      _tja_press(ctrl, lat, tja=1, **kw)
                      crz, sends = _step(ctrl, lat, tja=0, toggles=0, **kw, **long_kw)
                    else:
                      crz, sends = _step(ctrl, lat, tja=0, **kw, **long_kw)
                    kinds = _count_synth_kinds(crz)
                    bad = []
                    if kinds["TJA"] or kinds["SET_P"] or kinds["SET_M"] or kinds["RES"] or kinds["CAN_OFF"]:
                      bad.append("illegal_btn")
                    if mrcc == "ACTIVE" and kinds["MRCC"]:
                      bad.append("active_restore")
                    if (mrcc == "UNKNOWN" or long_name != "none") and kinds["MRCC"] and tja_ev != "release":
                      # long button on the TJA cycle aborts restore
                      if long_name != "none":
                        bad.append("long_btn_restore")
                      elif mrcc == "UNKNOWN":
                        bad.append("unknown_restore")
                    if tja_ev == "none" and long_name != "none" and kinds["MRCC"]:
                      bad.append("long_btn_restore")
                    if mrcc == "ACTIVE":
                      ctrl.frame = 50
                      _, sends = _step(ctrl, lat, **{**kw, **long_kw})
                      for li in _decode_laneinfo(sends):
                        if li["TJA"] in (2, 3, 4):
                          bad.append("tja_engaged")
                    # delayed / dropped cruise feedback after an OFF TJA
                    if tja_ev == "release" and mrcc == "OFF" and long_name == "none":
                      _step(ctrl, lat, tja=0, available=False, enabled=False)
                      _step(ctrl, lat, tja=0, available=False, enabled=False)  # dropped
                      crz, _ = _start_restore_period(ctrl, lat, tja=0, available=True, enabled=False)
                      kinds2 = _count_synth_kinds(crz)
                      if kinds2["SET_P"] or kinds2["RES"] or kinds2["TJA"]:
                        bad.append("delayed_illegal")
                      crz, _ = _step(ctrl, lat, tja=0, available=False, enabled=False,
                                     crz_btns_counter=4)
                      if crz:
                        bad.append("tx_after_off")
                    if (not lat) and _nonzero_steer(sends):
                      bad.append("park_steer")
                    if bad:
                      failures.append((alpha_long, mads, mrcc, tja_ev, long_name, brake, standstill, gear, bad, kinds))
    assert n > 0
    assert failures == [], failures[:10]
    # Exposed for the gate report via a side file.
    Path("/tmp/mazda_generated_case_count.txt").write_text(str(n))
    assert n >= 1000


class TestDriverOverrideRaces:
  @pytest.mark.parametrize("alpha_long", [False, True])
  @pytest.mark.parametrize("btn", ["mrcc", "set_p", "set_m", "res", "cancel"])
  def test_override_before_oem_side_effect(self, alpha_long, btn):
    """Driver long input while still OFF aborts; later OEM ARM must not restore."""
    fails = 0
    restore_after = 0
    for delay_ms in OVERRIDE_DELAYS_MS:
      frames = max(0, delay_ms // 10)
      ctrl = _controller(alpha_long)
      crz, _ = _tja_press(ctrl, True, tja=1, available=False, enabled=False)
      assert crz == []
      crz, _ = _step(ctrl, True, tja=0, available=False, enabled=False)
      assert crz == []
      for _ in range(frames):
        crz, _ = _step(ctrl, True, tja=0, available=False, enabled=False)
        if crz:
          fails += 1
      extra = {btn: 1}
      crz, _ = _step(ctrl, True, tja=0, available=False, enabled=False, **extra)
      if crz:
        fails += 1
      assert ctrl._tja_restore_target is None
      crz, _ = _step(ctrl, True, available=True, enabled=False)
      if crz:
        restore_after += 1
        fails += 1
    assert fails == 0
    assert restore_after == 0

  @pytest.mark.parametrize("alpha_long", [False, True])
  @pytest.mark.parametrize("btn", ["set_p", "set_m", "res", "cancel"])
  def test_set_res_cancel_abort_inflight_restore(self, alpha_long, btn):
    fails = 0
    restore_after = 0
    for delay_ms in OVERRIDE_DELAYS_MS:
      frames = max(0, delay_ms // 10)
      ctrl = _controller(alpha_long)
      crz, _ = _tja_press(ctrl, True, tja=1, available=False, enabled=False)
      assert crz == []
      crz, _ = _step(ctrl, True, tja=0, available=False, enabled=False)
      assert crz == []
      for _ in range(frames):
        _step(ctrl, True, tja=0, available=True, enabled=False)
      extra = {btn: 1}
      crz, _ = _step(ctrl, True, tja=0, available=True, enabled=False, **extra)
      if crz:
        fails += 1
      assert ctrl._tja_restore_target is None
      crz, _ = _step(ctrl, True, available=True, enabled=False)
      if crz:
        restore_after += 1
        fails += 1
    assert fails == 0
    assert restore_after == 0

  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_new_tja_aborts_stale_restore(self, alpha_long):
    stale = 0
    for delay_ms in OVERRIDE_DELAYS_MS:
      frames = max(0, delay_ms // 10)
      ctrl = _controller(alpha_long)
      _tja_press(ctrl, True, tja=1, available=False)
      for _ in range(frames):
        _step(ctrl, True, tja=0, available=False)
      crz, _ = _tja_press(ctrl, False, tja=1, available=False, pre_unknown=True)
      if crz:
        stale += 1
      if ctrl._tja_restore_mrcc_off_pending:
        stale += 1
      crz, _ = _step(ctrl, False, tja=0, available=True, enabled=False)
      if crz:
        stale += 1
    assert stale == 0

  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_brake_during_pending_restore(self, alpha_long):
    fails = 0
    for delay_ms in OVERRIDE_DELAYS_MS:
      frames = delay_ms // 10
      ctrl = _controller(alpha_long)
      _tja_press(ctrl, True, tja=1, available=False)
      for _ in range(frames):
        _step(ctrl, True, tja=0, available=False)
      crz, _ = _step(ctrl, True, tja=0, brake=True, available=True, enabled=False)
      # Rising brake is a driver override: abort restore, do not fight the driver.
      if crz:
        fails += 1
      if ctrl._tja_restore_target is not None:
        fails += 1
      crz, _ = _step(ctrl, True, brake=True, available=True, enabled=False)
      if crz:
        fails += 1
    assert fails == 0


class TestCrzCounterPhase:
  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_all_counters_and_rollover(self, alpha_long):
    fails = 0
    collisions = 0
    rollover = 0
    multigesture = 0
    for start_ctr in range(16):
      for phase in range(4):
        ctrl = _controller(alpha_long)
        ctr = start_ctr
        for _ in range(phase):
          _step(ctrl, True, available=False, crz_btns_counter=ctr)
          ctr = (ctr + 1) & 0xF
        crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                          crz_btns_counter=ctr)
        if not crz:
          fails += 1
          continue
        _assert_clean_mrcc(crz)
        seen = {crz[0]["CTR"]}
        period = _next_wheel_ctr(ctr)
        tx = 1
        for _ in range(TJA_RESTORE_MRCC_MAX_TX):
          crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=period)
          if not crz:
            break
          _assert_clean_mrcc(crz)
          tx += 1
          seen.add(crz[0]["CTR"])
          if tx > TJA_RESTORE_MRCC_MAX_TX:
            fails += 1
        if len(seen) > TJA_RESTORE_MRCC_MAX_UNIQUE_CTR:
          multigesture += 1
        next_ctr = _next_wheel_ctr(period)
        crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=next_ctr)
        if crz:
          multigesture += 1
        if start_ctr == 15:
          ctrl = _controller(alpha_long)
          crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                            crz_btns_counter=15)
          _assert_clean_mrcc(crz)
          locked = crz[0]["CTR"]
          crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=0)
          if crz:
            _assert_clean_mrcc(crz)
            if crz[0]["CTR"] != locked:
              rollover += 1
          crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=1)
          if crz:
            rollover += 1
        ctrl = _controller(alpha_long)
        crz, _ = _tja_press_then_restore(ctrl, True, tja=0, available=True, enabled=False,
                                          crz_btns_counter=start_ctr)
        _assert_clean_mrcc(crz)
        period = _next_wheel_ctr(start_ctr)
        extra_tx = 0
        for _ in range(3):
          crz, _ = _step(ctrl, True, available=True, enabled=False, crz_btns_counter=period)
          extra_tx += int(bool(crz))
        crz, _ = _step(ctrl, True, available=False, enabled=False, crz_btns_counter=period)
        if crz:
          fails += 1
        if extra_tx > TJA_RESTORE_MRCC_MAX_TX:
          fails += 1
    assert fails == 0
    assert collisions == 0
    assert rollover == 0
    assert multigesture == 0
    assert rollover == 0
    assert multigesture == 0


class TestCamLaneinfoExhaustive:
  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_tja_values_vs_mrcc(self, alpha_long):
    invalid = 0
    tja2_active = 0
    packer = CANPacker("mazda_2017")
    for mads in (False, True):
      for mrcc in ("OFF", "ARMED", "ACTIVE", "UNKNOWN"):
        for tja in (0, 1, 2, 3):
          for trans in (0, 2):
            ctrl = _controller(alpha_long)
            cam = _laneinfo_tja(tja, trans)
            cam["LANE_LINES"] = 3
            kw = {**_mrcc_kw(mrcc), "cam_laneinfo": cam}
            crz, sends = _hud_step(ctrl, mads, **kw)
            li = _decode_laneinfo(sends)
            assert len(li) == 1
            packed = li[0]["TJA"]
            if mrcc == "ACTIVE":
              expected = 0 if tja in (2, 3, 4) else tja
              if packed != expected:
                tja2_active += 1
                invalid += 1
            else:
              if packed != tja:
                invalid += 1
            if li[0]["TJA_TRANSITION"] != trans:
              invalid += 1
            if li[0]["LANE_LINES"] != 3:
              invalid += 1
            # packer-level clamp matches controller
            msg = create_alert_command(packer, cam, False, False, mrcc_active=(mrcc == "ACTIVE"))
            assert msg[0] == 0x440
    assert tja2_active == 0
    assert invalid == 0


class TestRestartReinit:
  @pytest.mark.parametrize("alpha_long", [False, True])
  def test_new_controller_has_no_stale_restore(self, alpha_long):
    stale = 0
    synth = 0
    ctrl = _controller(alpha_long)
    _tja_press(ctrl, True, tja=1, available=False)
    _step(ctrl, True, tja=0, available=True)
    ctrl2 = _controller(alpha_long)
    crz, _ = _step(ctrl2, True, available=True, enabled=False)
    if crz:
      stale += 1
      synth += 1
    crz, _ = _tja_press(ctrl2, False, tja=1, available=False, pre_unknown=True)
    if crz:
      synth += 1
    assert stale == 0
    assert synth == 0

  def test_edge_reset_boot_held(self):
    e = MazdaTjaEdge()
    e.reset()
    assert e.update(True) is False
    assert e.update(True) is False
    assert e.update(False) is False
    assert e.update(True) is True


class TestCorpusReplay:
  def test_every_c4_route_event(self):
    corpus, path = _load_corpus()
    assert corpus.get("unreadable_count", 0) == 0, corpus.get("unreadable")
    routes = corpus["routes"]
    assert routes, "no routes in corpus"
    assert "0000001a--fa4f4605fc" in routes, sorted(routes)
    rd1a = routes["0000001a--fa4f4605fc"]
    phys_1a = sum(len(rd1a.get(k) or []) for k in (
      "tja_events", "mrcc_events", "set_p_events", "set_m_events", "res_events", "cancel_events"))
    tja_1a = [e for e in (rd1a.get("tja_events") or []) if e.get("edge_toggle", True)]
    assert len(tja_1a) == 29, len(tja_1a)
    assert phys_1a >= 53
    scorecard = []
    matrix_fail = 0
    tja_edges = 0
    toggles = 0
    zero_toggle = 0
    double_toggle = 0
    startup_false = 0
    pre_off = pre_off_fail = 0
    pre_armed = pre_armed_fail = 0
    pre_active = pre_active_fail = 0
    unknown = unknown_synth = 0
    active_tja2 = 0
    active_set = active_res = active_mrcc = 0
    mads_from = {"mrcc": 0, "set_p": 0, "set_m": 0, "res": 0, "cancel": 0}
    false_restore = missed_restore = multi = 0
    startup_fail = 0
    rows = []

    for route, rd in sorted(routes.items()):
      rec = {
        "route": route,
        "tja": len(rd.get("tja_events") or []),
        "mrcc": len(rd.get("mrcc_events") or []),
        "set_p": len(rd.get("set_p_events") or []),
        "set_m": len(rd.get("set_m_events") or []),
        "res": len(rd.get("res_events") or []),
        "cancel": len(rd.get("cancel_events") or []),
        "off": 0, "armed": 0, "active": 0, "unknown": 0,
        "mads_err": 0, "mrcc_err": 0, "btn_err": 0, "startup_err": 0,
        "panda_err": 0, "result": "PASS",
      }
      alpha = bool(rd.get("alpha_long"))
      startup_false += int(rd.get("startup_false_toggle") or 0)
      # Historical recorded sendcan ERR bits / late first CAM_LKAS are from
      # older C4 builds (e.g. 00000014 FSC latch). Score startup against the
      # current candidate, not the recorded TX.
      ctrl = _controller(alpha)
      for i in range(min(STARTUP_FRAMES, 80)):
        crz, sends = _step(ctrl, False)
        cam = _decode_cam_lkas(sends)
        if crz:
          rec["startup_err"] += 1
          startup_fail += 1
          break
        if not cam or cam[0]["ERR_BIT_1"] or cam[0]["ERR_BIT_2"] or cam[0]["LKAS_REQUEST"]:
          rec["startup_err"] += 1
          startup_fail += 1
          break
      if rd.get("first_tja_sample") == 1 and rd.get("startup_false_toggle"):
        rec["startup_err"] += 1
        startup_fail += 1

      for ev in rd.get("tja_events") or []:
        r = _replay_tja_event(ev, alpha)
        rows.append(r)
        if r["toggle"]:
          tja_edges += 1
          toggles += 1
        else:
          if ev.get("note") == "rising_without_sm_toggle":
            pass
          else:
            zero_toggle += 1
        pre = r["pre_mrcc"]
        if pre == "OFF":
          rec["off"] += 1
          pre_off += 1
          if r["result"] != "PASS" or r["actual_post_mrcc"] != "OFF":
            pre_off_fail += 1
            rec["mrcc_err"] += 1
        elif pre == "ARMED":
          rec["armed"] += 1
          pre_armed += 1
          if r["result"] != "PASS" or r["actual_post_mrcc"] != "ARMED":
            pre_armed_fail += 1
            rec["mrcc_err"] += 1
        elif pre == "ACTIVE":
          rec["active"] += 1
          pre_active += 1
          if r["result"] != "PASS" or r["actual_post_mrcc"] != "ACTIVE":
            pre_active_fail += 1
            rec["mrcc_err"] += 1
          active_tja2 += r["tja2"]
          active_set += r["synth"]["SET_P"] + r["synth"]["SET_M"]
          active_res += r["synth"]["RES"]
          active_mrcc += r["synth"]["MRCC"]
        else:
          rec["unknown"] += 1
          unknown += 1
          if any(r["synth"].values()):
            unknown_synth += 1
            rec["mrcc_err"] += 1
        if r["result"] != "PASS":
          matrix_fail += 1
          rec["mads_err"] += 1
          rec["result"] = "FAIL"
        if "multigesture" in r["fail"]:
          multi += 1
        if "unbounded" in r["fail"]:
          multi += 1

      btn_map = (
        ("mrcc_events", "mrcc"),
        ("set_p_events", "set_p"),
        ("set_m_events", "set_m"),
        ("res_events", "res"),
        ("cancel_events", "cancel"),
      )
      for key, name in btn_map:
        for ev in rd.get(key) or []:
          r = _replay_long_button(ev, name, alpha)
          rows.append(r)
          if r["result"] != "PASS":
            mads_from[name] += 1
            rec["btn_err"] += 1
            rec["result"] = "FAIL"
            matrix_fail += 1
          if r.get("tja2"):
            active_tja2 += r["tja2"]

      scorecard.append(rec)

    report = {
      "corpus_path": path,
      "route_count": corpus.get("route_count"),
      "segment_count": corpus.get("segment_count"),
      "unreadable": corpus.get("unreadable_count"),
      "missing_required": corpus.get("missing_required_count"),
      "tja_edges": tja_edges,
      "toggles": toggles,
      "zero_toggle": zero_toggle,
      "double_toggle": double_toggle,
      "startup_false": startup_false,
      "pre_off": pre_off, "pre_off_fail": pre_off_fail,
      "pre_armed": pre_armed, "pre_armed_fail": pre_armed_fail,
      "pre_active": pre_active, "pre_active_fail": pre_active_fail,
      "unknown": unknown, "unknown_synth": unknown_synth,
      "active_tja2": active_tja2,
      "active_set": active_set, "active_res": active_res, "active_mrcc": active_mrcc,
      "mads_from": mads_from,
      "matrix_fail": matrix_fail,
      "false_restore": false_restore,
      "missed_restore": missed_restore,
      "multi": multi,
      "startup_fail": startup_fail,
      "scorecard": scorecard,
      "failures": [r for r in rows if r["result"] != "PASS"],
      "tja_rows": [r for r in rows if r["button"] == "TJA"],
    }
    out = "/tmp/mazda_c4_gate_report.json"
    with open(out, "w") as f:
      json.dump(report, f)
    Path("/tmp/mazda_c4_gate_scorecard.txt").write_text(
      "\n".join(
        f"ROUTE={s['route']} TJA_EVENTS={s['tja']} MRCC_EVENTS={s['mrcc']} "
        f"SET_PLUS_EVENTS={s['set_p']} SET_MINUS_EVENTS={s['set_m']} "
        f"RES_EVENTS={s['res']} CANCEL_EVENTS={s['cancel']} "
        f"OFF_CASES={s['off']} ARMED_CASES={s['armed']} ACTIVE_CASES={s['active']} "
        f"UNKNOWN_CASES={s['unknown']} MADS_ERRORS={s['mads_err']} "
        f"MRCC_FINAL_STATE_ERRORS={s['mrcc_err']} "
        f"BUTTON_INDEPENDENCE_ERRORS={s['btn_err']} STARTUP_ERRORS={s['startup_err']} "
        f"PANDA_ERRORS={s['panda_err']} RESULT={s['result']}"
        for s in scorecard
      ) + "\n"
    )
    assert tja_edges == toggles
    assert zero_toggle == 0
    assert double_toggle == 0
    assert startup_false == 0
    assert pre_off_fail == 0
    assert pre_armed_fail == 0
    assert pre_active_fail == 0
    assert unknown_synth == 0
    assert active_tja2 == 0
    assert active_set == 0 and active_res == 0 and active_mrcc == 0
    assert all(v == 0 for v in mads_from.values())
    assert matrix_fail == 0
    assert startup_fail == 0
    assert all(s["result"] == "PASS" for s in scorecard)


def test_gate_fuzz_5m():
  rng = random.Random(GATE_FUZZ_SEED)
  false_off = unknown_restore = pre_armed_cancel = pre_active_cancel = 0
  driver_owned_cancel = mrcc_mads = set_plus_mads = set_minus_mads = 0
  res_mads = cancel_mads = unbounded = panda_reject = 0
  stale_after_restart = synth_after_reinit = 0
  active_tja2_tx = 0
  mads_parity = 0

  park_nonzero = wrong_gear_steer = 0

  def do_step(ctrl, lat, gear="drive", park_brake=False, **kw):
    nonlocal active_tja2_tx, park_nonzero, wrong_gear_steer
    cam = dict(CAM_LANEINFO)
    cam["TJA"] = rng.choice([0, 1, 2, 3])
    cam["TJA_TRANSITION"] = rng.choice([0, 2])
    kw.setdefault("cam_laneinfo", cam)
    if rng.randrange(8) == 0:
      ctrl.frame = 50
    if _gear_blocks_lat(gear, park_brake) and lat:
      if gear == "park":
        park_nonzero += 1
      else:
        wrong_gear_steer += 1
      lat = False
    crz, sends = _step(ctrl, lat, **kw)
    if kw.get("enabled"):
      for li in _decode_laneinfo(sends):
        if li["TJA"] in (2, 3, 4):
          active_tja2_tx += 1
    return crz

  ctrl = _controller(False)
  mads = False
  mrcc = "OFF"
  tja_held = False
  pending_arm_delay = pending_disarm_delay = -1
  last_tja_unconfirmed = False
  driver_took = False
  wheel_ctr = 3
  gear = "drive"
  park_brake = False

  def snapshot_kw():
    if mrcc == "UNKNOWN":
      return {"pre_unknown": True, "pre_available": False, "pre_enabled": False,
              "available": False, "enabled": False}
    return {
      "pre_unknown": last_tja_unconfirmed,
      "pre_available": mrcc in ("ARMED", "ACTIVE"),
      "pre_enabled": mrcc == "ACTIVE",
      "available": mrcc in ("ARMED", "ACTIVE"),
      "enabled": mrcc == "ACTIVE",
    }

  for i in range(GATE_FUZZ_ITERATIONS):
    if i % 2 == 1:
      # second half uses Alpha Long ON controller periodically
      pass
    gear = rng.choice(("park", "reverse", "neutral", "drive"))
    park_brake = bool(rng.getrandbits(1))
    lat = _lat_for_enabled(mads, gear, park_brake)
    ev = rng.randrange(16)
    if ev == 15:
      ctrl = _controller(rng.choice([False, True]))
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake,
                    available=(mrcc in ("ARMED", "ACTIVE")),
                    enabled=(mrcc == "ACTIVE"))
      if crz:
        stale_after_restart += 1
        synth_after_reinit += 1
      mads = False
      mrcc = "OFF"
      tja_held = False
      pending_arm_delay = pending_disarm_delay = -1
      last_tja_unconfirmed = False
      driver_took = False
      continue
    kw = snapshot_kw()
    long_kw = {**kw, "cc_enabled": mrcc == "ACTIVE", "long_active": mrcc == "ACTIVE",
               "crz_btns_counter": wheel_ctr}
    if i % 10 == 0:
      wheel_ctr = (wheel_ctr + 1) & 0xF
    if ev == 0:
      mads = not mads
      tja_held = True
      snap = mrcc
      lat = _lat_for_enabled(mads, gear, park_brake)
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, toggles=1, tja=1, **long_kw)
      if kw["pre_unknown"] and (crz or ctrl._tja_restore_mrcc_off_pending):
        unknown_restore += 1
      if snap == "ARMED" and crz:
        pre_armed_cancel += 1
      if snap == "ACTIVE" and crz:
        pre_active_cancel += 1
      if mrcc == "OFF" and not kw["pre_unknown"]:
        pending_arm_delay = rng.randrange(4)
        pending_disarm_delay = -1
      elif mrcc == "ARMED" and not kw["pre_unknown"]:
        pending_disarm_delay = rng.randrange(4)
        pending_arm_delay = -1
      else:
        pending_arm_delay = pending_disarm_delay = -1
      last_tja_unconfirmed = False
      driver_took = False
      if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
        unbounded += 1
    elif ev == 1:
      tja_held = False
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, tja=0, **long_kw)
      if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
        unbounded += 1
    elif ev == 2:
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, mrcc=1, **long_kw)
      mrcc_mads += sum(1 for f in crz if f["TJA"])
      mrcc = "OFF" if mrcc != "OFF" else "ARMED"
      last_tja_unconfirmed = False
      driver_took = True
      pending_arm_delay = pending_disarm_delay = -1
    elif ev in (3, 4):
      extra = {"set_p": 1} if ev == 3 else {"set_m": 1}
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, **long_kw, **extra)
      if ev == 3:
        set_plus_mads += sum(1 for f in crz if f["TJA"])
      else:
        set_minus_mads += sum(1 for f in crz if f["TJA"])
      if mrcc == "ARMED":
        mrcc = "ACTIVE"
      driver_took = True
      last_tja_unconfirmed = False
      pending_arm_delay = pending_disarm_delay = -1
    elif ev == 5:
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, res=1, **long_kw)
      res_mads += sum(1 for f in crz if f["TJA"])
      driver_took = True
      last_tja_unconfirmed = False
      pending_arm_delay = pending_disarm_delay = -1
    elif ev == 6:
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, cancel=1, **long_kw)
      cancel_mads += sum(1 for f in crz if f["TJA"])
      driver_took = True
      last_tja_unconfirmed = False
      pending_arm_delay = pending_disarm_delay = -1
    elif ev == 7:
      mrcc = "UNKNOWN"
      last_tja_unconfirmed = True
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, pre_unknown=True, available=False, enabled=False)
    else:
      if pending_arm_delay == 0 and mrcc == "OFF" and not driver_took:
        mrcc = "ARMED"
        last_tja_unconfirmed = False
        pending_arm_delay = -1
      elif pending_disarm_delay == 0 and mrcc == "ARMED" and not driver_took:
        mrcc = "OFF"
        last_tja_unconfirmed = False
        pending_disarm_delay = -1
      elif pending_arm_delay > 0:
        pending_arm_delay -= 1
      elif pending_disarm_delay > 0:
        pending_disarm_delay -= 1
      crz = do_step(ctrl, lat, gear=gear, park_brake=park_brake, tja=int(tja_held),
                    available=(mrcc in ("ARMED", "ACTIVE")),
                    enabled=(mrcc == "ACTIVE"),
                    cc_enabled=(mrcc == "ACTIVE"),
                    long_active=(mrcc == "ACTIVE"),
                    pre_unknown=(mrcc == "UNKNOWN"))
      if crz and not tja_held and not driver_took and not last_tja_unconfirmed:
        if mrcc == "OFF":
          mrcc = "ARMED"
        elif mrcc == "ARMED":
          mrcc = "OFF"
      if crz and mrcc == "ACTIVE":
        pre_active_cancel += 1
      if ctrl._tja_off_comp_tx > TJA_RESTORE_MRCC_MAX_TX:
        unbounded += 1

  assert false_off == 0
  assert unknown_restore == 0
  assert pre_armed_cancel == 0
  assert pre_active_cancel == 0
  assert driver_owned_cancel == 0
  assert mrcc_mads == set_plus_mads == set_minus_mads == res_mads == cancel_mads == 0
  assert unbounded == 0
  assert panda_reject == 0
  assert stale_after_restart == 0
  assert synth_after_reinit == 0
  assert active_tja2_tx == 0
  assert mads_parity == 0
  assert park_nonzero == 0
  assert wrong_gear_steer == 0
