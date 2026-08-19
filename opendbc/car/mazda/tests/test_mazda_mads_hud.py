"""Experimental MADS GREEN HUD: default-off, family-gated, 2 Hz only."""

import pytest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.mazdacan import (HUD_GREEN, HUD_OFF, HUD_PASSTHROUGH, HUD_WHITE,
                                        OEM_LL1_HUD_FAMILY, OEM_LL1_HUD_GREEN, OEM_LL1_HUD_OFF,
                                        OEM_LL1_HUD_OFF_TJA2, OEM_LL1_HUD_OFF_TJA3,
                                        OEM_LL1_HUD_GREEN_VARIANTS,
                                        OEM_LL1_HUD_WHITE, STEER_ACTIVATION_HOLD_NS,
                                        create_alert_command)
from opendbc.car.mazda.values import CAR
from opendbc.sunnypilot.car.interfaces import _initialize_mazda
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

# Captured FSC 0x440 payload (route 4b, TJA stripped): TJA=0, LANE_LINES=1. Not a log replay.
ROUTE_4B_FSC_OFF = OEM_LL1_HUD_OFF
BOOT_PAYLOAD = bytes.fromhex("4261000000001040")  # NO_ERR_BIT-style, not OFF/WHITE family


def _cc(*, green=False, alpha_long=False):
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=alpha_long,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [],
                                     alpha_long, False, False)
  if green:
    CP_SP.flags |= MazdaFlagsSP.EXPERIMENTAL_MADS_GREEN_HUD.value
  return CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


def _cs(*, raw=ROUTE_4B_FSC_OFF, cam_lkas_live=True, err1=0, err2=0):
  return SimpleNamespace(
    out=SimpleNamespace(vEgoRaw=12.0, steeringTorque=0, brakePressed=False),
    cam_lkas_live=cam_lkas_live,
    cam_lkas={"ERR_BIT_1": err1, "ERR_BIT_2": err2, "LINE_NOT_VISIBLE": 0, "BIT_1": 1},
    cam_laneinfo={"TJA": 0, "LANE_LINES": 1, "LINE_VISIBLE": 0,
                  "LINE_NOT_VISIBLE": 1, "TJA_TRANSITION": 0},
    cam_laneinfo_raw=raw,
    crz_btns_counter=0,
    cancel_button=0,
    tja_button=0,
    accel_button=0,
    decel_button=0,
    lkas_allowed_speed=True,
  )


def _cc_msgs(*, lat_active=True, mads_enabled=True, mads_available=True):
  CC = structs.CarControl()
  CC.latActive = lat_active
  CC.actuators.torque = 0.2 if lat_active else 0.0
  CC = CC.as_reader()
  CC_SP = structs.CarControlSP()
  CC_SP.mads.available = mads_available
  CC_SP.mads.enabled = mads_enabled
  return CC, CC_SP


def _hud(sends):
  hits = [bytes(d) for a, d, _b in sends if a == 0x440]
  return hits[0] if hits else None


def _step(cc, CS, *, lat_active=True, mads_enabled=True, now_ns=0):
  CC, CC_SP = _cc_msgs(lat_active=lat_active, mads_enabled=mads_enabled)
  _, sends = cc.update(CC, CC_SP, CS, now_ns)
  return sends, now_ns + int(DT_CTRL * 1e9)


def _drive_to_active(cc, CS, *, lat_active=True, mads_enabled=True):
  """Advance past the 50 ms CAM_LKAS settle onto a HUD frame in LKAS_TX_ACTIVE."""
  now_ns = 0
  last_hud = None
  hold_frames = int(STEER_ACTIVATION_HOLD_NS / (DT_CTRL * 1e9)) + 2
  for _ in range(max(hold_frames, 50) + 1):
    sends, now_ns = _step(cc, CS, lat_active=lat_active, mads_enabled=mads_enabled, now_ns=now_ns)
    hud = _hud(sends)
    if hud is not None:
      last_hud = hud
    if cc.lkas_tx_state == mazdacan.LKAS_TX_ACTIVE and hud is not None:
      return last_hud, now_ns, sends
  # One more HUD tick at frame 50 if ACTIVE was reached mid-cycle.
  for _ in range(50):
    sends, now_ns = _step(cc, CS, lat_active=lat_active, mads_enabled=mads_enabled, now_ns=now_ns)
    hud = _hud(sends)
    if hud is not None and cc.lkas_tx_state == mazdacan.LKAS_TX_ACTIVE:
      return hud, now_ns, sends
  raise AssertionError(f"never ACTIVE+HUD, state={cc.lkas_tx_state} last={last_hud}")


def _decode_laneinfo(dat):
  cp = CANParser("mazda_2017", [("CAM_LANEINFO", float("nan"))], 0)
  cp.update([(0, [(0x440, dat, 0)])])
  v = cp.vl["CAM_LANEINFO"]
  return {k: int(v[k]) for k in (
    "TJA", "LANE_LINES", "LINE_VISIBLE", "LINE_NOT_VISIBLE", "TJA_TRANSITION",
    "ERR_BIT", "HANDS_ON_STEER_WARN", "HANDS_ON_STEER_WARN_2", "HANDS_WARN_3_BITS",
    "LDW_WARN_LL", "LDW_WARN_RL")}


class TestGreenTemplate:
  def test_green_is_tja4_ll2_with_zero_warn_bits(self):
    sig = _decode_laneinfo(OEM_LL1_HUD_GREEN)
    assert sig["TJA"] == 4
    assert sig["LANE_LINES"] == 2
    assert sig["TJA"] != 4 or sig["LANE_LINES"] != 1
    for k in ("ERR_BIT", "HANDS_ON_STEER_WARN", "HANDS_ON_STEER_WARN_2",
              "HANDS_WARN_3_BITS", "LDW_WARN_LL", "LDW_WARN_RL"):
      assert sig[k] == 0, k
    assert OEM_LL1_HUD_GREEN not in OEM_LL1_HUD_FAMILY


class TestFamilyGateUnit:
  def test_default_white_when_mads_on(self):
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=True,
                                           fsc_raw=OEM_LL1_HUD_OFF, green_allowed=False)
    assert dat == OEM_LL1_HUD_WHITE
    assert mode == HUD_WHITE

  def test_off_when_mads_off(self):
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=False,
                                           fsc_raw=OEM_LL1_HUD_WHITE, green_allowed=True)
    assert dat == OEM_LL1_HUD_OFF
    assert mode == HUD_OFF

  def test_green_only_when_allowed(self):
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=True,
                                           fsc_raw=OEM_LL1_HUD_OFF, green_allowed=True)
    assert dat == OEM_LL1_HUD_GREEN
    assert mode == HUD_GREEN

  # --- Route-4C TJA variant tests: param-off must be passthrough ---

  @pytest.mark.parametrize("fsc_raw", [OEM_LL1_HUD_OFF_TJA2, OEM_LL1_HUD_OFF_TJA3])
  def test_tja_variant_param_off_is_exact_passthrough(self, fsc_raw):
    """Default-off regression: 0x0a/0x0c variants must be byte-exact passthrough when param=0."""
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=True,
                                           fsc_raw=fsc_raw,
                                           green_hud_enabled=False, green_allowed=False)
    assert dat == fsc_raw, f"expected passthrough of {fsc_raw.hex()}, got {dat.hex()}"
    assert mode == HUD_PASSTHROUGH

  @pytest.mark.parametrize("fsc_raw", [OEM_LL1_HUD_OFF_TJA2, OEM_LL1_HUD_OFF_TJA3])
  def test_tja_variant_param_on_active_is_green(self, fsc_raw):
    """Route-4C regression: 0x0a/0x0c → GREEN when param=1 and all guards pass."""
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=True,
                                           fsc_raw=fsc_raw,
                                           green_hud_enabled=True, green_allowed=True)
    assert dat == OEM_LL1_HUD_GREEN
    assert mode == HUD_GREEN

  @pytest.mark.parametrize("fsc_raw", [OEM_LL1_HUD_OFF_TJA2, OEM_LL1_HUD_OFF_TJA3])
  def test_tja_variant_param_on_paused_is_white(self, fsc_raw):
    """Param on but green_allowed=False (e.g. MADS paused): variants → WHITE."""
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=True,
                                           fsc_raw=fsc_raw,
                                           green_hud_enabled=True, green_allowed=False)
    assert dat == OEM_LL1_HUD_WHITE
    assert mode == HUD_WHITE

  @pytest.mark.parametrize("fsc_raw", [OEM_LL1_HUD_OFF_TJA2, OEM_LL1_HUD_OFF_TJA3])
  def test_tja_variant_param_on_mads_off_is_off(self, fsc_raw):
    """Param on but MADS disabled: variants → OFF."""
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=False,
                                           fsc_raw=fsc_raw,
                                           green_hud_enabled=True, green_allowed=False)
    assert dat == OEM_LL1_HUD_OFF
    assert mode == HUD_OFF

  def test_tja_variants_not_in_baseline_family(self):
    """Structural: route-4C payloads must not be in the param-independent baseline family."""
    for v in OEM_LL1_HUD_GREEN_VARIANTS:
      assert v not in OEM_LL1_HUD_FAMILY, f"{v.hex()} must not be in OEM_LL1_HUD_FAMILY"

  def test_unknown_payload_passthrough_even_if_green_allowed(self):
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=True,
                                           fsc_raw=BOOT_PAYLOAD, green_allowed=True)
    assert dat == BOOT_PAYLOAD
    assert mode == HUD_PASSTHROUGH

  def test_no_raw_never_green(self):
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {"TJA": 0, "LANE_LINES": 1}, False, False,
                                           mads_enabled=True, fsc_raw=None, green_allowed=True)
    assert mode == HUD_PASSTHROUGH
    assert dat != OEM_LL1_HUD_GREEN


class TestParamDefaultOff:
  def test_setup_interfaces_default_does_not_set_flag(self):
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    _initialize_mazda(CP, CP_SP, {})
    assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_GREEN_HUD)

  def test_setup_interfaces_enables_flag(self):
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    _initialize_mazda(CP, CP_SP, {"MazdaExperimentalMadsGreenHud": "1"})
    assert CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_GREEN_HUD


class TestControllerHud:
  def test_param_off_stays_white_while_active(self):
    cc = _cc(green=False)
    hud, _, _ = _drive_to_active(cc, _cs())
    assert hud == OEM_LL1_HUD_WHITE
    assert cc.mads_hud_mode == HUD_WHITE

  def test_green_when_all_guards_true(self):
    cc = _cc(green=True)
    hud, _, _ = _drive_to_active(cc, _cs())
    assert hud == OEM_LL1_HUD_GREEN
    assert cc.mads_hud_mode == HUD_GREEN
    assert cc.mads_hud_tx == OEM_LL1_HUD_GREEN

  def test_mads_disabled_is_off(self):
    cc = _cc(green=True)
    CS = _cs()
    sends, _ = _step(cc, CS, lat_active=False, mads_enabled=False)
    assert _hud(sends) == OEM_LL1_HUD_OFF

  def test_enabled_not_steering_is_white(self):
    cc = _cc(green=True)
    CS = _cs()
    sends, _ = _step(cc, CS, lat_active=False, mads_enabled=True)
    assert _hud(sends) == OEM_LL1_HUD_WHITE
    assert cc.lkas_tx_state != mazdacan.LKAS_TX_ACTIVE

  def test_pause_falls_back_to_white(self):
    cc = _cc(green=True)
    CS = _cs()
    hud, now_ns, _ = _drive_to_active(cc, CS)
    assert hud == OEM_LL1_HUD_GREEN
    # latActive drop → AUTH_PAUSED; next HUD tick must be WHITE.
    found = None
    for _ in range(50):
      sends, now_ns = _step(cc, CS, lat_active=False, mads_enabled=True, now_ns=now_ns)
      hud = _hud(sends)
      if hud is not None:
        found = hud
        break
    assert found == OEM_LL1_HUD_WHITE
    assert cc.lkas_tx_state == mazdacan.LKAS_TX_AUTH_PAUSED

  def test_auth_loss_falls_back_to_white(self):
    self.test_pause_falls_back_to_white()

  def test_stale_camera_falls_back_to_white(self):
    cc = _cc(green=True)
    CS = _cs()
    hud, now_ns, _ = _drive_to_active(cc, CS)
    assert hud == OEM_LL1_HUD_GREEN
    CS.cam_lkas_live = False
    found = None
    for _ in range(50):
      sends, now_ns = _step(cc, CS, now_ns=now_ns)
      hud = _hud(sends)
      if hud is not None:
        found = hud
        break
    assert found == OEM_LL1_HUD_WHITE
    assert cc.lkas_tx_state == mazdacan.LKAS_TX_FAULT

  def test_cam_lkas_fault_falls_back_to_white(self):
    cc = _cc(green=True)
    CS = _cs()
    hud, now_ns, _ = _drive_to_active(cc, CS)
    assert hud == OEM_LL1_HUD_GREEN
    CS.cam_lkas["ERR_BIT_1"] = 1
    found = None
    for _ in range(50):
      sends, now_ns = _step(cc, CS, now_ns=now_ns)
      hud = _hud(sends)
      if hud is not None:
        found = hud
        break
    assert found == OEM_LL1_HUD_WHITE
    assert cc.lkas_tx_state == mazdacan.LKAS_TX_FAULT

  def test_recovery_returns_to_green(self):
    cc = _cc(green=True)
    CS = _cs()
    hud, now_ns, _ = _drive_to_active(cc, CS)
    assert hud == OEM_LL1_HUD_GREEN
    CS.cam_lkas_live = False
    for _ in range(50):
      sends, now_ns = _step(cc, CS, now_ns=now_ns)
      if _hud(sends) is not None:
        assert _hud(sends) == OEM_LL1_HUD_WHITE
        break
    CS.cam_lkas_live = True
    found = None
    for _ in range(60):
      sends, now_ns = _step(cc, CS, now_ns=now_ns)
      hud = _hud(sends)
      if hud is not None and cc.lkas_tx_state == mazdacan.LKAS_TX_ACTIVE:
        found = hud
        break
    assert found == OEM_LL1_HUD_GREEN

  def test_disengage_is_off(self):
    cc = _cc(green=True)
    CS = _cs()
    hud, now_ns, _ = _drive_to_active(cc, CS)
    assert hud == OEM_LL1_HUD_GREEN
    found = None
    for _ in range(50):
      sends, now_ns = _step(cc, CS, lat_active=False, mads_enabled=False, now_ns=now_ns)
      hud = _hud(sends)
      if hud is not None:
        found = hud
        break
    assert found == OEM_LL1_HUD_OFF

  def test_boot_payload_passthrough_not_green(self):
    cc = _cc(green=True)
    CS = _cs(raw=BOOT_PAYLOAD)
    hud, _, _ = _drive_to_active(cc, CS)
    assert hud == BOOT_PAYLOAD
    assert cc.mads_hud_mode == HUD_PASSTHROUGH

  def test_cadence_stays_2hz_with_green(self):
    cc = _cc(green=True)
    CS = _cs()
    now_ns = 0
    hud_frames = []
    for i in range(100):
      sends, now_ns = _step(cc, CS, now_ns=now_ns)
      if _hud(sends) is not None:
        hud_frames.append(i)
    assert hud_frames == [0, 50]
    assert len(hud_frames) == 2

  def test_never_emits_tja4_ll1(self):
    cc = _cc(green=True)
    hud, _, _ = _drive_to_active(cc, _cs())
    sig = _decode_laneinfo(hud)
    assert not (sig["TJA"] == 4 and sig["LANE_LINES"] == 1)

  def test_trial_state_is_latched_on_cs(self):
    cc = _cc(green=True)
    CS = _cs()
    _drive_to_active(cc, CS)
    assert CS.mads_hud_mode == HUD_GREEN
    assert CS.mads_hud_tx == OEM_LL1_HUD_GREEN
    assert CS.mads_hud_lat_active is True
    assert CS.mads_hud_cam_lkas_live is True
    assert CS.mads_hud_tx_state == mazdacan.LKAS_TX_ACTIVE
    assert CS.mads_hud_mads_enabled is True


class TestCaptured4bPayload:
  """Synthetic regression using the route-4b FSC OFF payload (not a log replay).

  FSC input stays OFF; outbound is WHITE unless GREEN guards fire.
  """

  def test_fsc_off_input_white_then_green(self):
    cc = _cc(green=True)
    CS = _cs(raw=ROUTE_4B_FSC_OFF)
    assert CS.cam_laneinfo_raw == ROUTE_4B_FSC_OFF

    sends, now_ns = _step(cc, CS, lat_active=False, mads_enabled=True)
    assert _hud(sends) == OEM_LL1_HUD_WHITE
    assert CS.cam_laneinfo_raw == ROUTE_4B_FSC_OFF

    hud, _, _ = _drive_to_active(cc, CS)
    assert CS.cam_laneinfo_raw == ROUTE_4B_FSC_OFF
    assert hud == OEM_LL1_HUD_GREEN
    assert _decode_laneinfo(hud)["TJA"] == 4
    assert _decode_laneinfo(hud)["LANE_LINES"] == 2

  def test_fsc_off_input_param_off_stays_white(self):
    cc = _cc(green=False)
    CS = _cs(raw=ROUTE_4B_FSC_OFF)
    hud, _, _ = _drive_to_active(cc, CS)
    assert CS.cam_laneinfo_raw == ROUTE_4B_FSC_OFF
    assert hud == OEM_LL1_HUD_WHITE


class TestCaptured4cVariants:
  """Regression for route-4C TJA_TRANSITION=2/3 variants (0x0a, 0x0c).

  With param off: byte-exact passthrough regardless of MADS state.
  With param on + guards healthy: continuous GREEN (no blink).
  """

  @pytest.mark.parametrize("fsc_raw", list(OEM_LL1_HUD_GREEN_VARIANTS))
  def test_param_off_variant_is_passthrough_not_white(self, fsc_raw):
    """Default-off: variant must not be silently mapped to WHITE."""
    cc = _cc(green=False)
    CS = _cs(raw=fsc_raw)
    now_ns = 0
    huds_seen = set()
    for _ in range(60):
      sends, now_ns = _step(cc, CS, lat_active=True, mads_enabled=True, now_ns=now_ns)
      h = _hud(sends)
      if h is not None:
        huds_seen.add(h)
    assert all(h == fsc_raw for h in huds_seen), (
      f"param-off variant {fsc_raw.hex()} was remapped: {[h.hex() for h in huds_seen]}")

  @pytest.mark.parametrize("fsc_raw", list(OEM_LL1_HUD_GREEN_VARIANTS))
  def test_param_on_variant_is_continuously_green(self, fsc_raw):
    """Route-4C fix: variant stays GREEN continuously while guards are healthy (no blink)."""
    cc = _cc(green=True)
    CS = _cs(raw=fsc_raw)
    hud, now_ns, _ = _drive_to_active(cc, CS)
    assert hud == OEM_LL1_HUD_GREEN, f"first GREEN tick: got {hud.hex() if hud else None}"
    # Run several more HUD ticks confirming no reversion to passthrough.
    green_ticks = 0
    for _ in range(200):
      sends, now_ns = _step(cc, CS, now_ns=now_ns)
      h = _hud(sends)
      if h is not None:
        assert h == OEM_LL1_HUD_GREEN, f"reverted to {h.hex()} after going GREEN"
        green_ticks += 1
      if green_ticks >= 3:
        break
    assert green_ticks >= 3, "never saw 3 consecutive GREEN ticks"

  @pytest.mark.parametrize("fsc_raw", list(OEM_LL1_HUD_GREEN_VARIANTS))
  def test_param_on_variant_mads_off_is_off(self, fsc_raw):
    packer = CANPacker("mazda_2017")
    _, dat, _, mode = create_alert_command(packer, {}, False, False, mads_enabled=False,
                                           fsc_raw=fsc_raw,
                                           green_hud_enabled=True, green_allowed=False)
    assert dat == OEM_LL1_HUD_OFF
    assert mode == HUD_OFF
