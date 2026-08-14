import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, rate_limit, structs, uds
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal import (RADAR_ADDR, RadarSessionManager, RadarSessionState, StopAndGoStateMachine,
                                            StopGoState, create_radar_session_msg)
from opendbc.car.mazda.values import CarControllerParams, Buttons

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Synthetic radar frames go to the car and to the camera; the panda only forwards
# received frames between those buses, not our own transmissions.
LONG_BUSES = (0, 2)

# Bound the wait for OEM MRCC to settle after a physical TJA press (~70–110 ms).
TJA_RESTORE_MRCC_TIMEOUT_FRAMES = 250  # 2.5 s at 100 Hz
# Physical successful MRCC is ~10 Hz, 2–4 frames, CTR increments while held,
# and there is never an idle MRCC=0 between those frames. CRZ_BTNS is same-bus
# (check_relay=false), so we cannot delete wheel idles. A 100 Hz burst that
# spans a wheel tick is press→idle→press (two logical presses). The only
# one-press primitive on this bus is to keep
# every synthetic MRCC=1 inside a single wheel period, then let the next idle
# be the release. Wait for the first CTR change after mismatch (full period),
# lock one CTR, TX at 100 Hz, stop on the next wheel CTR. Cap is one period.
TJA_RESTORE_MRCC_MAX_TX = 10
TJA_RESTORE_MRCC_MAX_UNIQUE_CTR = 1
TJA_RESTORE_MRCC_MAX_DURATION_MS = 100  # one 10 Hz wheel period at 100 Hz TX
# Compatibility names for startup LKAS tests that assert restore is not consumed.
TJA_OFF_COMPENSATION_TIMEOUT_FRAMES = TJA_RESTORE_MRCC_TIMEOUT_FRAMES
TJA_OFF_COMPENSATION_MAX_TX = TJA_RESTORE_MRCC_MAX_TX


def _crz_btns_held(CS) -> bool:
  return bool(getattr(CS, "tja_button", 0))


def _driver_long_button(CS, *, ignore_main: bool = False) -> bool:
  # SET/RES/CANCEL are never packed during MRCC restore, so they are always driver.
  # MRCC_BUTTON is the compensation signal; parser echoes of our own hold
  # must not abort. Physical MRCC is detected when we did not just transmit.
  return bool(
    getattr(CS, "accel_button", 0)
    or getattr(CS, "decel_button", 0)
    or getattr(CS, "resume_button", 0)
    or getattr(CS, "cancel_button", 0)
    or (getattr(CS, "main_button", 0) and not ignore_main)
  )


def _pre_tja_snapshot(CS) -> str:
  # UNKNOWN / stale (unconfirmed long button) must not look like OFF or ARMED.
  if getattr(CS, "tja_pre_cruise_unknown", False):
    return "UNKNOWN"
  if bool(getattr(CS, "tja_pre_cruise_enabled", False)):
    return "ACTIVE"
  if bool(getattr(CS, "tja_pre_cruise_available", False)):
    return "ARMED"
  return "OFF"


def _pre_tja_mrcc_off(CS) -> bool:
  return _pre_tja_snapshot(CS) == "OFF"


def _cruise_label(CS) -> str:
  if bool(CS.out.cruiseState.enabled):
    return "ACTIVE"
  if bool(CS.out.cruiseState.available):
    return "ARMED"
  return "OFF"


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.stop_and_go = StopAndGoStateMachine()
    self.virtual_resume_latched = False
    self.long_counter = 0
    self.radar_counter = 0
    self.radar_session = RadarSessionManager()
    self.accel_last = 0.
    # After an odd TJA toggle while latActive, keep torque at 0 until controlsd
    # drops latActive. Prevents packing nonzero CAM_LKAS from a stale CC while
    # Panda has already consumed the same CRZ_BTNS edge.
    self._tja_hold_zero = False
    self._lat_active_prev = False
    # Per physical TJA edge: snapshot confident MRCC (OFF/ARMED/ACTIVE/UNKNOWN).
    # After OEM TJA_BUTTON settles, restore with one bounded MRCC_BUTTON gesture
    # only if post-TJA state differs from OFF or ARMED. ACTIVE/UNKNOWN: no TX.
    self._tja_restore_target = None
    self._tja_off_comp_tx = 0
    self._tja_off_comp_wait = 0
    self._tja_off_comp_last_tx_frame = -1
    self._tja_restore_ctrs = set()
    self._tja_restore_locked_ctr = None
    self._tja_restore_wheel_ctr = None
    self._tja_restore_period_ctr = None
    self._tja_restore_mismatch_ctr = None
    self._tja_restore_wait_period = 0
    self._tja_restore_brake_at_edge = False

  @property
  def _tja_restore_mrcc_off_pending(self) -> bool:
    return self._tja_restore_target is not None

  @_tja_restore_mrcc_off_pending.setter
  def _tja_restore_mrcc_off_pending(self, value: bool) -> None:
    if value:
      self._tja_restore_target = "OFF"
    else:
      self._clear_tja_restore()

  @property
  def _tja_off_comp_pending(self) -> bool:
    return self._tja_restore_target is not None

  @_tja_off_comp_pending.setter
  def _tja_off_comp_pending(self, value: bool) -> None:
    if value:
      self._tja_restore_target = "OFF"
    else:
      self._clear_tja_restore()

  def _clear_tja_restore(self):
    self._tja_restore_target = None
    self._tja_off_comp_tx = 0
    self._tja_off_comp_wait = 0
    self._tja_off_comp_last_tx_frame = -1
    self._tja_restore_ctrs = set()
    self._tja_restore_locked_ctr = None
    self._tja_restore_wheel_ctr = None
    self._tja_restore_period_ctr = None
    self._tja_restore_mismatch_ctr = None
    self._tja_restore_wait_period = 0
    self._tja_restore_brake_at_edge = False

  def _stop_tja_restore(self):
    self._tja_restore_target = None

  def early_init_cam_lkas_keepalive(self, CS):
    """Pre-selfdriveInitializing: emit only inactive CAM_LKAS.

    Route 00000014: leftover mazda check_relay blocked stock 0x243 while card
    waited for selfdriveInitializing. Full CI.apply is not safe here — it can
    also pack CAM_LANEINFO HUD, ICBM/CRZ_BTNS, MRCC restore, and longitudinal
    radar frames. This path must not.
    """
    cam = getattr(CS, "cam_lkas", None)
    bit1 = 0
    if cam is not None:
      try:
        bit1 = int(cam["BIT_1"])
      except Exception:
        bit1 = 0
    lkas = {"BIT_1": bit1, "ERR_BIT_1": 0, "ERR_BIT_2": 0}
    msg = mazdacan.create_steering_control(self.packer, self.CP, self.frame, 0, lkas)
    self.frame += 1
    return [msg]

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    # Speed-dependent STEER_MAX (CX-5 2022: 1200 below 32 mph, 800 above)
    if hasattr(self.params, 'STEER_MAX_LOOKUP'):
      steer_max = round(float(np.interp(CS.out.vEgoRaw, self.params.STEER_MAX_LOOKUP[0],
                                         self.params.STEER_MAX_LOOKUP[1])))
    else:
      steer_max = self.params.STEER_MAX

    tja_toggles = int(getattr(CS, "tja_toggles_this_update", 0))
    if tja_toggles:
      if (tja_toggles % 2 == 1) and CC.latActive:
        self._tja_hold_zero = True
    if not CC.latActive:
      self._tja_hold_zero = False

    if getattr(CS, "tja_edge_reset", False):
      self._clear_tja_restore()
      CS.tja_edge_reset = False

    echo_main = (self._tja_off_comp_last_tx_frame == self.frame - 1)
    if _driver_long_button(CS, ignore_main=echo_main):
      self._clear_tja_restore()

    if tja_toggles:
      # Any new physical TJA aborts a previous edge's restore. Only a single
      # edge this cycle may start a new one; two edges in one batch fail closed.
      self._clear_tja_restore()
      if tja_toggles == 1:
        snap = _pre_tja_snapshot(CS)
        if snap in ("OFF", "ARMED"):
          self._tja_restore_target = snap
          self._tja_restore_brake_at_edge = bool(CS.out.brakePressed)

    if CC.latActive and not self._tja_hold_zero:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * steer_max))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params, steer_max)

    virtual_resume_sent = False
    tja_off_comp = self._maybe_tja_off_compensation(CC, CS)
    if tja_off_comp:
      can_sends.append(mazdacan.create_button_cmd(
        self.packer, self.CP, CS.crz_btns_counter, Buttons.MAIN,
        ctr=self._tja_restore_locked_ctr))
    if CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move. With openpilot longitudinal the
        # RES press asks the body ECU to leave its standstill hold; only meaningful from a stop.
        if not self.CP.openpilotLongitudinalControl or CS.out.standstill:
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
          virtual_resume_sent = self.CP.openpilotLongitudinalControl

    self.apply_torque_last = apply_torque

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.update_longitudinal(CC, CC_SP, CS, virtual_resume_sent))

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      # MRCC ACTIVE must never pack CAM_LANEINFO.TJA in {2,3,4} (OEM TJA-engaged).
      # MADS disabled must pack TJA=0 (Stage 2A HUD ownership; Event 38 leak).
      # Presentation plumbing only: CC_SP.mads.enabled is not a MADS/MRCC state change.
      can_sends.append(mazdacan.create_alert_command(
        self.packer, CS.cam_laneinfo, ldw, steer_required,
        bool(CS.out.cruiseState.enabled), bool(CC_SP.mads.enabled)))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    # Intelligent Cruise Button Management
    # Suppress ICBM CRZ_BTNS spam while cancel/resume are in flight or while the driver is
    # holding the wheel cancel button. Without this guard ICBM's interleaved cancel=0 frames
    # race the driver's cancel=1 frames on the bus and the body ECU drops the cancel intent.
    icbm_suppress = CC.cruiseControl.cancel or CC.cruiseControl.resume or CS.cancel_button == 1 or tja_off_comp
    if not icbm_suppress:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque
    # report what actually went on the wire, not the plan: the clip, the standstill hold values,
    # the slew limit, and the zero we send through a gas override all live in accel_last
    new_actuators.accel = self.accel_last

    self._lat_active_prev = bool(CC.latActive)
    self.frame += 1
    return new_actuators, can_sends

  def _maybe_tja_off_compensation(self, CC, CS) -> bool:
    """One bounded MRCC_BUTTON gesture to restore the pre-TJA cruise snapshot.

    Physical TJA is TJA_BUTTON only and may ARM or disarm OEM cruise. Final
    state must match a confident OFF or ARMED snapshot. ACTIVE and UNKNOWN
    never synthesize. Master toggle is MRCC_BUTTON. CAN_OFF is not.

    Route 00000019: contain the entire MRCC=1 burst in one 10 Hz wheel
    period so a physical idle cannot split it into two logical presses.
    """
    if self._tja_restore_target is None:
      return False
    if CC.cruiseControl.cancel:
      self._clear_tja_restore()
      return False
    echo_main = (self._tja_off_comp_last_tx_frame == self.frame - 1)
    if _driver_long_button(CS, ignore_main=echo_main):
      self._clear_tja_restore()
      return False
    if bool(CS.out.brakePressed) and not self._tja_restore_brake_at_edge:
      self._clear_tja_restore()
      return False
    if _crz_btns_held(CS):
      return False
    current = _cruise_label(CS)
    if current == "ACTIVE":
      # Deliberate longitudinal engagement, or an unexpected ACTIVE side-effect.
      # Do not invent RES/SET. Driver input already wins if it was a button.
      self._clear_tja_restore()
      return False
    if current == self._tja_restore_target:
      if self._tja_off_comp_tx:
        self._stop_tja_restore()
        return False
      # Snapshot already matches: wait for a possible delayed OEM side-effect.
      # RESTORE_WHEN_STATE_ALREADY_MATCHES_SNAPSHOT=0 — do not TX.
      self._tja_off_comp_wait += 1
      if self._tja_off_comp_wait > TJA_RESTORE_MRCC_TIMEOUT_FRAMES:
        self._stop_tja_restore()
      return False
    # Mismatch: OFF target + ARMED now, or ARMED target + OFF now.
    if self._tja_off_comp_tx >= TJA_RESTORE_MRCC_MAX_TX:
      self._stop_tja_restore()
      return False
    wheel_ctr = int(getattr(CS, "crz_btns_counter", 0)) % 16
    if self._tja_restore_period_ctr is None:
      if self._tja_restore_mismatch_ctr is None:
        self._tja_restore_mismatch_ctr = wheel_ctr
        return False
      if wheel_ctr == self._tja_restore_mismatch_ctr:
        # Fail closed if CarState never sees a wheel tick: starting anyway
        # would span a real idle we did not observe (second logical press).
        return False
      self._tja_restore_period_ctr = wheel_ctr
      self._tja_restore_locked_ctr = (wheel_ctr + 1) % 16
      self._tja_restore_ctrs.add(self._tja_restore_locked_ctr)
    elif wheel_ctr != self._tja_restore_period_ctr:
      # Next physical wheel frame would insert MRCC=0 and split the press.
      self._stop_tja_restore()
      return False
    if len(self._tja_restore_ctrs) > TJA_RESTORE_MRCC_MAX_UNIQUE_CTR:
      self._stop_tja_restore()
      return False
    self._tja_off_comp_tx += 1
    self._tja_off_comp_last_tx_frame = self.frame
    if self._tja_off_comp_tx >= TJA_RESTORE_MRCC_MAX_TX:
      self._stop_tja_restore()
    return True

  def update_longitudinal(self, CC, CC_SP, CS, virtual_resume_sent):
    can_sends = []

    # Radar session sequencing: hold off the teardown until the FSC's cold-boot
    # radar-presence check has cleared (carstate's settle timer), keep the radar in its
    # programming session while we own the bus, and on an onroad toggle-off return it
    # to the default session before card requests the process restart. Never yank the
    # radar out from under an active stock MRCC engagement (driver SET before the gate
    # passed on a warm boot): wait for the driver to disengage first.
    stock_radar_alive = CS.stock_radar_alive
    teardown_ok = CS.fsc_settled and not (stock_radar_alive and CS.out.cruiseState.enabled)
    session_state = self.radar_session.update(teardown_ok, stock_radar_alive, CC_SP.stockEcuHandBack)
    # synthetic radar frames flow while we own the bus, and keep flowing through the
    # hand-back so the camera never sees a radar gap
    radar_master = session_state in (RadarSessionState.SILENCED, RadarSessionState.HANDBACK)

    if self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
      if session_state == RadarSessionState.SILENCING:
        can_sends.append(create_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING))
      elif session_state == RadarSessionState.HANDBACK:
        can_sends.append(create_radar_session_msg(uds.SESSION_TYPE.DEFAULT))
      elif session_state == RadarSessionState.SILENCED:
        # keeps the radar in its diagnostic session, and with it the stock frames silenced
        can_sends.append(make_tester_present_msg(RADAR_ADDR, 0, suppress_response=True))

    # only trust a virtual resume once a RES frame has actually gone out on the bus
    if not CC.cruiseControl.resume or not CS.out.standstill:
      self.virtual_resume_latched = False
    elif virtual_resume_sent:
      self.virtual_resume_latched = True

    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    # A gas press is an override, not a disengagement. The command goes to zero as everywhere
    # else, but the engaged bits stay set off CC.enabled the way Honda drives ACC_CONTROL's
    # CONTROL_ON. Clearing them mid-decel takes the PCM out of ACC mode as the driver adds
    # throttle, so a light pedal input lands as a lurch and a rev flare; stock MRCC holds them
    # through 9 of 11 decel overrides (analyze_gas_override.py, 576 stock segments).
    gas_override = CC.enabled and (CC.cruiseControl.override or CS.out.gasPressed)
    long_engaged = CC.longActive or gas_override
    sm = self.stop_and_go
    state = sm.update(long_engaged, stopping, CS.out.standstill,
                      resume_pressed=bool(CS.resume_button),
                      virtual_resume=self.virtual_resume_latched,
                      gas_override=gas_override)

    accel = 0.
    if CC.longActive:
      accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      # Slew limit the plan-following command. accel_last is tracked through overrides too, so
      # taking control back when the driver lifts off ramps in instead of stepping. The hold and
      # resume commands below are byte-exact stock replays and bypass the limit.
      accel = rate_limit(accel, self.accel_last, CarControllerParams.ACCEL_WINDDOWN_LIMIT,
                         CarControllerParams.ACCEL_WINDUP_LIMIT)
      if state == StopGoState.HOLD:
        accel = CarControllerParams.ACCEL_HOLD
      elif state in (StopGoState.HOLD_LATCHED, StopGoState.HOLD_PASSIVE):
        accel = CarControllerParams.ACCEL_HOLD_LATCHED
      elif state == StopGoState.RESUMING:
        # brake-release window: let the car creep off the hold, never brake into it
        accel = max(accel, 0.)
    self.accel_last = accel

    lead_visible = CC.hudControl.leadVisible
    if radar_master and self.frame % CarControllerParams.RADAR_STEP == 0:
      synthetic_lead = long_engaged and (lead_visible or state != StopGoState.CRUISING)
      for bus in LONG_BUSES:
        can_sends.extend(mazdacan.create_radar_frames(bus, self.radar_counter, synthetic_lead))
      self.radar_counter += 1

    if radar_master and self.frame % CarControllerParams.LONG_STEP == 0:
      acc_available = CS.out.cruiseState.available
      # mirror the driver's distance setting on the dash; stock shows gap 2 by default
      gap = (int(CC.hudControl.leadDistanceBars) or 2) if (long_engaged or acc_available) else 0
      if long_engaged:
        has_lead = sm.radar_has_lead(lead_visible)
        phase = sm.ctrl_phase(lead_visible)
        acc_active_2 = sm.acc_active_2
      else:
        has_lead = False
        phase = 0
        acc_active_2 = False
      for bus in LONG_BUSES:
        can_sends.append(mazdacan.create_acc_command(self.packer, bus, self.long_counter, accel,
                                                     long_engaged, acc_available,
                                                     stopping=sm.stop_bits, resume_unlatching=sm.resume_unlatching))
        can_sends.append(mazdacan.create_crz_ctrl(self.packer, bus, long_engaged, acc_available, gap,
                                                  has_lead, phase, acc_active_2))
      self.long_counter += 1

    return can_sends
