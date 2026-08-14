from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.tja_edge import MazdaTjaEdge
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, CarControllerParams, MazdaSafetyFlags
from opendbc.sunnypilot.car.mazda.carstate_ext import CarStateExt

ButtonType = structs.CarState.ButtonEvent.Type

FSC_SETTLE_FRAMES = int(CarControllerParams.FSC_SETTLE_T / DT_CTRL)
STOCK_RADAR_ALIVE_FRAMES = int(CarControllerParams.STOCK_RADAR_ALIVE_T / DT_CTRL)
STOCK_RADAR_GUARD_FRAMES = int(CarControllerParams.STOCK_RADAR_GUARD_T / DT_CTRL)


class CarState(CarStateBase, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.cancel_button = 0
    self.resume_button = 0
    self.tja_button = 0
    self.main_button = 0
    self._tja_edge: MazdaTjaEdge | None = None
    self._tja_lkas_latched = False
    self.tja_toggles_this_update = 0
    self.tja_edge_reset = False
    self._panda_alive = True
    self._panda_was_dead = False
    if CP.safetyConfigs and (CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.TJA):
      self._tja_edge = MazdaTjaEdge()

    self.cruise_available = False
    self.cruise_enabled = False
    # Previous-cycle cruise, captured before this update's CRZ_CTRL/PEDALS parse.
    # Used as pre-TJA longitudinal state so we do not treat OEM's TJA→arm side
    # effect as "driver already had MRCC armed".
    self._prev_cruise_available = False
    self._prev_cruise_enabled = False
    self.tja_pre_cruise_available = False
    self.tja_pre_cruise_enabled = False
    # Driver MRCC/SET/RES/CANCEL seen since the last confirmed cruiseState change.
    # Until CRZ_CTRL/PEDALS moves, previous-cycle OFF is stale, not confident OFF.
    self._unconfirmed_long_btn = False
    self.tja_pre_cruise_unknown = False
    self.brake_pressed_prev = False
    self.stock_radar_silent_frames = 0
    self.cam_laneinfo_seen = False
    self.fsc_settled_frames = 0

  @property
  def fsc_settled(self) -> bool:
    return self.fsc_settled_frames >= FSC_SETTLE_FRAMES

  @property
  def stock_radar_alive(self) -> bool:
    return self.stock_radar_silent_frames < STOCK_RADAR_ALIVE_FRAMES

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD, 5)

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    if self.CP.openpilotLongitudinalControl:
      # The radar teardown silences the radar-owned CRZ_CTRL frame, so cruise state comes
      # from PEDALS: ACC_OFF means MRCC is armed but idle, ACC_ACTIVE means it is engaged.
      # Brake-only samples can arrive with both bits low mid-press; mirror the panda rx
      # guard and hold the previous state through them, else MADS sees a false
      # availability drop and force-disengages lateral.
      acc_armed = cp.vl["PEDALS"]["ACC_OFF"] == 1
      acc_active = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      brake_free = not ret.brakePressed and not self.brake_pressed_prev
      if acc_armed or acc_active:
        self.cruise_available = True
      elif brake_free:
        self.cruise_available = False
      if acc_armed or acc_active or self.cruise_enabled or brake_free:
        self.cruise_enabled = acc_active
      ret.cruiseState.available = self.cruise_available
      ret.cruiseState.enabled = self.cruise_enabled

      # Two-master guard: while the stock radar still broadcasts CRZ_INFO (teardown pending
      # or failed, or the radar recovered through its S3 timeout), our synthetic frames
      # would fight it on the bus, so block longitudinal engagement until it has been
      # silent for 1 second.
      if len(cp.vl_all["CRZ_INFO"]["CTR1"]) > 0:
        self.stock_radar_silent_frames = 0
      else:
        self.stock_radar_silent_frames += 1
      ret.accFaulted = self.stock_radar_silent_frames < STOCK_RADAR_GUARD_FRAMES

      # FSC settle timer (the radar teardown gate): the camera broadcasts a
      # boot-in-progress state on CAM_LANEINFO (NO_ERR_BIT + BIT2, pure boot markers
      # clearing at 2.8-6.0 s and never set again while driving), then runs a
      # radar-presence check in the following seconds. A latched fault (ERR_BIT) also
      # shows the boot markers clear, so it must hold the timer at zero. The seen latch
      # matters: before the first frame the parser reads all-zero, which would count as
      # settled.
      self.cam_laneinfo_seen |= len(cp_cam.vl_all["CAM_LANEINFO"]["LANE_LINES"]) > 0
      laneinfo = cp_cam.vl["CAM_LANEINFO"]
      settled = self.cam_laneinfo_seen and not any(laneinfo[s] for s in ("NO_ERR_BIT", "BIT2", "ERR_BIT"))
      self.fsc_settled_frames = self.fsc_settled_frames + 1 if settled else 0
    else:
      # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
      #       it should be used for carState.cruiseState.nonAdaptive instead
      ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    self.brake_pressed_prev = ret.brakePressed
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # stock lkas should be on
    # TODO: is this needed?
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    if self.CP.minSteerSpeed > 0:
      ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked
    else:
      # CX-5 2022: EPS accepts steering at all speeds regardless of LKAS_BLOCK.
      # Verified across 5.5M frames: LKAS_BLOCK never indicates a real steering failure.
      ret.steerFaultTemporary = False

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1

    # cruise control button events: distance, inc, dec, resume, cancel, TJA/LKAS, and main
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    prev_cancel_button = self.cancel_button
    prev_resume_button = self.resume_button
    prev_main_button = self.main_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    # On CX-5 2022 the wheel "+" button toggles SET_P (not RES); RES is the resume button.
    # Verified against route 0000019c--84a5408a38 seg2/3: holding "+" emits SET_P=1, body ECU increments CRZ_SPEED.
    self.accel_button = cp.vl["CRZ_BTNS"]["SET_P"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]
    # CAN_OFF carries the cancel intent. Without an event here, ICBM's readiness gate never
    # learns the driver is canceling, so it keeps spamming CRZ_BTNS with cancel=0 and the
    # body ECU treats the latest non-cancel frame as authoritative. Critical for cancel-safety.
    self.cancel_button = cp.vl["CRZ_BTNS"]["CAN_OFF"]
    self.resume_button = cp.vl["CRZ_BTNS"]["RES"]
    # Newer CX-5 wheels use MRCC_BUTTON; legacy wheels use MODE_X + MODE_Y.
    # Only the TJA platform reads MRCC_BUTTON so non-TJA Mazda button semantics stay unchanged.
    if self._tja_edge is not None:
      self.tja_button = cp.vl["CRZ_BTNS"]["TJA_BUTTON"]
      self.main_button = int(
        cp.vl["CRZ_BTNS"]["MRCC_BUTTON"] == 1 or
        (cp.vl["CRZ_BTNS"]["MODE_X"] == 1 and cp.vl["CRZ_BTNS"]["MODE_Y"] == 1)
      )
    else:
      self.main_button = int(cp.vl["CRZ_BTNS"]["MODE_X"] == 1 and cp.vl["CRZ_BTNS"]["MODE_Y"] == 1)

    lkas_events = []
    self.tja_toggles_this_update = 0
    long_rise_this_update = False
    if self._tja_edge is not None:
      # Step the edge SM on every physical CRZ_BTNS sample (vl_all), matching Panda RX.
      # Brake/gas/MRCC/SET/RES/CANCEL are not gesture conflicts.
      crz_all = cp.vl_all["CRZ_BTNS"]
      n_crz = len(crz_all["TJA_BUTTON"])
      # Pass 1: any MRCC/SET/RES/CANCEL rise in this batch, including samples
      # after the TJA edge. Previous-cycle OFF is then UNKNOWN, not confident OFF.
      sm_main = prev_main_button
      sm_set = prev_accel_button
      sm_dec = prev_decel_button
      sm_res = prev_resume_button
      sm_can = prev_cancel_button
      for i in range(n_crz):
        main_i = int(
          crz_all["MRCC_BUTTON"][i] == 1 or
          (crz_all["MODE_X"][i] == 1 and crz_all["MODE_Y"][i] == 1)
        )
        set_i = int(crz_all["SET_P"][i])
        dec_i = int(crz_all["SET_M"][i])
        res_i = int(crz_all["RES"][i])
        can_i = int(crz_all["CAN_OFF"][i])
        if ((main_i and not sm_main) or (set_i and not sm_set) or (dec_i and not sm_dec)
            or (res_i and not sm_res) or (can_i and not sm_can)):
          self._unconfirmed_long_btn = True
          long_rise_this_update = True
        sm_main, sm_set, sm_dec, sm_res, sm_can = main_i, set_i, dec_i, res_i, can_i
      toggle_count = 0
      for i in range(n_crz):
        if self._tja_edge.update(bool(crz_all["TJA_BUTTON"][i])):
          toggle_count += 1
      self.tja_toggles_this_update = toggle_count
      if toggle_count:
        # Previous-cycle cruise, not this cycle's CRZ_CTRL/PEDALS (TJA OEM arm).
        # UNKNOWN if a driver long button has not yet been confirmed.
        self.tja_pre_cruise_available = self._prev_cruise_available
        self.tja_pre_cruise_enabled = self._prev_cruise_enabled
        self.tja_pre_cruise_unknown = bool(self._unconfirmed_long_btn)
        # Do not mark TJA itself as an unconfirmed longitudinal button.
        # Route 00000019 event 53: TJA while ARMED stayed ARMED, so cruise
        # never moved and the next TJA snapshot became UNKNOWN — no restore.
        lkas_events = [
          structs.CarState.ButtonEvent(type=ButtonType.lkas, pressed=True)
          for _ in range(toggle_count)
        ]
        self._tja_lkas_latched = True
      elif self._tja_lkas_latched and not self.tja_button:
        lkas_events = [structs.CarState.ButtonEvent(type=ButtonType.lkas, pressed=False)]
        self._tja_lkas_latched = False

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
      *create_button_events(self.cancel_button, prev_cancel_button, {1: ButtonType.cancel}),
      *create_button_events(self.resume_button, prev_resume_button, {1: ButtonType.resumeCruise}),
      *lkas_events,
      *create_button_events(self.main_button, prev_main_button, {1: ButtonType.mainCruise}),
    ]

    CarStateExt.update(self, ret, ret_sp, can_parsers)

    new_avail = bool(ret.cruiseState.available)
    new_en = bool(ret.cruiseState.enabled)
    if new_avail != self._prev_cruise_available or new_en != self._prev_cruise_enabled:
      if long_rise_this_update:
        # CRZ_CTRL/PEDALS is parsed before buttons. A same-cycle cruise change
        # cannot prove this cycle's new MRCC/SET/RES/CANCEL; keep UNKNOWN.
        self._unconfirmed_long_btn = True
      else:
        self._unconfirmed_long_btn = False
    self._prev_cruise_available = new_avail
    self._prev_cruise_enabled = new_en

    return ret, ret_sp

  def reset_tja(self) -> None:
    """Panda reconnect / safety reinit: no boot-held synthetic edge, latch starts OFF."""
    if self._tja_edge is not None:
      self._tja_edge.reset()
      self._tja_lkas_latched = False
      self.tja_toggles_this_update = 0
      self._unconfirmed_long_btn = True
      self.tja_pre_cruise_unknown = True
      self.tja_edge_reset = True

  def on_panda_alive(self, alive: bool) -> None:
    if self._tja_edge is None:
      return
    if self._panda_alive and not alive:
      self._panda_was_dead = True
    if self._panda_was_dead and alive:
      self.reset_tja()
      self._panda_was_dead = False
    self._panda_alive = bool(alive)

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = []
    tja = bool(CP.safetyConfigs and (CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.TJA))
    if tja:
      # Registered (not lazy): single-TJA reads every CRZ_BTNS frame via vl_all so
      # userspace edge detection matches Panda's per-RX step.
      pt_messages.append(("CRZ_BTNS", 10))
    if CP.openpilotLongitudinalControl:
      # no liveness check: the stock frame is expected to disappear after the radar
      # teardown, and its presence is what the two-master guard watches for
      pt_messages.append(("CRZ_INFO", float("nan")))
    cam_messages = [
      # read through vl_all, which unlike vl has no lazy registration
      ("CAM_LANEINFO", 0),
      ("CAM_TRAFFIC_SIGNS", 0),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
