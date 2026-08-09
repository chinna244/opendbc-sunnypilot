#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.mazdacan import CRZ_BTNS_ADDR
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, MazdaSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def update(self, can_packets: tuple[int, list[CanData]] | list[tuple[int, list[CanData]]]) -> tuple[structs.CarState, structs.CarStateSP]:
    # Capture exact physical CRZ_BTNS bytes before parsing. CANParser only keeps decoded
    # signals and would drop undefined payload bits needed for the FSC clone.
    #
    # Match CANParser.update() input normalization: production and model tests pass one
    # (t, frames) packet; card may pass a list of packets.
    packets: list[tuple[int, list[CanData]]]
    if can_packets and not isinstance(can_packets[0], (list, tuple)):
      packets = [can_packets]  # type: ignore[list-item]
    else:
      packets = can_packets  # type: ignore[assignment]

    payloads: list[bytes] = []
    for _, frames in packets:
      for msg in frames:
        address, dat, src = msg
        if address == CRZ_BTNS_ADDR and src == 0 and len(dat) == 8:
          payloads.append(bytes(dat))
    self.CS.crz_btns_raw_payloads = payloads
    return super().update(can_packets)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]

    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    # 2022+ CX-5 EPS can steer to zero and has no hands-off lockout. Detected by EPS firmware
    # rather than by model, so an EPS swapped into an older Mazda is recognized as what it is.
    steer_to_zero = candidate == CAR.MAZDA_CX5_2022 or \
      any(fw.ecu == 'eps' and fw.fwVersion in STEER_TO_ZERO_EPS_FW for fw in car_fw)
    if not steer_to_zero:
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.alphaLongitudinalAvailable = candidate == CAR.MAZDA_CX5_2022
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.LONG.value
      # engagement stays with the car: the driver SETs on the wheel, the body ECU raises
      # PEDALS.ACC_ACTIVE, and the dash-owned CRZ_EVENTS setpoint survives the radar teardown
      ret.pcmCruise = True
      ret.radarUnavailable = True
      ret.stopAccel = -1.0  # stock MRCC holds raw -1024 (-1.024 m/s2) at a stop
      ret.longitudinalActuatorDelay = 0.36  # measured ~0.3 s dead time + ~0.3 s first-order lag

    # Older Mazdas are dashcam only for one reason: their EPS locks steering out after ~5 s of
    # hands-off and below 45 kph. That is a property of the EPS, not of the car, so a car with
    # the 2022+ EPS swapped in is controllable and lifts with it. Longitudinal stays keyed on the
    # model above: the radar and camera are not part of an EPS swap.
    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021) and not steer_to_zero

    ret.enableBsm = 0x477 in fingerprint[0]

    # command-to-torque lag is EPS firmware, so it follows the EPS. lagd learns the rest
    # (0.338 total on a CX-5 2022; initial = this + 0.2)
    ret.steerActuatorDelay = 0.14 if steer_to_zero else 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret
