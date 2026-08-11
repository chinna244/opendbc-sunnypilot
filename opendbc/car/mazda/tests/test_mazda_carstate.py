import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR

ButtonType = structs.CarState.ButtonEvent.Type


def _interface(car=CAR.MAZDA_CX5_2022, alpha_long=False):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(car, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, car, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  CI = CarInterface(CP, CP_SP)
  # Mazda pt messages such as CRZ_BTNS are lazily registered on the first CarState update.
  CI.update([])
  return CI


def _send_crz_btns(CI, packer, values):
  msg = packer.make_can_msg("CRZ_BTNS", 0, values)
  ret, _ = CI.update([(0, [msg])])
  return ret.buttonEvents


@pytest.mark.parametrize("alpha_long", [False, True])
def test_carstate_runs_with_real_parsers(alpha_long):
  # vl_all, unlike vl, has no lazy message registration: every message read through it
  # must be listed in get_can_parsers. The op-long FSC settle gate crashed card on its
  # first update when CAM_LANEINFO was missing from the cam parser (KeyError, 2026-07-29).
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
  assert CP.openpilotLongitudinalControl == alpha_long

  CI = CarInterface(CP, CP_SP)
  for _ in range(10):
    CI.update([])


@pytest.mark.parametrize(
  ("values", "button_type"),
  [
    ({"TJA_BUTTON": 1}, ButtonType.lkas),
    ({"MRCC_BUTTON": 1}, ButtonType.mainCruise),
    ({"CAN_OFF": 1}, ButtonType.cancel),
  ],
)
def test_new_wheel_button_press_release_events(values, button_type):
  CI = _interface()
  packer = CANPacker("mazda_2017")

  pressed = _send_crz_btns(CI, packer, values)
  assert [(event.type, event.pressed) for event in pressed] == [(button_type, True)]

  released = _send_crz_btns(CI, packer, {})
  assert [(event.type, event.pressed) for event in released] == [(button_type, False)]


def test_legacy_main_button_press_release_unchanged():
  CI = _interface(car=CAR.MAZDA_CX5)
  packer = CANPacker("mazda_2017")

  pressed = _send_crz_btns(CI, packer, {"MODE_X": 1, "MODE_Y": 1})
  assert [(event.type, event.pressed) for event in pressed] == [(ButtonType.mainCruise, True)]

  released = _send_crz_btns(CI, packer, {})
  assert [(event.type, event.pressed) for event in released] == [(ButtonType.mainCruise, False)]
