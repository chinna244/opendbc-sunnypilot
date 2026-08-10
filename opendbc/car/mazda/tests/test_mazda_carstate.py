import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
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
  # Production / model-test shape: one (timestamp, frames) packet, not a list of packets.
  ret, _ = CI.update((0, [msg]))
  return ret.buttonEvents


@pytest.mark.parametrize("alpha_long", [False, True])
def test_carstate_runs_with_real_parsers(alpha_long):
  CI = _interface(alpha_long=alpha_long)
  assert CI.CP.openpilotLongitudinalControl == alpha_long
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


@pytest.mark.parametrize(
  ("src_hex", "expected_hex"),
  [
    ("0001ffffffffff03", "0001ffffffffff03"),  # idle
    ("0009ffffffffff04", "0081feffffffff04"),  # physical TJA -> FSC MRCC
    ("0081feffffffff05", "0081feffffffff05"),  # stock-shaped physical MRCC unchanged
    ("0089ffffffffff06", "0081feffffffff06"),  # physical TJA+MRCC -> FSC MRCC
    ("0101ffffffffff07", "0101ffffffffff07"),  # CANCEL
    ("0401ffffffffff08", "0401ffffffffff08"),  # RESUME
    ("1001ffffffffff09", "1001ffffffffff09"),  # SET+
    ("2001ffffffffff0a", "2001ffffffffff0a"),  # SET-
    ("8001ffffffffff0b", "8001ffffffffff0b"),  # distance
  ],
)
def test_sanitized_crz_btns_clone_preserves_source(src_hex, expected_hex):
  src = bytes.fromhex(src_hex)
  clone = mazdacan.create_sanitized_crz_btns_clone(src, bus=2)
  assert clone.address == 0x9D
  assert clone.src == 2
  assert clone.dat == bytes.fromhex(expected_hex)
  assert clone.dat[1] & 0x08 == 0
  assert bool(clone.dat[1] & 0x80) == bool(src[1] & 0x80 or src[1] & 0x08)
  assert bool(clone.dat[2] & 0x01) == (False if src[1] & 0x08 else bool(src[2] & 0x01))
  # CTR nibble and every bit outside TJA/MRCC/companion remain identical.
  assert (clone.dat[3] >> 2) & 0x0F == (src[3] >> 2) & 0x0F
  for i, (a, b) in enumerate(zip(src, clone.dat, strict=True)):
    if i == 1:
      assert (a & ~0x88) == (b & ~0x88)
    elif i == 2:
      assert (a & ~0x01) == (b & ~0x01)
    else:
      assert a == b


def test_stock_capture_tja_to_mrcc_shape():
  source_tja = bytes.fromhex("0009fff400000000")
  captured_mrcc = bytes.fromhex("0081fef400000000")
  assert mazdacan.create_sanitized_crz_btns_clone(source_tja, bus=2).dat == captured_mrcc


def test_interface_captures_raw_and_controller_emits_bus2_clone():
  CI = _interface()
  # Physical-like frame with undefined trailing bytes that DBC packing would zero.
  src = bytes.fromhex("0009ffffffffff0c")
  msg = CanData(0x9D, src, 0)
  # Same single-packet shape used by test_models / panda_runner.
  CI.update((0, [msg]))
  assert CI.CS.crz_btns_raw_payloads == [src]

  CC = structs.CarControl().as_reader()
  CC_SP = structs.CarControlSP()
  _, can_sends = CI.apply(CC, CC_SP, now_nanos=0)
  clones = [m for m in can_sends if m[0] == 0x9D and m[2] == 2]
  assert len(clones) == 1
  assert clones[0][1] == bytes.fromhex("0081feffffffff0c")
  assert CI.CS.crz_btns_raw_payloads == []

  # TX echoes / non-bus0 frames must not be treated as physical sources.
  CI.update((0, [CanData(0x9D, src, 128)]))
  assert CI.CS.crz_btns_raw_payloads == []

  # Non-CRZ_BTNS frames ignored; list-of-packets shape still works (card path).
  other = CanData(0x202, bytes(8), 0)
  CI.update([(0, [other, msg])])
  assert CI.CS.crz_btns_raw_payloads == [src]
