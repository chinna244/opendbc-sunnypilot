"""Minimal single-press TJA edge detector.

Physical TJA is a toggle switch for MADS. Semantics (Panda and userspace must match):

  UNINITIALIZED + first sample 0 → ARMED
  UNINITIALIZED + first sample 1 → WAIT_FOR_RELEASE  (boot-held: no toggle)
  WAIT_FOR_RELEASE + 0 → ARMED
  ARMED + 0→1 → toggle once, then HELD
  HELD + 1 → HELD
  HELD + 1→0 → ARMED

A lost frame may miss a toggle. Duplicate/held frames must never create an extra toggle.
Brake/gas/MRCC/SET/RES/CANCEL are ignored — they are not gesture conflicts.
"""

from __future__ import annotations

UNINITIALIZED = 0
ARMED = 1
HELD = 2
WAIT_FOR_RELEASE = 3


class MazdaTjaEdge:
  def __init__(self) -> None:
    self.state = UNINITIALIZED

  def reset(self) -> None:
    self.state = UNINITIALIZED

  def update(self, tja: bool) -> bool:
    """Advance one physical CRZ_BTNS sample. Returns True iff this sample toggles MADS once."""
    pressed = bool(tja)
    toggle = False

    if self.state == UNINITIALIZED:
      self.state = WAIT_FOR_RELEASE if pressed else ARMED
    elif self.state == WAIT_FOR_RELEASE:
      if not pressed:
        self.state = ARMED
    elif self.state == ARMED:
      if pressed:
        toggle = True
        self.state = HELD
    elif self.state == HELD:
      if not pressed:
        self.state = ARMED

    return toggle
