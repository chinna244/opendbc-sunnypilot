"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntFlag


class MazdaFlagsSP(IntFlag):
  # Default-off experimental MADS HUD: exact captured GREEN while actively steering.
  EXPERIMENTAL_MADS_GREEN_HUD = 1
