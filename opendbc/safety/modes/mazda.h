#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_RADAR_STATIC  0x499U
#define MAZDA_RADAR_TRACK_1 0x361U
#define MAZDA_RADAR_TRACK_2 0x362U
#define MAZDA_RADAR_TRACK_3 0x363U
#define MAZDA_RADAR_TRACK_4 0x364U
#define MAZDA_RADAR_TRACK_5 0x365U
#define MAZDA_RADAR_TRACK_6 0x366U
#define MAZDA_RADAR_UDS     0x764U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

#define MAZDA_PARAM_LONGITUDINAL 1U
#define MAZDA_PARAM_TJA 2U

// Physical CRZ_BTNS bits (DBC Motorola start bit == GET_BIT flat numbering).
#define MAZDA_CRZ_BTNS_TJA_BIT 11U
#define MAZDA_CRZ_BTNS_MRCC_BIT 15U
#define MAZDA_CRZ_BTNS_SET_P_BIT 4U
#define MAZDA_CRZ_BTNS_SET_M_BIT 5U
#define MAZDA_CRZ_BTNS_RES_BIT 2U
#define MAZDA_CRZ_BTNS_CAN_OFF_BIT 0U
#define MAZDA_CRZ_BTNS_MODE_Y_BIT 13U
#define MAZDA_CRZ_BTNS_MODE_X_BIT 14U

// Per-edge MRCC restore. Physical TJA toggles MADS only. OEM TJA_BUTTON may
// also flip cruise OFF↔ARMED. After TJA release, if the post-TJA cruise state
// differs from the confident pre-edge snapshot, allow a bounded clean
// MRCC_BUTTON gesture to restore that snapshot.
//
// Stage 2M: at most TWO logical presses, each filling one 10 Hz wheel period
// with one locked CTR. Attempt #2 is only the next physical wheel period of
// the SAME TJA transaction, only while still mismatched, only inside 500 ms
// of TJA release. A third period / CTR / attempt is rejected. Spanning a
// wheel idle inside one attempt is a second press (route 00000019) and is
// still rejected. Pre-ACTIVE / UNKNOWN never authorize. Lateral does not gate.
#define MAZDA_RESTORE_NONE 0U
#define MAZDA_RESTORE_OFF 1U
#define MAZDA_RESTORE_ARMED 2U
#define MAZDA_RESTORE_WINDOW_US 500000U
#define MAZDA_RESTORE_MRCC_MAX_TX 10U
#define MAZDA_RESTORE_MAX_ATTEMPTS 2U
#define MAZDA_RESTORE_MRCC_MAX_TX_TOTAL 20U
#define MAZDA_RESTORE_CTR_UNSET 0xFFU

typedef enum {
  MAZDA_TJA_UNINITIALIZED = 0,
  MAZDA_TJA_ARMED = 1,
  MAZDA_TJA_HELD = 2,
  MAZDA_TJA_WAIT_FOR_RELEASE = 3,
} MazdaTjaEdgeState;

static bool mazda_longitudinal = false;
static bool mazda_tja_button = false;
static MazdaTjaEdgeState mazda_tja_edge_state = MAZDA_TJA_UNINITIALIZED;
static uint8_t mazda_restore_target = MAZDA_RESTORE_NONE;
static bool mazda_restore_released = false;
static bool mazda_restore_seen_mismatch = false;
static uint8_t mazda_restore_tx = 0;
static uint8_t mazda_restore_tx_total = 0;
static uint8_t mazda_restore_attempts = 0;
static uint32_t mazda_restore_ts = 0;
static uint8_t mazda_restore_ctr = MAZDA_RESTORE_CTR_UNSET;
static uint8_t mazda_last_rx_ctr = MAZDA_RESTORE_CTR_UNSET;
static uint8_t mazda_restore_period_ctr = MAZDA_RESTORE_CTR_UNSET;
static bool mazda_restore_need_new_period = false;

static void mazda_tja_edge_reset(void) {
  mazda_tja_edge_state = MAZDA_TJA_UNINITIALIZED;
}

static bool mazda_restore_mismatch(void);

static void mazda_restore_reset(void) {
  mazda_restore_target = MAZDA_RESTORE_NONE;
  mazda_restore_released = false;
  mazda_restore_seen_mismatch = false;
  mazda_restore_tx = 0;
  mazda_restore_tx_total = 0;
  mazda_restore_attempts = 0;
  mazda_restore_ts = 0;
  mazda_restore_ctr = MAZDA_RESTORE_CTR_UNSET;
  mazda_restore_period_ctr = MAZDA_RESTORE_CTR_UNSET;
  mazda_restore_need_new_period = false;
}

static bool mazda_restore_window_time_ok(void) {
  const uint32_t elapsed = safety_get_ts_elapsed(microsecond_timer_get(), mazda_restore_ts);
  return elapsed <= MAZDA_RESTORE_WINDOW_US;
}

// Finish the current logical press. Attempt #2 stays armed only for the same
// TJA transaction, still-mismatched OEM state, and open 500 ms window, and
// only after the next physical wheel period.
static void mazda_restore_end_attempt(void) {
  mazda_restore_attempts += 1U;
  mazda_restore_tx = 0;
  mazda_restore_ctr = MAZDA_RESTORE_CTR_UNSET;
  mazda_restore_need_new_period = true;
  if ((mazda_restore_attempts >= MAZDA_RESTORE_MAX_ATTEMPTS) ||
      (mazda_restore_tx_total >= MAZDA_RESTORE_MRCC_MAX_TX_TOTAL) ||
      !mazda_restore_released ||
      (mazda_restore_target == MAZDA_RESTORE_NONE) ||
      !mazda_restore_mismatch() ||
      !mazda_restore_window_time_ok()) {
    mazda_restore_reset();
  }
}

static bool mazda_crz_btns_restore_mrcc(const CANPacket_t *msg) {
  // MRCC_BUTTON only: TJA/SET/RES/CANCEL/MODE bits must be 0.
  return GET_BIT(msg, MAZDA_CRZ_BTNS_MRCC_BIT) &&
         !GET_BIT(msg, MAZDA_CRZ_BTNS_CAN_OFF_BIT) &&
         !GET_BIT(msg, MAZDA_CRZ_BTNS_RES_BIT) &&
         !GET_BIT(msg, MAZDA_CRZ_BTNS_SET_P_BIT) &&
         !GET_BIT(msg, MAZDA_CRZ_BTNS_SET_M_BIT) &&
         !GET_BIT(msg, MAZDA_CRZ_BTNS_TJA_BIT) &&
         !GET_BIT(msg, MAZDA_CRZ_BTNS_MODE_Y_BIT) &&
         !GET_BIT(msg, MAZDA_CRZ_BTNS_MODE_X_BIT);
}

static bool mazda_crz_btns_driver_long(const CANPacket_t *msg) {
  return GET_BIT(msg, MAZDA_CRZ_BTNS_MRCC_BIT) ||
         GET_BIT(msg, MAZDA_CRZ_BTNS_SET_P_BIT) ||
         GET_BIT(msg, MAZDA_CRZ_BTNS_SET_M_BIT) ||
         GET_BIT(msg, MAZDA_CRZ_BTNS_RES_BIT) ||
         GET_BIT(msg, MAZDA_CRZ_BTNS_CAN_OFF_BIT);
}

static bool mazda_restore_mismatch(void) {
  // Only restore when post-TJA cruise differs from the snapshot in an allowed way.
  // ACTIVE (controls_allowed) is never a restore target.
  bool mismatch = false;
  if (controls_allowed) {
    mismatch = false;
  } else if (mazda_restore_target == MAZDA_RESTORE_OFF) {
    mismatch = acc_main_on;
  } else if (mazda_restore_target == MAZDA_RESTORE_ARMED) {
    mismatch = !acc_main_on;
  } else {
    mismatch = false;
  }
  return mismatch;
}

static bool mazda_restore_window_open(void) {
  // MADS/lateral may be on or off: restore is per-edge, not session-owned.
  bool open = false;
  if (!mazda_tja_button || (mazda_restore_target == MAZDA_RESTORE_NONE) ||
      !mazda_restore_released || !mazda_restore_mismatch()) {
    open = false;
  } else if ((mazda_restore_attempts >= MAZDA_RESTORE_MAX_ATTEMPTS) ||
             (mazda_restore_tx_total >= MAZDA_RESTORE_MRCC_MAX_TX_TOTAL)) {
    mazda_restore_reset();
    open = false;
  } else if (mazda_restore_tx >= MAZDA_RESTORE_MRCC_MAX_TX) {
    mazda_restore_end_attempt();
    open = false;
  } else if (!mazda_restore_window_time_ok()) {
    mazda_restore_reset();
    open = false;
  } else if (mazda_restore_need_new_period) {
    open = false;
  } else {
    open = true;
  }
  return open;
}

static void mazda_restore_cruise_update(void) {
  if (controls_allowed) {
    // Became ACTIVE: driver longitudinal action. Never send MRCC to "preserve" ACTIVE.
    mazda_restore_reset();
  } else if (!mazda_restore_released || (mazda_restore_target == MAZDA_RESTORE_NONE)) {
    // no restore in flight
  } else if (mazda_restore_mismatch()) {
    mazda_restore_seen_mismatch = true;
  } else if (mazda_restore_seen_mismatch) {
    // Side-effect then returned to the snapshot: restore succeeded.
    mazda_restore_reset();
  } else {
    // snapshot still matches; keep waiting
  }
}

// Advance one physical CRZ_BTNS TJA sample. Returns true if and only if this sample toggles MADS once.
// Boot-held TJA (first sample = 1) does not toggle. Held frames do not repeat-toggle.
static bool mazda_tja_edge_update(const bool tja_pressed) {
  bool toggle = false;

  if (mazda_tja_edge_state == MAZDA_TJA_UNINITIALIZED) {
    mazda_tja_edge_state = tja_pressed ? MAZDA_TJA_WAIT_FOR_RELEASE : MAZDA_TJA_ARMED;
  } else if (mazda_tja_edge_state == MAZDA_TJA_WAIT_FOR_RELEASE) {
    if (!tja_pressed) {
      mazda_tja_edge_state = MAZDA_TJA_ARMED;
    }
  } else if (mazda_tja_edge_state == MAZDA_TJA_ARMED) {
    if (tja_pressed) {
      toggle = true;
      mazda_tja_edge_state = MAZDA_TJA_HELD;
    }
  } else if (mazda_tja_edge_state == MAZDA_TJA_HELD) {
    if (!tja_pressed) {
      mazda_tja_edge_state = MAZDA_TJA_ARMED;
    }
  } else {
    // Unknown state: fail closed. A missed toggle is allowed; an extra is not.
    mazda_tja_edge_state = MAZDA_TJA_UNINITIALIZED;
  }

  return toggle;
}

// With longitudinal control the stock radar is silenced and openpilot replays its frames,
// so allowed tx patterns are pinned to byte-exact stock captures wherever possible.

static bool mazda_radar_static_msg_valid(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) && (msg->data[1] == 0x08U) &&
         (msg->data[2] == 0xc0U) && (msg->data[3] == 0x00U) &&
         (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

static bool mazda_empty_radar_track_msg_valid(const CANPacket_t *msg) {
  bool valid = false;

  if ((msg->addr == MAZDA_RADAR_TRACK_1) || (msg->addr == MAZDA_RADAR_TRACK_2) ||
      (msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0xfeU) &&
            (msg->data[4] == 0x1fU);

    if (msg->addr == MAZDA_RADAR_TRACK_2) {
      valid = valid && (msg->data[5] == 0xc7U) && (msg->data[6] == 0x8cU) &&
              ((msg->data[7] & 0xf0U) == 0x80U);
    } else if ((msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x00U);
    } else {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x80U);
    }
  } else if ((msg->addr == MAZDA_RADAR_TRACK_5) || (msg->addr == MAZDA_RADAR_TRACK_6)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0x7fU) &&
            (msg->data[4] == 0xfbU) && (msg->data[5] == 0xffU) &&
            (msg->data[6] == 0x3fU) && ((msg->data[7] & 0xf0U) == 0xc0U);
  } else {
    // Caller only uses this for TRACK_1..TRACK_6.
  }

  return valid;
}

static bool mazda_synthetic_lead_radar_track_msg_valid(const CANPacket_t *msg) {
  return (msg->addr == MAZDA_RADAR_TRACK_4) &&
         (msg->data[0] == 0x0aU) && (msg->data[1] == 0x40U) &&
         (msg->data[2] == 0x00U) && (msg->data[3] == 0x00U) &&
         (msg->data[4] == 0x1dU) && (msg->data[5] == 0xc0U) &&
         (msg->data[6] == 0x00U) && ((msg->data[7] & 0xf0U) == 0x00U);
}

static bool mazda_radar_track_msg_valid(const CANPacket_t *msg) {
  return mazda_empty_radar_track_msg_valid(msg) ||
         (controls_allowed && mazda_synthetic_lead_radar_track_msg_valid(msg));
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((msg->addr == MAZDA_CRZ_CTRL) && !mazda_longitudinal) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 17U);
      mazda_restore_cruise_update();
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_tja_button) {
      // Single physical TJA rising edge toggles MADS/lateral exactly once.
      // Brake/gas/MRCC/SET/RES/CANCEL are not gesture conflicts.
      const bool tja_pressed = GET_BIT(msg, MAZDA_CRZ_BTNS_TJA_BIT);
      const bool was_held = (mazda_tja_edge_state == MAZDA_TJA_HELD);
      const bool toggle = mazda_tja_edge_update(tja_pressed);
      const bool released = was_held && !tja_pressed;
      const bool driver_long = mazda_crz_btns_driver_long(msg);
      const uint8_t rx_ctr = (msg->data[3] >> 2) & 0x0FU;
      mazda_last_rx_ctr = rx_ctr;

      mads_button_press = MADS_BUTTON_NOT_PRESSED;
      if (toggle) {
        // Every physical TJA edge: snapshot OEM cruise and toggle MADS.
        // Restore only if post-TJA cruise later differs from this snapshot.
        mazda_restore_reset();
        if (!acc_main_on && !controls_allowed) {
          mazda_restore_target = MAZDA_RESTORE_OFF;
        } else if (acc_main_on && !controls_allowed) {
          mazda_restore_target = MAZDA_RESTORE_ARMED;
        } else {
          // ACTIVE: never a restore target
        }
        if (controls_allowed_lateral) {
          mads_exit_controls(MADS_DISENGAGE_REASON_BUTTON);
        } else {
          mads_button_press = MADS_BUTTON_PRESSED;
        }
      } else if (released && (mazda_restore_target != MAZDA_RESTORE_NONE)) {
        mazda_restore_released = true;
        mazda_restore_ts = microsecond_timer_get();
        mazda_restore_tx = 0;
        if (mazda_restore_mismatch()) {
          mazda_restore_seen_mismatch = true;
        }
      } else {
        // held/idle sample: no snapshot change
      }

      if (driver_long && (mazda_restore_target != MAZDA_RESTORE_NONE)) {
        mazda_restore_reset();
      }
      // Held TJA after release is a new gesture; close the window.
      if (mazda_restore_released && tja_pressed) {
        mazda_restore_reset();
      }
      // Physical wheel idle on a new CTR after restore TX is the logical
      // release of this attempt. Attempt #2 may use that next period of the
      // same TJA transaction; a third period is rejected.
      if (mazda_restore_need_new_period && !tja_pressed && !driver_long &&
          (mazda_restore_period_ctr != MAZDA_RESTORE_CTR_UNSET) &&
          (rx_ctr != mazda_restore_period_ctr)) {
        mazda_restore_need_new_period = false;
        mazda_restore_period_ctr = MAZDA_RESTORE_CTR_UNSET;
        mazda_restore_ctr = MAZDA_RESTORE_CTR_UNSET;
      }
      if ((mazda_restore_tx > 0U) && !tja_pressed && !driver_long &&
          (mazda_restore_period_ctr != MAZDA_RESTORE_CTR_UNSET) &&
          (rx_ctr != mazda_restore_period_ctr)) {
        mazda_restore_end_attempt();
        if (mazda_restore_target != MAZDA_RESTORE_NONE) {
          // This RX sample is already the next physical period.
          mazda_restore_need_new_period = false;
          mazda_restore_period_ctr = MAZDA_RESTORE_CTR_UNSET;
          mazda_restore_ctr = MAZDA_RESTORE_CTR_UNSET;
        }
      }
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      // ensure the driver's cancel press always exits controls
      bool cancel = GET_BIT(msg, 0U);
      if (cancel) {
        controls_allowed = false;
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        // The radar teardown removes the stock CRZ_CTRL frame, so derive cruise state from
        // PEDALS: ACC_OFF (bit 2) means MRCC is armed but idle, ACC_ACTIVE (bit 3) means
        // engaged. Brake-only samples can arrive with both bits low mid-press; skip those
        // so they are not mistaken for an ACC-off edge.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;

        if (acc_armed || cruise_engaged_prev || (!brake && !brake_pressed_prev)) {
          acc_main_on = acc_armed;
          pcm_cruise_check(cruise_engaged);
        }
        mazda_restore_cruise_update();
      }
      brake_pressed = brake;
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  // Envelope sized for the CX-5 2022+ EPS, which the controller commands up to (max_torque 1200,
  // driver_torque_multiplier 15 vs upstream stock 800/1). SafetyModel.mazda is per-brand and can't
  // see the fingerprint/EPS, so these limits apply to every Mazda. Non-CX-5-EPS Mazdas self-cap
  // lower in the controller (values.py gates the tune on minSteerSpeed == 0), so this is only a
  // looser backstop for them — not a behavior change. Per-car gating would need a safety param.
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rate_up = 12,
    .max_rate_down = 25,
    .max_rt_delta = 384,
    .driver_torque_multiplier = 15,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // CRZ_INFO.ACCEL_CMD is raw units of 0.001 m/s2 (offset removed below), so this is the
  // ISO window: 2.0 / -3.5 m/s2. Stock MRCC itself commands down to raw -3891 in lead stops.
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool main_bus = msg->bus == (unsigned char)MAZDA_MAIN;
  bool long_replacement_bus = main_bus || (msg->bus == (unsigned char)MAZDA_CAM);

  // steer cmd checks
  if (main_bus && (msg->addr == MAZDA_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    // TJA platforms: lateral torque is owned solely by controls_allowed_lateral (MADS/TJA).
    // MRCC/longitudinal controls_allowed alone must not authorize nonzero steering.
    // Zero torque remains allowed for safe disengagement. Non-TJA Mazdas unchanged.
    if (mazda_tja_button && !controls_allowed_lateral && (desired_torque != 0)) {
      tx = false;
    } else if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
      tx = false;
    } else {
      // torque within limits
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    // the stock standby pattern pegs the command field high; allow it byte-exactly
    // (checksum included) instead of decoding it as a huge accel command
    bool stock_standby = (msg->data[0] == 0x01U) && (msg->data[1] == 0xffU) &&
                         (msg->data[2] == 0xe3U) && (msg->data[3] == 0xffU) &&
                         (msg->data[4] == 0xc0U) && (msg->data[5] == 0x00U) &&
                         ((msg->data[6] & 0xf0U) == 0x00U) &&
                         (msg->data[7] == ((0x5dU - msg->data[6]) & 0xffU));

    // 13-bit ACCEL_CMD: data[2] low bits, data[3], data[4] high bits, offset 4096
    const uint32_t raw_accel = (((uint32_t)msg->data[2] & 0x3U) << 11) |
                               ((uint32_t)msg->data[3] << 3) |
                               ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)raw_accel - 4096;
    if (!stock_standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    bool cruise_active = GET_BIT(msg, 3U);
    if (!controls_allowed && cruise_active) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_RADAR_STATIC)) {
    if (!mazda_radar_static_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr >= MAZDA_RADAR_TRACK_1) && (msg->addr <= MAZDA_RADAR_TRACK_6)) {
    if (!mazda_radar_track_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && main_bus && (msg->addr == MAZDA_RADAR_UDS)) {
    // only tester present and default/programming session control; flashing services stay blocked
    bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3eU) && (msg->data[2] == 0x80U);
    bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                           ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
    if (!tester_present && !session_control) {
      tx = false;
    }
  }

  // cruise buttons check
  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    const bool cancel_cmd = (msg->data[0] == 0x1U);
    if (mazda_tja_button && GET_BIT(msg, MAZDA_CRZ_BTNS_TJA_BIT)) {
      tx = false;
    }
    const bool restore_mrcc = mazda_crz_btns_restore_mrcc(msg);
    if (restore_mrcc) {
      if (!mazda_restore_window_open()) {
        tx = false;
      } else {
        // CRZ_BTNS CTR is DBC 29|4@0+ Motorola → data[3] bits 5..2.
        const uint8_t ctr = (msg->data[3] >> 2) & 0x0FU;
        if (mazda_restore_ctr == MAZDA_RESTORE_CTR_UNSET) {
          mazda_restore_ctr = ctr;
        } else if (ctr != mazda_restore_ctr) {
          // Second unique CTR is a second logical press. Reject it.
          tx = false;
          mazda_restore_reset();
        } else {
          // same locked CTR
        }
        if (tx) {
          if (mazda_restore_period_ctr == MAZDA_RESTORE_CTR_UNSET) {
            mazda_restore_period_ctr = mazda_last_rx_ctr;
          }
          mazda_restore_tx += 1U;
          mazda_restore_tx_total += 1U;
          if ((mazda_restore_tx >= MAZDA_RESTORE_MRCC_MAX_TX) ||
              (mazda_restore_tx_total >= MAZDA_RESTORE_MRCC_MAX_TX_TOTAL)) {
            mazda_restore_end_attempt();
          }
        }
      }
    } else if (!controls_allowed && !cancel_cmd) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed
      tx = false;
    } else {
      // cancel while disallowed, or any button while controls_allowed
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
  };

  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // no CRZ_CTRL check: the stock radar frame disappears after the teardown
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  mazda_tja_button = GET_FLAG(param, MAZDA_PARAM_TJA);
  mads_physical_button_only = mazda_tja_button;
  mazda_tja_edge_reset();
  mazda_restore_reset();
  acc_main_on = false;

  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};

#ifdef ALLOW_DEBUG
// Test-only accessors. Not compiled into car firmware (ALLOW_DEBUG is libsafety/mutation).
void set_mazda_tja_edge_state(uint8_t s) {
  mazda_tja_edge_state = (MazdaTjaEdgeState)s;
}

void set_mazda_restore_debug(uint8_t target, bool released, uint8_t tx) {
  mazda_restore_target = target;
  mazda_restore_released = released;
  mazda_restore_tx = tx;
  mazda_restore_tx_total = tx;
  mazda_restore_attempts = 0;
  mazda_restore_need_new_period = false;
  mazda_restore_ts = microsecond_timer_get();
  mazda_restore_seen_mismatch = true;
  mazda_restore_ctr = MAZDA_RESTORE_CTR_UNSET;
  mazda_restore_period_ctr = MAZDA_RESTORE_CTR_UNSET;
}
#endif
