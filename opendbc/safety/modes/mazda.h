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

// Stage 5B: userspace may TX a sanitized CRZ_BTNS clone to the camera bus only when it
// matches a recently received physical bus-0 source with TJA_BUTTON represented as MRCC.
#define MAZDA_CRZ_BTNS_TJA_BIT 11U
#define MAZDA_CRZ_BTNS_MRCC_BIT 15U
#define MAZDA_CRZ_BTNS_MRCC_COMPANION_BIT 16U
#define MAZDA_CRZ_BTNS_CLONE_QUEUE 8U
#define MAZDA_CRZ_BTNS_CLONE_TIMEOUT_US 100000U  // 100 ms freshness window

static bool mazda_longitudinal = false;

static uint8_t mazda_crz_btns_src[MAZDA_CRZ_BTNS_CLONE_QUEUE][8];
static uint32_t mazda_crz_btns_ts[MAZDA_CRZ_BTNS_CLONE_QUEUE];
static uint8_t mazda_crz_btns_q_r = 0U;
static uint8_t mazda_crz_btns_q_w = 0U;
static uint8_t mazda_crz_btns_q_n = 0U;

static void mazda_crz_btns_clone_reset(void) {
  mazda_crz_btns_q_r = 0U;
  mazda_crz_btns_q_w = 0U;
  mazda_crz_btns_q_n = 0U;
}

static void mazda_crz_btns_clone_enqueue(const CANPacket_t *msg) {
  // RX checks already require CRZ_BTNS length 8 before mazda_rx_hook runs.

  // Drop oldest if full so the queue tracks the freshest physical frames.
  if (mazda_crz_btns_q_n >= MAZDA_CRZ_BTNS_CLONE_QUEUE) {
    mazda_crz_btns_q_r = (uint8_t)((mazda_crz_btns_q_r + 1U) % MAZDA_CRZ_BTNS_CLONE_QUEUE);
    mazda_crz_btns_q_n--;
  }

  for (int i = 0; i < 8; i++) {
    mazda_crz_btns_src[mazda_crz_btns_q_w][i] = msg->data[i];
  }
  mazda_crz_btns_ts[mazda_crz_btns_q_w] = microsecond_timer_get();
  mazda_crz_btns_q_w = (uint8_t)((mazda_crz_btns_q_w + 1U) % MAZDA_CRZ_BTNS_CLONE_QUEUE);
  mazda_crz_btns_q_n++;
}

static void mazda_crz_btns_clone_drop_stale(void) {
  const uint32_t ts = microsecond_timer_get();
  while (mazda_crz_btns_q_n > 0U) {
    if (safety_get_ts_elapsed(ts, mazda_crz_btns_ts[mazda_crz_btns_q_r]) <= MAZDA_CRZ_BTNS_CLONE_TIMEOUT_US) {
      break;
    }
    mazda_crz_btns_q_r = (uint8_t)((mazda_crz_btns_q_r + 1U) % MAZDA_CRZ_BTNS_CLONE_QUEUE);
    mazda_crz_btns_q_n--;
  }
}

// True only when msg equals src with TJA_BUTTON cleared and, only for a physical TJA source,
// MRCC_BUTTON set and its captured bit-16 companion cleared. All other bits must remain
// identical, including CTR and undefined bits.
static bool mazda_crz_btns_sanitized_match(const CANPacket_t *msg, const uint8_t *src) {
  const bool tja_pressed = (src[MAZDA_CRZ_BTNS_TJA_BIT / 8U] &
                            (1U << (MAZDA_CRZ_BTNS_TJA_BIT % 8U))) != 0U;
  for (int i = 0; i < 8; i++) {
    uint8_t expected = src[i];
    if (i == (int)(MAZDA_CRZ_BTNS_TJA_BIT / 8U)) {
      expected &= (uint8_t)~(1U << (MAZDA_CRZ_BTNS_TJA_BIT % 8U));
    }
    if (tja_pressed && (i == (int)(MAZDA_CRZ_BTNS_MRCC_BIT / 8U))) {
      expected |= (uint8_t)(1U << (MAZDA_CRZ_BTNS_MRCC_BIT % 8U));
    }
    if (tja_pressed && (i == (int)(MAZDA_CRZ_BTNS_MRCC_COMPANION_BIT / 8U))) {
      expected &= (uint8_t)~(1U << (MAZDA_CRZ_BTNS_MRCC_COMPANION_BIT % 8U));
    }
    if (msg->data[i] != expected) {
      return false;
    }
  }
  return true;
}

static bool mazda_crz_btns_clone_tx_allowed(const CANPacket_t *msg) {
  // Caller only invokes this for bus-2 CRZ_BTNS; TX whitelist already requires length 8.

  mazda_crz_btns_clone_drop_stale();
  if (mazda_crz_btns_q_n == 0U) {
    return false;
  }

  if (!mazda_crz_btns_sanitized_match(msg, mazda_crz_btns_src[mazda_crz_btns_q_r])) {
    return false;
  }

  // Consume exactly one source frame per accepted replacement.
  mazda_crz_btns_q_r = (uint8_t)((mazda_crz_btns_q_r + 1U) % MAZDA_CRZ_BTNS_CLONE_QUEUE);
  mazda_crz_btns_q_n--;
  return true;
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
      }
      brake_pressed = brake;
    }

    // Capture physical CRZ_BTNS for the bus-2 sanitized-clone TX check. Bus-0 openpilot
    // synthesized button commands must not satisfy this path (they are TX, not RX).
    if (msg->addr == MAZDA_CRZ_BTNS) {
      mazda_crz_btns_clone_enqueue(msg);
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

    if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
      tx = false;
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
    int desired_accel = ((((int)msg->data[2] & 0x3) << 11) | (((int)msg->data[3]) << 3) | (((int)msg->data[4]) >> 5)) - 4096;
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

  // cruise buttons check (existing bus-0 synthesized cancel/resume path)
  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    // allow resume spamming while controls allowed, but
    // only allow cancel while controls not allowed
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
    }
  }

  // Stage 5B: bus-2 CRZ_BTNS must be an exact sanitized clone of a fresh physical source,
  // with physical TJA represented as MRCC for FSC state coherence.
  // Existing bus-0 button TX permission above must not satisfy or bypass this validator.
  if ((msg->bus == (unsigned char)MAZDA_CAM) && (msg->addr == MAZDA_CRZ_BTNS)) {
    if (!mazda_crz_btns_clone_tx_allowed(msg)) {
      tx = false;
    }
  }

  return tx;
}

// Block CRZ_BTNS (0x09D) bus 0 -> bus 2 so the FSC never sees physical TJA presses.
// openpilot still receives the original bus-0 RX for ButtonType.lkas / MADS.
// Stage 1 blocks the whole address (not payload-selective); bus 2 -> bus 0 is unchanged.
static bool mazda_fwd_hook(int bus_num, int addr) {
  return (bus_num == MAZDA_MAIN) && (addr == (int)MAZDA_CRZ_BTNS);
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
  };

  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
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
  acc_main_on = false;
  mazda_crz_btns_clone_reset();

  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .fwd = mazda_fwd_hook,
};
