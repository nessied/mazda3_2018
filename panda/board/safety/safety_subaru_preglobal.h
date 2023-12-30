const SteeringLimits SUBARU_PG_STEERING_LIMITS = {
  .max_steer = 2047,
  .max_rt_delta = 940,
  .max_rt_interval = 250000,
  .max_rate_up = 50,
  .max_rate_down = 70,
  .driver_torque_factor = 10,
  .driver_torque_allowance = 75,
  .type = TorqueDriverLimited,
};

const LongitudinalLimits SUBARU_PG_LONG_LIMITS = {
  .min_gas = 808,       // appears to be engine braking
  .max_gas = 3400,      // approx  2 m/s^2 when maxing cruise_rpm and cruise_throttle
  .inactive_gas = 1818, // this is zero acceleration
  .max_brake = 600,     // approx -3.5 m/s^2

  .min_transmission_rpm = 0,
  .max_transmission_rpm = 2400,
};

// Preglobal platform
// 0x161 is ES_CruiseThrottle
// 0x164 is ES_LKAS

#define MSG_SUBARU_PG_CruiseControl         0x144
#define MSG_SUBARU_PG_Throttle              0x140
#define MSG_SUBARU_PG_Wheel_Speeds          0xD4
#define MSG_SUBARU_PG_Brake_Pedal           0xD1
#define MSG_SUBARU_PG_ES_LKAS               0x164
#define MSG_SUBARU_PG_ES_Brake              0x160
#define MSG_SUBARU_PG_ES_Distance           0x161
#define MSG_SUBARU_PG_ES_DashStatus         0x166
#define MSG_SUBARU_PG_ES_DashStatus2        0x163
#define MSG_SUBARU_PG_ES_Status             0x162
#define MSG_SUBARU_PG_Steering_Torque       0x371

#define MSG_SUBARU_PG_Brake_Status          0xd3

#define SUBARU_PG_MAIN_BUS 0
#define SUBARU_PG_CAM_BUS  2

#define SUBARU_PG_COMMON_TX_MSGS()                           \
  {MSG_SUBARU_PG_ES_Distance,     SUBARU_PG_MAIN_BUS, 8},    \
  {MSG_SUBARU_PG_ES_LKAS,         SUBARU_PG_MAIN_BUS, 8},    \
  {MSG_SUBARU_PG_ES_DashStatus,   SUBARU_PG_MAIN_BUS, 8},    \
  {MSG_SUBARU_PG_ES_DashStatus2,  SUBARU_PG_MAIN_BUS, 8},    \

#define SUBARU_PG_COMMON_LONG_TX_MSGS()                  \
  {MSG_SUBARU_PG_ES_Brake,       SUBARU_PG_MAIN_BUS, 8}, \
  {MSG_SUBARU_PG_ES_Status,      SUBARU_PG_MAIN_BUS, 8}, \
  {MSG_SUBARU_PG_Brake_Status,   SUBARU_PG_CAM_BUS,  7}, \

const CanMsg SUBARU_PG_TX_MSGS[] = {
  SUBARU_PG_COMMON_TX_MSGS()
};
#define SUBARU_PG_TX_MSGS_LEN (sizeof(SUBARU_PG_TX_MSGS) / sizeof(SUBARU_PG_TX_MSGS[0]))

const CanMsg SUBARU_PG_LONG_TX_MSGS[] = {
  SUBARU_PG_COMMON_TX_MSGS()
  SUBARU_PG_COMMON_LONG_TX_MSGS()
};
#define SUBARU_PG_LONG_TX_MSGS_LEN (sizeof(SUBARU_PG_LONG_TX_MSGS) / sizeof(SUBARU_PG_LONG_TX_MSGS[0]))

bool subaru_preglobal_longitudinal = false;

// TODO: do checksum and counter checks after adding the signals to the outback dbc file
RxCheck subaru_preglobal_rx_checks[] = {
  {.msg = {{MSG_SUBARU_PG_Throttle,        SUBARU_PG_MAIN_BUS, 8, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{MSG_SUBARU_PG_Steering_Torque, SUBARU_PG_MAIN_BUS, 8, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{MSG_SUBARU_PG_CruiseControl,   SUBARU_PG_MAIN_BUS, 8, .frequency = 20U}, { 0 }, { 0 }}},
};


const int SUBARU_PG_PARAM_REVERSED_DRIVER_TORQUE = 1;
bool subaru_pg_reversed_driver_torque = false;


static void subaru_preglobal_rx_hook(CANPacket_t *to_push) {
  const int bus = GET_BUS(to_push);

  if (bus == SUBARU_PG_MAIN_BUS) {
    int addr = GET_ADDR(to_push);
    if (addr == MSG_SUBARU_PG_Steering_Torque) {
      int torque_driver_new;
      torque_driver_new = (GET_BYTE(to_push, 3) >> 5) + (GET_BYTE(to_push, 4) << 3);
      torque_driver_new = to_signed(torque_driver_new, 11);
      torque_driver_new = subaru_pg_reversed_driver_torque ? -torque_driver_new : torque_driver_new;
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (addr == MSG_SUBARU_PG_CruiseControl) {
      acc_main_on = GET_BIT(to_push, 48U) != 0U;
      bool cruise_engaged = GET_BIT(to_push, 49U) != 0U;
      pcm_cruise_check(cruise_engaged);
    }

    // update vehicle moving with any non-zero wheel speed
    if (addr == MSG_SUBARU_PG_Wheel_Speeds) {
      vehicle_moving = ((GET_BYTES(to_push, 0, 4) >> 12) != 0U) || (GET_BYTES(to_push, 4, 4) != 0U);
    }

    if (addr == MSG_SUBARU_PG_Brake_Pedal) {
      brake_pressed = ((GET_BYTES(to_push, 0, 4) >> 16) & 0xFFU) > 0U;
    }

    if (addr == MSG_SUBARU_PG_Throttle) {
      gas_pressed = GET_BYTE(to_push, 0) != 0U;
    }

    generic_rx_checks((addr == MSG_SUBARU_PG_ES_LKAS));
  }
}

static bool subaru_preglobal_tx_hook(CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  bool violation = false;

  if (subaru_preglobal_longitudinal) {
    tx = msg_allowed(to_send, SUBARU_PG_LONG_TX_MSGS, SUBARU_PG_LONG_TX_MSGS_LEN);
  } else {
    tx = msg_allowed(to_send, SUBARU_PG_TX_MSGS, SUBARU_PG_TX_MSGS_LEN);
  }

  // steer cmd checks
  if (addr == MSG_SUBARU_PG_ES_LKAS) {
    int desired_torque = ((GET_BYTES(to_send, 0, 4) >> 8) & 0x1FFFU);
    desired_torque = -1 * to_signed(desired_torque, 13);

    bool steer_req = (GET_BIT(to_send, 24U) != 0U);

    violation |= steer_torque_cmd_checks(desired_torque, steer_req, SUBARU_PG_STEERING_LIMITS);
  }

  // check es_brake brake_pressure limits
  if (addr == MSG_SUBARU_PG_ES_Brake) {
    int es_brake_pressure = GET_BYTES(to_send, 0, 2);
    violation |= longitudinal_brake_checks(es_brake_pressure, SUBARU_PG_LONG_LIMITS);
  }

  // check es_distance cruise_throttle limits
  if (addr == MSG_SUBARU_PG_ES_Distance) {
    int cruise_throttle = (GET_BYTES(to_send, 0, 2) & 0xFFFU);
    
    if (subaru_preglobal_longitudinal) {
      violation |= longitudinal_gas_checks(cruise_throttle, SUBARU_LONG_LIMITS);
    }
  }

  // check es_status transmission_rpm limits
  if (addr == MSG_SUBARU_PG_ES_Status) {
    int transmission_rpm = (GET_BYTES(to_send, 2, 2) & 0xFFFU);
    violation |= longitudinal_transmission_rpm_checks(transmission_rpm, SUBARU_LONG_LIMITS);
  }

  if (violation){
    tx = 0;
  }

  return tx;
}

static int subaru_preglobal_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  if (bus_num == SUBARU_PG_MAIN_BUS) {
    bool block_long = ((addr == MSG_SUBARU_PG_Brake_Status));

    bool block_msg = (subaru_preglobal_longitudinal && block_long);

    if (!block_msg) {
      bus_fwd = SUBARU_PG_CAM_BUS;  // Camera CAN
    }
  }

  if (bus_num == SUBARU_PG_CAM_BUS) {
    bool block_lkas = ((addr == MSG_SUBARU_PG_ES_LKAS) ||
                       (addr == MSG_SUBARU_PG_ES_DashStatus) ||
                       (addr == MSG_SUBARU_PG_ES_DashStatus2));
    bool block_long = ((addr == MSG_SUBARU_PG_ES_Brake) ||
                       (addr == MSG_SUBARU_PG_ES_Distance) ||
                       (addr == MSG_SUBARU_PG_ES_Status));

    bool block_msg = block_lkas || (subaru_preglobal_longitudinal && block_long);

    if (!block_msg) {
      bus_fwd = SUBARU_PG_MAIN_BUS;  // Main CAN
    }
  }

  return bus_fwd;
}

static safety_config subaru_preglobal_init(uint16_t param) {
  #ifdef ALLOW_DEBUG
    subaru_preglobal_longitudinal = GET_FLAG(param, SUBARU_PARAM_LONGITUDINAL);
  #endif
  
  subaru_pg_reversed_driver_torque = GET_FLAG(param, SUBARU_PG_PARAM_REVERSED_DRIVER_TORQUE);
  return BUILD_SAFETY_CFG(subaru_preglobal_rx_checks, SUBARU_PG_TX_MSGS);
}

const safety_hooks subaru_preglobal_hooks = {
  .init = subaru_preglobal_init,
  .rx = subaru_preglobal_rx_hook,
  .tx = subaru_preglobal_tx_hook,
  .fwd = subaru_preglobal_fwd_hook,
};
