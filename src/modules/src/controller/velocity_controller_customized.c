/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * position_estimator_pid.c: PID-based implementation of the position controller
 */

#include <math.h>
#include "num.h"

#include "log.h"
#include "param.h"
#include "pid.h"
#include "num.h"
#include "velocity_controller_customized.h"
#include "platform_defaults.h"
#include "math.h"

#ifndef M_PI
  #define M_PI   3.14159265358979323846
#endif
struct pidAxis_s {
  PidObject pid;

  stab_mode_t previousMode;
  float setpoint;

  float output;
};

struct this_s {
  struct pidAxis_s pidVX;
  struct pidAxis_s pidVY;
  struct pidAxis_s pidVZ;

//   struct pidAxis_s pidX;
//   struct pidAxis_s pidY;
//   struct pidAxis_s pidZ;

  uint16_t thrustBase; // approximate throttle needed when in perfect hover. More weight/older battery can use a higher value
  uint16_t thrustMin;  // Minimum thrust value to output
};

// Maximum roll/pitch angle permited
static float rLimit = PID_VEL_ROLL_MAX/(float)M_PI*180.0f;
static float pLimit = PID_VEL_PITCH_MAX/(float)M_PI*180.0f;
// static float rpLimitOverhead = 1.10f;
// Velocity maximums
static float xVelMax = PID_POS_VEL_X_MAX;
static float yVelMax = PID_POS_VEL_Y_MAX;
static float zVelMax = PID_POS_VEL_Z_MAX;
// static float velMaxOverhead = 1.10f;
static float zFactor = 0.8f;
static const float thrustScale = 1000.0f;
static float g = 9.81f;
static float mass = 0.0864f;

#define DT (float)(1.0f/POSITION_RATE)
// static bool posFiltEnable = PID_POS_XY_FILT_ENABLE;
static bool velFiltEnable = PID_VEL_XY_FILT_ENABLE;
// static float posFiltCutoff = PID_POS_XY_FILT_CUTOFF;
static float velFiltCutoff = PID_VEL_XY_FILT_CUTOFF;
// static bool posZFiltEnable = PID_POS_Z_FILT_ENABLE;
static bool velZFiltEnable = PID_VEL_Z_FILT_ENABLE;
// static float posZFiltCutoff = PID_POS_Z_FILT_CUTOFF;
#if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
static float velZFiltCutoff = PID_VEL_Z_FILT_CUTOFF_BARO_Z_HOLD;
#else
static float velZFiltCutoff = PID_VEL_Z_FILT_CUTOFF;
#endif

#ifndef UNIT_TEST
static struct this_s thisC = {
  .pidVX = {
    .pid = {
      .kp = 3.5,
      .ki = 0.001,
      .kd = PID_VEL_X_KD,
      .kff = PID_VEL_X_KFF,
    },
    .pid.dt = DT,
  },

  .pidVY = {
    .pid = {
      .kp = 3.5,
      .ki = 0.001,
      .kd = PID_VEL_Y_KD,
      .kff = PID_VEL_Y_KFF,
    },
    .pid.dt = DT,
  },
  #if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
    .pidVZ = {
      .pid = {
        .kp = PID_VEL_Z_KP_BARO_Z_HOLD,
        .ki = PID_VEL_Z_KI_BARO_Z_HOLD,
        .kd = PID_VEL_Z_KD_BARO_Z_HOLD,
        .kff = PID_VEL_Z_KFF_BARO_Z_HOLD,
      },
      .pid.dt = DT,
    },
  #else
    .pidVZ = {
      .pid = {
        .kp = 10.0,
        .ki = 20.0,
        .kd = PID_VEL_Z_KD,
        .kff = PID_VEL_Z_KFF,
      },
      .pid.dt = DT,
    },
  #endif
  #if CONFIG_CONTROLLER_PID_IMPROVED_BARO_Z_HOLD
    .thrustBase = PID_VEL_THRUST_BASE_BARO_Z_HOLD,
  #else
    .thrustBase = PID_VEL_THRUST_BASE,
  #endif
  .thrustMin  = PID_VEL_THRUST_MIN,
};
#endif
static float roll_d;
static float pitch_d;
static float yaw_d;
static float thrustRaw;

void velocityControllerCustomizedInit()
{
  pidInit(&thisC.pidVX.pid, thisC.pidVX.setpoint, thisC.pidVX.pid.kp, thisC.pidVX.pid.ki, thisC.pidVX.pid.kd,
      thisC.pidVX.pid.kff, thisC.pidVX.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&thisC.pidVY.pid, thisC.pidVY.setpoint, thisC.pidVY.pid.kp, thisC.pidVY.pid.ki, thisC.pidVY.pid.kd,
      thisC.pidVY.pid.kff, thisC.pidVY.pid.dt, POSITION_RATE, velFiltCutoff, velFiltEnable);
  pidInit(&thisC.pidVZ.pid, thisC.pidVZ.setpoint, thisC.pidVZ.pid.kp, thisC.pidVZ.pid.ki, thisC.pidVZ.pid.kd,
      thisC.pidVZ.pid.kff, thisC.pidVZ.pid.dt, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
}

// static float runPid(float input, struct pidAxis_s *axis, float setpoint, float dt) {
//   axis->setpoint = setpoint;

//   pidSetDesired(&axis->pid, axis->setpoint);
//   return pidUpdate(&axis->pid, input, true);
// }


float state_body_x, state_body_y, state_body_vx, state_body_vy;



void velocityControllerCustomized(float* thrust, attitude_t *attitude, const setpoint_t* setpoint,
                                                             const state_t *state)
{
  // thisC.pidVX.pid.outputLimit = pLimit * rpLimitOverhead;
  // thisC.pidVY.pid.outputLimit = rLimit * rpLimitOverhead;
  // // Set the output limit to the maximum thrust range
  // thisC.pidVZ.pid.outputLimit = (UINT16_MAX / 2 / thrustScale);
  //thisC.pidVZ.pid.outputLimit = (thisC.thrustBase - thisC.thrustMin) / thrustScale;
  yaw_d = setpoint->attitude.yaw;
  float cosyaw = cosf(yaw_d * (float)M_PI / 180.0f);
  float sinyaw = sinf(yaw_d * (float)M_PI / 180.0f);//negative sign because the yaw is in the opposite direction
  updateInt(&thisC.pidVX.pid, setpoint->velocity.x - state->velocity.x);
  updateInt(&thisC.pidVY.pid, setpoint->velocity.y - state->velocity.y);
  updateInt(&thisC.pidVZ.pid, setpoint->velocity.z - state->velocity.z);
  float u1 = thisC.pidVX.pid.kp * (setpoint->velocity.x - state->velocity.x) + thisC.pidVX.pid.ki * state->position.x; ;
  float u2 = thisC.pidVY.pid.kp * (setpoint->velocity.y - state->velocity.y) + thisC.pidVY.pid.ki * state->position.y; ;
  // float u3 = thisC.pidVZ.pid.kp * (setpoint->velocity.z - state->velocity.z) + thisC.pidVZ.pid.ki * thisC.pidVZ.pid.integ ;
  
  if (setpoint->mode.z == modeGround){
      // attitude->pitch = atanf(-(cosyaw * u1 + sinyaw * u2)/g/zFactor);
      pitch_d = atanf(-(cosyaw * u1 + sinyaw * u2)/g/zFactor);
      // attitude->roll = atanf(-cosf(attitude->pitch) * (sinyaw * u1 - cosyaw * u2)/g/zFactor);
      roll_d = atanf(-cosf(pitch_d) * (sinyaw * u1 - cosyaw * u2)/g/zFactor);
      thrustRaw = (g/cosf(pitch_d)/cosf(roll_d))*mass*zFactor;
    }
    // else if (setpoint->mode.z == modeSky)
    // {
      // // attitude->pitch = atanf(-(cosyaw * u1 + sinyaw * u2)/(g-u3));
      // pitch_d = atanf(-(cosyaw * u1 + sinyaw * u2)/(g-u3));
      // // attitude->roll = atanf(-cosf(attitude->pitch) * (sinyaw * u1 - cosyaw * u2)/(g-u3));
      // roll_d = atanf(-cosf(pitch_d) * (sinyaw * u1 - cosyaw * u2)/(g-u3));
      // thrustRaw = ((g-u3)/cosf(pitch_d)/cosf(roll_d))*mass;
    // }
    else
    {
      // attitude->pitch = 0;
      pitch_d = 0;
      // attitude->roll = 0;
      roll_d = 0;
      thrustRaw = 0;
    }
    
  // attitude->roll  = constrain(attitude->roll/(float)M_PI*180.0f,  -rLimit, rLimit);
  roll_d = constrain(roll_d,  -rLimit, rLimit);
  // attitude->pitch = constrain(attitude->pitch/(float)M_PI*180.0f, -pLimit, pLimit);
  pitch_d = constrain(pitch_d, -pLimit, pLimit);

  // Scale the thrust and add feed forward term
  *thrust = thrustRaw;
  // // Check for minimum thrust
  // if (*thrust < thisC.thrustMin) {
  //   *thrust = thisC.thrustMin;
  // }
    // saturate
  // *thrust = constrain(*thrust, 0, UINT16_MAX);
}

void velocityControllerCustomizedGetDesiredAttitude(float* roll, float* pitch, float* yaw)
{
  *roll = roll_d;
  *pitch = pitch_d;
  *yaw = yaw_d;
}

void velocityControllerCustomizedResetAllPID()
{
  pidReset(&thisC.pidVX.pid);
  pidReset(&thisC.pidVY.pid);
  pidReset(&thisC.pidVZ.pid);
}

void velocityControllerCustomizedResetAllfilters() {
  filterReset(&thisC.pidVX.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&thisC.pidVY.pid, POSITION_RATE, velFiltCutoff, velFiltEnable);
  filterReset(&thisC.pidVZ.pid, POSITION_RATE, velZFiltCutoff, velZFiltEnable);
}

/**
 * Log variables of the PID position controller
 *
 * Note: rename to posCtrlPID ?
 */
LOG_GROUP_START(customizedCtl)

/**
 * @brief PID controller target desired body-yaw-aligned velocity x [m/s]
 *
 * Note: Same as stabilizer log
 */

LOG_ADD(LOG_FLOAT, targetVX, &thisC.pidVX.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned velocity y [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVY, &thisC.pidVY.pid.desired)
/**
 * @brief PID controller target desired velocity z [m/s]
 *
 * Note: Same as stabilizer log
 */
LOG_ADD(LOG_FLOAT, targetVZ, &thisC.pidVZ.pid.desired)
/**
 * @brief PID controller target desired body-yaw-aligned position x [m]
 *
 * Note: Same as stabilizer log
 */

LOG_ADD(LOG_FLOAT, bodyVX, &state_body_vx)
/**
 * @brief PID state body-yaw-aligned velocity y [m/s]
 *
 */
LOG_ADD(LOG_FLOAT, bodyVY, &state_body_vy)
/**
 * @brief PID state body-yaw-aligned position x [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyX, &state_body_x)
/**
 * @brief PID state body-yaw-aligned position y [m]
 *
 */
LOG_ADD(LOG_FLOAT, bodyY, &state_body_y)

/**
 * @brief PID proportional output position x
 */
LOG_ADD(LOG_FLOAT, VXp, &thisC.pidVX.pid.outP)
/**
 * @brief PID integral output velocity x
 */
LOG_ADD(LOG_FLOAT, VXi, &thisC.pidVX.pid.outI)
/**
 * @brief PID derivative output velocity x
 */
LOG_ADD(LOG_FLOAT, VXd, &thisC.pidVX.pid.outD)
/**
 * @brief PID feedforward output velocity x
 */
LOG_ADD(LOG_FLOAT, VXff, &thisC.pidVX.pid.outFF)

/**
 * @brief PID proportional output velocity y
 */
LOG_ADD(LOG_FLOAT, VYp, &thisC.pidVY.pid.outP)
/**
 * @brief PID integral output velocity y
 */
LOG_ADD(LOG_FLOAT, VYi, &thisC.pidVY.pid.outI)
/**
 * @brief PID derivative output velocity y
 */
LOG_ADD(LOG_FLOAT, VYd, &thisC.pidVY.pid.outD)
/**
 * @brief PID feedforward output velocity y
 */
LOG_ADD(LOG_FLOAT, VYff, &thisC.pidVY.pid.outFF)

/**
 * @brief PID proportional output velocity z
 */
LOG_ADD(LOG_FLOAT, VZp, &thisC.pidVZ.pid.outP)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZi, &thisC.pidVZ.pid.outI)
/**
 * @brief PID integral output velocity z
 */
LOG_ADD(LOG_FLOAT, VZd, &thisC.pidVZ.pid.outD)
/**
 * @brief PID feedforward output velocity z
 */
LOG_ADD(LOG_FLOAT, VZff, &thisC.pidVZ.pid.outFF)


// LOG_ADD(LOG_FLOAT, rollD, &roll_d)
// LOG_ADD(LOG_FLOAT, pitchD, &pitch_d)
// LOG_ADD(LOG_FLOAT, yawD, &yaw_d)
LOG_ADD(LOG_FLOAT, thrust, &thrustRaw)
LOG_ADD(LOG_FLOAT, mass, &mass)
LOG_ADD(LOG_FLOAT, g, &g)
LOG_ADD(LOG_FLOAT, zFactor, &zFactor)


LOG_GROUP_STOP(customizedCtl)

/**
 * Tuning settings for the gains of the PID
 * controller for the velocity of the Crazyflie ¨
 * in the body-yaw-aligned X & Y and global Z directions.
 */
PARAM_GROUP_START(customizedPid)
/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKp, &thisC.pidVX.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKi, &thisC.pidVX.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKd, &thisC.pidVX.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the body-yaw-aligned X direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vxKFF, &thisC.pidVX.pid.kff)

/**
 * @brief Proportional gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKp, &thisC.pidVY.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKi, &thisC.pidVY.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the body-yaw-aligned Y direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKd, &thisC.pidVY.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the body-yaw-aligned Y direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vyKFF, &thisC.pidVY.pid.kff)

/**
 * @brief Proportional gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKp, &thisC.pidVZ.pid.kp)
/**
 * @brief Integral gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKi, &thisC.pidVZ.pid.ki)
/**
 * @brief Derivative gain for the velocity PID in the global Z direction
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKd, &thisC.pidVZ.pid.kd)
/**
 * @brief Feedforward gain for the velocity PID in the global direction (in degrees per m/s)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, vzKFF, &thisC.pidVZ.pid.kff)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zFactor, &zFactor)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, mass, &mass)


PARAM_GROUP_STOP(customizedPid)

/**
 * Tuning settings for the gains of the PID
 * controller for the position of the Crazyflie ¨
 * in the body-yaw-aligned X & Y and global Z directions.
 */
PARAM_GROUP_START(posCtlPid)
/**
 * @brief Proportional gain for the position PID in the body-yaw-aligned X direction
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, thrustBase, &thisC.thrustBase)
/**
 * @brief Min. thrust value to output
 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, thrustMin, &thisC.thrustMin)

/**
 * @brief Roll absolute limit
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, rLimit,  &rLimit)
/**
 * @brief Pitch absolute limit
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pLimit,  &pLimit)
/**
 * @brief Maximum body-yaw-aligned X velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, xVelMax, &xVelMax)
/**
 * @brief Maximum body-yaw-aligned Y velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yVelMax, &yVelMax)
/**
 * @brief Maximum Z Velocity
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zVelMax,  &zVelMax)

PARAM_GROUP_STOP(posCtlPid)


