#include "stabilizer_types.h"

#include "attitude_controller_customized.h"
// #include "velocity_controller_customized.h"
#include "controller_customized.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
// static float actuatorThrust = 0;

// static float cmd_thrust;
static float cmd_roll=0.0;
static float cmd_pitch=0.0;
static float cmd_yaw=0.0;
static float roll_d=0.0;
static float pitch_d=0.0;
static float yaw_d=0.0;
bool start = false;
bool forcestop = false;
// static float mass = 0.0864f;
// static float Ixx = 0.0000020011f;
// static float Iyy = 0.0000069007f;
// static float Izz = 0.001064235f;
// static float zFactor = 0.8f;
// static float g = 9.81f;
#ifndef M_PI
  #define M_PI   3.14159265358979323846
#endif

// static inter_disturbance_t inter_disturbance={
//     .u1 = 0.0,
//     .u2 = 0.0,
//     .u3 = 0.0,
//     .u4 = 0.0,
// };
// static disturbance_t disturbance = {
//     .u1 = 0.0,
//     .u2 = 0.0,
//     .u3 = 0.0,
//     .u4 = 0.0,
// };
// static K_obs_t K_obs = {
//     .K1 = 0.0,
//     .K2 = 0.0,
//     .K3 = 0.0,
//     .K4 = 0.0,
// };


// void updateDisturbance(const state_t *state, control_t *control,const sensorData_t *sensors){
//     float eulerRollActual = state->attitude.roll* (float)M_PI / 180.0f;
//     float eulerPitchActual = state->attitude.pitch* (float)M_PI / 180.0f;
//     float eulerYawActual = - state->attitude.yaw * (float)M_PI / 180.0f;
//     float omega_x = sensors->gyro.x* (float)M_PI / 180.0f;
//     float omega_y = sensors->gyro.y* (float)M_PI / 180.0f;
//     float omega_z = -sensors->gyro.z* (float)M_PI / 180.0f;
//     inter_disturbance.u1 += ATTITUDE_UPDATE_DT * K_obs.K1 * (mass*zFactor*g/cosf(eulerRollActual)/cosf(eulerPitchActual) - control->thrustSi);
//     disturbance.u1 = inter_disturbance.u1 - mass*
// }

void controllerCustomizedInit(void)
{
  attitudeControllerCustomizedInit(ATTITUDE_UPDATE_DT);
  // velocityControllerCustomizedInit();
}

bool controllerCustomizedTest(void){
  bool pass = true;

  pass &= attitudeControllerCustomizedTest();

  return pass;
}

void controllerCustomized(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep){

  control->controlMode = controlModeForceTorque;
  // attitudeDesired.yaw = setpoint->attitude.yaw;
  // velocityControllerCustomized(&actuatorThrust, &attitudeDesired, setpoint, state);
  // velocityControllerCustomizedGetDesiredAttitude(&roll_d, &pitch_d, &yaw_d);
  // attitudeDesired.roll = roll_d;
  // attitudeDesired.pitch = pitch_d;
  roll_d = setpoint->attitude.roll;
  pitch_d = setpoint->attitude.pitch;
  yaw_d = setpoint->attitude.yaw;
  attitudeDesired.roll = setpoint->attitude.roll;
  attitudeDesired.pitch = setpoint->attitude.pitch;
  attitudeDesired.yaw = setpoint->attitude.yaw;
  attitudeControllerCustomized(sensors,&attitudeDesired,setpoint->attitudeRate.yaw,state, setpoint->mode.z, setpoint->locmode);
  attitudeControllerCustomizedGetActuatorOutput(&cmd_roll, &cmd_pitch, &cmd_yaw);
  control->thrustSi = setpoint->thrust;
  // control->thrustSi = actuatorThrust;
  // control->thrustSi = 0.005;
  control->torqueX = cmd_roll;
  control->torqueY = cmd_pitch;
  control->torqueZ = cmd_yaw;
  // if (setpoint->mode.z != modeGround && setpoint->mode.z != modeSky){
  //   control->thrustSi = 0.0;
  //   control->torqueX = 0.0;
  //   control->torqueY = 0.0;
  //   control->torqueZ = 0.0;
  //   attitudeControllerCustomizedResetAllPID();
  //   velocityControllerCustomizedResetAllPID();
  // }
  // if (setpoint->velocity_body){
  //   attitudeControllerCustomizedResetAllPID();
  //   velocityControllerCustomizedResetAllPID();
  //   // velocityControllerCustomizedResetAllfilters();
  // }
  if (setpoint->start){
    start = true;
  }
  if (setpoint->forcestop){
    forcestop = true;
  }
  if ((!start) || forcestop){
    control->thrustSi = 0.0;
    control->torqueX = 0.0;
    control->torqueY = 0.0;
    control->torqueZ = 0.0;
    attitudeControllerCustomizedResetAllPID();
  }
  if(setpoint->reset){
    attitudeControllerCustomizedResetAllPID();
    // velocityControllerCustomizedResetAllPID();
  }
}

LOG_GROUP_START(customizedCtl)

// // LOG_ADD(LOG_FLOAT, thrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, rollTorque, &cmd_roll)
LOG_ADD(LOG_FLOAT, pitchTorque, &cmd_pitch)
LOG_ADD(LOG_FLOAT, yawTorque, &cmd_yaw)
LOG_ADD(LOG_FLOAT, rollD, &roll_d)
LOG_ADD(LOG_FLOAT, pitchD, &pitch_d)
LOG_ADD(LOG_FLOAT, yawD, &yaw_d)
// LOG_ADD(LOG_FLOAT, thrust, &actuatorThrust)

LOG_GROUP_STOP(customizedCtl)

// PARAM_GROUP_START(customizedPid)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Ixx, &Ixx)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Iyy, &Iyy)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Izz, &Izz)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, mass, &mass)
// PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, zFactor, &zFactor)

// PARAM_GROUP_STOP(customizedPid)