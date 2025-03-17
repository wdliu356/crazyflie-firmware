#include <stdbool.h>

#include "stabilizer_types.h"

#include "attitude_controller_customized_new.h"
#include "pid.h"
#include "param.h"
#include "log.h"
#include "commander.h"
#include "platform_defaults.h"
#include "math.h"

#ifndef M_PI
  #define M_PI   3.14159265358979323846
#endif
static bool attFiltEnable = ATTITUDE_LPF_ENABLE;
// static bool rateFiltEnable = ATTITUDE_RATE_LPF_ENABLE;
static float attFiltCutoff = ATTITUDE_LPF_CUTOFF_FREQ;
// static float omxFiltCutoff = ATTITUDE_ROLL_RATE_LPF_CUTOFF_FREQ;
// static float omyFiltCutoff = ATTITUDE_PITCH_RATE_LPF_CUTOFF_FREQ;
// static float omzFiltCutoff = ATTITUDE_YAW_RATE_LPF_CUTOFF_FREQ;
// static float yawMaxDelta = YAW_MAX_DELTA;
// static float mass = 0.0864f;

// float Ixx = 0.0000020011f;
// float Iyy = 0.0000069007f;
// float Izz = 0.001064235f;
// float armLength = 0.041;
// float k_roll_p = 1.0f;
// float k_roll_d = 1.0f;
// float k_roll_i = 1.0f;
// float k_pitch_p = 1.0f;
// float k_pitch_d = 1.0f;
// float k_pitch_i = 1.0f;
// float k_yaw_p = 1.0f;
// float k_yaw_d = 1.0f;
// float k_yaw_i = 1.0f;

float Ixx;
float Iyy;
float Izz;
float armLength;
float eulerRollDesired;//* (float)M_PI / 180.0f;
float eulerPitchDesired;//* (float)M_PI / 180.0f;
float eulerYawDesired;//* (float)M_PI / 180.0f;
float eulerRollActual;
float eulerPitchActual;
float eulerYawActual;
float rollError;
float pitchError;
float yawError;
float spring;
float Iyy_frame;
float Izz_frame;
float angle_diff;
float omega_x;
float omega_y;
float omega_z;
// static inline int16_t saturateSignedInt16(float in)
// {
//   // don't use INT16_MIN, because later we may negate it, which won't work for that value.
//   if (in > INT16_MAX)
//     return INT16_MAX;
//   else if (in < -INT16_MAX)
//     return -INT16_MAX;
//   else
//     return (int16_t)in;
// }

PidObject pidRollC = {
  .kp = 0.0,
  .ki = 0.0,
  .kd = 0.0,
  .kff = PID_ROLL_KFF,
};

PidObject pidPitchC = {
  .kp = 0.0,
  .ki = 0.0,
  .kd = 0.0,
  .kff = PID_PITCH_KFF,
};

PidObject pidYawC = {
  .kp = 0.0,
  .ki = 0.0,
  .kd = 0.0,
  .kff = PID_YAW_KFF,
};

PidObject pidRollC_f = {
  .kp = 0.0,
  .ki = 0.0,
  .kd = 0.0,
  .kff = PID_ROLL_KFF,
};

PidObject pidPitchC_f = {
  .kp = 0.0,
  .ki = 0.0,
  .kd = 0.0,
  .kff = PID_PITCH_KFF,
};

PidObject pidYawC_f = {
  .kp = 0.0,
  .ki = 0.0,
  .kd = 0.0,
  .kff = PID_YAW_KFF,
};

static float rollTorque=0.0;
static float pitchTorque=0.0;
static float yawTorque=0.0;
static stab_mode_t pre_mode = modeGround;

static bool isInit;

void attitudeControllerCustomizedNewInit(const float updateDt)
{
  if (isInit)
    return;

//   pidInit(&pidRollCRate, updateDt);
//   pidInit(&pidPitchCRate, updateDt);
//   pidInit(&pidYawCRate, updateDt);
  pidInit(&pidRollC,  0, pidRollC.kp,  pidRollC.ki,  pidRollC.kd,  pidRollC.kff,  updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidPitchC, 0, pidPitchC.kp, pidPitchC.ki, pidPitchC.kd, pidPitchC.kff, updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidYawC,   0, pidYawC.kp,   pidYawC.ki,   pidYawC.kd,   pidYawC.kff,   updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidRollC_f,  0, pidRollC_f.kp,  pidRollC_f.ki,  pidRollC_f.kd,  pidRollC_f.kff,  updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidPitchC_f, 0, pidPitchC_f.kp, pidPitchC_f.ki, pidPitchC_f.kd, pidPitchC_f.kff, updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidInit(&pidYawC_f,   0, pidYawC_f.kp,   pidYawC_f.ki,   pidYawC_f.kd,   pidYawC_f.kff,   updateDt,
      ATTITUDE_RATE, attFiltCutoff, attFiltEnable);
  pidRollC.integ = 0.0;
  pidPitchC.integ = 0.0;
  pidYawC.integ = 0.0;
  pidRollC_f.integ = 0.0;
  pidPitchC_f.integ = 0.0;
  pidYawC_f.integ = 0.0;

  isInit = true;
}
float omega_dot_x = 0.0f;
float omega_dot_y = 0.0f;
float omega_dot_z = 0.0f;

bool attitudeControllerCustomizedNewTest()
{
  return isInit;
}

void attitudeControllerCustomizedNew(
  const sensorData_t *sensors,const state_t *state, const setpoint_t *setpoint)
{
    if (!isInit)
        return;
    if (pre_mode != setpoint->mode.z){
      pidRollC.integ = 0.0;
      pidPitchC.integ = 0.0;
      pidYawC.integ = 0.0;
      pidRollC_f.integ = 0.0;
      pidPitchC_f.integ = 0.0;
      pidYawC_f.integ = 0.0;
    }
    // Correct attitude PID
    if (setpoint->testMode == true){
      float eulerRollActual = setpoint->attitude.roll;
      float eulerPitchActual = setpoint->attitude.pitch;
      float eulerYawActual = setpoint->attitude.yaw;
      rollError = - eulerRollActual;
      pitchError = - eulerPitchActual;
      yawError = - eulerYawActual;
      omega_x = setpoint->attitudeRate.roll;
      omega_y = setpoint->attitudeRate.pitch;
      omega_z = setpoint->attitudeRate.yaw;
      angle_diff = - eulerRollActual;
      
    }
    else{
      float eulerRollDesired =setpoint->attitude.roll;//* (float)M_PI / 180.0f;
      float eulerPitchDesired = setpoint->attitude.pitch;//* (float)M_PI / 180.0f;
      float eulerYawDesired = setpoint->attitude.yaw;//* (float)M_PI / 180.0f;
      float eulerRollActual = state->attitude.roll* (float)M_PI / 180.0f;
      float eulerPitchActual = -state->attitude.pitch* (float)M_PI / 180.0f;
      // float eulerYawActual = state->attitude.yaw * (float)M_PI / 180.0f;
      rollError = eulerRollDesired - eulerRollActual;
      pitchError = eulerPitchDesired - eulerPitchActual;
      yawError = eulerYawDesired - setpoint->yaw_fb;
      // yawError = eulerYawDesired - eulerYawActual;

      if (yawError > (float)M_PI){
        yawError -= 2.0f*(float)M_PI;
      }
      else if (yawError < -(float)M_PI){
        yawError += 2.0f*(float)M_PI;
      }
      omega_x = sensors->gyro.x* (float)M_PI / 180.0f;
      omega_y = sensors->gyro.y* (float)M_PI / 180.0f;
      omega_z = sensors->gyro.z* (float)M_PI / 180.0f;
      angle_diff = setpoint->frameroll - eulerRollActual;
      // angle_diff = 0.0f;
    }
    if (angle_diff > (float)M_PI){
      angle_diff -= 2.0f*(float)M_PI;
    }
    else if (angle_diff < -(float)M_PI){
      angle_diff += 2.0f*(float)M_PI;
    }
    if (omega_x > (float)1.){
      omega_x = (float)1.;
    }
    if (omega_x < (float)-1.){
      omega_x = (float)-1.;
    }
    if (omega_y > (float)1.){
      omega_y = (float)1.;
    }
    if (omega_y < (float)-1.){
      omega_y = (float)-1.;
    }
    if (omega_z > (float)1.){
      omega_z = (float)1.;
    }
    if (omega_z < (float)-1.){
      omega_z = (float)-1.;
    }
    

    float I_new00 = Ixx;

    float I_new11 = Iyy + Iyy_frame*(float)(cos(angle_diff)*cos(angle_diff))+Izz_frame*(float)(sin(angle_diff)*sin(angle_diff));
    float I_new12 = -Iyy_frame*(float)(cos(angle_diff)*sin(angle_diff))+Izz_frame*(float)(sin(angle_diff)*cos(angle_diff));

    float I_new21 = -Iyy_frame*(float)(cos(angle_diff)*sin(angle_diff))+Izz_frame*(float)(sin(angle_diff)*cos(angle_diff));
    float I_new22 = Izz + Iyy_frame*(float)(sin(angle_diff)*sin(angle_diff))+Izz_frame*(float)(cos(angle_diff)*cos(angle_diff));

    updateInt(&pidRollC, rollError);
    updateInt(&pidPitchC, pitchError);
    updateInt(&pidYawC, yawError);
    updateInt(&pidRollC_f, rollError);
    updateInt(&pidPitchC_f, pitchError);
    updateInt(&pidYawC_f, yawError);
    
    if (setpoint->mode.z == modeGround){
      omega_dot_x = -pidRollC.kd*omega_x + pidRollC.kp*rollError + pidRollC.ki*pidRollC.integ;
      omega_dot_y = -pidPitchC.kd*omega_y + pidPitchC.kp*pitchError + pidPitchC.ki*pidPitchC.integ;
      omega_dot_z = -pidYawC.kd*(omega_z - (float)0.0) + pidYawC.kp*yawError + pidYawC.ki*pidYawC.integ; 
      // I_new12 = 0;
      // I_new21 = 0;

    }
    else if (setpoint->mode.z == modeSky)
    {
      omega_dot_x = -pidRollC_f.kd*omega_x + pidRollC_f.kp*rollError + pidRollC_f.ki*pidRollC_f.integ;
      omega_dot_y = -pidPitchC_f.kd*omega_y + pidPitchC_f.kp*pitchError + pidPitchC_f.ki*pidPitchC_f.integ;
      omega_dot_z = -pidYawC_f.kd*(omega_z - (float)0.0) + pidYawC_f.kp*yawError + pidYawC_f.ki*pidYawC_f.integ;
    }

    if (setpoint->locmode){
      rollTorque = 0.0;
      // yawTorque = 0.0;
      pidYawC.integ = 0.0;
      pidRollC.integ = 0.0;
      pidPitchC.integ = 0.0;
    }
    else{
      rollTorque = I_new00*omega_dot_x + omega_y*(I_new22*omega_z+I_new21*omega_y) - omega_z*(I_new11*omega_y+I_new12*omega_z);//-spring*angle_diff;
      // if (angle_diff > (float)0.1 || angle_diff < (float)-0.1){
      //   rollTorque = rollTorque - spring*angle_diff; 
      // }
      
    }
    // rollTorque = 0.0;
    pitchTorque = (I_new11*omega_dot_y + I_new12*omega_dot_z + omega_z*(I_new00*omega_x) - omega_x*(I_new22*omega_z+I_new21*omega_y));
    yawTorque = I_new22*omega_dot_z + I_new21*omega_dot_y + omega_x*(I_new11*omega_y+I_new12*omega_z) - omega_y*(I_new00*omega_x);

    // rollTorque = I_new00*omega_dot_x;
    // pitchTorque = I_new11*omega_dot_y + I_new12*omega_dot_z;
    // yawTorque = I_new22*omega_dot_z + I_new21*omega_dot_y;


    
    if(rollTorque > 0.1f)
        rollTorque = 0.1f;
    if(rollTorque < -0.1f)
        rollTorque = -0.1f;
    // rollTorque = 0.0f;
    if(pitchTorque > 0.1f)
        pitchTorque = 0.1f;
    if(pitchTorque < -0.1f)
        pitchTorque = -0.1f;
    // pitchTorque = 0.0f;
    if (yawTorque > 0.05f)
        yawTorque = 0.05f;
    if (yawTorque < -0.05f)
        yawTorque = -0.05f;
    // yawTorque = 0.0f;
    pre_mode = setpoint->mode.z;
}

void attitudeControllerCustomizedNewResetAllPID()
{
    pidReset(&pidRollC);
    pidReset(&pidPitchC);
    pidReset(&pidYawC);
}
void attitudeControllerCustomizedNewGetActuatorOutput(float* roll, float* pitch, float* yaw)
{
  *roll = rollTorque;
  *pitch = pitchTorque;
  *yaw = yawTorque;
}

void attitudeControllerCustomizedNewGetPIDOutput(float* roll, float* pitch, float* yaw)
{
  *roll = omega_dot_x;
  *pitch = omega_dot_y;
  *yaw = omega_dot_z;
}



/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll 
 */
PARAM_GROUP_START(customizedPid)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp, &pidRollC.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki, &pidRollC.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd, &pidRollC.kd)
/**
 * @brief Feedforward gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff, &pidRollC.kff)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp, &pidPitchC.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki, &pidPitchC.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd, &pidPitchC.kd)
/**
 * @brief Feedforward gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff, &pidPitchC.kff)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp, &pidYawC.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki, &pidYawC.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd, &pidYawC.kd)
/**
 * @brief Feedforward gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff, &pidYawC.kff)
/**
 * @brief Low pass filter enable
 */
PARAM_ADD(PARAM_INT8 | PARAM_PERSISTENT, attFiltEn, &attFiltEnable)
/**
 * @brief Low pass filter cut-off frequency (Hz)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, attFiltCut, &attFiltCutoff)

PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Ixx, &Ixx)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Iyy, &Iyy)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Izz, &Izz)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, armLength, &armLength)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, spring, &spring)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Iyy_frame, &Iyy_frame)
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, Izz_frame, &Izz_frame)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kp_f, &pidRollC_f.kp)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_ki_f, &pidRollC_f.ki)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kd_f, &pidRollC_f.kd)
/**
 * @brief Feedforward gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, roll_kff_f, &pidRollC_f.kff)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kp_f, &pidPitchC_f.kp)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_ki_f, &pidPitchC_f.ki)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kd_f, &pidPitchC_f.kd)
/**
 * @brief Feedforward gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, pitch_kff_f, &pidPitchC_f.kff)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kp_f, &pidYawC_f.kp)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_ki_f, &pidYawC_f.ki)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kd_f, &pidYawC_f.kd)
/**
 * @brief Feedforward gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, yaw_kff_f, &pidYawC_f.kff)

PARAM_GROUP_STOP(customizedPid)

// LOG_GROUP_START(controlTarget)
// LOG_ADD(LOG_FLOAT, rollError, &rollError)
// LOG_ADD(LOG_FLOAT, pitchError, &pitchError)
// LOG_ADD(LOG_FLOAT, yawError, &yawError)
// LOG_ADD(LOG_FLOAT, eulerRollDesired, &eulerRollDesired)
// LOG_ADD(LOG_FLOAT, eulerPitchDesired, &eulerPitchDesired)
// LOG_GROUP_STOP(controlTarget)

