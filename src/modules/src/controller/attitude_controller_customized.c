#include <stdbool.h>

#include "stabilizer_types.h"

#include "attitude_controller_customized.h"
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
  .kp = 0.35,
  .ki = 0.4,
  .kd = -400,
  .kff = PID_ROLL_KFF,
};

PidObject pidPitchC = {
  .kp = 0.5,
  .ki = 0.2,
  .kd = -250,
  .kff = PID_PITCH_KFF,
};

PidObject pidYawC = {
  .kp = 0.4,
  .ki = 0.1,
  .kd = -2,
  .kff = PID_YAW_KFF,
};

PidObject pidRollC_f = {
  .kp = 0.35,
  .ki = 0.4,
  .kd = -400,
  .kff = PID_ROLL_KFF,
};

PidObject pidPitchC_f = {
  .kp = 0.5,
  .ki = 0.2,
  .kd = -250,
  .kff = PID_PITCH_KFF,
};

PidObject pidYawC_f = {
  .kp = 0.4,
  .ki = 0.1,
  .kd = -2,
  .kff = PID_YAW_KFF,
};

static float rollTorque=0.0;
static float pitchTorque=0.0;
static float yawTorque=0.0;

static bool isInit;

void attitudeControllerCustomizedInit(const float updateDt)
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

  isInit = true;
}

bool attitudeControllerCustomizedTest()
{
  return isInit;
}

void attitudeControllerCustomized(
       const sensorData_t *sensors,const attitude_t *attitude,const float desired_yaw_rate,const state_t *state, const stab_mode_t mode)
{
    if (!isInit)
        return;
    // if (mode == modeGround){
    //   Ixx = 0.0000020011f;
    // }
    // else{
    //   Ixx = 0.0000020011f + 0.076f*0.076f*0.0084107f*2.0f + 0.0000029976f;
    // }
    float I1 = (Iyy - Izz) / Ixx;
    float I2 = (Izz - Ixx) / Iyy;
    float I3 = (Ixx - Iyy) / Izz;
    // Correct attitude PID
    float eulerRollDesired = attitude->roll;//* (float)M_PI / 180.0f;
    float eulerPitchDesired = attitude->pitch;//* (float)M_PI / 180.0f;
    float eulerYawDesired = attitude->yaw;//* (float)M_PI / 180.0f;
    float eulerRollActual = state->attitude.roll* (float)M_PI / 180.0f;
    float eulerPitchActual = state->attitude.pitch* (float)M_PI / 180.0f;
    float eulerYawActual = - state->attitude.yaw * (float)M_PI / 180.0f;
    float rollError = eulerRollDesired - eulerRollActual;
    float pitchError = eulerPitchDesired - eulerPitchActual;
    float yawError = eulerYawDesired - eulerYawActual;
    if (yawError > (float)M_PI){
      yawError -= 2.0f*(float)M_PI;
    }
    else if (yawError < -(float)M_PI){
      yawError += 2.0f*(float)M_PI;
    }
    updateInt(&pidRollC, rollError);
    updateInt(&pidPitchC, pitchError);
    updateInt(&pidYawC, yawError);
    updateInt(&pidRollC_f, rollError);
    updateInt(&pidPitchC_f, pitchError);
    updateInt(&pidYawC_f, yawError);
    // pidRollC.integ += rollError*pidRollC.dt;
    // pidPitchC.integ += pitchError*pidPitchC.dt;
    // pidRollC.integ = pidRollC.integ > 1.0f ? 1.0f : pidRollC.integ;
    // pidRollC.integ = pidRollC.integ < -1.0f ? -1.0f : pidRollC.integ;
    // pidPitchC.integ = pidPitchC.integ > 1.0f ? 1.0f : pidPitchC.integ;
    // pidPitchC.integ = pidPitchC.integ < -1.0f ? -1.0f : pidPitchC.integ;
    float omega_x = sensors->gyro.x* (float)M_PI / 180.0f;
    float omega_y = -sensors->gyro.y* (float)M_PI / 180.0f;
    float omega_z = -sensors->gyro.z* (float)M_PI / 180.0f;// minus sign because of coordinate system. Might need further modification
    if (mode == modeGround){
      rollTorque = (Ixx*(pidRollC.kd*omega_x - omega_y*omega_z*I1) + pidRollC.kp*rollError + pidRollC.ki*pidRollC.integ);// - 0.1f*eulerRollActual);
      pitchTorque = (Iyy*(pidPitchC.kd*omega_y - omega_x*omega_z*I2) + pidPitchC.kp*pitchError + pidPitchC.ki*pidPitchC.integ);
      yawTorque = (Izz*(pidYawC.kd*(omega_z - desired_yaw_rate) - omega_x*omega_y*I3) + pidYawC.kp*yawError + pidYawC.ki*pidYawC.integ);
    }
    else
    {
      rollTorque = (Ixx*(pidRollC_f.kd*omega_x - omega_y*omega_z*I1) + pidRollC_f.kp*rollError + pidRollC_f.ki*pidRollC_f.integ);// - 0.1f*eulerRollActual);
      pitchTorque = (Iyy*(pidPitchC_f.kd*omega_y - omega_x*omega_z*I2) + pidPitchC_f.kp*pitchError + pidPitchC_f.ki*pidPitchC_f.integ);
      yawTorque = (Izz*(pidYawC_f.kd*(omega_z - desired_yaw_rate) - omega_x*omega_y*I3) + pidYawC_f.kp*yawError + pidYawC_f.ki*pidYawC_f.integ);
    }
    // rollTorque = (Ixx*(pidRollC.kd*omega_x - omega_y*omega_z*I1) + pidRollC.kp*rollError + pidRollC.ki*pidRollC.integ);
    if(rollTorque > 0.2f)
        rollTorque = 0.2f;
    if(rollTorque < -0.2f)
        rollTorque = -0.2f;
    // rollTorque = 0.0f;
    if(pitchTorque > 0.2f)
        pitchTorque = 0.2f;
    if(pitchTorque < -0.2f)
        pitchTorque = -0.2f;
    // pitchTorque = 0.0f;
    if (yawTorque > 0.1f)
        yawTorque = 0.1f;
    if (yawTorque < -0.1f)
        yawTorque = -0.1f;
    // yawTorque = 0.0f;
}

void attitudeControllerCustomizedResetAllPID()
{
    pidReset(&pidRollC);
    pidReset(&pidPitchC);
    pidReset(&pidYawC);
}
void attitudeControllerCustomizedGetActuatorOutput(float* roll, float* pitch, float* yaw)
{
  *roll = rollTorque;
  *pitch = pitchTorque;
  *yaw = yawTorque;
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

