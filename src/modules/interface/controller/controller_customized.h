/**
Customized controller for wheeled quadcopter in ground mode
 */
#ifndef __CONTROLLER_CUSTOMIZED_H__
#define __CONTROLLER_CUSTOMIZED_H__

#include "stabilizer_types.h"

void controllerCustomizedInit(void);
bool controllerCustomizedTest(void);
void controllerCustomized(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);
// struct inter_disturbance_t{
//     float u1;
//     float u2;
//     float u3;
//     float u4;
// };
// struct disturbance_t{
//     float u1;
//     float u2;
//     float u3;
//     float u4;
// };
// struct K_obs_t{
//     float K1;
//     float K2;
//     float K3;
//     float K4;
// };
#endif //__CONTROLLER_PID_H__
