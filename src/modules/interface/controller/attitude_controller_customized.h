/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * attitude_controller.h: PID-based attitude controller
 */

#ifndef ATTITUDE_CONTROLLER_CUSTOMIZED_H_
#define ATTITUDE_CONTROLLER_CUSTOMIZED_H_

#include <stdbool.h>
#include <stdint.h>
#include "stabilizer_types.h"

void attitudeControllerCustomizedInit(const float updateDt);
bool attitudeControllerCustomizedTest();
void attitudeControllerCustomized(
       const sensorData_t *sensors,const attitude_t *attitude,const float desired_yaw_rate,const state_t *state, const stab_mode_t mode, const bool locmode, const float frameroll, const float yaw_fb);

/**
 * Reset controller roll, pitch and yaw PID's.
 */
void attitudeControllerCustomizedResetAllPID();

/**
 * Get the actuator output.
 */
void attitudeControllerCustomizedGetActuatorOutput(float* roll, float* pitch, float* yaw);

void attitudeControllerCustomizedGetPIDOutput(float* roll, float* pitch, float* yaw);


#endif /* ATTITUDE_CONTROLLER_CUSTOMIZED_H_ */
