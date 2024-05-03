/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "controller_pid.h"

#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"


// This needs to be here bc of the app framework
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

/* ------------------ State Restricted Controller ----------------- */

#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"
#include "platform_defaults.h"

// CONSTANTS

static struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};


static float THRUST_MIN = 1;
static float THRUST_MAX = 18;
static float ROTATION_MAX = 30;

// Static gains
static float TRANS_KP_FIXED[] = {4.0f,4.0f,4.0f};
static float TRANS_KD_FIXED[] = {3.75f,3.75f,3.75f};
static float ROT_KP_FIXED[] = {150.0f,150.0f,150.0f};
static float ROT_KD_FIXED[] = {50.0f,50.0f,50.0f};

// Dynamic gains
float trans_kp[] = {4.0f,4.0f,4.0f};
float trans_kd[] = {3.75f,3.75f,3.75f};
float rot_kp[] = {150.0f,150.0f,150.0f};
float rot_kd[] = {50.0f,50.0f,50.0f};


// Adaptive gains
static float LAMBDA[] = {1.5, 0.005, 0.05, 0.005};
static float ALPHA[] = {6, 0.005, 8, 0.005};


// Restrictions
// to do



//Struct for logging
static bool isInit = false;

//signum
int sgn(number){
  if (number < 0) {
    return -1;
  }
  else if (number > 0){
    return 1;
  }
  else{
  return 0;
  }
}

void controllerOutOfTreeInit() {
  if (isInit) {
    return;
  }

  struct quat unit_q = qeye();

  isInit = true;
}

#define UPDATE_RATE RATE_100_HZ
static float DELTA_T = 0.01;

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  //To do
  
  
  struct quat unit_q = qeye();
  struct vec z_vec = mkvec(0,0,1);

  struct quat orientationError = qeye(); //init
  struct quat orientationDes = qeye(); //init

  float omega[3] = {0};
  omega[0] = radians(sensors->gyro.x);
  omega[1] = radians(sensors->gyro.y);
  omega[2] = radians(sensors->gyro.z);

  struct vec posError = mkvec(setpoint->position.x - state->position.x,
                              setpoint->position.y - state->position.y,
                              setpoint->position.z - state->position.z);

  if (RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {
    // Input variables
    struct vec accDes = vzero();
    float thrustDes = 0;




    // Current attitude
    struct quat orientation = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w);

    // Errors
    struct vec posErrorPrev = posError; 
    struct vec posError = mkvec(setpoint->position.x - state->position.x,
                                setpoint->position.y - state->position.y,
                                setpoint->position.z - state->position.z);
    float posErrorArray[] = {posError.x, posError.y, posError.z};

    struct vec velError = vdiv(posError, DELTA_T);
    float velErrorArray[] = {velError.x, velError.y, velError.z};

    // ------- Translational control -------

    // Calculate gain adaptation
    for(int i = 0; i < 3; i++){
      //To do: add restrictions
      float trans_kp_dot = (LAMBDA[0])*(posErrorArray[i])*sgn(posErrorArray[i]) + LAMBDA[1]*(TRANS_KP_FIXED[i] - trans_kp[i]);
      float trans_kd_dot = LAMBDA[2]*(velErrorArray[i]) + LAMBDA[3]*(TRANS_KD_FIXED[i] - trans_kd[i]);

      trans_kp[i] = trans_kp[i] + trans_kp_dot * DELTA_T;
      trans_kd[i] = trans_kd[i] + trans_kd_dot * DELTA_T;
    }
    struct vec trans_kp_vec = mkvec(trans_kp[0], trans_kp[1], trans_kp[2]);
    struct vec trans_kd_vec = mkvec(trans_kd[0], trans_kd[1], trans_kd[2]);

    // Translational control law equation
    struct vec trans_control = vadd(
      veltmul(trans_kp_vec,posError),
      veltmul(trans_kd_vec,velError)) ;
    trans_control.z -= GRAVITY_MAGNITUDE;
    trans_control = vscl(-CF_MASS, trans_control);
    float norm_trans_control = vmag(trans_control);
    struct vec control_direction = vzero();

    if (norm_trans_control != 0){
      control_direction = vdiv(trans_control,norm_trans_control);
    }

    struct quat curr_thrust_force_vectorq = qqmul(orientation,unit_q);
    curr_thrust_force_vectorq = qqmul(curr_thrust_force_vectorq,qinv(orientation));
    struct vec curr_thrust_force_vector = mkvec(curr_thrust_force_vectorq.x, curr_thrust_force_vectorq.y, curr_thrust_force_vectorq.z); //Fth
    float control_thrust = trans_control.z/curr_thrust_force_vector.z; //Fu

    // orientationDes = control_direction.z

    







        



  }
  
}

bool controllerOutOfTreeTest(){
  return true;
}