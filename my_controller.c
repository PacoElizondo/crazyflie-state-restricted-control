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

static const struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};


static const float THRUST_MIN = 1;
static const float THRUST_MAX = 40;
static const float ROTATION_MAX = 30;


// const gains
static const float TRANS_KP_FIXED[] = {10.0f,10.0f,10.0f};
static const float TRANS_KD_FIXED[] = {6.0f,6.0f,6.0f};
static const float ROT_KP_FIXED[] = {150.0f,150.0f,150.0f};
static const float ROT_KD_FIXED[] = {50.0f,50.0f,50.0f};

// Dynamic gains
float trans_kp[] = {10.0f,10.0f,10.0f};
float trans_kd[] = {6.0f,6.0f,6.0f};
float rot_kp[] = {150.0f,150.0f,150.0f};
float rot_kd[] = {50.0f,50.0f,50.0f};


// Adaptive gains
static const float LAMBDA[] = {0.7f, 0.01f, 0.5f, 0.5f};
static const float ALPHA[] = {1.5f, 0.05f, 2.0f, 0.1f};

// Restrictions
// to do 


// Init store variables
static float pos_error_stored[] = {0.0f, 0.0f, 0.0f};
static float vel_error_stored[] = {0.0f, 0.0f, 0.0f};
static float omega_stored[] = {0.0f, 0.0f, 0.0f};
// static float orientation_stored[] = {0.0f, 0.0f, 0.0f, 0.0f};
static float orientation_error_stored[] = {0.0f, 0.0f, 0.0f, 0.0f};
// static float angular_velocity_stored[] = {0.0f,0.0f,0.0f};
static float angular_velocity_error_stored[] = {0.0f, 0.0f, 0.0f};

//Struct for logging
static bool isInit = false;

// Auxiliary functions
//signum
static inline int signum(float n){
  if (n < 0) {
    return -1;
  }
  else if (n > 0){
    return 1;
  }
  else{
  return 0;
  }
}

static inline struct quat load_q_from_array(float const *d) {
	return mkquat(d[0], d[1], d[2], d[3]);
}

static inline void store_from_q(struct quat q, float *d) {
	d[0] = (float)q.x; d[1] = (float)q.y; d[2] = (float)q.z; d[3] = (float)q.w;
}

// Quaternion to axis-angle
static inline struct vec rotvec(struct quat q){
  q = qnormalize(q);
  float ang = 2*acosf(q.w);
  float axis[] = {q.x, q.y, q.z};
  float magnitude = sqrtf(q.x*q.x + q.y*q.y + q.z*q.z);
  float rv[] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 3; i++){
    rv[i] = (magnitude != 0) ? (ang * axis[i]) / magnitude : 0;
  }
  return mkvec(rv[0],rv[1],rv[2]);
}

// From: https://la.mathworks.com/help/nav/ref/quaternion.log.html
static inline struct quat qlog(struct quat q){
    float norm_v = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    float norm_q = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    float log_scalar = logf(norm_q);
    float log_factor = (norm_v != 0) ? acosf(q.w / norm_q) / norm_v : 0;
    struct quat result;
    result.w = log_scalar;
    result.x = log_factor * q.x;
    result.y = log_factor * q.y;
    result.z = log_factor * q.z;
    return result;

}

// exp(q) = exp(a)*( cos(||v||) + (v/||v||)*sin(||v||) )
static inline struct quat qexp(struct quat q){
    float norm_v = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    float exp_scalar = expf(q.w) * cosf(norm_v);
    float exp_factor = (norm_v != 0) ? (expf(q.w) * sinf(norm_v) / norm_v) : 0;
    struct quat result;
    result.w = exp_scalar;
    result.x = exp_factor * q.x;
    result.y = exp_factor * q.y;
    result.z = exp_factor * q.z;
    return result;
}

void controllerOutOfTreeInit() {
  if (isInit) {
    return;
  }

  isInit = true;
}

#define UPDATE_RATE RATE_100_HZ
static const float DELTA_T = 0.01f;
static float t = 0;

void controllerOutOfTree(control_t *control,
                          const setpoint_t *setpoint,
                          const sensorData_t *sensors,
                          const state_t *state,
                          const stabilizerStep_t stabilizerStep) {

  struct quat z_q = mkquat(0,0,1,0);
  struct vec z_vec = mkvec(0,0,1);

  float omega[3] = {0};
  omega[0] = radians(sensors->gyro.x);
  omega[1] = radians(sensors->gyro.y);
  omega[2] = radians(sensors->gyro.z);

  static float control_thrust;
  static struct vec control_torque;

  if (RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {

    // Current attitude
    struct quat orientation = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w
    );

    struct vec posError = mkvec(
      state->position.x - setpoint->position.x,
      state->position.y - setpoint->position.y,
      state->position.z - setpoint->position.z
    );

    pos_error_stored[0] = posError.x;
    pos_error_stored[1] = posError.y;
    pos_error_stored[2] = posError.z;


    // Velocity error
    struct vec velError = mkvec(
      state->velocity.x - setpoint->velocity.x,
      state->velocity.y - setpoint->velocity.y,
      state->velocity.z - setpoint->velocity.z
    );
    vel_error_stored[0] = velError.x;
    vel_error_stored[1] = velError.y;
    vel_error_stored[2] = velError.z;
    // Angular velocity from gyroscope

    omega_stored[0] = omega[0];
    omega_stored[1] = omega[1];
    omega_stored[2] = omega[2];

    t += DELTA_T;
    
    // ----- Translational control ------


    for(int i = 0; i < 3; i++){
      float trans_kp_dot = LAMBDA[0]*pos_error_stored[i] + LAMBDA[1]*(TRANS_KP_FIXED[i] - trans_kp[i]);
      float trans_kd_dot = LAMBDA[2]*vel_error_stored[i] + LAMBDA[3]*(TRANS_KD_FIXED[i] - trans_kd[i]);

      trans_kp[i] = trans_kp[i] + trans_kp_dot * DELTA_T;
      trans_kd[i] = trans_kd[i] + trans_kd_dot * DELTA_T;
    };

    struct vec trans_kp_vec = mkvec(trans_kp[0], trans_kp[1], trans_kp[2]);
    struct vec trans_kd_vec = mkvec(trans_kd[0], trans_kd[1], trans_kd[2]);
    struct vec trans_control = vadd(
    veltmul(trans_kp_vec,posError),
    veltmul(trans_kd_vec,velError)) ;
    trans_control.z -= GRAVITY_MAGNITUDE;
    trans_control = vscl(-CF_MASS, trans_control);
    float norm_trans_control = vmag(trans_control);
    struct vec control_direction = vzero();

    if (norm_trans_control != 0){
      // control_direction = vdiv(trans_control,norm_trans_control);
      // trans_control = vscl(THRUST_MAX*tanhf(norm_trans_control/THRUST_MAX), control_direction);
      control_direction = vdiv(trans_control,norm_trans_control);
    }

    struct quat curr_thrust_force_vectorq = qqmul(orientation,z_q);
    curr_thrust_force_vectorq = qqmul(curr_thrust_force_vectorq,qinv(orientation));
    struct vec curr_thrust_force_vector = mkvec(curr_thrust_force_vectorq.x, curr_thrust_force_vectorq.y, curr_thrust_force_vectorq.z); //Fth
    control_thrust = trans_control.z/vdot(z_vec,curr_thrust_force_vector); //Fu


    

    struct vec vcross_temp = vcross(z_vec,control_direction);
    struct quat orientationDes = mkquat(vcross_temp.x,vcross_temp.y,vcross_temp.z,vdot(z_vec, control_direction));
    orientationDes = qlog(orientationDes);
    orientationDes = mkquat(0.5f*orientationDes.x, 0.5f*orientationDes.y, 0.5f*orientationDes.z, 0.5f*orientationDes.w);
    orientationDes = qexp(orientationDes);
    orientationDes = qnormalize(orientationDes);




    // orientationDes = qeye();

    // control_thrust = 0.5f*posError.z;

    // Orientation error
    // struct quat orientationErrorPrev = load_q_from_array(orientation_error_stored);
    struct quat orientationError = qqmul(orientation, qinv(orientationDes));
    orientationError = qnormalize(orientationError);
    store_from_q(orientationError, orientation_error_stored);

    // Angular Velocity Error
    struct vec orientationErrorVector = rotvec(orientationError); // Euler angle representation
    // struct vec angVelocityErrorVector = vdiv(rotvec(qqmul(orientationError,qinv(orientationErrorPrev))),DELTA_T);
    // angular_velocity_error_stored[0] = angVelocityErrorVector.x;
    // angular_velocity_error_stored[1] = angVelocityErrorVector.y;
    // angular_velocity_error_stored[2] = angVelocityErrorVector.z;


    //Angular velocity error from setpoint
    struct vec angVelocityErrorVector = mkvec(
      omega[0] - setpoint->attitudeRate.pitch,
      omega[1] - setpoint->attitudeRate.roll,
      omega[2] - setpoint->attitudeRate.yaw
    );

    // Invert reference
    if (vmag(orientationErrorVector) > M_PI_F || vmag(orientationErrorVector) < -M_PI_F){
      orientationDes = qneg(orientationDes);
      orientationError = qqmul(orientationDes,qinv(orientation));
    };

    // Rotational adaptive gains
    for(int i = 0; i < 3; i++){
      float rot_kp_dot = ALPHA[0]*orientation_error_stored[i] + ALPHA[1]*(ROT_KP_FIXED[i] - rot_kp[i]);
      float rot_kd_dot = ALPHA[2]*angular_velocity_error_stored[i] + ALPHA[3]*(ROT_KD_FIXED[i] - rot_kd[i]);

      rot_kp[i] = rot_kp[i] + rot_kp_dot * DELTA_T;
      rot_kd[i] = rot_kd[i] + rot_kd_dot * DELTA_T;
    };

    struct vec rot_kp_vec = mkvec(rot_kp[0],rot_kp[1],rot_kp[2]);
    struct vec rot_kd_vec = mkvec(rot_kd[0],rot_kd[1],rot_kd[2]);

    // orientation_control = (-ang_kp.*orientation_error_vector)-(ang_kd.*angular_velocity_error);
    // torque = INERTIA*(orientation_control + cross(angular_velocity,INERTIA*angular_velocity));
    struct vec orientation_control = vsub(veltmul(vneg(rot_kp_vec),orientationErrorVector),veltmul(rot_kd_vec,angVelocityErrorVector));
    // struct vec control_torque = mvmul(CRAZYFLIE_INERTIA,vadd(orientation_control,vcross(angVelocityVector,mvmul(CRAZYFLIE_INERTIA,angVelocityVector))));
    control_torque = mvmul(CRAZYFLIE_INERTIA,orientation_control);

    // Max and min control
    // 
  }
    // Control Input
  if (setpoint->mode.z == modeDisable) {
  control->thrustSi = 0.0f;
  control->torque[0] =  0.0f;
  control->torque[1] =  0.0f;
  control->torque[2] =  0.0f;
  } else {
  // control the body torques
  control->thrustSi = control_thrust;
  control->torqueX  = control_torque.x;
  control->torqueY  = control_torque.y;
  control->torqueZ  = control_torque.z;
  }  

  // todo: reset variables

  control->controlMode = controlModeForceTorque;


  
}

bool controllerOutOfTreeTest(){
  return true;
}