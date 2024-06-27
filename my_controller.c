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


static const float THRUST_MIN = 0.15f;
static const float THRUST_MAX = 30.0f;
static const float ROTATION_MAX = 0.015f;
static const float ANGLE_MAX = 0.6f;


// const gains
static const float TRANS_KP_FIXED[] = {10.0f,10.0f,7.0f};
static const float TRANS_KD_FIXED[] = {6.0f,6.0f,3.75f};
static const float ROT_KP_FIXED[] = {90.0f,90.0f,90.0f};
static const float ROT_KD_FIXED[] = {40.0f,40.0f,40.0f};

// Dynamic gains
float trans_kp[] = {10.0f,10.0f,7.0f};
float trans_kp_applied[] = {10.0f, 10.0f, 7.0f};
float trans_kd[] = {6.0f,6.0f,3.75f};
float rot_kp[] = {90.0f,90.0f,90.0f};
float rot_kd[] = {40.0f,40.0f,40.0f};



// Adaptive gains
static const float LAMBDA_RESTRICTION[] = {1.0f, 20.0f, 1.5f, 0.5f};
static const float LAMBDA_Z[] = {8.0f, 1.0f, 1.5f, 0.5f};
static const float ALPHA[] = {3.0f, 0.05f, 1.0f, 0.05f};


// Restrictions
static const float RESTRICTION_RADIUS = 0.7f;
static const float LAMBDA_MAX = 0.5f * 0.5f;
static const float ORIGIN[] = {1.8f, 1.0f, 1.0f};

// Init store variables
static float position_error_origin[] = {0.0f, 0.0f, 0.0f};
static float pos_error_prev[] = {0.0f, 0.0f, 0.0f};
static float vel_error_stored[] = {0.0f, 0.0f, 0.0f};
static float omega_stored[] = {0.0f, 0.0f, 0.0f};
// static float orientation_stored[] = {0.0f, 0.0f, 0.0f, 0.0f};
static float orientation_error_stored[] = {0.0f, 0.0f, 0.0f, 0.0f};
// static float angular_velocity_stored[] = {0.0f,0.0f,0.0f};
static float angular_velocity_error_stored[] = {0.0f, 0.0f, 0.0f};

// Input variables
static float control_thrust;
static struct vec control_torque;

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

static inline float qmag(struct quat q){
  float norm_q = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return norm_q;
}

static inline struct quat qsmul(struct quat q, float s){
  struct quat qs = mkquat(q.x*s , q.y*s , q.z*s , q.w*s);
  return qs;
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

  if (RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {

    t += DELTA_T;

    // Current attitude
    struct quat orientation = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w
    );

    // Angular velocity from gyroscope
    omega_stored[0] = omega[0];
    omega_stored[1] = omega[1];
    omega_stored[2] = omega[2];
    
    // Velocity error
    // struct vec velError = mkvec(
    //   state->velocity.x - setpoint->velocity.x,
    //   state->velocity.y - setpoint->velocity.y,
    //   state->velocity.z - setpoint->velocity.z
    // );
    // vel_error_stored[0] = velError.x;
    // vel_error_stored[1] = velError.y;
    // vel_error_stored[2] = velError.z;

    pos_error_prev[0] = position_error_origin[0];
    pos_error_prev[1] = position_error_origin[1];
    pos_error_prev[2] = position_error_origin[2];

    // Position relative to the origin
    float position_origin[] = {
      state->position.x - ORIGIN[0],
      state->position.y - ORIGIN[1],
      state->position.z
    };
    
    
    float desired_position_origin[] = {
      setpoint->position.x - ORIGIN[0],
      setpoint->position.y - ORIGIN[1],
      setpoint->position.z
    };
    // struct vec desired_position_origin_vec = mkvec(desired_position_origin[0], desired_position_origin[1], desired_position_origin[2]);

    
    struct vec position_error_origin_vec = mkvec(position_error_origin[0], position_error_origin[1], position_error_origin[2]);

    // Position restriction limits
    float r = sqrt((position_origin[0] * position_origin[0]) + (position_origin[1] * position_origin[1]));
    float rd = sqrt((desired_position_origin[0] * desired_position_origin[0]) + (desired_position_origin[1] * desired_position_origin[1]));
    float reduction_variable = 0.9999f;
    float increment_variable = 0.05f;
    
    float razimuth = atan2f(position_origin[1], position_origin[0]);
    float dazimuth = atan2f(desired_position_origin[1], desired_position_origin[0]);

    if(r > RESTRICTION_RADIUS*reduction_variable){
      position_origin[0] = RESTRICTION_RADIUS*reduction_variable*cosf(razimuth);
      position_origin[1] = RESTRICTION_RADIUS*reduction_variable*sinf(razimuth);
      r = sqrt((position_origin[0] * position_origin[0]) + (position_origin[1] * position_origin[1]));

    }

    if(rd > RESTRICTION_RADIUS){
      desired_position_origin[0] = RESTRICTION_RADIUS*cosf(dazimuth) + increment_variable;
      desired_position_origin[1] = RESTRICTION_RADIUS*sinf(dazimuth) + increment_variable;
      rd = sqrt((desired_position_origin[0] * desired_position_origin[0]) + (desired_position_origin[1] * desired_position_origin[1]));
    }



    
    // if (desired_position_origin[0]*signum(desired_position_origin[0]) > 
    //   RESTRICTION_RADIUS*cosf(azimuth)*increment_variable*signum(RESTRICTION_RADIUS*cosf(azimuth))){
    //   desired_position_origin[0] = RESTRICTION_RADIUS*cosf(azimuth)*increment_variable;
    //   position_error_origin[0] = position_origin[0] - desired_position_origin[0];
    //   position_error_origin_vec.x = position_error_origin[0];
    // }
    // if (desired_position_origin[1]*signum(desired_position_origin[1]) > 
    //   RESTRICTION_RADIUS*sinf(azimuth)*increment_variable*signum(RESTRICTION_RADIUS*sinf(azimuth))){
    //   desired_position_origin[1] = RESTRICTION_RADIUS*cosf(azimuth)*increment_variable;
    //   position_error_origin[1] = position_origin[1] - desired_position_origin[1];
    //   position_error_origin_vec.y = position_error_origin[1];
    // }

    

    position_error_origin[0] = position_origin[0] - desired_position_origin[0];
    position_error_origin[1] = position_origin[1] - desired_position_origin[1];
    position_error_origin[2] = position_origin[2] - desired_position_origin[2];
    position_error_origin_vec = mkvec(position_error_origin[0], position_error_origin[1], position_error_origin[2]);

    

    for(int i = 0; i < 3; i++){
      vel_error_stored[i] = ((position_error_origin[i] - pos_error_prev[i])/DELTA_T)*0.99f;
    }

    struct vec velError = mkvec(vel_error_stored[0], vel_error_stored[1], vel_error_stored[2]);
    
    
    

    // ----- Translational control ------
    
    float trans_kp_dot;
    float trans_kd_dot;
    for(int i = 0; i < 3; i++){
      if(i == 0){
        trans_kp_dot = (-LAMBDA_RESTRICTION[0]/(RESTRICTION_RADIUS - r))*position_error_origin[i]*signum(position_error_origin[i]) + LAMBDA_RESTRICTION[1]*(TRANS_KP_FIXED[i] - trans_kp[i]);
        trans_kp[i] = trans_kp[i] + trans_kp_dot * DELTA_T;
        trans_kd_dot = LAMBDA_RESTRICTION[2]*vel_error_stored[i] + LAMBDA_RESTRICTION[3]*(TRANS_KD_FIXED[i] - trans_kd[i]);
      }
      if(i == 1){
        trans_kp_dot = (-LAMBDA_RESTRICTION[0]-0.5f/(RESTRICTION_RADIUS - r))*position_error_origin[i]*signum(position_error_origin[i]) + LAMBDA_RESTRICTION[1]*(TRANS_KP_FIXED[i] - trans_kp[i]);
        trans_kp[i] = trans_kp[i] + trans_kp_dot * DELTA_T;
        trans_kd_dot = LAMBDA_RESTRICTION[2]*vel_error_stored[i] + LAMBDA_RESTRICTION[3]*(TRANS_KD_FIXED[i] - trans_kd[i]);
      }
      else {
        trans_kp_dot = LAMBDA_Z[0]*position_error_origin[2] + LAMBDA_Z[1]*(TRANS_KP_FIXED[2] - trans_kp[2]);
        trans_kp[2] = trans_kp[2] + trans_kp_dot * DELTA_T;
        trans_kd_dot = LAMBDA_Z[2]*vel_error_stored[2] + LAMBDA_Z[3]*(TRANS_KD_FIXED[2] - trans_kd[2]);
      }
      
      trans_kd[i] = trans_kd[i] + trans_kd_dot * DELTA_T;
      if(rd < r && trans_kp[i] < 0){
        desired_position_origin[0] = RESTRICTION_RADIUS*cosf(dazimuth);
        desired_position_origin[1] = RESTRICTION_RADIUS*sinf(dazimuth);
        rd = sqrt((desired_position_origin[0] * desired_position_origin[0]) + (desired_position_origin[1] * desired_position_origin[1])); 
      }

      if(trans_kp[i] < 1.0f && trans_kp[i] > -1.0f){
        trans_kp_applied[i] = 1.0f*signum(trans_kp_applied[i]);
      }
      else{
        trans_kp_applied[i] = trans_kp[i];
      }

      // if(trans_kp_applied[i] < -200.0f){
      //   trans_kp_applied[i] = -200.0f;
      // }


    };
    position_error_origin[0] = position_origin[0] - desired_position_origin[0];
    position_error_origin[1] = position_origin[1] - desired_position_origin[1];
    position_error_origin[2] = position_origin[2] - desired_position_origin[2];
    position_error_origin_vec = mkvec(position_error_origin[0], position_error_origin[1], position_error_origin[2]);

    struct vec trans_kp_vec = mkvec(trans_kp_applied[0], trans_kp_applied[1], trans_kp_applied[2]);
    struct vec trans_kd_vec = mkvec(trans_kd[0], trans_kd[1], trans_kd[2]);
    struct vec trans_control = vadd(
      veltmul(trans_kp_vec,position_error_origin_vec),
      veltmul(trans_kd_vec,velError)
    );
    trans_control.z -= GRAVITY_MAGNITUDE;
    trans_control = vscl(-CF_MASS, trans_control);
    float norm_trans_control = vmag(trans_control);
    struct vec control_direction = vzero();

    if (norm_trans_control != 0){
      control_direction = vdiv(trans_control,norm_trans_control);
      trans_control = vscl(THRUST_MAX*tanhf(norm_trans_control/THRUST_MAX), control_direction);
    }

    norm_trans_control = vmag(trans_control);
    control_direction = vdiv(trans_control,norm_trans_control);


    struct quat curr_thrust_force_vectorq = qqmul(orientation,z_q);
    curr_thrust_force_vectorq = qqmul(curr_thrust_force_vectorq,qinv(orientation));
    struct vec curr_thrust_force_vector = mkvec(curr_thrust_force_vectorq.x, curr_thrust_force_vectorq.y, curr_thrust_force_vectorq.z); //Fth
    control_thrust = trans_control.z/vdot(z_vec,curr_thrust_force_vector); //Fu
    // if (control_thrust < 0.21f){
    //   control_thrust = 0.21f;
    // }

    

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
    float orientationDes_norm = qmag(orientationDes);
    float orientation_norm = qmag(orientation);
    if (orientationDes_norm > orientation_norm - 0.01f){
      orientationDes_norm = orientation_norm - 0.01f;
      orientationDes = qsmul(orientationDes,orientationDes_norm);
    }

  
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

    // if(vmag(orientationErrorVector) != 0){
    //   struct vec orientation_direction = vdiv(orientationErrorVector, vmag(orientationErrorVector));
    //   orientationErrorVector = vscl(ANGLE_MAX*tanhf(vmag(orientationErrorVector)/ANGLE_MAX), orientation_direction);
    // }

    // orientation_control = (-ang_kp.*orientation_error_vector)-(ang_kd.*angular_velocity_error);
    // torque = INERTIA*(orientation_control + cross(angular_velocity,INERTIA*angular_velocity));
    struct vec orientation_control = vsub(veltmul(vneg(rot_kp_vec),orientationErrorVector),veltmul(rot_kd_vec,angVelocityErrorVector));
    // struct vec control_torque = mvmul(CRAZYFLIE_INERTIA,vadd(orientation_control,vcross(angVelocityVector,mvmul(CRAZYFLIE_INERTIA,angVelocityVector))));

    // float orientation_control_norm = vmag(orientation_control);
    // if(orientation_control_norm != 0){
    //   orientation_control = vscl(ROTATION_MAX*tanhf(orientation_control_norm/ROTATION_MAX),vdiv(orientation_control,orientation_control_norm));
    // }
    control_torque = mvmul(CRAZYFLIE_INERTIA,orientation_control);
    
    
    // if(vmag(control_torque) != 0){
    //   struct vec torque_direction = vdiv(control_torque,vmag(control_torque));
    //   control_torque = vscl(ROTATION_MAX*tanhf(vmag(control_torque)/ROTATION_MAX), torque_direction);
    // }

    
    

    
    
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

LOG_GROUP_START(adaptive_control)
/**
 * @brief Thrust
 */
LOG_ADD(LOG_FLOAT, thrust, &control_thrust)
/**
 * @brief Torque x
 */
LOG_ADD(LOG_FLOAT, torque_x, &control_torque.x)
/**
 * @brief Torque y
 */
LOG_ADD(LOG_FLOAT, torque_y, &control_torque.y)
/**
 * @brief Torque z
 */
LOG_ADD(LOG_FLOAT, torque_z, &control_torque.z)
LOG_GROUP_STOP(adaptive_control)