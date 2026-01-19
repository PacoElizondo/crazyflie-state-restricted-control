"""
This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.
"""
import logging
import time
import pandas as pd
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import numpy as np
import matplotlib.pyplot as plt

plt.style.use('_mpl-gallery')


class LogArrayHolder(object):
    def __init__(self):
        self.log_x = np.empty(0)
        self.log_y = np.empty(0)
        self.log_z = np.empty(0)
        self.log_thrust = np.empty(0)
        self.log_torque_x = np.empty(0)
        self.log_torque_y = np.empty(0)
        self.log_torque_z = np.empty(0)
        self.log_kp_x = np.empty(0)
        self.log_kp_y = np.empty(0)
        self.log_kp_z = np.empty(0)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    # bat = data["pm.vbat"]

    # px = log_array.log_x
    # py = log_array.log_x
    # pz = log_array.log_x

    log_array.log_x = np.append(log_array.log_x, x)
    log_array.log_y = np.append(log_array.log_y, y)
    log_array.log_z = np.append(log_array.log_z, z)

    # print('pos: ({}, {}, {})'.format(x, y, z))
    # try:
    #     with open("temp/last_v.txt", "w") as file:
    #         file.write(str(bat))
    # except:
    #     print("Busy")


def gain_callback(timestamp, data, logconf):
    kpx = data['adaptive_control.trans_x']
    kpy = data['adaptive_control.trans_y']
    kpz = data['adaptive_control.trans_z']
    # print('input: ({}, {}, {}, {},)'.format(th, tx, ty, tz))

    log_array.log_kp_x = np.append(log_array.log_kp_x, kpx)
    log_array.log_kp_y = np.append(log_array.log_kp_y, kpy)
    log_array.log_kp_z = np.append(log_array.log_kp_z, kpz)    

def start_gain_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('adaptive_control.trans_x', 'float')
    log_conf.add_variable('adaptive_control.trans_y', 'float')
    log_conf.add_variable('adaptive_control.trans_z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(gain_callback)
    log_conf.start() 


def input_callback(timestamp, data, logconf):
    th = data['adaptive_control.thrust']
    tx = data['adaptive_control.torque_x']
    ty = data['adaptive_control.torque_y']
    tz = data['adaptive_control.torque_z']
    # print('input: ({}, {}, {}, {},)'.format(th, tx, ty, tz))

    log_array.log_thrust = np.append(log_array.log_thrust, th)
    log_array.log_torque_x = np.append(log_array.log_torque_x, tx)
    log_array.log_torque_y = np.append(log_array.log_torque_y, ty)
    log_array.log_torque_z = np.append(log_array.log_torque_z, tz)

def start_input_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('adaptive_control.thrust', 'float')
    log_conf.add_variable('adaptive_control.torque_x', 'float')
    log_conf.add_variable('adaptive_control.torque_y', 'float')
    log_conf.add_variable('adaptive_control.torque_z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(input_callback)
    log_conf.start() 



def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    
    # log_conf.add_variable("pm.vbat", "float")
    

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start() 



def run_sequence(scf, sequence, path_name):
    cf = scf.cf
    for position in range(len(sequence)):
        # for i in range(30):
            # x = sequence[position][0]
            # y = sequence[position][1]
            # z = sequence[position][2]
            # print(print(f'Sent: {x} {y} {z}'))
            cf.commander.send_position_setpoint(sequence[position][0], sequence[position][1], sequence[position][2]+0.1, 0)
                                                
            time.sleep(0.2)
            

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing



def set_controller(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.controller', '5')
    time.sleep(2.1)

def data_for_cylinder_restriction(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

if __name__ == '__main__':
    # URI to the Crazyflie to connect to
    origin = [1.8, 1.0, 1.0]
    log_array = LogArrayHolder()
    
    xy_offset = 0.2

    uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
    # path_name = "trajectory_square"
    # path_name = "trajectory"
    path_name = "trajectory_moving_loop"
    traj = pd.read_csv(f'{path_name}.csv')
    sequence = traj.values.tolist()
    
    print("Loading drivers")
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print("Drivers loaded")
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Resetting estimator")
        reset_estimator(scf)
        start_position_printing(scf)
        start_input_printing(scf)
        start_gain_printing(scf)
        print("Setting controller to outOfTree")
        set_controller(scf)
        print("Running sequence")
        # while True:
        for i in range (0,20):
            scf.cf.commander.send_position_setpoint(origin[0], origin[1], i/20, 0)
            time.sleep(0.1)
        run_sequence(scf, sequence, path_name)
        for i in range(0,20):
            scf.cf.commander.send_position_setpoint(origin[0], origin[1], origin[2], 0)
            time.sleep(0.1)
        for i in range (0,40):
            scf.cf.commander.send_position_setpoint(origin[0], origin[1], origin[2]-i/50,0)
            time.sleep(0.15)



    desired_position = pd.DataFrame(traj)

    restriction_radius = 0.7
    height = 1.8
    # rx = np.empty(0)
    # ry = np.empty(0)
    # rz = np.empty(0)

    # for i in range(0,1000):
    #     rx = np.append(rx, np.cos(i)*r + origin[0])
    #     ry = np.append(ry, np.sin(i)*r + origin[1])
    #     rz = np.append(rz, origin[2]   + origin[2])


    rx, ry, rz = data_for_cylinder_restriction(origin[0],origin[1],restriction_radius,height)
    

    position = pd.DataFrame({'X': log_array.log_x,'Y': log_array.log_y,'Z': log_array.log_z})
    position.to_csv('log_position.csv', index=False)
    ax1 = plt.figure(1).add_subplot(projection='3d')
    ax1.plot(log_array.log_x, log_array.log_y, log_array.log_z, label='trajectory')
    ax1.plot(desired_position.X, desired_position.Y, desired_position.Z, label='desired', color='r', linestyle='--')
    ax1.plot_surface(rx,ry,rz, alpha = 0.5, color='yellow')


    inputs = pd.DataFrame({'Thrust': log_array.log_thrust, 'Torque x': log_array.log_torque_x, 'Torque y': log_array.log_torque_y, 'Torque z': log_array.log_torque_z})
    inputs.to_csv('log_inputs.csv', index = False)
    ax2 = plt.figure(2).add_subplot()
    ax2.plot(log_array.log_thrust, label='Thrust', color='k')
    ax3 = plt.figure(3).add_subplot()
    ax3.plot(log_array.log_torque_x, label='Torque x', color='r')
    ax3.plot(log_array.log_torque_y, label='Torque y', color='g')
    ax3.plot(log_array.log_torque_z, label='Torque z', color='b')

    gains = pd.DataFrame({'kp x': log_array.log_kp_x, 'kp y': log_array.log_kp_y, 'kp z': log_array.log_kp_z})
    gains.to_csv('log_gains.csv', index = False)
    ax4 = plt.figure(4).add_subplot()
    ax4.plot(log_array.log_kp_x, label='kp x', color='r')
    ax4.plot(log_array.log_kp_y, label='kp y', color='g')
    ax4.plot(log_array.log_kp_z, label='kp z', color='b')

    plt.show()

    
