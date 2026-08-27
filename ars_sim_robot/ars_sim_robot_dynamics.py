import numpy as np
from numpy import *

import math

# ROS
import rclpy
from rclpy.time import Time

import tf_transformations


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers




class ArsSimRobotDynamics:

  #######

  #
  flag_disp_pitch_roll = None

  #
  robot_velo_cmd_flag = None

  #
  robot_velo_cmd_sat = None

  #
  robot_dyn_const_cmd = None
  robot_dyn_const_lin = None
  robot_dyn_const_ang = None

  #
  mass_quadrotor = None

  #
  gravity = None

  # aerodynamics coef per mass unit
  aerodynamics_coef = None


  #
  time_stamp_ros = None

  #
  flag_cmd_control_enabled = None
  #
  flag_robot_motion_enabled = None

  #
  # m/s
  robot_velo_lin_cmd_in = None
  # rad/s
  robot_velo_ang_cmd_in = None

  #
  # m/s
  robot_velo_lin_cmd_ef_robot = None
  # rad/s
  robot_velo_ang_cmd_ef_robot = None

  #
  robot_posi = None
  robot_atti_quat = None

  #
  robot_velo_lin_robot = None
  robot_velo_ang_robot = None

  #
  robot_acce_lin_robot = None
  robot_acce_ang_robot = None

  


  #########

  def __init__(self):

    #
    #
    self.gravity=9.81


    #
    self.flag_disp_pitch_roll = True

    #
    self.robot_velo_cmd_flag = {'lin': {'x': True, 'y': True, 'z': True}, 
                                'ang': {'z': True}}

    #
    self.robot_velo_cmd_sat = {'lin': {'x': [-5.0, 5.0], 'y': [-5.0, 5.0], 'z': [-5.0, 5.0]}, 
                                'ang': {'z': [-5.0, 5.0]}}


    #
    self.robot_dyn_const_cmd = { 'u_x': {'k': 1.0, 'T': 0.1}, 
                              'u_y': {'k': 1.0, 'T': 0.1},
                              'u_z': {'k': 1.0, 'T': 0.1},
                              'u_phi': {'k': 1.0, 'T': 0.1} 
                              }

    #
    self.robot_dyn_const_lin = { 'vx': {'k': 1.0, 'T': 0.8355}, 
                              'vy': {'k': 1.0, 'T': 0.7701},
                              'vz': {'k': 1.0, 'T': 0.5013},
                              }

    #
    self.robot_dyn_const_ang = { 'wz': {'k': 1.0, 'T': 0.5142} 
                              }

    #
    self.mass_quadrotor=2.0


    # aerodynamics coef per mass unit
    self.aerodynamics_coef = { 'x': 0.2, 'y': 0.2, 'z': 0.2 }
    


    #
    self.time_stamp_ros = Time()

    #
    self.flag_cmd_control_enabled = True
    self.flag_robot_motion_enabled = True

    #
    self.robot_velo_lin_cmd_in = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_in = np.zeros((3,), dtype=float)

    self.robot_velo_lin_cmd_ef_robot = np.zeros((3,), dtype=float)
    self.robot_velo_ang_cmd_ef_robot = np.zeros((3,), dtype=float)

    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()

    self.robot_velo_lin_robot = np.zeros((3,), dtype=float)
    self.robot_velo_ang_robot = np.zeros((3,), dtype=float)

    self.robot_acce_lin_robot = np.zeros((3,), dtype=float)
    self.robot_acce_ang_robot = np.zeros((3,), dtype=float)

    #
    return


  @staticmethod
  def saturate(value, low_value=-inf, upp_value=inf):
    if(value < low_value):
      value = low_value
    if(value > upp_value):
      value = upp_value
    return value


  @staticmethod
  def computeSign(value):
    sign_value=1.0
    if(value>0.0):
      sign_value=1.0
    else:
      sign_value=-1.0
    return sign_value


  def setTimeStamp(self, time_stamp_ros):

    self.time_stamp_ros = time_stamp_ros

    return

  def getTimeStamp(self):

    return self.time_stamp_ros


  def setRobotSimDescription(self, robot_sim_description):

    #
    self.flag_disp_pitch_roll = robot_sim_description['flag_disp_pitch_roll']

    #
    self.robot_velo_cmd_flag = robot_sim_description['robot_velo_cmd_flag']

    #
    self.robot_velo_cmd_sat = robot_sim_description['robot_velo_cmd_sat']

    #
    self.robot_dyn_const_cmd = robot_sim_description['robot_dyn_const_cmd']

    #
    self.robot_dyn_const_lin = robot_sim_description['robot_dyn_const_lin']

    #
    self.robot_dyn_const_ang = robot_sim_description['robot_dyn_const_ang']

    #
    self.mass_quadrotor = robot_sim_description['mass_quadrotor']

    #
    self.aerodynamics_coef = robot_sim_description['aerodynamics_coef']

    # End
    return


  def setRobotVelCmd(self, lin_vel_cmd, ang_vel_cmd):

    if(self.robot_velo_cmd_flag['lin']['x']):
      self.robot_velo_lin_cmd_in[0] = lin_vel_cmd[0]
      self.robot_velo_lin_cmd_in[0] = self.saturate(self.robot_velo_lin_cmd_in[0], self.robot_velo_cmd_sat['lin']['x'][0], self.robot_velo_cmd_sat['lin']['x'][1])
    else:
      self.robot_velo_lin_cmd_in[0] = 0.0

    if(self.robot_velo_cmd_flag['lin']['y']):
      self.robot_velo_lin_cmd_in[1] = lin_vel_cmd[1]
      self.robot_velo_lin_cmd_in[1] = self.saturate(self.robot_velo_lin_cmd_in[1], self.robot_velo_cmd_sat['lin']['y'][0], self.robot_velo_cmd_sat['lin']['y'][1])
    else:
      self.robot_velo_lin_cmd_in[1] = 0.0

    if(self.robot_velo_cmd_flag['lin']['z']):
      self.robot_velo_lin_cmd_in[2] = lin_vel_cmd[2]
      self.robot_velo_lin_cmd_in[2] = self.saturate(self.robot_velo_lin_cmd_in[2], self.robot_velo_cmd_sat['lin']['z'][0], self.robot_velo_cmd_sat['lin']['z'][1])
    else:
      self.robot_velo_lin_cmd_in[2] = 0.0

    if(self.robot_velo_cmd_flag['ang']['z']):
      self.robot_velo_ang_cmd_in[2] = ang_vel_cmd[2]
      self.robot_velo_ang_cmd_in[2] = self.saturate(self.robot_velo_ang_cmd_in[2], self.robot_velo_cmd_sat['ang']['z'][0], self.robot_velo_cmd_sat['ang']['z'][1])
    else:
      self.robot_velo_ang_cmd_in[2] = 0.0

    return


  def getRobotPosi(self):
    return self.robot_posi


  def getRobotAttiQuat(self):
    return self.robot_atti_quat


  def getRobotVeloLinWorld(self):
    return ars_lib_helpers.Conversions.convertVelLinFromRobotToWorld(self.robot_velo_lin_robot, self.robot_atti_quat, False)


  def getRobotVeloLinRobot(self):
    return self.robot_velo_lin_robot


  def getRobotVeloAngRobot(self):
    return self.robot_velo_ang_robot


  def getRobotVeloAngWorld(self):
    return ars_lib_helpers.Conversions.convertVelAngFromRobotToWorld(self.robot_velo_ang_robot, self.robot_atti_quat, False)


  def getRobotAcceLinWorld(self):
    return ars_lib_helpers.Conversions.convertVelLinFromRobotToWorld(self.robot_acce_lin_robot, self.robot_atti_quat, False)


  def getRobotAcceLinRobot(self):
    return self.robot_acce_lin_robot


  def getRobotAcceAngRobot(self):
    return self.robot_acce_ang_robot


  def getRobotAcceAngWorld(self):
    return ars_lib_helpers.Conversions.convertVelAngFromRobotToWorld(self.robot_acce_ang_robot, self.robot_atti_quat, False)


  def simRobot(self, time_stamp_ros):

    # Delta time
    delta_time_ros = time_stamp_ros - self.time_stamp_ros
    delta_time = delta_time_ros.nanoseconds/1e9


    # Calculations

    #
    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(self.robot_atti_quat)

    #
    robot_velo_lin_world = ars_lib_helpers.Conversions.convertVelLinFromRobotToWorld(self.robot_velo_lin_robot, self.robot_atti_quat, False)


    #
    if(self.flag_cmd_control_enabled is False):
      #
      self.robot_velo_lin_cmd_in[0] = 0.0
      self.robot_velo_lin_cmd_in[1] = 0.0
      self.robot_velo_lin_cmd_in[2] = 0.0
      #
      self.robot_velo_ang_cmd_in[2] = 0.0



    if(self.flag_robot_motion_enabled is False):

      # Update state
      #
      self.robot_velo_lin_cmd_ef_robot = np.zeros((3,), dtype=float)
      self.robot_velo_ang_cmd_ef_robot = np.zeros((3,), dtype=float)
      #
      self.robot_posi = self.robot_posi
      self.robot_velo_lin_robot = np.zeros((3,), dtype=float)
      self.robot_acce_lin_robot = np.zeros((3,), dtype=float)
      #
      self.robot_atti_quat = self.robot_atti_quat
      self.robot_velo_ang_robot = np.zeros((3,), dtype=float)
      self.robot_acce_ang_robot = np.zeros((3,), dtype=float)


    else:

      ## Control Comd

      # Control command - Linear
      new_robot_velo_lin_cmd_ef_robot = np.zeros((3,), dtype=float)
      new_robot_velo_lin_cmd_ef_robot[0] = self.robot_velo_lin_cmd_ef_robot[0] * (1.0 - delta_time/self.robot_dyn_const_cmd['u_x']['T']) + delta_time/self.robot_dyn_const_cmd['u_x']['T']*self.robot_dyn_const_cmd['u_x']['k']*self.robot_velo_lin_cmd_in[0]
      new_robot_velo_lin_cmd_ef_robot[1] = self.robot_velo_lin_cmd_ef_robot[1] * (1.0 - delta_time/self.robot_dyn_const_cmd['u_y']['T']) + delta_time/self.robot_dyn_const_cmd['u_y']['T']*self.robot_dyn_const_cmd['u_y']['k']*self.robot_velo_lin_cmd_in[1]
      new_robot_velo_lin_cmd_ef_robot[2] = self.robot_velo_lin_cmd_ef_robot[2] * (1.0 - delta_time/self.robot_dyn_const_cmd['u_z']['T']) + delta_time/self.robot_dyn_const_cmd['u_z']['T']*self.robot_dyn_const_cmd['u_z']['k']*self.robot_velo_lin_cmd_in[2]

      # Control command - Angular
      new_robot_velo_ang_cmd_ef_robot = np.zeros((3,), dtype=float)
      new_robot_velo_ang_cmd_ef_robot[2] = self.robot_velo_ang_cmd_ef_robot[2] * (1.0 - delta_time/self.robot_dyn_const_cmd['u_phi']['T']) + delta_time/self.robot_dyn_const_cmd['u_phi']['T']*self.robot_dyn_const_cmd['u_phi']['k']*self.robot_velo_ang_cmd_in[2]


      ## State:linear

      # Accel lin
      new_robot_acce_lin_robot = np.zeros((3,), dtype=float)
      new_robot_acce_lin_robot[0] = 1/self.robot_dyn_const_lin['vx']['T']*(-self.robot_velo_lin_robot[0]+self.robot_dyn_const_lin['vx']['k']*self.robot_velo_lin_cmd_ef_robot[0])
      new_robot_acce_lin_robot[1] = 1/self.robot_dyn_const_lin['vy']['T']*(-self.robot_velo_lin_robot[1]+self.robot_dyn_const_lin['vy']['k']*self.robot_velo_lin_cmd_ef_robot[1])
      new_robot_acce_lin_robot[2] = 1/self.robot_dyn_const_lin['vz']['T']*(-self.robot_velo_lin_robot[2]+self.robot_dyn_const_lin['vz']['k']*self.robot_velo_lin_cmd_ef_robot[2])

      # Velo lin
      new_robot_velo_lin_robot = np.zeros((3,), dtype=float)
      new_robot_velo_lin_robot[0] = self.robot_velo_lin_robot[0] + delta_time * self.robot_acce_lin_robot[0]
      new_robot_velo_lin_robot[1] = self.robot_velo_lin_robot[1] + delta_time * self.robot_acce_lin_robot[1]
      new_robot_velo_lin_robot[2] = self.robot_velo_lin_robot[2] + delta_time * self.robot_acce_lin_robot[2]

      # Position
      new_robot_posi = np.zeros((3,), dtype=float)
      new_robot_posi[0] = self.robot_posi[0] + delta_time * robot_velo_lin_world[0]
      new_robot_posi[1] = self.robot_posi[1] + delta_time * robot_velo_lin_world[1]
      new_robot_posi[2] = self.robot_posi[2] + delta_time * robot_velo_lin_world[2]



      ## State: Orientation
      
      #
      delta_rho = delta_time*self.robot_velo_ang_robot[2]
      dq_simp_robot = np.zeros((2,), dtype=float)
      dq_simp_robot[0] = math.cos(0.5*delta_rho) # dqw
      dq_simp_robot[1] = math.sin(0.5*delta_rho) # dqz
      #
      new_robot_atti_quat_simp = np.zeros((2,), dtype=float)
      new_robot_atti_quat_simp = ars_lib_helpers.Quaternion.quatSimpProd(robot_atti_quat_simp, dq_simp_robot)


      # Yaw
      new_robot_atti_ang_yaw = ars_lib_helpers.Quaternion.angleFromQuatSimp(new_robot_atti_quat_simp)


      # Pitch and roll
      if(self.flag_disp_pitch_roll):

        #
        sign_robot_velo_lin_x_robot=self.computeSign(self.robot_velo_lin_robot[0])
        sign_robot_velo_lin_y_robot=self.computeSign(self.robot_velo_lin_robot[1])
        sign_robot_velo_lin_z_robot=self.computeSign(self.robot_velo_lin_robot[2])

        #
        thrust_specific_lin_x = self.robot_acce_lin_robot[0]+sign_robot_velo_lin_x_robot*1.0/self.mass_quadrotor*self.aerodynamics_coef['x']*self.robot_velo_lin_robot[0]*self.robot_velo_lin_robot[0]
        thrust_specific_lin_y = self.robot_acce_lin_robot[1]+sign_robot_velo_lin_y_robot*1.0/self.mass_quadrotor*self.aerodynamics_coef['y']*self.robot_velo_lin_robot[1]*self.robot_velo_lin_robot[1]
        thrust_specific_lin_z = self.robot_acce_lin_robot[2]+self.gravity+sign_robot_velo_lin_z_robot*1.0/self.mass_quadrotor*self.aerodynamics_coef['z']*self.robot_velo_lin_robot[2]*self.robot_velo_lin_robot[2]

        #
        new_robot_atti_ang_pitch =  math.atan2( thrust_specific_lin_x , thrust_specific_lin_z )
        new_robot_atti_ang_roll  = -math.atan2( thrust_specific_lin_y , thrust_specific_lin_z )

        #
        new_robot_atti_ang_pitch = self.saturate(new_robot_atti_ang_pitch, -math.pi/4.0, math.pi/4.0)
        new_robot_atti_ang_roll = self.saturate(new_robot_atti_ang_roll, -math.pi/4.0, math.pi/4.0)

      else:
        new_robot_atti_ang_pitch = 0.0
        new_robot_atti_ang_roll = 0.0

      #
      new_robot_atti_quat_tf = tf_transformations.quaternion_from_euler(new_robot_atti_ang_roll, new_robot_atti_ang_pitch, new_robot_atti_ang_yaw, axes='sxyz')
      new_robot_atti_quat = np.roll(new_robot_atti_quat_tf, 1)


      # Accel ang
      new_robot_acce_ang_robot = np.zeros((3,), dtype=float)
      new_robot_acce_ang_robot[2] = 1/self.robot_dyn_const_ang['wz']['T']*(-self.robot_velo_ang_robot[2]+self.robot_dyn_const_ang['wz']['k']*self.robot_velo_ang_cmd_ef_robot[2])



      # Velo ang
      new_robot_velo_ang_robot = np.zeros((3,), dtype=float)
      new_robot_velo_ang_robot[2] = self.robot_velo_ang_robot[2] + delta_time * self.robot_acce_ang_robot[2]



      ## Update state
      #
      self.robot_velo_lin_cmd_ef_robot = new_robot_velo_lin_cmd_ef_robot
      self.robot_velo_ang_cmd_ef_robot = new_robot_velo_ang_cmd_ef_robot
      #
      self.robot_posi = new_robot_posi
      self.robot_velo_lin_robot = new_robot_velo_lin_robot
      self.robot_acce_lin_robot = new_robot_acce_lin_robot
      #
      self.robot_atti_quat = new_robot_atti_quat
      self.robot_velo_ang_robot = new_robot_velo_ang_robot
      self.robot_acce_ang_robot = new_robot_acce_ang_robot


    # Update timestamp
    self.time_stamp_ros = time_stamp_ros

    # End
    return  
