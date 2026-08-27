import numpy as np
from numpy import *

import os

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader



# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Accel
from geometry_msgs.msg import AccelStamped
from geometry_msgs.msg import TransformStamped

import tf2_ros


#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers


#
from ars_sim_robot.ars_sim_robot_dynamics import *




class ArsSimRobotDynamicsRos(Node):

  #######


  # Robot frame
  robot_frame = None

  # World frame
  world_frame = None

  # Sim step 
  # time step
  sim_step_time = None
  # Timer
  sim_step_timer = None

  # Pub timer
  # time step
  pub_step_time = None
  # Timer
  pub_step_timer = None

  # Robot command
  flag_sub_robot_vel_cmd_unstamped = False
  robot_vel_cmd_unstamped_sub = None
  robot_vel_cmd_stamped_sub = None

  # Robot pose publisher
  robot_pose_pub = None

  # Robot velocity publisher
  robot_vel_world_pub = None
  robot_vel_robot_pub = None

  # Robot acceleration publisher
  robot_acc_world_pub = None
  robot_acc_robot_pub = None

  # Robot cmd control enabled subscriber
  robot_cmd_control_enabled_sub = None
  # Robot motion enabled subscriber
  robot_motion_enabled_sub = None

  # tf2 broadcaster
  tf2_broadcaster = None

  # Param
  robot_sim_description_yaml_file_name = None
  robot_sim_description = None


  # Robot dynamics simulator
  robot_dynamics = ArsSimRobotDynamics()
  


  #########

  def __init__(self, node_name='ars_sim_robot_dynamics_node'):

    # Init ROS
    super().__init__(node_name)

    #
    self.__init(node_name)

    return


  def __init(self, node_name='ars_sim_robot_dynamics_node'):
  
    # Package path
    try:
      pkg_path = get_package_share_directory('ars_sim_robot')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")


    
    #### READING PARAMETERS ###
    
    # Robot description
    default_robot_sim_description_yaml_file_name = os.path.join(pkg_path, 'config', 'config_sim_robot_dji_m100.yaml')
    # Declare the parameter with a default value
    self.declare_parameter('robot_sim_description_yaml_file', default_robot_sim_description_yaml_file_name)
    # Get the parameter value
    robot_sim_description_yaml_file_name_str = self.get_parameter('robot_sim_description_yaml_file').get_parameter_value().string_value
    self.get_logger().info(robot_sim_description_yaml_file_name_str)
    # Store the absolute path
    self.robot_sim_description_yaml_file_name = os.path.abspath(robot_sim_description_yaml_file_name_str)


    #
    self.robot_dynamics.setTimeStamp(self.get_clock().now())


    # Load robot dynamics description
    with open(self.robot_sim_description_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.robot_sim_description = yaml.load(file, Loader=SafeLoader)['sim_robot']

    if(self.robot_sim_description is None):
      self.get_logger().info("Error loading robot dynamics sim description")
    else:
      self.get_logger().info("Robot sim description:")
      self.get_logger().info(str(self.robot_sim_description))

    # Set robot dynamics description param
    self.robot_dynamics.setRobotSimDescription(self.robot_sim_description)

    # Set other robot dynamics description param
    self.robot_frame = self.robot_sim_description['robot_frame']
    self.world_frame = self.robot_sim_description['world_frame']
    self.sim_step_time = self.robot_sim_description['sim_step_time']
    self.pub_step_time = self.robot_sim_description['pub_step_time']



    # Init state
    self.robot_dynamics.robot_posi = [0.0, 0.0, 1.0]
    self.robot_dynamics.robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    self.robot_dynamics.robot_velo_lin_robot = [0.0, 0.0, 0.0]
    self.robot_dynamics.robot_velo_ang_robot = [0.0, 0.0, 0.0]

    
    # End
    return


  def open(self):

    # Subcribers
    # Robot cmd subscriber
    if(self.flag_sub_robot_vel_cmd_unstamped):
      self.robot_vel_cmd_unstamped_sub = self.create_subscription(Twist, 'robot_cmd', self.robotVelCmdUnstampedCallback, qos_profile=10)
    # Robot cmd stamped subscriber
    self.robot_vel_cmd_stamped_sub = self.create_subscription(TwistStamped, 'robot_cmd_stamped', self.robotVelCmdStampedCallback, qos_profile=10)
    # Robot cmd control enabled
    robot_cmd_control_enabled_qos_profile = rclpy.qos.QoSProfile(depth=1)
    robot_cmd_control_enabled_qos_profile.history=rclpy.qos.HistoryPolicy.KEEP_LAST
    robot_cmd_control_enabled_qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    robot_cmd_control_enabled_qos_profile.reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
    self.robot_cmd_control_enabled_sub = self.create_subscription(Bool, 'robot_cmd_control_enabled', self.robotControlEnabledCallback, robot_cmd_control_enabled_qos_profile)
    # Robot motion enabled
    robot_motion_enabled_qos_profile = rclpy.qos.QoSProfile(depth=1)
    robot_motion_enabled_qos_profile.history=rclpy.qos.HistoryPolicy.KEEP_LAST
    robot_motion_enabled_qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    robot_motion_enabled_qos_profile.reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
    self.robot_motion_enabled_sub = self.create_subscription(Bool, 'robot_motion_enabled', self.robotMotionEnabledCallback, robot_motion_enabled_qos_profile)


    # Publishers
    #
    self.robot_pose_pub = self.create_publisher(PoseStamped, 'robot_pose', qos_profile=10)
    #
    self.robot_vel_world_pub = self.create_publisher(TwistStamped, 'robot_velocity_world', qos_profile=10)
    #
    self.robot_vel_robot_pub = self.create_publisher(TwistStamped, 'robot_velocity_robot', qos_profile=10)
    #
    self.robot_acc_world_pub = self.create_publisher(AccelStamped, 'robot_acceleration_world', qos_profile=10)
    #
    self.robot_acc_robot_pub = self.create_publisher(AccelStamped, 'robot_acceleration_robot', qos_profile=10)


    # Tf2 broadcasters
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)


    # Timers
    #
    self.sim_step_timer = self.create_timer(self.sim_step_time, self.simStepTimerCallback)
    #
    self.pub_step_timer = self.create_timer(self.pub_step_time, self.pubStepTimerCallback)


    # End
    return


  def run(self):

    rclpy.spin(self)

    return


  def simStepTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

    # Robot
    self.robot_dynamics.simRobot(time_stamp_current)
    
    # End
    return

  def pubStepTimerCallback(self):

    # Perform the simulation step -> Not needed
    # Get time
    # time_stamp_current = self.get_clock().now()
    # Robot
    # self.robot_dynamics.simRobot(time_stamp_current)

    # Keep the last state already advanced by simStepTimerCallback; and publish it

    # Publish the last state
    # Pose
    self.robotPosePub()
    # Velocity
    self.robotVelPub()
    # Acceleration
    self.robotAccPub()

    # End
    return

    
  def robotVelCmdUnstampedCallback(self, robot_vel_cmd_msg):

    robot_velo_lin_cmd = np.zeros((3,), dtype=float)
    robot_velo_lin_cmd[0] = robot_vel_cmd_msg.linear.x
    robot_velo_lin_cmd[1] = robot_vel_cmd_msg.linear.y
    robot_velo_lin_cmd[2] = robot_vel_cmd_msg.linear.z

    robot_velo_ang_cmd = np.zeros((3,), dtype=float)
    robot_velo_ang_cmd[0] = robot_vel_cmd_msg.angular.x
    robot_velo_ang_cmd[1] = robot_vel_cmd_msg.angular.y
    robot_velo_ang_cmd[2] = robot_vel_cmd_msg.angular.z

    self.robot_dynamics.setRobotVelCmd(robot_velo_lin_cmd, robot_velo_ang_cmd)

    return


  def robotVelCmdStampedCallback(self, robot_vel_cmd_stamped_msg):

    robot_velo_lin_cmd = np.zeros((3,), dtype=float)
    robot_velo_lin_cmd[0] = robot_vel_cmd_stamped_msg.twist.linear.x
    robot_velo_lin_cmd[1] = robot_vel_cmd_stamped_msg.twist.linear.y
    robot_velo_lin_cmd[2] = robot_vel_cmd_stamped_msg.twist.linear.z

    robot_velo_ang_cmd = np.zeros((3,), dtype=float)
    robot_velo_ang_cmd[0] = robot_vel_cmd_stamped_msg.twist.angular.x
    robot_velo_ang_cmd[1] = robot_vel_cmd_stamped_msg.twist.angular.y
    robot_velo_ang_cmd[2] = robot_vel_cmd_stamped_msg.twist.angular.z

    self.robot_dynamics.setRobotVelCmd(robot_velo_lin_cmd, robot_velo_ang_cmd)

    return


  def robotControlEnabledCallback(self, flag_cmd_control_enabled):

    self.robot_dynamics.flag_cmd_control_enabled = flag_cmd_control_enabled.data

    return


  def robotMotionEnabledCallback(self, flag_robot_motion_enabled):

    self.robot_dynamics.flag_robot_motion_enabled = flag_robot_motion_enabled.data

    return


  def robotPosePub(self):

    #
    time_stamp_current = self.robot_dynamics.getTimeStamp()

    #
    robot_posi = self.robot_dynamics.getRobotPosi()
    robot_atti_quat = self.robot_dynamics.getRobotAttiQuat()


    #
    robot_pose_msg = PoseStamped()

    robot_pose_msg.header = Header()
    robot_pose_msg.header.stamp = time_stamp_current.to_msg()
    robot_pose_msg.header.frame_id = self.world_frame

    robot_pose_msg.pose.position.x = robot_posi[0]
    robot_pose_msg.pose.position.y = robot_posi[1]
    robot_pose_msg.pose.position.z = robot_posi[2]

    robot_pose_msg.pose.orientation.w = robot_atti_quat[0]
    robot_pose_msg.pose.orientation.x = robot_atti_quat[1]
    robot_pose_msg.pose.orientation.y = robot_atti_quat[2]
    robot_pose_msg.pose.orientation.z = robot_atti_quat[3]

    # Tf2
    tf2_msg = TransformStamped()

    tf2_msg.header.stamp = time_stamp_current.to_msg()
    tf2_msg.header.frame_id = self.world_frame
    tf2_msg.child_frame_id = self.robot_frame

    tf2_msg.transform.translation.x = robot_posi[0]
    tf2_msg.transform.translation.y = robot_posi[1]
    tf2_msg.transform.translation.z = robot_posi[2]

    tf2_msg.transform.rotation.w = robot_atti_quat[0]
    tf2_msg.transform.rotation.x = robot_atti_quat[1]
    tf2_msg.transform.rotation.y = robot_atti_quat[2]
    tf2_msg.transform.rotation.z = robot_atti_quat[3]


    self.robot_pose_pub.publish(robot_pose_msg)

    self.tf2_broadcaster.sendTransform(tf2_msg)


    # End
    return


  def robotVelPub(self):

    #
    time_stamp_current = self.robot_dynamics.getTimeStamp()

    robot_velo_lin_robot = self.robot_dynamics.getRobotVeloLinRobot()
    robot_velo_ang_robot = self.robot_dynamics.getRobotVeloAngRobot()

    robot_velo_lin_world = self.robot_dynamics.getRobotVeloLinWorld()
    robot_velo_ang_world = self.robot_dynamics.getRobotVeloAngWorld()


    #
    robot_velo_robot_msg = TwistStamped()

    robot_velo_robot_msg.header.stamp = time_stamp_current.to_msg()
    robot_velo_robot_msg.header.frame_id = self.robot_frame

    robot_velo_robot_msg.twist.linear.x = robot_velo_lin_robot[0]
    robot_velo_robot_msg.twist.linear.y = robot_velo_lin_robot[1]
    robot_velo_robot_msg.twist.linear.z = robot_velo_lin_robot[2]

    robot_velo_robot_msg.twist.angular.x = robot_velo_ang_robot[0]
    robot_velo_robot_msg.twist.angular.y = robot_velo_ang_robot[1]
    robot_velo_robot_msg.twist.angular.z = robot_velo_ang_robot[2]

    
    #
    robot_velo_world_msg = TwistStamped()

    robot_velo_world_msg.header.stamp = time_stamp_current.to_msg()
    robot_velo_world_msg.header.frame_id = self.world_frame

    robot_velo_world_msg.twist.linear.x = robot_velo_lin_world[0]
    robot_velo_world_msg.twist.linear.y = robot_velo_lin_world[1]
    robot_velo_world_msg.twist.linear.z = robot_velo_lin_world[2]

    robot_velo_world_msg.twist.angular.x = robot_velo_ang_world[0]
    robot_velo_world_msg.twist.angular.y = robot_velo_ang_world[1]
    robot_velo_world_msg.twist.angular.z = robot_velo_ang_world[2]



    self.robot_vel_robot_pub.publish(robot_velo_robot_msg)

    self.robot_vel_world_pub.publish(robot_velo_world_msg)

    #
    return


  def robotAccPub(self):

    #
    time_stamp_current = self.robot_dynamics.getTimeStamp()

    robot_acce_lin_robot = self.robot_dynamics.getRobotAcceLinRobot()
    robot_acce_ang_robot = self.robot_dynamics.getRobotAcceAngRobot()

    robot_acce_lin_world = self.robot_dynamics.getRobotAcceLinWorld()
    robot_acce_ang_world = self.robot_dynamics.getRobotAcceAngWorld()


    #
    robot_acce_robot_msg = AccelStamped()

    robot_acce_robot_msg.header.stamp = time_stamp_current.to_msg()
    robot_acce_robot_msg.header.frame_id = self.robot_frame

    robot_acce_robot_msg.accel.linear.x = robot_acce_lin_robot[0]
    robot_acce_robot_msg.accel.linear.y = robot_acce_lin_robot[1]
    robot_acce_robot_msg.accel.linear.z = robot_acce_lin_robot[2]

    robot_acce_robot_msg.accel.angular.x = robot_acce_ang_robot[0]
    robot_acce_robot_msg.accel.angular.y = robot_acce_ang_robot[1]
    robot_acce_robot_msg.accel.angular.z = robot_acce_ang_robot[2]

    
    #
    robot_acce_world_msg = AccelStamped()

    robot_acce_world_msg.header.stamp = time_stamp_current.to_msg()
    robot_acce_world_msg.header.frame_id = self.world_frame

    robot_acce_world_msg.accel.linear.x = robot_acce_lin_world[0]
    robot_acce_world_msg.accel.linear.y = robot_acce_lin_world[1]
    robot_acce_world_msg.accel.linear.z = robot_acce_lin_world[2]

    robot_acce_world_msg.accel.angular.x = robot_acce_ang_world[0]
    robot_acce_world_msg.accel.angular.y = robot_acce_ang_world[1]
    robot_acce_world_msg.accel.angular.z = robot_acce_ang_world[2]



    self.robot_acc_robot_pub.publish(robot_acce_robot_msg)

    self.robot_acc_world_pub.publish(robot_acce_world_msg)

    #
    return
