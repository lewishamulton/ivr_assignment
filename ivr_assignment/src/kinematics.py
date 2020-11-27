#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64


class kinematics:

  # Defines publisher and subscriber
  def __init__(self):
    self.cv_x_estimates = []
    self.cv_y_estimates = []
    self.cv_z_estimates = []

    self.fk_estimates = []
    self.cv_estimates = []

    # initialize the node named image_processing
    rospy.init_node('kinematics', anonymous=True)
    # initialize a publisher to send predicted x y and z coordinates of end effector
    self.kin_x = rospy.Publisher("kinematics_x",Float64, queue_size = 10)
    self.kin_y = rospy.Publisher("kinematics_y",Float64, queue_size = 10)
    self.kin_z = rospy.Publisher("kinematics_z",Float64, queue_size = 10)
    
    #subscribers for visual estimates
    self.cv_estimate_x = rospy.Subscriber("cv_estimate_x",Float64,self.cvx_callback)
    self.cv_estimate_y = rospy.Subscriber("cv_estimate_y",Float64,self.cvy_callback)
    self.cv_estimate_z = rospy.Subscriber("cv_estimate_z",Float64,self.cvz_callback)

    #set up publisher to send joint angles to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command",Float64,queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command",Float64,queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command",Float64,queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command",Float64,queue_size=10)


    #start time for robot
    self.time_trajectory = rospy.get_time()

    
    self.joint_test()
    self.cv_estimates = [[self.cv_x_estimates[i],self.cv_y_estimates[i],self.cv_z_estimates[i]] for i in range(len(self.cv_x_estimates))]
    self.showResults()


    


  def calculateForwardKinematics(self,theta1,theta2,theta3,theta4):
    m1 = self.calculateTMatFromDHParams(-np.pi/2,0,2.5,theta1)
    m2 = self.calculateTMatFromDHParams(np.pi/2,0,0,theta2)
    m3 = self.calculateTMatFromDHParams(-np.pi/2,3.5,0,theta3)
    m4 = self.calculateTMatFromDHParams(np.pi/2,0,3,theta4)
    final = np.matmul(m1,np.matmul(m2,m3,m4))
    return final[:,3][:3]
  
  def calculateTMatFromDHParams(self, alpha, a, d, theta):
    t = [np.cos(theta),-np.sin(theta)*np.cos(alpha),np.sin(theta)*np.sin(alpha), a*np.cos(theta),
         np.sin(theta),np.cos(theta)*np.cos(alpha),-np.cos(theta)*np.sin(alpha), a*np.sin(theta),
         0, np.sin(alpha), np.cos(alpha), d,
         0,0,0,1]
    t = np.array(t).reshape((4,4))
    return t


    


  # Recieve data from camera 1, process it, and publish
  def joint_test(self):
    joint_positions = [[-1.5707963267948966, 0.5235987755982988, -0.7853981633974483, 0.5235987755982988],
                       [1.5707963267948966, 0.7853981633974483, -0.5235987755982988, -0.7853981633974483],
                       [0.5235987755982988, 1.5707963267948966, 0.5235987755982988, -0.7853981633974483],
                       [-0.7853981633974483, -0.7853981633974483, -1.5707963267948966, 1.5707963267948966],
                       [1.5707963267948966, 1.5707963267948966, 1.5707963267948966, -0.7853981633974483],
                       [1.5707963267948966, -1.5707963267948966, -0.5235987755982988, -0.5235987755982988],
                       [1.5707963267948966, -0.7853981633974483, -0.7853981633974483, -0.5235987755982988],
                       [-0.5235987755982988, 0.5235987755982988, 0.7853981633974483, -0.7853981633974483],
                       [0.5235987755982988, 0.5235987755982988, -0.7853981633974483, 0.5235987755982988],
                       [0.5235987755982988, 0.7853981633974483, -0.7853981633974483, 0.5235987755982988]]

    for joint in joint_positions:
      self.robot_joint1_pub.publish(joint[0])
      self.robot_joint2_pub.publish(joint[1])
      self.robot_joint3_pub.publish(joint[2])
      self.robot_joint4_pub.publish(joint[3])

      FK_estimate = self.calculateForwardKinematics(joint[0],joint[1],joint[2],joint[3])
      self.fk_estimates.append(FK_estimate)
      time.sleep(4)

  def showResults(self):
    for i in range(len(self.fk_estimates)):
      print("FK estimate: ",self.fk_estimates[i])
      print("CV estimate: ",self.cv_esimates[i])
      
  def cvx_callback(self,data):
    self.cv_x_estimates.append(data)
    
  def cvy_callback(self,data):
    self.cv_y_estimates.append(data)
    
  def cvz_callback(self,data):
    self.cv_z_estimates.append(data)
    
    
    

# call the class
def main(args):
  k = kinematics()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
