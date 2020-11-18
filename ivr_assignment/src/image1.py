#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import image2


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)

    # initialises a subscriber to recieve messages from camera 2
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback1)

    #set up publisher to send joint angles to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command",Float64,queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command",Float64,queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command",Float64,queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command",Float64,queue_size=10)

    #sets up publisher of estimated joints via computer vision from camera 1
    self.est1_joint2_pub = rospy.Publisher("/robot/joint2_estimated",Float64,queue_size=10)
    self.est1_joint3_pub = rospy.Publisher("/robot/joint3_estimated",Float64,queue_size=10)
    self.est1_joint4_pub = rospy.Publisher("/robot/joint4_estimated",Float64,queue_size=10)

    #sets up subscribers to get list of estimated joints from camera 2
    self.est2_joint3_sub = rospy.Subscriber("/robot/joint3_c2_estimated",Float64,self.get_camera2_joint3)

    #sets up variables for estimated angles from camera 2
    self.est2_joint2 = 0.0
    self.est2_joint3 = 0.0
    self.est2_joint4 = 0.0



    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    #start time for robot
    self.time_trajectory = rospy.get_time()


  def defineSinusoidalTrajectory(self):
    #get current time
    cur_time = np.array([rospy.get_time() - self.time_trajectory])
    print("Time:",cur_time)
    joint2_trajectory = float(np.pi/2*np.sin(np.pi/15*cur_time))
    joint3_trajectory = float(np.pi/2*np.sin(np.pi/18*cur_time))
    joint4_trajectory = float(np.pi/2*np.sin(np.pi/20*cur_time))
    return np.array([joint2_trajectory,joint3_trajectory,joint4_trajectory])

  def detect_red(self,image):
    mask = cv2.inRange(image,(0,0,100),(0,0,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return np.array([cx,cy])
    #blob is blocked or out of camera view = zerodivision
    except ZeroDivisionError:
        #try camera 2 view v
        print("Switch camera")

  def detect_green(self,image):
    mask = cv2.inRange(image,(0,100,0),(0,255,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    cv2.imwrite('green_copy.png', mask)
    M = cv2.moments(mask)
    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return np.array([cx,cy])
    #blob is blocked or out of camera view = zerodivision
    except ZeroDivisionError:
        #try camera 2 view
        print("Switch camera")


  def detect_blue(self,image):
    mask = cv2.inRange(image,(100,0,0),(255,0,0))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)
    cv2.imwrite('blue_copy.png', mask)
    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])


  def detect_yellow(self,image):
    mask = cv2.inRange(image,(0,100,100),(0,255,255))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask,kernel,iterations=3)

    M = cv2.moments(mask)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return np.array([cx,cy])




  def pixelToMeter(self,image):
    blue_blob = self.detect_blue(image)
    green_blob = self.detect_green(image)

    dist = np.sum((blue_blob - green_blob)**2)
    return 3.5/np.sqrt(dist)


  def detect_end_effector(self,image):
    #finds end position respect to blue blob
    red_blob = self.detect_red(image)
    blue_blob = self.detect_blue(image)

    p = self.pixelToMeter(image)

    return p*(blue_blob - red_blob)


  def detect_joint2(self,image):

    p = self.pixelToMeter(image)
    green_blob = self.detect_green(image)
    blue_blob = self.detect_blue(image)

    joint2_pos = p*(blue_blob - green_blob)

    j2_angle = np.arctan2(joint2_pos[0],joint2_pos[1])
    return j2_angle

  def detect_joint3(self,image,j2_angle):

    p = self.pixelToMeter(image)
    green_blob = self.detect_green(image)
    blue_blob = self.detect_blue(image)

    joint3_pos = p*(blue_blob - green_blob)

    j3_angle = np.arctan2(joint3_pos[0],joint3_pos[1])-j2_angle
    return j3_angle

  def detect_joint4(self,image,j2_angle,j3_angle):

    p = self.pixelToMeter(image)
    blue_blob = self.detect_blue(image)
    red_blob = self.detect_red(image)

    joint4_pos = p*(blue_blob-red_blob)

    j4_angle = np.arctan2(joint4_pos[0],joint4_pos[1]) - j2_angle - j3_angle
    return j4_angle


  def get_camera2_joint3(self,data):
    self.est2_joint3 = data.data




































  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)

    #Get joint angles
    j_angle = self.defineSinusoidalTrajectory()



    # Publish the results
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))


      self.robot_joint2_pub.publish(j_angle[0])
      self.robot_joint3_pub.publish(j_angle[1])
      self.robot_joint4_pub.publish(j_angle[2])


      est_j2 = self.detect_joint2(self.cv_image1)
      est_j3 = self.est2_joint3
      est_j4 = self.detect_joint4(self.cv_image1,est_j2,est_j3)

      self.est1_joint2_pub.publish(est_j2)
      print("J2 Actual Angle:",j_angle[0]," Estimated Angle:",est_j2)
      print("J3 Actual Angle:",j_angle[1]," Estimated Angle:",est_j3)
      print("J4 Actual Angle:",j_angle[2]," Estimated Angle:",est_j4)








    except CvBridgeError as e:
      print(e)



# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
