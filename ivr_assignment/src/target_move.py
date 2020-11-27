#!/usr/bin/env python3


import rospy
import sys
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math



class chamfer_match:

    def __init__(self):

        rospy.init_node('image_processing', anonymous=True)

        # global vars of x,y and Zs of sphere from camera 1 and camera 2
        self.sphere_c1_y = 0
        self.sphere_c1_z = 0
        self.sphere_c2_x = 0
        self.sphere_c2_z = 0

        # global vars of x,y and Zs of yellow blob from camera 1 and camera 2
        self.yellow_c1_y = 0
        self.yellow_c1_z = 0
        self.yellow_c2_x = 0
        self.yellow_c2_z = 0

        self.bridge = CvBridge()
        #get image data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.get_image_data)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.get_image_data2)

        #publish estimated x,y,z coords in meters
        self.est_x = rospy.Publisher("sphere/est_x",Float64,queue_size=10)
        self.est_y = rospy.Publisher("sphere/est_y",Float64,queue_size=10)
        self.est_z = rospy.Publisher("sphere/est_z",Float64,queue_size=10)


    def get_image_data(self,data):
        # Recieve the image
        try:
          self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")

          cnts,mask = self.detect_orange(self.cv_image1)
          #print(cnts)

          centres = self.get_cnts_centres(cnts)

          #In order to get image samples uncomment and run main once
          #then kill the process with os.kill() in the program or outwith
          #get_cropped_image_samples(centres,mask)

          #import in our template image of sphere
          template = cv2.imread("image_sample1.jpg")
          template = cv2.inRange(template,(200,200,200),(255,255,255))
          sphere_centre = self.get_sphere_target(template,mask,centres)
          self.sphere_c1_y = sphere_centre[0]
          self.sphere_c1_z = sphere_centre[1]



          #gets yellow blob coords
          yellow_blob = self.detect_yellow(self.cv_image1)
          self.yellow_c1_y = yellow_blob[0]
          self.yellow_c1_z = yellow_blob[1]


          #does the same for cv_image2
          cnts,mask = self.detect_orange(self.cv_image2)
          centres = self.get_cnts_centres(cnts)
          sphere_centre = self.get_sphere_target(template,mask,centres)
          self.sphere_c2_x = sphere_centre[0]
          self.sphere_c2_z = sphere_centre[1]

          p = self.pixelToMeter(self.cv_image1)
          xyz = self.get_sphere_3d_coords()

          #puts sphere coords into meters
          xyz_m = xyz*p

          self.est_x.publish(xyz_m[0])
          self.est_y.publish(xyz_m[1])
          self.est_z.publish(xyz_m[2])
          print("Est x:",xyz_m[0])
          print("Est y:",xyz_m[1])
          print("Est z:",xyz_m[2])





        except CvBridgeError as e:
          print(e)

    def get_image_data2(self,data):
        try:
            self.cv_image2 =  self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def get_sphere_3d_coords(self):
        #gets the avg of 2 views coords to get z coord in 3d
        z_coord = math.floor((self.sphere_c1_z+self.sphere_c2_z)/2)

        #returns 3d coords
        return np.array([self.sphere_c2_x,self.sphere_c1_y,z_coord])

    #puts our pixel coords into meters
    def pixelToMeter(self,image):
      blue_blob = self.detect_blue(image)
      yellow_blob = self.detect_yellow(image)

      dist = np.sum((yellow_blob - blue_blob)**2)
      return 2.5/np.sqrt(dist)

    # returns contours of orange masked parts of image
    # and orange masked image
    def detect_orange(self,image):
        mask = cv2.inRange(image,(5,50,50),(15,255,255))
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        ret,thresh = cv2.threshold(mask,127,255,0)
        contours,_ = cv2.findContours(mask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_TC89_L1)

        return contours,mask

    #uses detect_yellow/detect_blue to get ratio/distance from base frame
    def detect_yellow(self,image):
        mask = cv2.inRange(image,(0,100,100),(0,255,255))
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)

        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return np.array([cx,cy])

    def detect_blue(self,image):
        mask = cv2.inRange(image,(100,0,0),(255,0,0))
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return np.array([cx,cy])

    # gets centres of contours of orange masked image
    def get_cnts_centres(self,cnts):
        num = 0
        centres = []
        for i in range(len(cnts)):
            M = cv2.moments(cnts[i])
            centres.append((int(M['m10']/M['m00']),int(M['m01']/M['m00'])))
        return centres

    def get_cropped_image_samples(self,centres,mask):
        for i in range(len(centres)):
            centre = centres[i]
            y = int(centre[1])
            x= int(centre[0])
            cropped_img = mask[y-20:y+20,x-20:x+20]
            cv2.imwrite("image_sample"+str(i)+".jpg",cropped_img)

    # returns coordinate centre of sphere
    def get_sphere_target(self,template,current_mask,centres):
        # list of summed values of templates and dist_trans ROI
        sum_list = np.array([])
        # loops through each orange object
        for i in range(len(centres)):
            centre = centres[i]
            y = int(centre[1])
            x = int(centre[0])
            # gets region of interest to compare to sample_img
            ROI = current_mask[y-20:y+20,x-20:x+20]
            # checks its same size as template
            ROI = ROI[0:template.shape[0],0:template.shape[1]]

            # apply distance transform and times it with templates
            dist_trans = cv2.distanceTransform(cv2.bitwise_not(ROI),cv2.DIST_L2,0)
            sum_list = np.append(sum_list,np.sum(dist_trans*template))

        return centres[np.argmin(sum_list)]






# Publish data
def move():
  #rospy.init_node('target_pos_cmd', anonymous=True)
  rate = rospy.Rate(30) # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint1_pub = rospy.Publisher("/target/x_position_controller/command", Float64, queue_size=10)
  robot_joint2_pub = rospy.Publisher("/target/y_position_controller/command", Float64, queue_size=10)
  robot_joint3_pub = rospy.Publisher("/target/z_position_controller/command", Float64, queue_size=10)
  robot_joint4_pub = rospy.Publisher("/target2/x2_position_controller/command", Float64, queue_size=10)
  robot_joint5_pub = rospy.Publisher("/target2/y2_position_controller/command", Float64, queue_size=10)
  robot_joint6_pub = rospy.Publisher("/target2/z2_position_controller/command", Float64, queue_size=10)
  t0 = rospy.get_time()

  while not rospy.is_shutdown():

    cur_time = np.array([rospy.get_time()])-t0
    #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))


    x_d = 2.5* np.cos(cur_time * np.pi/15)
    y_d = 2.5* np.sin(cur_time * np.pi/15)
    z_d = 1* np.sin(cur_time * np.pi/15)
    joint1=Float64()
    joint1.data= 0.5 + x_d
    joint2=Float64()
    joint2.data= 0 + y_d
    joint3=Float64()
    joint3.data= 7 + z_d
    robot_joint1_pub.publish(joint1)
    robot_joint2_pub.publish(joint2)
    robot_joint3_pub.publish(joint3)
    x_d = 2+ 2* np.cos(cur_time * np.pi/15)
    y_d = 2.5+ 1.5* np.sin(cur_time * np.pi/15)
    joint4=Float64()
    joint4.data=  x_d
    joint5=Float64()
    joint5.data=  y_d
    joint6=Float64()
    joint6.data= 7.5
    robot_joint4_pub.publish(joint4)
    robot_joint5_pub.publish(joint5)
    robot_joint6_pub.publish(joint6)
    rate.sleep()


def main(args):
    cf = chamfer_match()
    move()




# run the code if the node is called
if __name__ == '__main__':
  try:
    main(sys.argv)

  except rospy.ROSInterruptException:
    pass
