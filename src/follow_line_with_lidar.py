#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_husky import MoveHusky
from laser_test import LidarData
from sensor_msgs.msg import LaserScan


class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.move_obj = MoveHusky()
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.response_flag = 0

        self.image_error = 0.0
        self.scan_distance = 0.0

        #self.lidar_obj = LidarData()

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        '''
        RGB = [124, 118, 0]     BGR = [0,118,124]       HSV= [[[ 35 255 255]]]
        '''

        height, width, channels = cv_image.shape
        descentre = 200
        rows_to_watch = 20
        crop_img = cv_image[(height)/2 + descentre :(height)/2+(descentre+rows_to_watch)][1:width]

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([10,100,100])
        upper_yellow = np.array([50,255,255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            #self.lidar_obj.readLaser()

        except ZeroDivisionError: #here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            self.move_obj.clean_class()
            cy, cx = height/2, width/2
            print("zero exception")
            self.response_flag = 1
            self.state_controller()

        
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)

        cv2.waitKey(1)

        #Proportional control
        error = cx - width/2
        '''
        twist_obj = Twist();
        twist_obj.linear.x = 0.4;
        twist_obj.angular.z = -error/100
        rospy.loginfo("Angular value = "+str(twist_obj.angular.z))
        self.move_obj.move_robot(twist_obj)
        '''
        self.image_error = -error/200
        self.response_flag = 3
        self.state_controller()

        def cleanup(self):
            self.move_obj.clean_class()
            cv2.destroyAllWindows()

    def scan_callback(self,scan_data):
        for i in range(3):   
            if len(scan_data.ranges) < 300:
                print("less than 300 readings")
                self.response_flag = 3
            else:
                print "Middle = "+ str(scan_data.ranges[360])
                print " tamanho = "+str(len(scan_data.ranges))
                self.response_flag = 3
                if scan_data.ranges[360] < 0.6:
                    self.response_flag = 2
        self.state_controller()


    def state_controller(self):
        if self.response_flag < 3:
            self.move_obj.clean_class()
        else:
            twist_obj = Twist()
            twist_obj.linear.x = 0.4
            twist_obj.angular.z = self.image_error
            rospy.loginfo("Angular Value = "+str(twist_obj.angular.z))
            self.move_obj.move_robot(twist_obj)




def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        line_follower_object.cleanup()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    main()
