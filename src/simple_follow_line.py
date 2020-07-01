#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from visual_path_husky.msg import SysStatus
from std_msgs.msg import Bool
from move_husky import MoveHusky


class LineFollower(object):

    def __init__(self):

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.safety_subscriber = rospy.Subscriber("/safety_controller", SysStatus, self.safety_callback)
        self.line_feedback_publisher = rospy.Publisher('/line_control', Bool)
        self.move_obj = MoveHusky()
        self.safety_flag = SysStatus()
        self.line_feedback = Bool()

        self.no_blob = False
        self.comeback_flag = False
        self.counter = 0
        self.counter_2 = 0

    def safety_callback(self,s_data):
        self.safety_flag = s_data
        #print("safety flag is: ") + str(self.safety_flag)

        if self.safety_flag.flag:
            self.counter = 0
            self.cleanup()


    def camera_callback(self,data):

        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        '''
        It might be necessary adjust the collors to your enviroment.
        RGB = [124, 118, 0]     BGR = [0,118,124]       HSV= [[[ 35 255 255]]]
        ---- NEW ---
        MIN_RGB = [31 30 9]     MAX_RGB = [57,54,0]
        '''

        height, width, channels = cv_image.shape
        descentre = 10
        rows_to_watch = 300
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([10,100,30])
        upper_yellow = np.array([40,255,70])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)

        come_back_holder = self.no_blob
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            self.no_blob = False
        except ZeroDivisionError:
            cy, cx = height/2, width/2
            self.no_blob = True
            rospy.loginfo("No line detected")


        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)
        '''
        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        '''
        cv2.waitKey(1)

        error = cx - width/2
        #Proportional control
        v = 0.3
        w = -error/100
        w_max = 1.5

        if come_back_holder == True and self.no_blob == False:
            self.comeback_flag = True


        if m['m10'] > 1700000000 and self.comeback_flag:
            w = -1
            self.counter_2 += 1
            print(m['m10'])

        if self.counter_2 > 10:
            self.comeback_flag = False
            self.counter_2 = 0


        if w > w_max:
            w = w_max
        if w < -w_max:
            w = -w_max

        twist_obj = Twist();
        twist_obj.linear.x = v
        twist_obj.angular.z = w

        self.line_feedback = False

        if not self.safety_flag.flag and not self.no_blob :
            self.move_obj.line_move_robot(twist_obj)
            self.line_feedback = True
            self.counter += 1
            if self.counter > 10:
                self.line_feedback = False


        self.line_feedback_publisher.publish(self.line_feedback)



    def cleanup(self):
        #self.move_obj.line_clean_class()
        #cv2.destroyAllWindows()

        if self.safety_flag.flag == True:
            rospy.loginfo("Waiting...")




def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    rate = rospy.Rate(10)
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
