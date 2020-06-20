#! /usr/bin/env python
import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from visual_path_husky.msg import SysStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32

def main():
    rospy.init_node('safety_controller')
    rate = rospy.Rate(3)

    scan_message = LaserScan()
    status = SysStatus()


    #calculates the estimated size of the obstacle
    def obstacleSize(scan_message, first_index):
        i = first_index

        while scan_message.ranges[i] < 5 and i < 720:
            i += 1

        last_index = i - 1
        angle = (last_index - first_index) * scan_message.angle_increment
        #cos law
        virtual_size = math.sqrt (pow(scan_message.ranges[first_index],2) + pow(scan_message.ranges[last_index],2) - 2 * scan_message.ranges[first_index] * scan_message.ranges[last_index] * math.cos(angle))
        status.obstacle_size = virtual_size
        safety_publisher.publish(status)
        return virtual_size
        '''
        print("first index: ") + str(first_index) + ("last index: ") + str(last_index)
        print("first measure: ") + str(scan_message.ranges[first_index]) + ("last measure: ") + str(scan_message.ranges[last_index])
        print("angle: ") + str(angle) + ("virtual size ") + str(virtual_size)
        print(" ")
        '''

    #get position of the robot when an obstacle is found
    def getPosition(flag_keeper):

        if flag_keeper != status.flag :
            position = rospy.wait_for_message('/odometry/filtered', Odometry)

            quaternion = (position.pose.pose.orientation.x, position.pose.pose.orientation.y, position.pose.pose.orientation.z, position.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)

            status.initial_position[0] = position.pose.pose.position.x
            status.initial_position[1] = position.pose.pose.position.y
            status.initial_position[2] = euler[2]

            flag_keeper = status.flag



    def scanCallback(scan_data):
        if len(scan_data.ranges) < 270: # those reading are somehow wrong
            return

        else:
            global flag_keeper
            flag_keeper = status.flag
            angle_min = -math.pi/2
            angle_max = math.pi/2

            global readings_filter
            readings_filter = Int32()

            #calculates the index of data of interest
            index = int (angle_max / scan_data.angle_increment)
            size = len(scan_data.ranges)
            status.flag = False

            # search obstacles between angles of interest

            for i in range(index*2):
                current_index = int(size/2 - index + i)
                if scan_data.ranges[current_index] < 1.5 and scan_data.ranges[current_index] > 0.1:
                    size = obstacleSize(scan_data, current_index)
                    if(size > 0.1):
                        status.flag = True
                        getPosition(flag_keeper)
                        obstacleSize(scan_data, current_index)
                        break

            status.distance_to_obstacle = scan_data.ranges[360] # This reading must be improved. Do not work properly yet
            safety_publisher.publish(status)



    safety_publisher = rospy.Publisher('safety_controller', SysStatus, queue_size=5)
    sub = rospy.Subscriber('/scan', LaserScan, scanCallback )

    rospy.spin()

if __name__ == '__main__':
    main()
