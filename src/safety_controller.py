#! /usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

def main():
    rospy.init_node('safety_controller')
    rate = rospy.Rate(3)


    def scanCallback(scan_data):
        if len(scan_data.ranges) < 270:
            return

        else:
            if scan_data.ranges[360] < 3.0:
                safety_publisher.publish(True)
            else:
                safety_publisher.publish(False)


    safety_publisher = rospy.Publisher('safety_controller', Bool, queue_size=5)
    sub = rospy.Subscriber('/scan', LaserScan, scanCallback )

    rospy.spin()

if __name__ == '__main__':
    main()
