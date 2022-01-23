#! /usr/bin/env python3
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None


def clbk_laser(msg):
    max_distance = 10
    regions = {
        'sag': min(max_distance, min(msg.ranges[0:144])),
        'sag_on': min(max_distance, min(msg.ranges[144:288])),
        'on': min(max_distance, min(msg.ranges[288:432])),
        'sol_on': min(max_distance, min(msg.ranges[432:576])),
        'sol': min(max_distance, min(msg.ranges[576:720]))
    }

    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['on'] > 1 and regions['sol_on'] > 1 and regions['sag_on'] > 1:
        state_description = 'engel_yok'
        linear_x = 0.6
        angular_z = 0
    elif regions['on'] < 1 and regions['sol_on'] > 1 and regions['sag_on'] > 1:
        state_description = 'engel_onde'
        linear_x = 0
        angular_z = -0.3
    elif regions['on'] > 1 and regions['sol_on'] > 1 and regions['sag_on'] < 1:
        state_description = 'engel_sag_onde'
        linear_x = 0
        angular_z = -0.3
    elif regions['on'] > 1 and regions['sol_on'] < 1 and regions['sag_on'] > 1:
        state_description = 'engel_sol_onde'
        linear_x = 0
        angular_z = 0.3
    elif regions['on'] < 1 and regions['sol_on'] > 1 and regions['sag_on'] < 1:
        state_description = 'engel_onde_ve_sag_onde'
        linear_x = 0
        angular_z = -0.3
    elif regions['on'] < 1 and regions['sol_on'] < 1 and regions['sag_on'] > 1:
        state_description = 'engel_onde_ve_sol_onde'
        linear_x = 0
        angular_z = 0.3
    elif regions['on'] < 1 and regions['sol_on'] < 1 and regions['sag_on'] < 1:
        state_description = 'engel_onde_sol_onde_ve_sag_onde'
        linear_x = 0
        angular_z = -0.3
    elif regions['on'] > 1 and regions['sol_on'] < 1 and regions['sag_on'] < 1:
        state_description = 'engel_sag_onde_ve_sol_onde'
        linear_x = 0
        angular_z = -0.3
    else:
        state_description = 'bilinmeyen durum'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():
    global pub

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)

    rospy.spin()


if __name__ == '__main__':
    main()
