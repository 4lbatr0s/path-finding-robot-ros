#! /usr/bin/env python3
import math
from std_srvs.srv import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf import transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import rospy

# import ros stuff
# import ros message
# import ros service


srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180)  # 5 degrees
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param('initial_x')
initial_position_.y = rospy.get_param('initial_y')
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['hedefe git', 'duvari takip et']
state_ = 0
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0
# 0 - hedefe git
# 1 - duvari takip et


def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def clbk_laser(msg):
    global regions_
    dist_max = 10
    regions_ = {
        'sag':  min(min(msg.ranges[0:144]), dist_max),
        'sag_on': min(min(msg.ranges[144:288]), dist_max),
        'on':  min(min(msg.ranges[288:432]), dist_max),
        'sol_on':  min(min(msg.ranges[432:576]), dist_max),
        'sol':   min(min(msg.ranges[576:720]), dist_max),
    }


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "Durum degistirildi:{}".format(state_desc_[state])
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)


def distance_to_line(p0):
    # p0 bizim robotumuzun halihazirdaki konum bilgisidir.
    # p1 ve p2 degerlerimiz hayali cizgimizi tanımlamaktadırlar.
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_
    # here goes the equation
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x)
                      * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_

    rospy.init_node('bug0')

    sub_laser = rospy.Subscriber(
        '/m2wr/laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')

    srv_client_go_to_point_ = rospy.ServiceProxy(
        '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    srv_client_set_model_state = rospy.ServiceProxy(
        '/gazebo/set_model_state', SetModelState)

    # set robot position
    model_state = ModelState()
    model_state.model_name = 'm2wr'
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)

    # initialize going to the point
    change_state(0)

    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        # hayali cizgimize olan uzakligin hesaplandıgı nokta.
        distance_position_to_line = distance_to_line(position_)

        if state_ == 0:
            if regions_['on'] > 0.15 and regions_['on'] < 1:
                change_state(1)

        elif state_ == 1:
            if count_state_time_ > 5 and \
               distance_position_to_line < 0.1:  # eger cizgimize olan uzaklık 0.1'den ufaksa, go_to_point.py scriptini calistir.
                change_state(0)

        # zamani olcuyoruz, cünkü her hayali cizgiye geldiğimizde mutlaka düz hareket edip bir engelle karsılacagız
        count_loop_ = count_loop_ + 1
        # bu nedenle engelle karsılastıgımızda biraz bekleyip ondan sonra takip islemi yapmak istedigimizden zaman olcumu yapıyoruz.
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rospy.loginfo("Cizgiye uzaklik:{}, konum: {}{}".format(distance_to_line(
            position_), position_.x, position_.y))
        rate.sleep()


if __name__ == "__main__":
    main()
