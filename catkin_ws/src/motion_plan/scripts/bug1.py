#! /usr/bin/env python3

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *

import math

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
state_desc_ = ['hedefe git',
               'engelin etrafindan dolas', 'en yakin noktaya git']
state_ = 0
# robotun baslangic noktasi burda saklanacak
circumnavigate_starting_point_ = Point()
# en yakin nokta degeri burda saklanacak.
circumnavigate_closest_point_ = Point()
count_state_time_ = 0  # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point

# callbacks


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
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        # bug0 dan farklı olarak en yakın noktaya gitmek icin yine wall following scriptini kullandık
        resp = srv_client_go_to_point_(False)
        # cünkü en yakin noktayla robot arasında bir cisim kalmayana dek duvarı takip ederek hareket edeceğiz
        resp = srv_client_wall_follower_(True)


# robot ile ulasmak istedigimiz nokta arasındaki uzaklıgı hesaplayan fonksiyon.
def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global circumnavigate_closest_point_, circumnavigate_starting_point_
    global count_loop_, count_state_time_

    rospy.init_node('bug1')

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

    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        if state_ == 0:
            if regions_['on'] > 0.15 and regions_['on'] < 1:
                circumnavigate_closest_point_ = position_
                circumnavigate_starting_point_ = position_
                change_state(1)

        elif state_ == 1:
            # eger halihazırda bulundugumuz nokta, onceki en yakin pozisyon degerinden kucukse, yeni en yakin noktamizi bulundugumuz nokta olarak belirle.
            if calc_dist_points(position_, desired_position_) < calc_dist_points(circumnavigate_closest_point_, desired_position_):
                circumnavigate_closest_point_ = position_

            # robotu calistirdigimizda oncelikle engellerin cevresini dolasip baslangic noktasina donecek
            # daha sonrasinda en yakin noktaya gitmeye calisacak
            # fakat robotun baslangic noktasina ne kadar yakin oldugunu olcmek icin 5 saniye kadar bekleyecegiz
            # robot baslangic noktasindan "engelin etrafindan dolas" durumuna gecerek tekrar baslangic noktasina
            # donecegi zaman zarfini 5 saniye gectikten sonra anlik olarak hesaplamaya basliyoruz.
            if count_state_time_ > 5 and \
               calc_dist_points(position_, circumnavigate_starting_point_) < 0.2:
                change_state(2)

        elif state_ == 2:
            # if robot reaches (is close to) closest point
            if calc_dist_points(position_, circumnavigate_closest_point_) < 0.2:
                change_state(0)

        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rate.sleep()


if __name__ == "__main__":
    main()
