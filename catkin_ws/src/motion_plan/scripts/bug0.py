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
from std_srvs.srv import *  # servis.

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
state_desc_ = ['hedefe git', 'duvarı takip et']
state_ = 0
# 0 - hedefe git
# 1 - duvarı takip et


# robotun pozisyon bilgisini alan fonksiyon
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

# robotun lazerini kullanmamızı saglayan fonksiyon.


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
    global state_, state_desc_  # servisleri burda tanımladık.
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "Durum degisti->{}".format(state_desc_[state])
    rospy.loginfo(log)
    if state_ == 0:  # en onemli kisim state 0 ise go_to_point.py scriptini aktif et, degilse follow_wall.py scripitin aktif et, digerini pasif hale getir.
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)


# main fonksiyonunda belirttigimiz donusleri bununla yapacagiz, bu sayede donusleri alirken en uzun mesafeyi kullanmamis olacagiz.
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_

    rospy.init_node('bug0')  # bug0 node'ı baslatildi.

    sub_laser = rospy.Subscriber(
        '/robotcar/laserjob/scanner', LaserScan, clbk_laser)  # lazerimize abone oluyoruz cunku cisimleri tespit etmemiz gerek.
    # odometriye abone oluyoruz cunku robotun anlık konumunu bilmek istiyoruz.
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    # aynı klasor icindeki bu scripti kullan
    rospy.wait_for_service('/go_to_point_switch')
    # aynı klasor icindeki bu scripti kullan
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')

    # change state fonksiyonunda tanımladıgımız servisleri burda kullanılabilir hale getiriyoruz.
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

    # state machine'imiz olmalı, iki stateimiz var, birincisi hedefe gitme, ikincisi ise duvarı takip etme.
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ is None:
            continue

        if state_ == 0:
            # eger robotun onunde 0.15 ile 1 birim arasında uzaklıkta bir cisim varsa
            if regions_['on'] > 0.15 and regions_['on'] < 1:
                change_state(1)  # duvar takibi moduna gec

        # eger duvar takibindeysek önce desired_yaw(robotun yol aldıgını yon)'e bakıyoruz, hata payımıza bakıyoruz
        elif state_ == 1:
            desired_yaw = math.atan2(
                desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)

            # <30 eger error_yaw pi/6'dan ufaksa ve robotun 1.5 metre onunde bir engel yoksa, hedefe_git durumuna don
            if math.fabs(err_yaw) < (math.pi / 6) and \
               regions_['on'] > 1.5 and regions_['sag_on'] > 1 and regions_['sol_on'] > 1:
                print('30 dereceden kucuk')
                change_state(0)

            # <30 && >90 error_yaw'ın pozitif olması hedefimizin robotun sol tarafında kaldıgını gostermektedir.
            if err_yaw > 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['sol'] > 1.5 and regions_['sol_on'] > 1:  # robotumuzun solunda bir engel yoksa state degistir sola donup git.
                print('30 ile 90 derece arasinda sol tarafa')
                change_state(0)
            # eger error_yaw negatif bir degere sahipse gitmek istedigimiz hedef robotumuzun saginda kaliyor demektir.
            if err_yaw < 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['sag'] > 1.5 and regions_['sag_on'] > 1:  # robotumuzun sag tarafinda bir engel yoksa state degistir.
                print('30 ile 90 derece arasinda sag tarafa')
                change_state(0)

        rate.sleep()  # olagan loop kontrolu.(rospy ozelligi.)


if __name__ == "__main__":
    main()
