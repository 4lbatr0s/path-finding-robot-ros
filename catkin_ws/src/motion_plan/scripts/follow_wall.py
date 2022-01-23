#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False  # robottan gelen mesajlara göre True ve false olacak.
pub_ = None

regions_ = {
    'sag': 0,
    'sag_on': 0,
    'on': 0,
    'sol_on': 0,
    'sol': 0
}

state_ = 0

state_dict_ = {
    0: 'duvari bul',
    1: 'sola don',
    2: 'duvari takip et'
}


def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Bitti!'
    return res


def clbk_laser(msg):  # karsisindaki alani bes parcaya bolmus durumda fakat sadece ondeki uc parcayi kullanacak
    global regions_
    dist_max = 10
    regions_ = {
        'sag':  min(min(msg.ranges[0:144]), dist_max),
        'sag_on': min(min(msg.ranges[144:288]), dist_max),
        'on':  min(min(msg.ranges[288:432]), dist_max),
        'sol_on':  min(min(msg.ranges[432:576]), dist_max),
        'sol':   min(min(msg.ranges[576:720]), dist_max)
    }
    take_action()

# t+1 anındaki durum t anından farklıysa state(durum) degistirir.


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Duvar takibi - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

    # lazerden degerleri alır ve algoritmanın durumunu degistirir.


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    dist = 1.5
    # robotun 1.5 metre mesafesi bossa bir sey yapma
    # robotun 1.5 metre uzaklıgındaki alanda bir engel varsa, duruma gore duvarın sagına gec
    # duvari takip et.
    if regions['on'] > dist and regions['sol_on'] > dist and regions['sag_on'] > dist:
        state_description = 'durum 1 - engel yok'
        change_state(0)
    elif regions['on'] < dist and regions['sol_on'] > dist and regions['sag_on'] > dist:
        state_description = 'durum 2 - on'
        change_state(1)
    elif regions['on'] > dist and regions['sol_on'] > dist and regions['sag_on'] < dist:
        state_description = 'durum 3 - sag_on'
        change_state(2)
    elif regions['on'] > dist and regions['sol_on'] < dist and regions['sag_on'] > dist:
        state_description = 'durum 4 - sol_on'
        change_state(0)
    elif regions['on'] < dist and regions['sol_on'] > dist and regions['sag_on'] < dist:
        state_description = 'durum 5 - on ve sag_on'
        change_state(1)
    elif regions['on'] < dist and regions['sol_on'] < dist and regions['sag_on'] > dist:
        state_description = 'durum 6 - on ve sol_on'
        change_state(1)
    elif regions['on'] < dist and regions['sol_on'] < dist and regions['sag_on'] < dist:
        state_description = 'durum 7 - on & sol_on & sag_on'
        change_state(1)
    elif regions['on'] > dist and regions['sol_on'] < dist and regions['sag_on'] < dist:
        state_description = 'durum 8 - sol_on & sag_on'
        change_state(0)
    else:
        state_description = 'bilinmeyen durum'
        rospy.loginfo(regions)


# duvarı henuz bulmadiysan 0.2 birim ileri, git ve saga don.
def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.3
    return msg


def turn_left():  # duvari buldugunda yani state 1'de sola don duvari sagina al
    msg = Twist()
    msg.angular.z = -0.3
    return msg


def follow_the_wall():  # duvari sagina aldiktan sonra 0.3 birim ileri dogru git.
    global regions_

    msg = Twist()
    msg.linear.x = 0.3
    return msg


def main():
    global pub_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:  # eger bu script False durumdaysa, yani bug dosyasında kullanılmıyorsa bir şey yapma, kullanılıyorsa hareket işlemlerini gerçekleştir.
            rate.sleep()
            continue

        msg = Twist()
        if state_ == 0:  # duvar bulmuyorsan x ekseninde 0.2 birim ilerle, ve z ekseninde -0.3 birim dön yani saga don
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        else:
            rospy.logerr('Bilinmeyen durum!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
