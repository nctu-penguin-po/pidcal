#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import time
import random as rd

import Tkinter as tk

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
posture_data=[0,0,0]
depth_data = 0
fc = 0
tc = 0

depth_center = 0
vol = 0

def motor_cb(data):
    global depth_center
    data = data.data
    s = data[4]+data[5]+data[6]+data[7]
    print s
    if s > 7000:
        depth_center = depth_center+0.01
    elif s < 6500 and depth_center > 0.02: 
        depth_center = depth_center-0.01

rospy.init_node('dummy',anonymous=True)

rospy.Subscriber('/motor', Int32MultiArray, motor_cb)

pub1 = rospy.Publisher('/posture',numpy_msg(Floats),queue_size=10)
pub2 = rospy.Publisher('/depth',Float32,queue_size=10)
pub3 = rospy.Publisher('/forward_command',Int32,queue_size=10)
pub4 = rospy.Publisher('/turn_command',Int32,queue_size=10)
pub5 = rospy.Publisher('/voltage',Float32,queue_size=10)
pub7 = rospy.Publisher('/compass_yaw',Float32,queue_size=10)
pub8 = rospy.Publisher('/trigger_command',Int32,queue_size=10)
pub9 = rospy.Publisher('/block/yaw/err',Float32,queue_size=10)



now_time = 0

while now_time<60:
    pos_data = np.array([rd.uniform(-10, 10), rd.uniform(-10, 10), rd.uniform(-10, 10)])
    pub1.publish(pos_data)
    pub2.publish(depth_center+rd.uniform(-0.15, 0.15))
    pub5.publish(vol)
    pub8.publish(0)
    pub7.publish(rd.uniform(-10, 10))
    pub9.publish(rd.uniform(-5, 5))
    time.sleep(0.1)
    now_time = now_time+0.1
    if now_time > 5:
        vol = 16
