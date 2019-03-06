#!/usr/bin/env python
# license removed for brevity

import numpy as np
import pylab as plt
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time
import math

x_data = []
y_data = []

def compass_cb(data):
    #print('get')
    global x_data, y_data
    data = data.data
    x_data.append(data[0])
    y_data.append(data[1])
    #print('get')

rospy.init_node('magnetic_calibration',anonymous=True)
pub1 = rospy.Publisher('state',Int32,queue_size=10)
time.sleep(1)
pub1.publish(10)
time.sleep(2)
rospy.Subscriber('compass', Int32MultiArray, compass_cb)
pub1.publish(1)
while not rospy.is_shutdown():
    if len(x_data) == 10:
        pub1.publish(2)
    if len(x_data) == 20:
        pub1.publish(4)
    if len(x_data) == 30:
        pub1.publish(8)
    if len(x_data) == 40:
        pub1.publish(16)
    if len(x_data) == 50:
        pub1.publish(32)
    if len(x_data) == 60:
        pub1.publish(64)
    if len(x_data) == 70:
        pub1.publish(128)
    if len(x_data) > 80:
        pub1.publish(1)
        break

xList = list(x_data)
yList = list(y_data)
xList.sort()
yList.sort()

xMin = sum(xList[0:5])/5.
xMax = sum(xList[-5:])/5.
yMin = sum(yList[0:5])/5.
yMax = sum(yList[-5:])/5.
xAve = (xMin+xMax)/2.
yAve = (yMin+yMax)/2.
plt.plot(x_data, y_data, 'b.')
print xAve, yAve
for i in range(len(xList)):
    x_data[i] = x_data[i]-xAve
    y_data[i] = y_data[i]-yAve
m, b = np.polyfit(x_data, y_data, 1)

theta = math.atan(m)
print theta

plt.plot(x_data, y_data, 'r.')

for i in range(len(x_data)):
    x_data[i] = x_data[i]*math.cos(theta)-y_data[i]*math.sin(theta)
    y_data[i] = x_data[i]*math.sin(theta)+y_data[i]*math.cos(theta)
plt.plot(x_data, y_data, 'g.')

xList = list(x_data)
yList = list(y_data)
xList.sort()
yList.sort()
xMin = sum(xList[0:5])/5.
xMax = sum(xList[-5:])/5.
yMin = sum(yList[0:5])/5.
yMax = sum(yList[-5:])/5.

scale = (xMax-xMin)/(yMax-yMin)
print scale

for i in range(len(x_data)):
    x_data[i] = float(x_data[i])/scale

plt.plot(x_data, y_data, 'y.')
plt.show()
