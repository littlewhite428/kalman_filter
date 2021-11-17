#!/usr/bin/python
import rospy
import numpy as np
from kalman_filter import kalman_filter
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

velocity = 0.0
TF_height = 0.0

def cali_cb(msg):
    global velocity
    velocity = -msg.twist.twist.linear.z
    # rospy.loginfo_throttle(1,str(velocity))

def TF_cb(msg):
    global TF_height
    TF_height = msg.data
    # rospy.loginfo_throttle(1,str(TF_height))

rospy.init_node('height filter')
rospy.Subscriber('/drone_1/mavros/global_position/local',Odometry,cali_cb)
rospy.Subscriber('/TF02/Height',Float32,TF_cb)
kf_pub = rospy.Publisher('/kalman/Height',Float32,queue_size=10)

delta_time = 0.1
r = rospy.Rate(1 / delta_time)

A = np.array([[1]])
B = np.array([[delta_time]])
H = np.array([[1]])
Q = np.array([[0.003*delta_time]])
R = np.array([[0.30]])
P0 = np.array([[1e4]])
x0 = np.array([0])


kf = kalman_filter(A,B,H,Q,R,P0,x0,delta_time)
while not rospy.is_shutdown():
    out,k = kf.iter(velocity,TF_height)
    kf_msg = Float32()
    kf_msg.data = float(out)
    kf_pub.publish(kf_msg)
    rospy.loginfo_throttle(0.5,k)
    r.sleep()


