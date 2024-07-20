#!/usr/bin/env python
import rospy
from dynamic_reconfigure.server import Server
from smm_screws.cfg import JointVelocitiesConfig
from smm_screws.msg import JointVel

def callback(config, level):
    vel_msg = JointVel()
    vel_msg.velocities = [config.joint1, config.joint2, config.joint3]
    pub.publish(vel_msg)
    return config

if __name__ == "__main__":
    rospy.init_node("joint_velocities_publisher", anonymous=True)
    pub = rospy.Publisher("/joint_velocities", JointVel, queue_size=10)
    srv = Server(JointVelocitiesConfig, callback)
    rospy.spin()
