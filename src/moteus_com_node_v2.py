#!/usr/bin/env python3 -B

import rospy
from geometry_msgs.msg import Pose

import asyncio
import math
import moteus
import numpy as np
import matplotlib.pyplot as plt

th_base = 0
th_shoulder = 0
th_elbow = 0

def callback_joints(data):
    global th_base
    th_base = data.orientation.x/(2*math.pi)
    th_shoulder = data.orientation.y/(2*math.pi)
    th_elbow = data.orientation.z/(2*math.pi)
    rospy.loginfo(rospy.get_caller_id() + "base angle: %s rad, %s rev", data.data,th_base)

async def main():
    rospy.init_node('moteus_com_node', anonymous=True)
    rospy.Subscriber("/msv_3dof_arm/joints_vector", Pose, callback_joints)
    rate = rospy.Rate(120)

    #Controllers
    controller_Base = moteus.Controller(id=1)
    await controller_Base.set_stop()
    controller_Shoulder = moteus.Controller(id=2)
    await controller_Shoulder.set_stop()
    controller_Elbow = moteus.Controller(id=3)
    await controller_Elbow.set_stop()

    while not rospy.is_shutdown():
        
        state_Base = await controller_Base.set_position(position=math.nan,stop_position=-th_base*4,velocity=0.5, query=True)
        state_Shoulder = await controller_Shoulder.set_position(position=math.nan,stop_position=th_shoulder*240,velocity=12, query=True)
        state_Elbow = await controller_Elbow.set_position(position=math.nan,stop_position=th_elbow*20-0.5,velocity=2, query=True)
        
        print("posElbow desired: ",th_elbow)
        print("Position1:", state_Elbow.values[moteus.Register.POSITION], "Velocity1:", state_Base.values[moteus.Register.VELOCITY])
        print()

        rate.sleep()


if __name__ == '__main__':
    asyncio.run(main())