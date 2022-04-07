#!/usr/bin/env python3 -B

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose

import asyncio
import math
import moteus
import numpy as np
import matplotlib.pyplot as plt

th_base = 0
th_shoulder = 0
th_elbow = 0

def callback_base(data):
    global th_base
    th_base = (data.data-3.14159)/(2*math.pi)
    rospy.loginfo(rospy.get_caller_id() + "base angle: %s rad, %s rev", data.data,th_base)

def callback_shoulder(data):
    global th_shoulder
    th_shoulder = data.data/(2*math.pi)
    rospy.loginfo(rospy.get_caller_id() + "shoulder angle: %s rad, %s rev", data.data,th_shoulder)

def callback_elbow(data):
    global th_elbow
    th_elbow = (data.data+3.14159)/(2*math.pi)
    rospy.loginfo(rospy.get_caller_id() + "elbow angle: %s rad, %s rev", data.data,th_elbow)

async def main():
    rospy.init_node('moteus_com_node', anonymous=True)
    rospy.Subscriber("/msv_3dof_arm/joint_base_position_controller/command", Float64, callback_base)
    rospy.Subscriber("/msv_3dof_arm/joint_shoulder_position_controller/command", Float64, callback_shoulder)
    rospy.Subscriber("/msv_3dof_arm/joint_elbow_position_controller/command", Float64, callback_elbow)
    pub_base = rospy.Publisher('moteus_base_reading', Pose, queue_size=1)
    pub_shoulder = rospy.Publisher('moteus_shoulder_reading', Pose, queue_size=4)
    pub_elbow = rospy.Publisher('moteus_elbow_reading', Pose, queue_size=4)
    rate = rospy.Rate(120)

    base_data = Pose()
    shoulder_data = Pose()
    elbow_data = Pose()

    #Controllers
    qr = moteus.QueryResolution()   #some important values are not requested by default
    qr.q_current = moteus.F32
    qr.d_current = moteus.F32

    controller_Base = moteus.Controller(id=1, query_resolution=qr)
    await controller_Base.set_stop()
    controller_Shoulder = moteus.Controller(id=2, query_resolution=qr)
    await controller_Shoulder.set_stop()
    controller_Elbow = moteus.Controller(id=3, query_resolution=qr)
    await controller_Elbow.set_stop()
    

    while not rospy.is_shutdown():
        
        state_Base = await controller_Base.set_position(position=math.nan,stop_position=-th_base*4,velocity=0.5, query=True)
        state_Shoulder = await controller_Shoulder.set_position(position=math.nan,stop_position=th_shoulder*240,velocity=14, query=True)
        state_Elbow = await controller_Elbow.set_position(position=math.nan,stop_position=th_elbow*20+0.25,velocity=3, query=True)
        
        base_data.position.x = state_Base.values[moteus.Register.POSITION]
        base_data.position.y = state_Base.values[moteus.Register.TORQUE]
        base_data.position.z = state_Base.values[moteus.Register.VELOCITY]
        base_data.orientation.x = state_Base.values[moteus.Register.Q_CURRENT]
        base_data.orientation.y = state_Base.values[moteus.Register.D_CURRENT]
        base_data.orientation.z = state_Base.values[moteus.Register.VOLTAGE]

        shoulder_data.position.x = state_Shoulder.values[moteus.Register.POSITION]
        shoulder_data.position.y = state_Shoulder.values[moteus.Register.TORQUE]
        shoulder_data.position.z = state_Shoulder.values[moteus.Register.VELOCITY]
        shoulder_data.orientation.x = state_Shoulder.values[moteus.Register.Q_CURRENT]
        shoulder_data.orientation.y = state_Shoulder.values[moteus.Register.D_CURRENT]
        shoulder_data.orientation.z = state_Shoulder.values[moteus.Register.VOLTAGE]

        elbow_data.position.x = state_Elbow.values[moteus.Register.POSITION]
        elbow_data.position.y = state_Elbow.values[moteus.Register.TORQUE]
        elbow_data.position.z = state_Elbow.values[moteus.Register.VELOCITY]
        elbow_data.orientation.x = state_Elbow.values[moteus.Register.Q_CURRENT]
        elbow_data.orientation.y = state_Elbow.values[moteus.Register.D_CURRENT]
        elbow_data.orientation.z = state_Elbow.values[moteus.Register.VOLTAGE]

        pub_base.publish(base_data)
        pub_shoulder.publish(shoulder_data)
        pub_elbow.publish(elbow_data)
        print("posBase des: ",th_base,"  posShoulder des: ",th_shoulder,"  posElbow des: ",th_elbow)
        print()

        rate.sleep()


if __name__ == '__main__':
    asyncio.run(main())