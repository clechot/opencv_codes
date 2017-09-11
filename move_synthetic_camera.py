#!/usr/bin/env python

import numpy as np

import rospy

import math

from std_msgs.msg import Float32
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from tf import transformations

model_state = ModelState()

def callback_pose(data):
    global model_state
    #model_state.pose = data
    #model_state.pose.position.x = 2.3 + data.position.x
    #model_state.pose.position.y = -0.6 + data.position.y
    #model_state.pose.position.z = 1.0 + data.position.z

    model_state.pose.position.x = model_state.pose.position.x + data.position.x
    model_state.pose.position.y = model_state.pose.position.y + data.position.y
    model_state.pose.position.z = model_state.pose.position.z + data.position.z

    move_camera()


def callback_angle(data):
    # Compute the rotation quaternion: rotation about the z-axis.
    z_axis = (0, 0, 1)

    quaternion = transformations.quaternion_about_axis(
        angle=np.pi+np.pi*data.data, axis=z_axis)

    model_state.pose.orientation.x = quaternion[0]
    model_state.pose.orientation.y = quaternion[1]
    model_state.pose.orientation.z = quaternion[2]
    model_state.pose.orientation.w = quaternion[3]

    # In order to move the virtual camera on an arc
    global model_state
    model_state.pose.position.x = 1.9*math.cos(np.pi*data.data) + 0.48
    model_state.pose.position.y = 1.9*math.sin(np.pi*data.data) -0.6
    model_state.pose.position.z = 1.0

    move_camera()

def move_camera():
    global model_state

    model_state.model_name = "synthetic_camera"
    
    try:
        service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)			
        rospy.loginfo('Setting model state: {}'.format(model_state.pose))

        ret = service.call(model_state)
        print('Service called: returned {}'.format(ret.success))

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def run():
    rospy.loginfo('Waiting for service..')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    rospy.loginfo('Sending pose')		        
    rospy.Subscriber("/desired_camera/pose",  Pose, callback_pose)
    rospy.Subscriber("/desired_camera/angle",  Float32, callback_angle)
      

        # print('Called service: {}'.format(model_state))   #


if __name__ == '__main__':
    rospy.init_node('move_synthetic_camera')
    run()
    rospy.spin()

