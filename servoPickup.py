#!/usr/bin/env python2
# encoding: utf-8
import sys
import rospy
import numpy as np
import tf
import math as m
from hiwonder_servo_controllers.bus_servo_control import set_servos
from hiwonder_kinematics.kinematics_control import set_pose_target, set_joint_value_target, get_joint_position

from hiwonder_servo_msgs.msg import MultiRawIdPosDur, ServoStateList
from hiwonder_interfaces.srv import SetRobotPose, SetJointValue, GetRobotPose, SetLink, GetLink, SetJointRange, GetJointRange
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

def go_to_joint_state(state):	
    servo_data = state
    res = set_joint_value_target(state)
    print res
    set_servos(joints_pub, 1.5, ((10, servo_data[5]), (5, servo_data[4]), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
    rospy.sleep(2)

def go_to_pose(position, pitch, pitch_range=[-180, 180], resolution=1):
    res = set_pose_target(position,pitch,pitch_range,resolution)
    print ""
    print "Current Pulse: ", res[2]
    print "Goal Pulse:    ", res[1]
    print "RPY:           ", res[3]
    print "min_variance:  ", res[4]
    if res[1]:
       servo_data = res[1]
       set_servos(joints_pub, 1.5, ((5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
    rospy.sleep(2)

def gripper(state):
    set_servos(joints_pub, 1.5, ((10, state),))
    rospy.sleep(2)

def get_pose():
    res = get_joint_position()
    print res.pose

def get_tag_pose():
    rospy.loginfo("Waiting for AprilTag detection...")
    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
    detected_tags = {}

    if not msg.detections:
        rospy.loginfo("No AprilTag detected")
        return
    else:
        tf_listener = tf.TransformListener()
        rospy.sleep(1)  # Allow TF buffer to fill
        for tag in msg.detections:
            tag_id = tag.id[0]
            tag_pose = PoseStamped()
            tag_pose.header.frame_id = "robot_1/depth_cam_color_frame" 
            tag_pose.pose = tag.pose.pose.pose  # Extract AprilTag pose

            # Transform the tag pose to `robot_1/base_footprint`
            try:
                transformed_pose = tf_listener.transformPose("robot_1/base_footprint", tag_pose)
                detected_tags[tag_id] = transformed_pose.pose
                #print tag_id                
                #print transformed_pose.pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF transform failed!")
    #print detected_tags    
    return detected_tags

def see_and_grab(pose):
    pose = pose
   
    errorx = (0.028*m.sin(45))
    errory = (0.028*m.cos(45))
    z = 0.245
    '''
    errorz = 0 
    '''
    if pose.position.y < 0.08 and pose.position.y > -0.08:
        go_to_pose([pose.position.x-(0.05*m.sin(45)), pose.position.y-(0.05*m.cos(45))+0.025, 0.32],45,[-180,180],1)
        go_to_pose([pose.position.x, pose.position.y, z],45,[-180,180],1)
        gripper(1000)
        go_to_pose([pose.position.x-(0.05*m.sin(45)), pose.position.y-(0.05*m.cos(45))+0.025, 0.32],45,[-180,180],1)
        go_to_joint_state([500,500,500,500,500,1000])
        go_to_pose([-0.14, 0, 0.3],0,[-180,180],1)
        gripper(0)
        go_to_joint_state([500,688,213,50,500,0])
    else:
        go_to_joint_state([500,688,213,50,500,0])
    
    

if __name__ == '__main__':	
    rospy.init_node('ArmNode') 
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2)
    go_to_joint_state([500,688,213,50,500,0])
    tag = get_tag_pose()    
    for i in tag:
        print tag[i]
        see_and_grab(tag[i])

    #get_pose()
    #go_to_joint_state([500,688,213,50,500,1000])
    #go_to_pose([0.14, 0, 0.22],45,[-180,180],1)
    #go_to_pose([pose.position.x, pose.position.y, pose.position.z-0.1],45,[-180,180],1)
    #go_to_pose([pose.position.x, pose.position.y, pose.position.z],45,[-180,180],1)
    
    print "Run Complete" 
