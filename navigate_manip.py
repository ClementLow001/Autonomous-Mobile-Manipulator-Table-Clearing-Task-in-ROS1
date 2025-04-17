#!/usr/bin/env python2
# encoding: utf-8
import sys
import rospy
import tf
import numpy as np
import actionlib
import math as m
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from hiwonder_servo_controllers.bus_servo_control import set_servos
from hiwonder_kinematics.kinematics_control import set_pose_target, set_joint_value_target

from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_interfaces.srv import SetRobotPose, SetJointValue, GetRobotPose, SetLink, GetLink, SetJointRange, GetJointRange
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

def movebase_client(x,y,oz,ow):
    # Create an action client for move_base
    client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
    client.wait_for_server()

    # Set the goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "robot_1/map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Goal Pose
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow

    actionGoal = MoveBaseActionGoal()
    actionGoal.goal = goal

    # Send goal to move_base
    client.send_goal(actionGoal.goal)

    # Wait for the robot to reach the goal
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal Reached! Adjusting orientation...")

        # Extract the desired final orientation
        quaternion = (
            goal.target_pose.pose.orientation.x,
            goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w
        )
        _, _, desired_yaw = euler_from_quaternion(quaternion)

        # Adjust orientation precisely
        adjust_orientation(desired_yaw)

    return 1
def adjust_orientation(target_yaw):
    """Rotates the robot to the exact target yaw after reaching the goal"""
    rospy.loginfo("Adjusting orientation to: {:.2f} radians".format(target_yaw))

    cmd_pub = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=10)
    tf_listener = tf.TransformListener()
    
    rate = rospy.Rate(10)  # 10 Hz loop
    twist = Twist()
    threshold = 0.1  # Small angle threshold (radians)
    max_turn_speed = 0.3  # Maximum angular velocity

    while not rospy.is_shutdown():
        try:
            # Get the current robot orientation from TF
            (trans, rot) = tf_listener.lookupTransform('/robot_1/map', '/robot_1/base_link', rospy.Time(0))
            _, _, current_yaw = euler_from_quaternion(rot)

            # Compute yaw error
            yaw_error = target_yaw - current_yaw
            yaw_error = m.atan2(m.sin(yaw_error), m.cos(yaw_error))  # Normalize to [-pi, pi]

            # If the error is small enough, stop
            if abs(yaw_error) < threshold:
                rospy.loginfo("Orientation adjusted!")
                break

            # Apply proportional control
            twist.angular.z = max(-max_turn_speed, min(max_turn_speed, 0.5 * yaw_error))  # Scale rotation speed
            cmd_pub.publish(twist)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    # Stop rotation
    twist.angular.z = 0.0
    cmd_pub.publish(twist)

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

            # Transform the tag pose to `robot_1/base_link`
            try:
                transformed_pose = tf_listener.transformPose("robot_1/base_footprint", tag_pose)
                detected_tags[tag_id] = transformed_pose.pose
                
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF transform failed!")
    #print detected_tags    
    return detected_tags

def see_and_grab(pose):
    pose = pose

    errorx = (0.05*m.sin(45))
    errory = (0.05*m.cos(45))
    z = 0.245
    '''
    errorz = 0 
    '''
    if pose.position.y < 0.08 and pose.position.y > -0.08:
        go_to_pose([pose.position.x-errorx, pose.position.y-errory +0.025, 0.32],45,[-180,180],1)
        go_to_pose([pose.position.x, pose.position.y, z],45,[-180,180],1)
        gripper(1000)
        go_to_pose([pose.position.x-errorx, pose.position.y-errory +0.025, 0.32],45,[-180,180],1)
        go_to_joint_state([500,500,500,500,500,1000])
        go_to_pose([-0.14, 0, 0.3],0,[-180,180],1)
        gripper(0)
        go_to_joint_state([500,688,213,50,500,0])
    else:
        go_to_joint_state([500,688,213,50,500,0])

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.1)
    try:
        result = movebase_client(1.01665759087, 1.13363933563, -0.711490486324, 0.702695729225)#X, Y, yaw, w                        -0.711490486324, 0.702695729225 | -0.700465598598, 0.713686167149
        if result:
            print result
            go_to_joint_state([500,688,213,50,500,0])
            tag = get_tag_pose()
            for i in tag:
                see_and_grab(tag[i])
        result = movebase_client(1.49807107449, 0.873422980309, 0.999981751017, 0.00604132712413)#X, Y, yaw, w
        if result:
            print result
            tag = get_tag_pose()
            for i in tag:
                see_and_grab(tag[i])
        result = movebase_client(0, 0, 0, 1)#X, Y, yaw, w
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation_Manipulator Test ShutDown")
