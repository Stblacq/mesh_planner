#!/usr/bin/env python

import rospy
import random
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import csv

current_state = None

csv_file_path = '/home/developer/workspace/src/pluto_robot/pluto_navigation/scripts/results.csv'


def odom_callback(msg):
    global current_state
    current_state = msg.pose.pose

def apply_random_control():
    control = Twist()
    control.linear.x = random.uniform(-1, 3)
    control.angular.z = random.uniform(-1, 1)
    cmd_vel_pub.publish(control)
    return control

def write_to_csv(position, orientation, control, next_position, next_orientation):
    with open(csv_file_path, 'a') as file:
        writer = csv.writer(file)

        if file.tell() == 0:
            writer.writerow(["current_position_x", "current_position_y", "current_position_z", "current_orientation_x", 
                             "current_orientation_y", "current_orientation_z", "current_orientation_w", "control_linear_x", 
                             "control_angular_z", "next_position_x", "next_position_y", "next_position_z", 
                             "next_orientation_x", "next_orientation_y", "next_orientation_z", "next_orientation_w"])

        writer.writerow([position[0], position[1], position[2], orientation[0], orientation[1], orientation[2], orientation[3],
                         control.linear.x, control.angular.z, next_position[0], next_position[1], next_position[2],
                         next_orientation[0], next_orientation[1], next_orientation[2], next_orientation[3]])

def main():
    global cmd_vel_pub 

    rospy.init_node('random_control_robot', anonymous=True)

    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, odom_callback)

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if current_state is not None:
            initial_state = current_state

            control = apply_random_control()

            time.sleep(1)
            next_state = current_state 

            write_to_csv(
                [initial_state.position.x, initial_state.position.y, initial_state.position.z],
                [initial_state.orientation.x, initial_state.orientation.y, initial_state.orientation.z, initial_state.orientation.w],
                control,
                [next_state.position.x, next_state.position.y, next_state.position.z],
                [next_state.orientation.x, next_state.orientation.y, next_state.orientation.z, next_state.orientation.w]
            )

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
