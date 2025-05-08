#! /usr/bin/env python3
import os
import numpy as np
import rospkg
import yaml
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import rospy
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
import csv
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pi, atan2, sqrt
import rospkg
import os
import numpy as np
import time

plt.ion()

X = None
Y = None
yaw = None
race_started = False
trajectory_x = []
trajectory_y = []
trajectory_time = []
start_time = None
checkpoints_hit = []


Goal_reached = 0

rp = rospkg.RosPack()
path = rp.get_path('f1tenth_racing')
file_path = os.path.join(path, 'config','params.yaml')
with open(file_path, "r") as file:
    params = yaml.safe_load(file)

odom_topic = params['odom_topic']
command_topic = params['command_topic']

waypoints = []
csv_path = os.path.join(path,'csv/gp_centerline.csv')
with open(csv_path, 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        x = float(row['x'])
        y = float(row['y'])
        waypoints.append((x, y))

data = waypoints

waypoints_x = [wp[0] for wp in waypoints]
waypoints_y = [wp[1] for wp in waypoints]

def odom_callback(msg):
    global X, Y, yaw
    X = msg.pose.pose.position.x
    Y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw_ = euler_from_quaternion(orientation_list)
    yaw = yaw_

def start_callback(msg):
    global race_started
    if msg.data:
        rospy.loginfo("Race has started!")
        race_started = True

def plot_time_vs_trajectory():
    distances = [0.0]
    for i in range(1, len(trajectory_x)):
        dx = trajectory_x[i] - trajectory_x[i - 1]
        dy = trajectory_y[i] - trajectory_y[i - 1]
        distances.append(distances[-1] + np.sqrt(dx**2 + dy**2))

    plt.ioff()
    plt.figure()
    plt.plot(trajectory_time, distances, 'r-')
    plt.title('Time vs Trajectory Distance')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance Traveled (m)')
    plt.grid()
    plt.show()

def pp():
    global X, Y, yaw, start_time
    global prev_error_linear, integral_linear, prev_error_angular, integral_angular, prev_time
    steering = AckermannDrive()

    L = 0.324  
    speed = 1.0
    K_dd = 0.3 
    min_ld = 0.7   
    max_ld = 1.3
    goal_index = 0
    rate = rospy.Rate(10)

    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'b-', label='Actual Path')
    ax.plot(waypoints_x, waypoints_y, 'r--', label='Planned Path')
    ax.set_title('Real-Time Trajectory')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid()
    ax.legend()



    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        if X is None or Y is None or yaw is None:
            print("None")
            rate.sleep()
            continue
        while not race_started and not rospy.is_shutdown():
            rospy.loginfo_once("Waiting for race start...")
            rate.sleep()

        current_time = rospy.get_time() - start_time
        trajectory_x.append(X)
        trajectory_y.append(Y)
        trajectory_time.append(current_time)

        line.set_xdata(trajectory_x)
        line.set_ydata(trajectory_y)
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.001)

        if goal_index >= len(waypoints):
            rospy.loginfo(f"Lap completed in {current_time:.2f} seconds")
            steering.steering_angle = 0.0
            steering.speed = 0.0
            pub.publish(steering)
            break

        goal_x, goal_y = waypoints[goal_index]
        dx = goal_x - X
        dy = goal_y - Y
        distance = np.sqrt(dx**2 + dy**2)

        if goal_index % 100 == 0 and goal_index not in checkpoints_hit:
            rospy.loginfo(f"Checkpoint {goal_index} reached at ({X:.2f}, {Y:.2f}), dist={distance:.2f}m")
            checkpoints_hit.append(goal_index)

        if distance < 0.8:
            goal_index += 1
            continue
        l_d = np.clip(K_dd * speed, min_ld, max_ld)
        goal_yaw = np.arctan2(dy, dx)
        alpha = (goal_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        delta = -np.arctan2(2 * L * np.sin(alpha), l_d)
        delta = max(min(delta, 0.4), -0.4)


        steering.steering_angle = delta
        steering.speed = speed
        pub.publish(steering)

        rate.sleep()

    plot_time_vs_trajectory()
    missing = [i for i in range(0, 1757, 100) if i not in checkpoints_hit]
    rospy.loginfo(f"Missing checkpoints: {missing}")

if __name__ == '__main__':
    rospy.init_node('PP')
    rospy.Subscriber('/race_start', Bool, start_callback)
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    pub = rospy.Publisher(command_topic, AckermannDrive, queue_size=10)
    pp()































# import os
# import numpy as np
# import rospkg
# import yaml
# from ackermann_msgs.msg import AckermannDrive
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped
# import rospy
# from tf.transformations import euler_from_quaternion
# import matplotlib.pyplot as plt
# from std_msgs.msg import Bool
# import csv

# plt.ion()

# X = None
# Y = None
# yaw = None
# race_started = False
# trajectory_x = []
# trajectory_y = []
# trajectory_time = []
# start_time = None
# checkpoints_hit = []

# rp = rospkg.RosPack()
# path = rp.get_path('f1tenth_racing')
# file_path = os.path.join(path, 'config','params.yaml')
# with open(file_path, "r") as file:
#     params = yaml.safe_load(file)

# odom_topic = params['odom_topic']
# command_topic = params['command_topic']

# waypoints = []
# csv_path = os.path.join(path,'csv/gp_centerline.csv')
# with open(csv_path, 'r') as csvfile:
#     reader = csv.DictReader(csvfile)
#     for row in reader:
#         x = float(row['x'])
#         y = float(row['y'])
#         waypoints.append((x, y))

# waypoints_x = [wp[0] for wp in waypoints]
# waypoints_y = [wp[1] for wp in waypoints]

# def odom_callback(msg):
#     global X, Y, yaw
#     X = msg.pose.pose.position.x
#     Y = msg.pose.pose.position.y
#     orientation_q = msg.pose.pose.orientation
#     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     roll, pitch, yaw_ = euler_from_quaternion(orientation_list)
#     yaw = yaw_

# def start_callback(msg):
#     global race_started
#     if msg.data:
#         rospy.loginfo("Race has started!")
#         race_started = True

# def plot_time_vs_trajectory():
#     distances = [0.0]
#     for i in range(1, len(trajectory_x)):
#         dx = trajectory_x[i] - trajectory_x[i - 1]
#         dy = trajectory_y[i] - trajectory_y[i - 1]
#         distances.append(distances[-1] + np.sqrt(dx**2 + dy**2))

#     plt.ioff()
#     plt.figure()
#     plt.plot(trajectory_time, distances, 'r-')
#     plt.title('Time vs Trajectory Distance')
#     plt.xlabel('Time (s)')
#     plt.ylabel('Distance Traveled (m)')
#     plt.grid()
#     plt.show()

# def pp():
#     global X, Y, yaw, start_time
#     L = 0.324
#     speed = 2.0
#     goal_index = 0
#     rate = rospy.Rate(10)

#     fig, ax = plt.subplots()
#     line, = ax.plot([], [], 'b-', label='Actual Path')
#     ax.plot(waypoints_x, waypoints_y, 'r--', label='Planned Path')
#     ax.set_title('Real-Time Trajectory')
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.grid()
#     ax.legend()

#     # while not race_started and not rospy.is_shutdown():
#     #     rospy.loginfo_once("Waiting for race start...")
#     #     rate.sleep()

#     start_time = rospy.get_time()

#     while not rospy.is_shutdown():
#         if X is None or Y is None or yaw is None:
#             rate.sleep()
#             continue

#         current_time = rospy.get_time() - start_time
#         trajectory_x.append(X)
#         trajectory_y.append(Y)
#         trajectory_time.append(current_time)

#         line.set_xdata(trajectory_x)
#         line.set_ydata(trajectory_y)
#         ax.relim()
#         ax.autoscale_view()
#         plt.pause(0.001)

#         if goal_index >= len(waypoints):
#             rospy.loginfo(f"Lap completed in {current_time:.2f} seconds")
#             steering = AckermannDrive()
#             steering.steering_angle = 0.0
#             steering.speed = 0.0
#             pub.publish(steering)
#             break

#         goal_x, goal_y = waypoints[goal_index]
#         dx = goal_x - X
#         dy = goal_y - Y
#         distance = np.sqrt(dx**2 + dy**2)

#         if goal_index % 100 == 0 and goal_index not in checkpoints_hit:
#             rospy.loginfo(f"Checkpoint {goal_index} reached at ({X:.2f}, {Y:.2f}), dist={distance:.2f}m")
#             checkpoints_hit.append(goal_index)

#         if distance < 0.8:
#             goal_index += 1
#             continue

#         goal_yaw = np.arctan2(dy, dx)
#         alpha = (goal_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
#         delta = -np.arctan2(2 * L * np.sin(alpha), distance)
#         delta = max(min(delta, 0.4), -0.4)

#         steering = AckermannDrive()
#         steering.steering_angle = delta
#         steering.speed = speed
#         pub.publish(steering)

#         rate.sleep()

#     plot_time_vs_trajectory()
#     missing = [i for i in range(0, 1757, 100) if i not in checkpoints_hit]
#     rospy.loginfo(f"Missing checkpoints: {missing}")

# if __name__ == '__main__':
#     rospy.init_node('PP')
#     rospy.Subscriber('/race_start', Bool, start_callback)
#     rospy.Subscriber(odom_topic, Odometry, odom_callback)
#     pub = rospy.Publisher(command_topic, AckermannDrive, queue_size=10)
#     pp()
