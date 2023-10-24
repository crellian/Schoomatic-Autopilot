#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf2_ros

import message_filters
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from carla_msgs.msg import CarlaEgoVehicleControl
from carla_navigation.msg import FloatArray

from carla_navigation.teb import TEB

# LINEAR_VELOCITY = 740
# ANGULAR_VELOCITY = 1500
# ROBOT_WIDTH = 0.65
# WHEEL_RADIUS = 0.37

class Base:
    def __init__(self):
        self.way_points = []
        self.pos = None
        self.enable = False  # True when receive a goal and begin navigation
        self.mk_msg = MarkerArray()
        self.tf_buffer = None

        self.vel_setpoint_pub = None
        self.ang_setpoint_pub = None
        self.vel_state_pub = None
        self.ang_state_pub = None
        self.cmd_pub = None
        self.marker_pub = None


    def run(self):
        rospy.init_node('base', anonymous=True)

        # tf transformer
        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # global plan subscriber
        plan_sub = rospy.Subscriber(
            '/plan', FloatArray, self.plan_callback)
        # sensor subscriber
        odom_sub = rospy.Subscriber(
            "/carla/ego_vehicle/odometry", Odometry, self.sensor_callback)
        # costmap subscriber
        map_sub = rospy.Subscriber(
            "/move_base_node/local_costmap/costmap", OccupancyGrid, self.map_callback)
        # local plan subscriber
        teb_sub = rospy.Subscriber(
            "/teb_planner/teb_poses", PoseArray, self.teb_callback)

        # control effort subscriber
        vel_ctrl_effort_sub = message_filters.Subscriber(
            "/vel/control_effort", Float64)
        ang_ctrl_effort_sub = message_filters.Subscriber(
            "/ang/control_effort", Float64)
        cs = message_filters.ApproximateTimeSynchronizer(
            [vel_ctrl_effort_sub, ang_ctrl_effort_sub], queue_size=10, slop=0.01, allow_headerless=True)
        cs.registerCallback(self.controller_callback)

        # setpoint publisher
        self.vel_setpoint_pub = rospy.Publisher(
            '/vel/setpoint', Float64, queue_size=1)
        self.ang_setpoint_pub = rospy.Publisher(
            '/ang/setpoint', Float64, queue_size=1)
        # state publisher
        self.vel_state_pub = rospy.Publisher(
            '/vel/state', Float64, queue_size=1)
        self.ang_state_pub = rospy.Publisher(
            '/ang/state', Float64, queue_size=1)

        # velocity command publisher
        self.cmd_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)

        # marker publisher
        self.marker_pub = rospy.Publisher(
            '/carla/debug_marker', MarkerArray, queue_size=1)

        print("Controller is ready. Waiting for incoming data..")

        while self.marker_pub.get_num_connections() < 1:
            continue

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.mk_msg)  # publish marker messages to carla client at 10hz
            rate.sleep()

    def plan_callback(self, plan):
        self.enable = True
        self.way_points = []
        for float_list in plan.lists:
            self.way_points.append(float_list.elements)

    def sensor_callback(self, odom):
        # retrieve current position
        self.pos = odom.pose.pose

    def map_callback(self, map):
        if not self.enable:
            return

        # select waypoints from global plan
        if self.way_points:
            # current set of waypoints
            length = min(5, len(self.way_points))
            selected_waypoints = self.way_points[:length]
        else:
            # arrived at final goal
            print("Goal has been reached")
            vel_msg = CarlaEgoVehicleControl()
            vel_msg.throttle = 0
            vel_msg.steer = 0
            self.cmd_pub.publish(vel_msg)
            self.enable = False
            return

        dst = math.sqrt(math.pow((selected_waypoints[-1][0] - self.pos.position.x), 2) +
                        math.pow((selected_waypoints[-1][1] - self.pos.position.y), 2))
        if dst < 1:
            # arrived at the last waypoint in this set, remove the whole set from global waypoints
            print("Waypoint (" + str(selected_waypoints[-1][0]) +
                  ", " + str(selected_waypoints[-1][1]) + ") is achieved!\n\n")
            self.way_points = self.way_points[length:]
            return

        # compute local plan by TEB
        local_planner = TEB(selected_waypoints, map)
        local_planner.plan()

        return

    def teb_callback(self, teb_poses):
        if not self.enable:
            return

        local_waypoints = teb_poses.poses

        if not local_waypoints:
            print("TEB Planner failed to compute a path..")
            return

        # message templates
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.scale.x, marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.05, 0.0, 1.0, 0.0, 1.0
        marker.lifetime = rospy.Duration(secs=0.12)

        # create messages sent to the carla client for debugging
        for i in range(len(local_waypoints)):
            point = Point()
            point.x, point.y, point.z = float(local_waypoints[i].position.x), float(local_waypoints[i].position.y), 0.5
            marker.points.append(point)
        # prepare messages
        marker_array.markers.append(marker)
        self.mk_msg = marker_array

        # publish state and setpoint
        self.vel_state_pub.publish(0)
        self.ang_state_pub.publish(0)
        for i in range(len(local_waypoints)):
            if self.euclidean_distance((self.pos.position.x, self.pos.position.y),
                                       (local_waypoints[i].position.x, local_waypoints[i].position.y)) >= 1:
                # translate local waypoint to ego_vehicle coordinates
                pt = PointStamped()
                pt.header.stamp = rospy.Time(0)
                pt.header.frame_id = "map"
                pt.point.x, pt.point.y, pt.point.z = local_waypoints[i].position.x, local_waypoints[i].position.y, 0
                pt_ego = self.tf_buffer.transform(pt, "ego_vehicle", timeout=rospy.Duration(10))
                self.vel_setpoint_pub.publish(float(pt_ego.point.x))
                self.ang_setpoint_pub.publish(float(pt_ego.point.y))
                print("forward difference: "+str(float(pt_ego.point.x)))
                print("side difference: " + str(float(pt_ego.point.y)))
                return

    def controller_callback(self, vel, ang):
        if not self.enable:
            return

        # publish cmd_vel
        vel_msg = CarlaEgoVehicleControl()
        vel_msg.throttle = vel.data
        vel_msg.steer = ang.data
        print("Control effort, linear: " + str(vel.data) + ", angular: " + str(ang.data))
        print("left velocity: "+str((vel.data - ang.data * 4) * 1000)+", right velocity: "+str((vel.data + ang.data * 4) * 1000))
        self.cmd_pub.publish(vel_msg)

        return

    def euclidean_distance(self, start, goal):
        return math.sqrt(math.pow((goal[0] - start[0]), 2) + math.pow((goal[1] - start[1]), 2))

if __name__ == '__main__':
    base = Base()
    base.run()
    
