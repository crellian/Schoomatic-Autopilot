#!/usr/bin/env python

import rospy

from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path


def publish_obstacle_msg(grid_map):
    obstacle_msg = ObstacleArrayMsg()
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "map"

    # Add point obstacles
    cnt = 0
    height, width = grid_map.info.height, grid_map.info.width
    res = grid_map.info.resolution
    ori_x, ori_y = grid_map.info.origin.position.x, grid_map.info.origin.position.y
    for i in range(0, height):
        for j in range(0, width):
            if grid_map.data[i*width + j] > 0:
                obstacle_msg.obstacles.append(ObstacleMsg())
                obstacle_msg.obstacles[cnt].id = cnt
                obstacle_msg.obstacles[cnt].radius = res
                obstacle_msg.obstacles[cnt].polygon.points = [Point32()]

                obstacle_msg.obstacles[cnt].polygon.points[0].x = ori_x+res*j
                obstacle_msg.obstacles[cnt].polygon.points[0].y = ori_y+res*i
                obstacle_msg.obstacles[cnt].polygon.points[0].z = 0

                cnt += 1
    obs_pub = rospy.Publisher('/teb_planner/obstacles', ObstacleArrayMsg, queue_size=1)

    while obs_pub.get_num_connections() < 1:
        continue

    obs_pub.publish(obstacle_msg)

    return

def publish_waypoinys_msg(waypoints):
    via_points_msg = Path()
    via_points_msg.header.stamp = rospy.Time.now()
    via_points_msg.header.frame_id = "map"

    for waypoint in waypoints:
        # Add via-points
        point = PoseStamped()
        point.pose.position.x = waypoint[0]
        point.pose.position.y = waypoint[1]

        via_points_msg.poses.append(point)

    wp_pub = rospy.Publisher('/teb_planner/via_points', Path, queue_size=1)
    while wp_pub.get_num_connections() < 1:
        continue
    wp_pub.publish(via_points_msg)

    return

class TEB:
    def __init__(self, waypoints, map):
        self.waypoints = waypoints
        self.grid_map = map

    def plan(self):
        publish_obstacle_msg(self.grid_map)
        publish_waypoinys_msg(self.waypoints)

        return

