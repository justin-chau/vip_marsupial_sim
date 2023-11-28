#!/usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import MapMetaData, OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Vector3, Point, PoseStamped
from rrt import RRT, RRTVertex
import numpy as np
from typing import Optional
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def create_sphere_marker(position: Point = Point(x=0, y=0), color: ColorRGBA = ColorRGBA(r=1, a=1)) -> Marker:
  marker: Marker = Marker()
  marker.header.frame_id = 'map'
  marker.type = Marker.SPHERE
  marker.pose.position = position
  marker.pose.orientation = Quaternion(w=1)
  marker.color = color
  marker.scale = Vector3(x=0.1, y=0.1, z=0.1)

  return marker

if __name__ == '__main__':
  rospy.init_node('multi_planner')

  mother_costmap: OccupancyGrid = rospy.wait_for_message('/mother_inflation/costmap/costmap', OccupancyGrid)
  baby_costmap: OccupancyGrid = rospy.wait_for_message('/baby_inflation/costmap/costmap', OccupancyGrid)
  map_metadata: MapMetaData = mother_costmap.info

  min_x = map_metadata.origin.position.x
  min_y = map_metadata.origin.position.y

  max_x = min_x + map_metadata.resolution * map_metadata.height
  max_y = min_y + map_metadata.resolution * map_metadata.width

  start = Point(x=0, y=0)
  goal = Point(x=4, y=-2)

  forward_rrt = RRT(start=(start.x, start.y), goal=(goal.x, goal.y), x_range=(min_x, max_x), y_range=(min_y, max_y), occupancy_grid=mother_costmap, delta=0.2)

  reverse_rrt = RRT(start=(goal.x, goal.y), goal=(start.x, start.y), x_range=(min_x, max_x), y_range=(min_y, max_y), occupancy_grid=baby_costmap, delta=0.2)

  start_marker_publisher = rospy.Publisher('/multi_planner/visualization/start', Marker, queue_size=10, latch=True)
  goal_marker_publisher = rospy.Publisher('/multi_planner/visualization/goal', Marker, queue_size=10, latch=True)

  forward_rrt_publisher = rospy.Publisher('/multi_planner/visualization/forward_rrt', Marker, queue_size=10, latch=True)
  reverse_rrt_publisher = rospy.Publisher('/multi_planner/visualization/reverse_rrt', Marker, queue_size=10, latch=True)

  exchange_point_publisher = rospy.Publisher('/multi_planner/visualization/exchange_point', Marker, queue_size=10, latch=True)

  checked_map_publisher = rospy.Publisher('/multi_planner/visualization/checked_map', OccupancyGrid, queue_size=10, latch=True)

  move_base_client = actionlib.SimpleActionClient('/mother/move_base', MoveBaseAction)
  move_base_client.wait_for_server()

  start_marker = create_sphere_marker(start)
  goal_marker = create_sphere_marker(goal, color=ColorRGBA(g=1, a=1))
  
  exchange_point: Optional[Point] = None
  best_exchange_goal_distance = np.inf

  start_marker_publisher.publish(start_marker)
  goal_marker_publisher.publish(goal_marker)
  

  current_rrt = forward_rrt

  for i in range(10000):
    random_vertex = current_rrt.random_vertex()
    stepped_vertex = current_rrt.extend(random_vertex)
    
    connected_vertex: Optional[RRTVertex] = None

    if stepped_vertex is not None:

      if current_rrt is forward_rrt:
        connected_vertex = reverse_rrt.connect(stepped_vertex)
        current_rrt = reverse_rrt
      else:
        connected_vertex = forward_rrt.connect(stepped_vertex)
        current_rrt = forward_rrt
      
      
      if connected_vertex is not None and connected_vertex == stepped_vertex and connected_vertex.distance(RRTVertex([goal.x, goal.y])) < best_exchange_goal_distance:
        best_exchange_goal_distance = connected_vertex.distance(RRTVertex([goal.x, goal.y]))
        exchange_point = Point(x=connected_vertex.point[0], y=connected_vertex.point[1])

    forward_rrt_publisher.publish(forward_rrt.get_marker_visualization(color=ColorRGBA(r=1, a=1)))
    reverse_rrt_publisher.publish(reverse_rrt.get_marker_visualization(color=ColorRGBA(g=1, a=1)))

    if exchange_point is not None:
      exchange_point_publisher.publish(create_sphere_marker(exchange_point, color=ColorRGBA(b=1, a=1)))

  
  if exchange_point is not None:
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose = PoseStamped()
    move_base_goal.target_pose.header.frame_id = 'map'
    move_base_goal.target_pose.pose.position = exchange_point
    move_base_goal.target_pose.pose.orientation = Quaternion(w=1)

    move_base_client.send_goal_and_wait(move_base_goal)