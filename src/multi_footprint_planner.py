#!/usr/bin/env python3

import rospy
from std_msgs.msg import ColorRGBA
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import MapMetaData
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Vector3, Point
from rrt import RRT

def create_sphere_marker(position: Point = Point(x=0, y=0), color: ColorRGBA = ColorRGBA(r=1, a=1)) -> Marker:
  marker: Marker = Marker()
  marker.header.frame_id = 'map'
  marker.type = Marker.SPHERE
  marker.pose.position = position
  marker.pose.orientation = Quaternion(w=1)
  marker.color = color
  marker.scale = Vector3(x=0.25, y=0.25, z=0.25)

  return marker

if __name__ == '__main__':
  rospy.init_node('multi_planner')
  
  rate = rospy.Rate(100)
  
  get_global_map = rospy.ServiceProxy('/static_map', GetMap)
  global_map: GetMapResponse = get_global_map()
  map_metadata: MapMetaData = global_map.map.info

  min_x = map_metadata.origin.position.x
  min_y = map_metadata.origin.position.y

  max_x = min_x + map_metadata.resolution * map_metadata.height
  max_y = min_y + map_metadata.resolution * map_metadata.width

  start = Point(x=0, y=0)
  goal = Point(x=4, y=-2)

  forward_rrt = RRT(start=(start.x, start.y), goal=(goal.x, goal.y), x_range=(min_x, max_x), y_range=(min_y, max_y), occupancy_grid= global_map.map, delta=0.2)

  reverse_rrt = RRT(start=(goal.x, goal.y), goal= (start.x, start.y), x_range=(min_x, max_x), y_range=(min_y, max_y), occupancy_grid= global_map.map, delta=0.2)

  start_marker_publisher = rospy.Publisher('/multi_planner/visualization/start', Marker, queue_size=10, latch=True)
  goal_marker_publisher = rospy.Publisher('/multi_planner/visualization/goal', Marker, queue_size=10, latch=True)

  forward_rrt_publisher = rospy.Publisher('/multi_planner/visualization/forward_rrt', Marker, queue_size=10, latch=True)
  reverse_rrt_publisher = rospy.Publisher('/multi_planner/visualization/reverse_rrt', Marker, queue_size=10, latch=True)

  start_marker = create_sphere_marker(start)
  goal_marker = create_sphere_marker(goal, color=ColorRGBA(g=1, a=1))

  start_marker_publisher.publish(start_marker)
  goal_marker_publisher.publish(goal_marker)

  while True:
    forward_rrt.random_extend()
    reverse_rrt.random_extend()

    forward_rrt_publisher.publish(forward_rrt.get_marker_visualization(color=ColorRGBA(r=1, a=1)))
    reverse_rrt_publisher.publish(reverse_rrt.get_marker_visualization(color=ColorRGBA(g=1, a=1)))

    rate.sleep()


