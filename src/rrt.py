from __future__ import annotations

from typing import Tuple, List, Optional
import numpy as np
from numpy.typing import *
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Quaternion, Vector3, Point
from nav_msgs.msg import OccupancyGrid

def bresenham_line(x0, y0, x1, y1) -> List[Tuple[int, int]]:
    # Bresenham's line algorithm in Python
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x, y))
    return cells

class RRTVertex:
  def __init__(self, point: Tuple[float, float], parent: Optional[RRTVertex] = None) -> None:
    self.point: NDArray = np.array(point)
    self.parent = parent

  def __eq__(self, other: object) -> bool:
    return isinstance(other, RRTVertex) and self.point[0] == other.point[0] and self.point[1] == other.point[1]
  
  def __ne__(self, other: object) -> bool:
    return not self.__eq__(other)

  def distance(self, other: RRTVertex):
    return np.linalg.norm(self.point - other.point)

class RRT:
  def __init__(self, start: Tuple[float, float], goal: Tuple[float, float], x_range: Tuple[float, float], y_range: Tuple[float, float], delta: float, occupancy_grid: OccupancyGrid, goal_bias: float = 0.2) -> None:
    self.start: RRTVertex = RRTVertex(start)
    self.goal: RRTVertex = RRTVertex(goal)
    self.x_range = x_range
    self.y_range = y_range
    self.delta = delta
    self.occupancy_grid = occupancy_grid
    self.goal_bias = goal_bias
    self.vertices: List[RRTVertex] = [self.start]

  def random_vertex(self) -> RRTVertex:
    if np.random.random() < self.goal_bias:
      return RRTVertex(self.goal.point)
    else:
      return RRTVertex((np.random.uniform(self.x_range[0], self.x_range[1]), np.random.uniform(self.y_range[0], self.y_range[1])))
  
  def nearest_neighbor(self, new_vertex: RRTVertex) -> RRTVertex:
    nearest_vertex = self.vertices[0]
    nearest_distance = np.inf

    for vertex in self.vertices:
      distance = vertex.distance(new_vertex)

      if distance < nearest_distance:
        nearest_distance = distance
        nearest_vertex = vertex

    return nearest_vertex
  
  def get_occupancy_data_position(self, grid_cell: Tuple[int, int]):
    return grid_cell[1] * self.occupancy_grid.info.width + grid_cell[0]

  def get_occupancy_data(self, grid_cell: Tuple[int, int]):
    return self.occupancy_grid.data[self.get_occupancy_data_position(grid_cell)]
  
  def world_to_grid(self, point: Tuple[float, float]) -> Tuple[int, int]:
    grid_x = np.floor((point[0] - self.occupancy_grid.info.origin.position.x) / self.occupancy_grid.info.resolution)
    grid_y = np.floor((point[1] - self.occupancy_grid.info.origin.position.y) / self.occupancy_grid.info.resolution)
    return (int(grid_x), int(grid_y))
  
  def has_edge_collision(self, v0: RRTVertex, v1: RRTVertex):
    v0_grid = self.world_to_grid(v0.point)
    v1_grid = self.world_to_grid(v1.point)

    path_cells = bresenham_line(v0_grid[0], v0_grid[1], v1_grid[0], v1_grid[1])

    for cell in path_cells:
      if self.get_occupancy_data(cell) != 0:
        return True

    return False
  
  def step_vertex(self, nearest_neighbor: RRTVertex, vertex: RRTVertex) -> Optional[RRTVertex]:
    if nearest_neighbor.distance(vertex) <= self.delta and not self.has_edge_collision(nearest_neighbor, vertex):
      return vertex
    
    else:
      direction_vector = vertex.point - nearest_neighbor.point
      direction_vector_norm = np.linalg.norm(direction_vector)

      if direction_vector_norm == 0:
        return None

      normalized_direction_vector = direction_vector / direction_vector_norm
      new_point = (normalized_direction_vector * self.delta) + nearest_neighbor.point
      new_vertex = RRTVertex(new_point)

      if new_point[0] >= self.x_range[0] and new_point[0] <= self.x_range[1] and new_point[1] >= self.y_range[0] and new_point[1] <= self.y_range[1] and not self.has_edge_collision(nearest_neighbor, new_vertex):
        return new_vertex
      
      else:
        return None

  def extend(self, vertex: RRTVertex) -> RRTVertex:
    nearest_neighbor = self.nearest_neighbor(vertex)
    stepped_vertex = self.step_vertex(nearest_neighbor, vertex)

    if stepped_vertex is not None:
      stepped_vertex.parent = nearest_neighbor
      self.vertices.append(stepped_vertex)

      return stepped_vertex

    return None

  def connect(self, vertex: RRTVertex) -> Optional[bool]:
    vertex_copy = RRTVertex([vertex.point[0], vertex.point[1]])

    stepped_vertex = self.extend(vertex_copy)

    while stepped_vertex != vertex and stepped_vertex is not None:
      stepped_vertex = self.extend(vertex_copy)

    return stepped_vertex

  def get_marker_visualization(self, color: ColorRGBA) -> Marker:
    tree_marker = Marker()
    tree_marker.header.frame_id = 'map'
    tree_marker.type = Marker.LINE_LIST
    tree_marker.pose.orientation = Quaternion(w=1)
    tree_marker.color = color
    tree_marker.scale = Vector3(x=0.01)

    for vertex in self.vertices:
      if vertex.parent is not None:
        
        tree_marker.points.append(Point(x=vertex.point[0], y=vertex.point[1]))
        tree_marker.points.append(Point(x=vertex.parent.point[0], y=vertex.parent.point[1]))

    return tree_marker