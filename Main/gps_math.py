from math import atan2, pi

def angle_to_goal(delta_x, delta_y):
  angle =  atan2(delta_x, delta_y) * 180 / pi
  angle += 360 if angle < 0 else 0
  return angle

def get_delta_x_and_y_from_robot_to_goal(robot_x, robot_y, goal_x, goal_y):
  return goal_x - robot_x, goal_y - robot_y

d_x, d_y = get_delta_x_and_y_from_robot_to_goal(1, 1.5, -1, -1)
print(d_x, d_y, angle_to_goal(d_x, d_y))