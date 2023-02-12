from math import pi, sqrt
import matplotlib.pyplot as plt
import numpy as np


global red_cartridge_rps
global max_torque_on_motor_w_red_gear_cartridge

RED_CARTRIDGE_RPS = 100 / 60

MAX_TORQUE_ON_MOTOR_W_RED_GEAR_CARTRIDGE = 0.247821

PERCENT_TO_REAL_NUMBER = 100

THEORETICAL_MAX_SPEED_OF_MOTORS_UNDER_NO_LOAD = 125

motor_count_N = 4
drivetrain_rps = 200 / 60
wheel_radius_IN = 2
robot_weight_LB = 22

coefficient_of_friction = 0.5

def find_max_speed_as_a_percentage(robot_weight_LB, wheel_radius_IN, drivetrain_rps):
  # Compute the max speed as a percentage of 100

  max_speed_as_a_percentage = 125 - sqrt(2) * coefficient_of_friction * (wheel_radius_IN * drivetrain_rps * robot_weight_LB / (MAX_TORQUE_ON_MOTOR_W_RED_GEAR_CARTRIDGE * RED_CARTRIDGE_RPS * motor_count_N))

  # Clamp values between [0, 100] percent
  max_speed_as_a_percentage = max(min(max_speed_as_a_percentage, 100), 0)

  return max_speed_as_a_percentage

def compute_speed(robot_weight_LB, wheel_radius_IN, drivetrain_rps):
  # Compute the max speed as a percentage of 100

  max_speed_as_a_percentage = 125 - coefficient_of_friction * (wheel_radius_IN * drivetrain_rps * robot_weight_LB / (MAX_TORQUE_ON_MOTOR_W_RED_GEAR_CARTRIDGE * RED_CARTRIDGE_RPS * motor_count_N))

  # Clamp values between [0, 100] percent
  max_speed_as_a_percentage = max(min(max_speed_as_a_percentage, 100), 0)

  speed = 2 * pi / PERCENT_TO_REAL_NUMBER * drivetrain_rps * wheel_radius_IN * max_speed_as_a_percentage
  return speed


weights_of_robot = np.linspace(10, 22, 100)
radius_of_wheels = np.array([2.75/2, 3.25/2, 4/2])


print(compute_speed(17, 3.25/2,drivetrain_rps) / compute_speed(22, 4/2,drivetrain_rps))

compute_speed = np.vectorize(compute_speed)

x_data = [weights_of_robot] * len(radius_of_wheels)
y_data = np.array([radius_of_wheels] * len(weights_of_robot)).transpose()
z_data = []
for radius in radius_of_wheels:
  z_data.append(compute_speed(weights_of_robot, radius, drivetrain_rps))
  plt.plot(weights_of_robot, compute_speed(weights_of_robot, radius, drivetrain_rps))

z_data = np.transpose(z_data)
plt.plot(robot_weight_LB, compute_speed(robot_weight_LB, wheel_radius_IN, drivetrain_rps), marker="x", label="Current Weight")

plt.title("Max Speed Of Motors As A Percentage Vs. Robot Weight")
plt.legend()
plt.show()

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.contour(x_data, y_data, z_data.transpose(), 50, cmap='binary')
ax.set_xlabel('x')
ax.set_ylabel('y')
plt.show()