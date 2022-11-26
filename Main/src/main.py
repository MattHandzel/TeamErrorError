# region VEXcode Generated Robot Configuration
from math import sin, cos
import math
import time
from vex import *
import random
import vex
import sys

# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
controller_2 = Controller(PARTNER)

left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(
    left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
flywheel_motor_1 = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
flywheel_motor_2 = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)

led_a = Led(brain.three_wire_port.a)
inertial = Inertial(Ports.PORT9)
index_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
roller_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
intake_motor = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
roller_optical = Optical(Ports.PORT16)

indexer = Pneumatics(brain.three_wire_port.a)
expansion = Pneumatics(brain.three_wire_port.b)



gps = Gps(Ports.PORT3)

global g
g = -9.81

global RAD_TO_DEG
global DEG_TO_RAD
RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

global r2o2
r2o2 = math.sqrt(2) / 2

# wait for rotation sensor to fully initialize
wait(30, MSEC)

def f(*args):
    message = ""
    for arg in args:
        if type(arg) != str:
            arg = str(arg)
        message += " " + arg
    return message

def sign(num):
    '''
    Returns the sign of the number
    '''
    if num == 0:
        return 1.0
    return abs(num) / num


def clamp(num, _max, _min):
    '''
    Clamps the number between the max and min
    '''
    return max(min(num, _max), _min)


def rotate_vector_2d(x, y, theta):
    '''
    Rotates a vector by theta degrees
    '''
    x_old = x
    x = x * math.cos(theta) - y * math.sin(theta)
    y = x_old * math.sin(theta) + y * math.cos(theta)

    return x,y 

class Vector:
    '''
    Vector class I wrote because basic python lists are lame
    '''

    def __init__(self, data):
        self.data = data

    def __add__(self, other):
        assert len(other) == len(self.data)
        return [other[i] + self.data[i] for i in range(len(self.data))]

    def __sub__(self, other):
        assert len(other) == len(self.data)
        return Vector([other[i] - self.data[i] for i in range(len(self.data))])

    def __getitem__(self, key):
        return self.data[key]

    def __len__(self):
        return len(self.data)

    def __repr__(self):
        return "Vector:\t" + repr(self.data)

class PID:
    '''
    Your standard PID controller (look up on wikipedia if you don't know what it is)
    '''
    previous_value = None

    integral_error = 0
    derivative_error = 0
    proportional_error = 0

    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def update(self, _value, delta_time=None):
        '''
        Updates the PID controller with the new value, optional delta_time parameter means you can use this for non-constant time steps
        '''
        if delta_time != None:
            self.integral_error += _value * delta_time
        else:
            self.integral_error += _value

        if self.previous_value != None:
            # Compute derivative term
            self.derivative_error = _value - self.previous_value

        self.previous_value = _value

        return _value * self.kP + self.integral_error * self.kI + self.derivative_error * self.kD

    def set_constants(self, kP, kI, kD):
        '''
        Updates the constants of the PID controller
        '''
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def reset(self):
        '''
        Resets the PID controller
        '''
        self.integral_error = 0
        self.previous_value = None

class TargetPoint:
    '''
    This is the target point class, it can be used for PID controller/path following in the future (i haven't done it yet),
    but how it intends to work is to have the robot go to a list of target points, if it is within a threshold then it is allowed
    to go to the next target point
    '''

    def __init__(self, _x, _y, _total_tolerance_or_just_x_tolerance, _tolerance_y=-1):
        self.x = _x
        self.y = _y
        if _tolerance_y == -1:
            self.tolerance_x = _total_tolerance_or_just_x_tolerance
            self.tolerance_y = _total_tolerance_or_just_x_tolerance
        else:
            self.tolerance_x = _total_tolerance_or_just_x_tolerance
            self.tolerance_y = _tolerance_y

    def check_within(self, _x, _y):
        return abs(self.x - _x) <= self.tolerance_x and abs(self.y - _y) <= self.tolerance_y


class GameObject:
    '''
    If we want to introduce game object (like say the goal or barriers), we have have to robot look up important information about the game object
    '''

    def __init__(self, x_pos, y_pos):
        self.x_pos = x_pos
        self.y_pos = y_pos

##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###

class Robot(GameObject):
    '''
    This is the big-boy class, this is the robot class, this is the class that controls the robot, there is a lot of stuff here
    '''
    theta_shooter: float = 20
    theta_robot: float = 0
    length: float = 38.1

    # Set the offset for the flywheel from the center of the robot
    flywheel_offset_x = 0   
    flywheel_offset_y = 0

    flywheel_angle = 45 * DEG_TO_RAD

    previous_update_time: float = 0

    wheel_gear_ratio = 18

    wheel_max_rpm: float = 200
    wheel_diameter_CM: float = 10.5

    # In order to get this, it is ticks for the specific gear ratio we're using divided by the circumeference of our wheel
    wheel_distance_CM_to_TICK_coefficient: float = (wheel_gear_ratio / 6 * 300) / \
        (math.pi * wheel_diameter_CM)

    max_velocity: float
    max_acceleration: float

    total_theta = 0

    # PID controller constants
    motor_PID_kP = 10
    motor_PID_kI = 5
    motor_PID_kD = 0
    
    # Init the PID controllers for the robot
    left_motor_a_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    right_motor_a_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    left_motor_b_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    right_motor_b_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    
    pos_PID_kP = 0.1
    pos_PID_kI = 0
    pos_PID_kD = 0
    x_pos_PID = PID(pos_PID_kP,pos_PID_kI,pos_PID_kD)
    y_pos_PID = PID(pos_PID_kP,pos_PID_kI,pos_PID_kD)
    
    rotation_PID_kP = 0.1
    rotation_PID_kI = 0
    rotation_PID_kD = 0
    rotation_PID = PID(rotation_PID_kP,rotation_PID_kI,rotation_PID_kD)

    previous_x_from_encoders = 0
    previous_y_from_encoders = 0

    # used to keep track of time in auto and driver mode respectively, use it for nicely logging data, can be used during either modes for end game/pathfinding rules
    autonomous_timer = Timer()
    driver_controlled_timer = Timer()

    previous_x_from_gps = 0
    previous_y_from_gps = 0

    total_x_from_encoders = 0
    total_y_from_encoders = 0

    flywheel_motor_1_PID = PID(2, 0, 0)
    flywheel_motor_2_PID = PID(2, 0, 0)

    flywheel_speed = 0

    delta_time = 0

    target_reached = False
    position_tolerance = 3 # tolerance to target position in cm
    orientation_tolerance = 8 # tolerance to target orientation in degrees
    
    red_goal = GameObject(0, 100)
    blue_goal = GameObject(100, 0)
    # State dictionary will hold ALL information about the robot
    '''
    '''
    state = {
        # Orientation
        "x_pos" : 0,
        "y_pos" : 0,
        "x_vel" : 0,
        "y_vel" : 0,
        "x_gps" : 0,
        "y_gps" : 0,
        "x_enc" : 0,
        "y_enc" : 0,
        "theta" : 0,
        "theta_vel" : 0,

        "time" : 0,
        # Actuators
        "flywheel_speed" : 0,
        "intake_speed" : 0,

        # Commands
        "using_gps" : False,
        "is_shooting": False,
        "slow_mode" : False,
        "drone_mode" : False,
        "autonomous" : False
    }

    target_state = {}

    x_vel_PID = PID(0.1, 0, 0)
    y_vel_PID = PID(0.1, 0, 0)
    theta_vel_PID = PID(0.1, 0, 0)

    def __init__(self, x_pos=0, y_pos=0, theta=0):
        '''
        Initializes the robot class, computes the max velocity and acceleration
        params -
            x_pos - set the robot's x position
            y_pos - set the robot's y position
            theta - set the robot's theta
        '''
        super().__init__(x_pos, y_pos)
        self.theta = theta

        # what our max velocity "should" be (can go higher or lower)
        self.max_velocity = ((self.wheel_max_rpm / 60) *
                             math.pi * 2 * self.wheel_diameter_CM / math.sqrt(2))
        # This number in the divisor means it will speedup/slow down in that many seeconds
        self.max_acceleration = 2 * self.max_velocity / 0.05

        # Set origin of the gps
        gps.set_origin(0,0)

        self.set_target_state(self.state)
        self.previous_state = self.state
    
    def init(self):
        self.set_target_state(self.state)
        self.previous_state = self.state
        
    

    def update(self):
        '''
        This is a VERY important function, it should be called once every [0.1 - 0.01] seconds (the faster the better, especially for controls)
        It updates the robot's position based off of the encoders and the gyro
        '''

        self.delta_time = (getattr(time, "ticks_ms")() / 1000) - (self.previous_state["time"])

        # prevent divide by zero
        if self.delta_time == 0:
            return
        
        # Update the previous state before doing state estimation
        self.previous_state = self.state.copy()
        self.estimate_state()


        self.position_update()
        self.flywheel_update()
        # self.intake_update()

    
    def position_update(self):
        '''
        Updates the position of the robot
        '''

        # These are the target position for each wheel, you find the target positions by finding the direction the wheels
        # are pointing in and then multiplying by the sin(theta) for the x and cos(theta) for theta
        # Speed towards target will be controlled by PID loop
        delta_x = self.target_state["x_pos"] - self.x_pos
        delta_y = self.target_state["y_pos"] - self.y_pos
        delta_theta = self.target_state["theta"] - self.theta

        # Turn via the shortest path
        if delta_theta > 180:
            delta_theta -= 360
        elif delta_theta < -180:
            delta_theta += 360

        # target_x_vel = self.target_state["x_vel"]
        # target_y_vel = self.target_state["y_vel"]
        # target_theta_vel = self.target_state["theta_vel"]

        delta_theta = math.sqrt(abs(delta_theta)) * sign(delta_theta) 
        target_x_vel = ((clamp(delta_x, self.position_tolerance, -self.position_tolerance) * 100 / self.position_tolerance) ** 2) / 100 * sign(delta_x)
        target_y_vel = ((clamp(delta_y, self.position_tolerance, -self.position_tolerance) * 100 / self.position_tolerance) ** 2) / 100 * sign(delta_y)
        target_theta_vel = ((clamp(delta_theta, self.orientation_tolerance, -self.orientation_tolerance) * 100 / self.orientation_tolerance) ** 2) / 100 * sign(delta_theta)
        
        # target_theta_vel = clamp(target_theta_vel, 100, -100)
        # target_x_vel = self.x_vel_PID.update(delta_x)
        # target_y_vel = self.y_vel_PID.update(delta_y)
        # target_theta_vel = self.theta_vel_PID.update(delta_theta)
        
        # target_x_vel = clamp(target_x_vel,100, -100)
        # target_y_vel = clamp(target_y_vel,100, -100) 
        # target_theta_vel = clamp(target_theta_vel,100, -100)

        # print("target_x_vel: ", target_x_vel, "target_y_vel: ", target_y_vel, "target_theta_vel: ", target_theta_vel)
        
        # # If we're close enough to the target, stop moving
        # if abs(delta_x) <= self.position_tolerance or abs(delta_y) <= self.position_tolerance:
        #     self.drive(0,0,self.target_state["theta_vel"])
        #     return

        # # Prevent divide by 0 error
        # theta_to_target = 0
        # if delta_x != 0:
        #     theta_to_target = math.atan(delta_y / delta_x)
            
        #     # If theta_to_target is less than 0, then add 180 degrees
        #     theta_to_target += math.pi * (delta_x < 0)

        # Spin the motors (in case they were stopped before, might be able to be ommitted later)
        left_motor_a.spin(FORWARD)
        left_motor_b.spin(FORWARD)
        right_motor_a.spin(FORWARD)
        right_motor_b.spin(FORWARD)

        # Cool driving toggle (basically you rotate the target direction vector based on)
        # the robots heading (https://stackoverflow.com/questions/14607640/rotating-a-vector-in-3d-space)
        if self.drone_mode:
            target_x_vel, target_y_vel = rotate_vector_2d(target_x_vel, target_y_vel, self.theta * DEG_TO_RAD)

        if self.slow_mode:
            target_x_vel = target_x_vel / 4
            target_y_vel = target_y_vel / 4
            target_theta_vel = target_theta_vel / 4

        # HAVE DAVID TRY THIS OUT (sqrt the input might be better for da vid)
        # x_vector = (abs(_x_vector) ** (0.5)) * 10 * sign(_x_vector)
        # y_vector = (abs(_y_vector) ** (0.5)) * 10 * sign(_y_vector)
        # r_vector = (abs(_r_vector) ** (0.5)) * 10 * sign(_r_vector)

        # Get the motor powers
        left_motor_a_target_velocity = target_x_vel + target_y_vel + target_theta_vel
        right_motor_a_target_velocity = -target_x_vel + target_y_vel - target_theta_vel
        left_motor_b_target_velocity = -target_x_vel + target_y_vel + target_theta_vel
        right_motor_b_target_velocity = target_x_vel + target_y_vel - target_theta_vel

        # Constant-ish acceleration profile. Basically, our acceleration is constant except for when we are close to starting or stopping (to prevent jerks/slides)
        slow_down_speed_threshold = 5

        max_acceleration_left_motor_a = self.max_acceleration
        max_acceleration_right_motor_a = self.max_acceleration
        max_acceleration_left_motor_b = self.max_acceleration
        max_acceleration_right_motor_b = self.max_acceleration

        # If we aren't moving from a stop to a start or vice versa, decrease acceleration
        if abs(left_motor_a.velocity(PERCENT)) < slow_down_speed_threshold:
            max_acceleration_left_motor_a = self.max_acceleration * 2

        if abs(right_motor_a.velocity(PERCENT)) < slow_down_speed_threshold:
            max_acceleration_right_motor_a = self.max_acceleration * 2

        if abs(left_motor_b.velocity(PERCENT)) < slow_down_speed_threshold:
            max_acceleration_left_motor_b = self.max_acceleration * 2

        if abs(right_motor_b.velocity(PERCENT)) < slow_down_speed_threshold:
            max_acceleration_right_motor_b = self.max_acceleration * 2

        # Okie doki, this line is gonna need some documentation, but first some labels:
        # (self.max_acceleration / self.max_velocity * 100) means the maximum acceleration of our robot IN A PERCENTAGE (i.e. 10 percent per second, 50 percent per second, -10 percent, per second)
        # (self.max_acceleration / self.max_velocity * 100) * self.delta_time is the maximum change in velocity of the robot FOR THIS RUN THROUGH THE LOOP (by multipling by delta time we get how long its been since the last loop, which means we can change our velocity by that change in time time acceleration), Ex. if its been 0.1 seconds since the last loop, then our change in velocity can be 10 percent, but if it was 1 second then our change in velocity can be 100 percent beceause its been such a long time (if you still don't understand then learn what integration is)
        # the goal of the following lines is to clamp the change in velocity to not go over some number (max_acceleration),
        # so in order to do that we clamp motor_target_velocity between the motor_current_velocity plus the maximum change in velocity for this run of the loop, repeat for each wheel
        left_motor_a_target_velocity = clamp(left_motor_a_target_velocity, (max_acceleration_left_motor_a / self.max_velocity * 100) * self.delta_time + left_motor_a.velocity(
            PERCENT), -(max_acceleration_left_motor_a / self.max_velocity * 100) * self.delta_time + left_motor_a.velocity(PERCENT))
        right_motor_a_target_velocity = clamp(right_motor_a_target_velocity, (max_acceleration_right_motor_a / self.max_velocity * 100) * self.delta_time + right_motor_a.velocity(
            PERCENT), -(max_acceleration_right_motor_a / self.max_velocity * 100) * self.delta_time + right_motor_a.velocity(PERCENT))
        left_motor_b_target_velocity = clamp(left_motor_b_target_velocity, (max_acceleration_left_motor_b / self.max_velocity * 100) * self.delta_time + left_motor_b.velocity(
            PERCENT), -(max_acceleration_left_motor_b / self.max_velocity * 100) * self.delta_time + left_motor_b.velocity(PERCENT))
        right_motor_b_target_velocity = clamp(right_motor_b_target_velocity, (max_acceleration_right_motor_b / self.max_velocity * 100) * self.delta_time + right_motor_b.velocity(
            PERCENT), -(max_acceleration_right_motor_b / self.max_velocity * 100) * self.delta_time + right_motor_b.velocity(PERCENT))

        # Accelerate the motors to the target velocity. 
        left_motor_a.set_velocity(left_motor_a_target_velocity, PERCENT)
        right_motor_a.set_velocity(right_motor_a_target_velocity, PERCENT)
        left_motor_b.set_velocity(left_motor_b_target_velocity, PERCENT)
        right_motor_b.set_velocity(right_motor_b_target_velocity, PERCENT)

        # Display content
        # print("left_motor_a_target_velocity ", left_motor_a_target_velocity)
        # print("right_motor_a_target_velocity ", right_motor_a_target_velocity)
        # print("left_motor_b_target_velocity ", left_motor_b_target_velocity)
        # print("right_motor_b_target_velocity ", right_motor_b_target_velocity)
        # print("")
            
    
    def estimate_state(self):
        '''
        Estimates the state of the robot
        '''
        self.state["time"] = getattr(time, "ticks_ms")() / 1000
        self.theta_vel = inertial.gyro_rate(ZAXIS, VelocityUnits.DPS)

        # is a magic number that makes the gyro work better (experimentall I found that when the gyro reported that it spun 1 time, it actually overshot by about 3 degrees)
        self.total_theta += self.theta_vel * self.delta_time / 0.99375 / 2
        self.theta = self.total_theta - (self.total_theta // 360 * 360)
        
        # Use encoders to get our x and y positions so that we can take the derivative and get our velocity
        self.x_enc, self.y_enc = self.get_position_from_encoders()

        delta_x_from_encoders = self.x_enc - self.previous_state["x_enc"]
        delta_y_from_encoders = self.y_enc - self.previous_state["y_enc"]

        # delta_x_from_encoders *= 0.86
        # delta_y_from_encoders *= 0.86 

        delta_x_from_encoders, delta_y_from_encoders = rotate_vector_2d(delta_x_from_encoders, delta_y_from_encoders, -self.theta * DEG_TO_RAD)

        # Get the velocity of the robot in deg/s for 4 wheels
        self.x_vel = delta_x_from_encoders / self.delta_time
        self.y_vel = delta_y_from_encoders / self.delta_time


        # Velocity of robot from gps perspective
        x_from_gps = 0
        y_from_gps = 0
        alpha = 0
        
        if self.using_gps and gps.quality() >= 100: 
            # Update alpha to value that uses gps
            alpha = 0.9
            x_from_gps = gps.x_position(DistanceUnits.CM)
            y_from_gps = gps.y_position(DistanceUnits.CM)

            delta_x_from_gps = gps.x_position(DistanceUnits.CM) - self.previous_x_from_gps 
            delta_y_from_gps = gps.y_position(DistanceUnits.CM) - self.previous_y_from_gps 

            self.previous_x_from_gps = gps.x_position(DistanceUnits.CM)
            self.previous_y_from_gps = gps.y_position(DistanceUnits.CM)
            # print(abs(x_from_gps-self.x_pos), abs(y_from_gps-self.y_pos))
            if abs(x_from_gps-self.x_pos) > 100 or abs(y_from_gps-self.y_pos) > 100:
                self.x_pos = x_from_gps
                self.y_pos = y_from_gps

        # If we have not moved (from the encoders point of view), and the gps is not changing that much, then use the rolling average from the gps
        # If gps is enabled then the low and high pass filter will make the x and y position more stable, if gps is not enabled then the formula won't use gps data (alpha would equal 0)
        self.x_pos += delta_x_from_encoders * (1-alpha) # + (x_from_gps-self.x_pos) * alpha 
        self.y_pos += delta_y_from_encoders * (1-alpha) # + (y_from_gps-self.y_pos) * alpha
        
        self.x_from_gps = x_from_gps
        self.y_from_gps = y_from_gps

        self.total_x_from_encoders += delta_x_from_encoders
        self.total_y_from_encoders += delta_y_from_encoders

        if abs(self.state["x_pos"] - self.target_state["x_pos"]) < self.position_tolerance and abs(self.state["y_pos"] - self.target_state["y_pos"]) < self.position_tolerance:
            # print("TARGET STATE REACHED")
            self.target_reached = True

##### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ###
    def set_target_state(self, _state):
        '''
        Updates the state of the robot
        '''
        # This makes it so that in case the new target state doesn't have a value, instead of just deleting that value, we don't change it
        for key in _state.keys():
            if key in self.state:
                self.target_state[key] = _state[key]
                
                if key == "theta":
                    self.target_state[key] = _state[key] % 360

        self.update_constants()
        self.target_reached = False
    
    def update_constants(self):
        self.drone_mode = self.target_state["drone_mode"]
        self.slow_mode = self.target_state["slow_mode"]
        self.using_gps = self.target_state["using_gps"]

    def print_debug_info(self):
        print("left_motor_a_position:", left_motor_a.position(DEGREES), "left_motor_b_position:", left_motor_b.position(DEGREES), "right_motor_a_position:", right_motor_a.position(DEGREES), "right_motor_b_position:", right_motor_b.position(DEGREES))

    def set_time_to_top_speed(self, _time):
        '''
        Change the maximum acceleration of the robot so that it will reach max velocity in the given time
        '''
        self.max_acceleration = 2 * self.max_velocity / _time

    def reset_motor_positions(self):
        '''
        Resets the motor positions
        '''
        left_motor_a.set_position(0, DEGREES)
        right_motor_a.set_position(0, DEGREES)
        left_motor_b.set_position(0, DEGREES)
        right_motor_b.set_position(0, DEGREES)

    def has_moved(self):
        '''
        Function returns true if the values of the encoders for the motors have
        changed AT ALL
        '''
        return self.x_vel == 0 and self.y_vel == 0

    def set_team(self, _color):
        '''
        Sets the team for the robot so that we can move the rollers to our team color, or shoot disks just into our goal, etc.
        '''
        _color = _color.lower()
        print("INPUT FROM SET_TEAM IS:\t", _color)
        if _color == "red":
            self.notBlue = True
            self.goal = red_goal
            return
        elif _color == "blue":
            self.goal = blue_goal
            self.notBlue = False
            return
        raise Exception("broo you dodn't set a proper team color")

    def reset_theta(self):
        '''
        Resets the theta of the robot to 0
        '''
        self.total_theta = 0

    def stop_moving(self):
        '''
        Stops all of the motors of the robot (this is good for when we want to switch from run_to_position mode to just spin mode (my nomaclature))
        '''
        left_motor_a.stop()
        left_motor_b.stop()
        right_motor_a.stop()
        right_motor_b.stop()

    def print(self, message):
        '''
        Prints a message on the screen and into the console with the current time and mode we're using
        '''
        message = f(" " ,((self.autonomous_timer.time() / 1000)), message)
        if self.autonomous_timer.value() <= 15:
            message = "AUTO" + message
        elif self.driver_controlled_timer.value() < 105:
            message = "DRIVER" + message
        else:
            message = "MATCH_OVER" + message
        print(message)
        # brain.screen.print(message)
        

##### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ###
    def turn_to_roller(self):
        '''
        Turns the robot to the roller, will NOT turn to roller if not in the right quadrant
        '''

        # Compute the angle to the roller based on the robot's current position and what "quadrant" it is in
        x_pos, y_pos = self.get_position()
        roller_angle: float = -1
        if x_pos > 0 and y_pos > 0:
            roller_angle = 180
            # We are in quadrant #1
            if abs(x_pos) < abs(y_pos):
                roller_angle += 90

        if x_pos < 0 and y_pos < 0:
            # We are in quadrant #3
            roller_angle = 0

            if abs(x_pos) > abs(y_pos):
                roller_angle = 90
        
        if roller_angle == -1:
            self.print("Not in a proper quadrant...")
            return

        Thread(self.turn_to_heading,(roller_angle,))

    def get_position_from_encoders(self):
        '''
        Computes the robot's current position from encoders, something that's very cool is that the robot rotating doesn't change this
        '''
        encoders = self.get_current_wheel_encoder_values()

        # Compute the position from the encoders, we get this by summing up the encoders based on if they contribute to the robot moving in one direction
        # and then divide it from the cm/deg coefficient which means that we turn the encoder values into cm, and we divide by 4 because there are 4 wheels
        x_value = (encoders[0] - encoders[1] - encoders[2] + encoders[3]) / (self.wheel_distance_CM_to_TICK_coefficient * r2o2) / 4
        y_value = (encoders[0] + encoders[1] + encoders[2] + encoders[3]) / (self.wheel_distance_CM_to_TICK_coefficient * r2o2) / 4
        return x_value, y_value

    def get_position(self):
        return self.x_pos, self.y_pos
        # return self.get_position_from_encoders()

    # Target position in cm
    def go_to_position(self, _target_x, _target_y, _pid=False, _pidDistanceThreshold=1):
        '''
        Function that will move the robot to a target position
        params: _target_x is the target x position in cm (from the origin)
                _target_y is the target y position in cm (from the origin)
                _pid is a boolean that will determine if the robot will use PID or not
                _pidDistanceThreshold is the distance threshold in cm that the robot uses to determin if it's close enough to the target point
        '''
        # Offset the target position by the robot's current position
        delta_x = _target_x - self.x_pos
        delta_y = _target_y - self.y_pos

        # self.x_pos = _target_x
        # self.y_pos =_target_y

        if not _pid:
            # These are the target position for each wheel, you find the target positions by finding the direction the wheels
            # are pointing in and then multiplying by the sin(theta) for the x and cos(theta) for theta

            x_pos, y_pos = self.get_position()

            while abs(x_pos - _target_x) > 5 or abs(y_pos - _target_y) > 5:
                x_pos, y_pos = self.get_position()

                # Speed towards target will be controlled by PID loop
                error_x = _target_x - x_pos
                error_y = _target_y - y_pos
                
                # Prevent divide by 0 error
                theta_to_target = 0
                if error_x != 0:
                    theta_to_target = math.atan(error_y / (error_x))
                    
                    # If theta_to_target is less than 0, then add 180 degrees
                    theta_to_target += math.pi * (error_x < 0)

                print(x_pos, y_pos, theta_to_target * RAD_TO_DEG)

                self.drive(math.cos(theta_to_target) * 20, math.sin(theta_to_target)
                           * 20, 0, square_input=False)
                wait(0.1, SECONDS)
            print("Done!") 

        else:
            left_motor_a_target_position = delta_x * self.wheel_distance_CM_to_TICK_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_TICK_coefficient * r2o2
            right_motor_a_target_position = -delta_x * self.wheel_distance_CM_to_TICK_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_TICK_coefficient * r2o2
            left_motor_b_target_position = -delta_x * self.wheel_distance_CM_to_TICK_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_TICK_coefficient * r2o2
            right_motor_b_target_position = delta_x * self.wheel_distance_CM_to_TICK_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_TICK_coefficient * r2o2

            # PID loop here for each wheel
            x_pos, y_pos = self.get_position()
            
            # Reset all PID controllers
            self.left_motor_a_PID.reset()
            self.right_motor_a_PID.reset()
            self.left_motor_b_PID.reset()
            self.right_motor_b_PID.reset()

            while abs(x_pos - _target_x) > _pidDistanceThreshold or abs(y_pos - _target_y) > _pidDistanceThreshold:
                x_pos, y_pos = self.get_position()

                # Speed towards target will be controlled by PID loop
                error_x = _target_x - x_pos
                error_y = _target_y - y_pos

                # Prevent divide by 0 error
                theta_to_target = 0
                if error_x != 0:
                    theta_to_target = math.atan(error_y / (error_x))
                    
                    # If theta_to_target is less than 0, then add 180 degrees
                    theta_to_target += math.pi * (error_x < 0)

                # output_x = x_magnitude_PID.update(error_x, self.delta_time)
                # output_y = y_magnitude_PID.update(error_y, self.delta_time)

                # output_magnitude = magnitude_PID.update(
                    # (error_x ** 2 + error_y ** 2) ** (0.5), self.delta_time)

                # self.drive(output_magnitude * cos(theta_to_target),
                        #    output_magnitude * sin(theta_to_target), 0, square_input=False)

                # Gonne be sleeping based off of self.delta time so that this loop doesn't update more frequently then delta_time (to make sure the PID loop's I error term doesn't explode quickly)
                wait(self.delta_time, SECONDS)

            
            print("Exiting PID loop...")

    def forever_update(self, _delay=0.01):
        '''
        This is used in a Thread() so that we run the update function constantly (because its very freaking important)
        '''
        while True:
            self.update()
            wait(_delay, SECONDS)

    def run_autonomous(self, procedure):
        '''
        Given an autonomous procedure, this function will run the procedure
        procedure has very important requirements optional params:
            setX - sets the x position of the robot
            setY - sets the y position of the robot
            setTheta - sets the theta of the robot
            x - move the robot to that x position
            y - move the robot to that y position
            theta - rotate the robot to that theta
            wait - wait for that many seconds
            actions - a list of actions to run
                unload - unload the disc
                load - load the disc
                shoot - shoot the disc
                spin_roller - spin the roller
            message - a message to print to the console and brain (for debugging purposes)

        '''
        # Update loop
        self.update()
        self.autonomous_timer.reset()
        
        for step in procedure:
            target_state = {
                "x_vel" : 10,
                "y_vel" : 10,
            }
            if "setX" in step:
                self.x_pos = step["setX"]
            if "setY" in step:
                self.y_pos = step["setY"]
            # if "actions" in step.keys():
            #     if "unload" in step["actions"]:
            #         r.unload()
            #     if "load" in step["actions"]:
            #         r.load()
            #     if "stop_intake" in step["actions"]:
            #         r.stop_intake()

            #     if "spin_roller" in step["actions"]:
            #         r.spin_roller()

            if "message" in step.keys():
                self.print(step["message"])

            # TODO: add threading here
            if "x" in step.keys() and "y" in step.keys():
                print("Going to position:", step["x"], step["y"])
                target_state["x_pos"] = step["x"]
                target_state["y_pos"] = step["y"]
            
            self.set_target_state(target_state)

            while not self.target_reached:
                self.update()
                wait(0.1, SECONDS)

    def turn_to_heading(self, _heading, _wait=True):
        '''
        Makes the robot turn to a specific heading
        '''
        # There are two ways to approach this, one of them is to compute the positions
        # that the motors need to go to in order to turn the robot (that number can be)
        # calculated by figuring out the circumference that the robot will rotate around
        # and then divide that by the number of degrees, or it can be found experimentally

        # Another way to approach this is to use a PID-esque loop where we check our current heading
        # at each timestep and drive the robot at a specific velocity until the desired heading is reached

        # self.print(f"Turning to heading {_heading}")
        return

    # if you are readign this code and are not me (Matt Handzel) and don't know what this is, then look up python getters and setters
##### PROPERTIES ##### PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES ##### 
    @property
    def position(self):
        return (self.x_pos, self.y_pos)

    @position.setter
    def position(self, _x, _y):
        self.x_pos = _x
        self.y_pos = _y
    @property
    def using_gps(self):
        return self.state["using_gps"]
    
    @using_gps.setter
    def using_gps(self, _using_gps):
        self.state["using_gps"] = _using_gps
    
    @property
    def x_pos(self):
        return self.state["x_pos"]
    
    @x_pos.setter
    def x_pos(self, _x):
        self.state["x_pos"] = _x
    
    @property
    def x_enc(self):
        return self.state["x_enc"]
    
    @x_enc.setter
    def x_enc(self, _x_enc):
        self.state["x_enc"] = _x_enc

    @property
    def y_enc(self):
        return self.state["y_enc"]
    
    @y_enc.setter
    def y_enc(self, _y_enc):
        self.state["y_enc"] = _y_enc

    @property
    def y_pos(self):
        return self.state["y_pos"]
    
    @y_pos.setter
    def y_pos(self, _y):
        self.state["y_pos"] = _y
    
    @property
    def theta(self):
        return self.state["theta"]
    
    @theta.setter
    def theta(self, _theta):
        self.state["theta"] = _theta
    
    @property
    def x_vel(self):
        return self.state["x_vel"]
    
    @x_vel.setter
    def x_vel(self, _x_vel):
        self.state["x_vel"] = _x_vel
    
    @property
    def y_vel(self):
        return self.state["y_vel"]
    
    @y_vel.setter
    def y_vel(self, _y_vel):
        self.state["y_vel"] = _y_vel
    
    @property
    def theta_vel(self):
        return self.state["theta_vel"]
    
    @theta_vel.setter
    def theta_vel(self, _theta_vel):
        self.state["theta_vel"] = _theta_vel
    
    @property
    def x_gps(self):
        return self.state["x_gps"]
    
    @x_gps.setter
    def x_gps(self, _x_gps):
        self.state["x_gps"] = _x_gps
    
    @property
    def y_gps(self):
        return self.state["y_gps"]
    
    @y_gps.setter
    def y_gps(self, _y_gps):
        self.state["y_gps"] = _y_gps
    
    @property
    def theta_gps(self):
        return self.state["theta_gps"]
    
    @theta_gps.setter
    def theta_gps(self, _theta_gps):
        self.state["theta_gps"] = _theta_gps
    
    @property
    def drone_mode(self):
        return self.state["drone_mode"]
        
    @drone_mode.setter
    def drone_mode(self, _drone_mode):
        self.state["drone_mode"] = _drone_mode

    @property
    def slow_mode(self):
        return self.state["slow_mode"]
    
    @slow_mode.setter
    def slow_mode(self, _slow_mode):
        self.state["slow_mode"] = _slow_mode
    
    def unload(self):
        intake_motor.set_velocity(-50, PERCENT)

    def load(self):
        intake_motor.set_velocity(15, PERCENT)

    def stop_intake(self):
        intake_motor.set_velocity(0, PERCENT)


    def get_current_wheel_encoder_values(self):
        '''
        Returns a vector of all of the encoder values
        '''
        return Vector([left_motor_a.position(DEGREES), right_motor_a.position(DEGREES), left_motor_b.position(DEGREES), right_motor_b.position(DEGREES)])

    def turn_to_object(self, _gameobject):
        '''
        This function will have the robot turn to face the object
        '''
        # Get the delta theta needed for the robot to turn
        delta_theta = (get_heading_to_object(
            r, _gameobject) - inertial.heading(DEGREES))

        # Makes delta theta to go from [0, 360)
        while delta_theta < 0:
            delta_theta += 360
        while delta_theta >= 360:
            delta_theta -= 360

        # Makes delta theta to be from [-180, 180] so that the robot turns the fastest way possible
        if delta_theta >= 180:
            delta_theta -= 360
        elif delta_theta < -180:
            delta_theta += 360

        # Use a PID loop or just basic controls to have the robot turn towards the object here


##### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ###
    def set_flywheel_speed(self, speed):
        self.flywheel_speed = speed
        
    
    def flywheel_update(self):
        if self.flywheel_speed != 0:
            self.flywheel_pid(self.flywheel_speed)
        else:
            flywheel_motor_1.set_velocity(0, PERCENT)
            flywheel_motor_2.set_velocity(0, PERCENT)
        
    
    def flywheel_pid(self, speed):
        MAX_FLYWHEEL_SPEED = 100 / 60
        speed /= MAX_FLYWHEEL_SPEED

        error_1 = -flywheel_motor_1.velocity(VelocityUnits.PERCENT) - speed
        error_2 = -flywheel_motor_2.velocity(VelocityUnits.PERCENT) - speed
        output_1 = self.flywheel_motor_1_PID.update(error_1)
        output_2 = self.flywheel_motor_2_PID.update(error_2)

        
        output_1 = output_1 if abs(output_1) < 100 else 100 * sign(output_1)
        output_2 = output_2 if abs(output_2) < 100 else 100 * sign(output_2)
        output_1 /= MAX_FLYWHEEL_SPEED
        output_2 /= MAX_FLYWHEEL_SPEED
        output_1 -= speed
        output_2 -= speed
        flywheel_motor_1.set_velocity(output_1, PERCENT)
        flywheel_motor_2.set_velocity(output_2, PERCENT)

    def shoot_disk(self):
        '''
        This function will shoot a disk from the flywheel
        '''
        self.is_shooting = True
        # Compute speed that the flywheel needs to speed

        # Set the motors to the speed

        # Index a disk out of the magazine
        # index_motor.spin_for(FORWARD, 2, TURNS, True)
        indexer.open()
        wait(0.5, SECONDS)
        indexer.close()
        
        self.is_shooting = False
        # Launch disk
        pass

    def compute_speed_of_flywheel(self):
        '''
        Because we know our distance to the goal, we can compute the speed that the flywheel needs to launch in order to make the disk in
        '''
        # Equation to find the speed that the flywheel needs:

        self.update()
        goal = blue_goal
        if self.notBlue:
            goal = red_goal

        delta_x = goal.x_pos - self.x_pos - self.flywheel_offset_x
        delta_y = goal.y_pos - self.y_pos - self.flywheel_offset_y

        # TODO: Factor in the position of the shooter

        # Compute the linear speed that the disk needs to be launched to
        # math.cos(theta_shooter * DEG_TO_RAD)

        # distance_to_goal * theta_shooter
        pass

# ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS
    @staticmethod
    def spin_roller_forward():
        roller_motor.set_velocity(100, PERCENT)

    def auto_spin_roller(self):
        '''
        Using an optical sense, we can detect when its our team's color and if it is not then we spin the roller until we do see our team's color
        '''

        # If we are on the red team
        if self.notBlue:
            # Thresholds the roller being the "blue" color
            pass

    def spin_roller(self):
        '''
        Spin the roller (used during auto mode) before we had the auto_spin_roller funciton
        '''
        # Set the velocity to an amount to spin the roller
        # SPINING WITH POSITIVE POWER MAKES THE ROLLERS GO COUNTER CLOCKWISE
        roller_motor.set_velocity(-15, PERCENT)
        # Drive the robot to the right at half speed
        self.stop_moving()
        t = Timer()

        t.reset()
        while t.time() < 500:  # Accellerate
            self.drive(40, 0, 0)

        while t.time() < 500:  # Decellerate
            self.drive(0, 0, 0)

        roller_motor.set_velocity(0, PERCENT)

    @staticmethod
    def spin_roller_backward():
        roller_motor.set_velocity(-100, PERCENT)

##### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ###
    def drive(self, x_vector, y_vector, r_vector, constant_acceleration=True, square_input=True):
        '''
        Drives the robot.
        params - 
            x_vector - the speed that the robot will move in the x direction
            y_vector - the speed that the robot will move in the y direction
            r_vector - the speed that the robot will rotate
            field_based_driving - if true, the robot will drive in the same direction regarless of the robot's orientation
            constant_acceleration - if true, the robot will accelerate at a constant rate
            square_input - if true, the robot will square the input to make it easier to control
        '''
        left_motor_a.spin(FORWARD)
        left_motor_b.spin(FORWARD)
        right_motor_a.spin(FORWARD)
        right_motor_b.spin(FORWARD)

        # Cool driving toggle (basically you rotate the target direction vector based on)
        # the robots heading (https://stackoverflow.com/questions/14607640/rotating-a-vector-in-3d-space)
        if self.drone_mode:
            x_vector, y_vector = rotate_vector_2d(x_vector, y_vector, self.theta * DEG_TO_RAD)

        # Square the input vectors and divide by 100 for better controls
        if square_input:
            x_vector = (x_vector ** 2) / 100 * sign(x_vector)
            y_vector = (y_vector ** 2) / 100 * sign(y_vector)
            r_vector = (r_vector ** 2) / 100 * sign(r_vector)

        if self.slow_mode:
            x_vector = x_vector / 4
            y_vector = y_vector / 4
            r_vector = r_vector / 4

        # HAVE DAVID TRY THIS OUT (sqrt the input might be better for da vid)
        # x_vector = (abs(_x_vector) ** (0.5)) * 10 * sign(_x_vector)
        # y_vector = (abs(_y_vector) ** (0.5)) * 10 * sign(_y_vector)
        # r_vector = (abs(_r_vector) ** (0.5)) * 10 * sign(_r_vector)

        # Get the motor powers
        left_motor_a_target_velocity = x_vector + y_vector + r_vector
        right_motor_a_target_velocity = -x_vector + y_vector - r_vector
        left_motor_b_target_velocity = -x_vector + y_vector + r_vector
        right_motor_b_target_velocity = x_vector + y_vector - r_vector

        # Constant-ish acceleration profile. Basically, our acceleration is constant except for when we are close to starting or stopping (to prevent jerks/slides)
        if constant_acceleration:

            slow_down_speed_threshold = 5

            max_acceleration_left_motor_a = self.max_acceleration
            max_acceleration_right_motor_a = self.max_acceleration
            max_acceleration_left_motor_b = self.max_acceleration
            max_acceleration_right_motor_b = self.max_acceleration

            # If we aren't moving from a stop to a start or vice versa, decrease acceleration
            if abs(left_motor_a.velocity(PERCENT)) < slow_down_speed_threshold:
                max_acceleration_left_motor_a = self.max_acceleration * 2

            if abs(right_motor_a.velocity(PERCENT)) < slow_down_speed_threshold:
                max_acceleration_right_motor_a = self.max_acceleration * 2

            if abs(left_motor_b.velocity(PERCENT)) < slow_down_speed_threshold:
                max_acceleration_left_motor_b = self.max_acceleration * 2

            if abs(right_motor_b.velocity(PERCENT)) < slow_down_speed_threshold:
                max_acceleration_right_motor_b = self.max_acceleration * 2

            # Okie doki, this line is gonna need some documentation, but first some labels:
            # (self.max_acceleration / self.max_velocity * 100) means the maximum acceleration of our robot IN A PERCENTAGE (i.e. 10 percent per second, 50 percent per second, -10 percent, per second)
            # (self.max_acceleration / self.max_velocity * 100) * self.delta_time is the maximum change in velocity of the robot FOR THIS RUN THROUGH THE LOOP (by multipling by delta time we get how long its been since the last loop, which means we can change our velocity by that change in time time acceleration), Ex. if its been 0.1 seconds since the last loop, then our change in velocity can be 10 percent, but if it was 1 second then our change in velocity can be 100 percent beceause its been such a long time (if you still don't understand then learn what integration is)
            # the goal of the following lines is to clamp the change in velocity to not go over some number (max_acceleration),
            # so in order to do that we clamp motor_target_velocity between the motor_current_velocity plus the maximum change in velocity for this run of the loop, repeat for each wheel
            left_motor_a_target_velocity = clamp(left_motor_a_target_velocity, (max_acceleration_left_motor_a / self.max_velocity * 100) * self.delta_time + left_motor_a.velocity(
                PERCENT), -(max_acceleration_left_motor_a / self.max_velocity * 100) * self.delta_time + left_motor_a.velocity(PERCENT))
            right_motor_a_target_velocity = clamp(right_motor_a_target_velocity, (max_acceleration_right_motor_a / self.max_velocity * 100) * self.delta_time + right_motor_a.velocity(
                PERCENT), -(max_acceleration_right_motor_a / self.max_velocity * 100) * self.delta_time + right_motor_a.velocity(PERCENT))
            left_motor_b_target_velocity = clamp(left_motor_b_target_velocity, (max_acceleration_left_motor_b / self.max_velocity * 100) * self.delta_time + left_motor_b.velocity(
                PERCENT), -(max_acceleration_left_motor_b / self.max_velocity * 100) * self.delta_time + left_motor_b.velocity(PERCENT))
            right_motor_b_target_velocity = clamp(right_motor_b_target_velocity, (max_acceleration_right_motor_b / self.max_velocity * 100) * self.delta_time + right_motor_b.velocity(
                PERCENT), -(max_acceleration_right_motor_b / self.max_velocity * 100) * self.delta_time + right_motor_b.velocity(PERCENT))

            # Accelerate the motors to the target velocity. 
            left_motor_a.set_velocity(left_motor_a_target_velocity, PERCENT)
            right_motor_a.set_velocity(right_motor_a_target_velocity, PERCENT)
            left_motor_b.set_velocity(left_motor_b_target_velocity, PERCENT)
            right_motor_b.set_velocity(right_motor_b_target_velocity, PERCENT)

        else:
            # Alpha controls how responsigve the driving is, higher alpha means more reponsibe
            alpha = 0.4

            if abs(x_vector) < 3 and abs(y_vector) < 3 and abs(r_vector) < 3:
                alpha = 0.8

            left_motor_a_target_velocity = left_motor_a.velocity(
                PERCENT) * (1 - alpha) + left_motor_a_target_velocity * alpha
            right_motor_a_target_velocity = right_motor_a.velocity(
                PERCENT) * (1 - alpha) + right_motor_a_target_velocity * alpha
            left_motor_b_target_velocity = left_motor_b.velocity(
                PERCENT) * (1 - alpha) + left_motor_b_target_velocity * alpha
            right_motor_b_target_velocity = right_motor_b.velocity(
                PERCENT) * (1 - alpha) + right_motor_b_target_velocity * alpha

        # Display content
        # print("left_motor_a_target_velocity ", left_motor_a_target_velocity)
        # print("right_motor_a_target_velocity ", right_motor_a_target_velocity)
        # print("left_motor_b_target_velocity ", left_motor_b_target_velocity)
        # print("right_motor_b_target_velocity ", right_motor_b_target_velocity)
        # print("")

        # Set motor powers
        left_motor_a.set_velocity(left_motor_a_target_velocity, PERCENT)
        right_motor_a.set_velocity(right_motor_a_target_velocity, PERCENT)
        left_motor_b.set_velocity(left_motor_b_target_velocity, PERCENT)
        right_motor_b.set_velocity(right_motor_b_target_velocity, PERCENT)
    
    def drive_using_position(self, delta_x, delta_y, delta_r, use_pid = False):
        target_x = self.x_pos + delta_x
        target_y = self.y_pos + delta_y
        target_r = self.theta + delta_r

    @staticmethod
    def intake(_intake_speed):
        # There could be a sensor that detects if theres a disk and then turn on the intake
        # intake_motor.set_velocity(100, PERCENT)

        # UNCOMMENT THIS LINE TO MAKE MOTOR INTAKE SPIN VARIABLE
        intake_motor.set_velocity(_intake_speed, PERCENT)


########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########

########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ##########

simple_path = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : 0.0, 'y' : 1.254751286449391},{ 'x' : 0.0, 'y' : 3.1368782161234776},{ 'x' : 0.0, 'y' : 5.64638078902226},{ 'x' : 0.0, 'y' : 8.783259005145766},{ 'x' : 0.0, 'y' : 11.292761578044576},{ 'x' : -0.6273756432246955, 'y' : 15.057015437392778},{ 'x' : -0.6273756432246955, 'y' : 18.82126929674098},{ 'x' : -0.6273756432246955, 'y' : 23.212898799313876},{ 'x' : 0.0, 'y' : 26.977152658662078},{ 'x' : 0.0, 'y' : 28.23190394511147},{ 'x' : 0.0, 'y' : 31.99615780445967},{ 'x' : 0.0, 'y' : 35.76041166380787},{ 'x' : 0.0, 'y' : 40.15204116638077},{ 'x' : 0.0, 'y' : 44.543670668953666},{ 'x' : 0.627375643224724, 'y' : 47.05317324185248},{ 'x' : 0.627375643224724, 'y' : 50.81742710120065},{ 'x' : 1.2547512864494195, 'y' : 55.20905660377355},{ 'x' : 1.2547512864494195, 'y' : 58.97331046312175},{ 'x' : 1.2547512864494195, 'y' : 62.110188679245255},{ 'x' : 1.2547512864494195, 'y' : 66.50181818181815},{ 'x' : 1.882126929674115, 'y' : 70.26607204116635},{ 'x' : 1.882126929674115, 'y' : 74.65770154373925},{ 'x' : 1.882126929674115, 'y' : 78.42195540308745},{ 'x' : 1.882126929674115, 'y' : 82.18620926243565},{ 'x' : 2.5095025728988105, 'y' : 85.32308747855916},{ 'x' : 2.5095025728988105, 'y' : 89.08734133790736},{ 'x' : 3.7642538593482016, 'y' : 92.85159519725556},{ 'x' : 4.391629502572897, 'y' : 95.98847341337904},{ 'x' : 5.019005145797621, 'y' : 99.12535162950255},{ 'x' : 5.6463807890223165, 'y' : 101.63485420240136},{ 'x' : 6.273756432247012, 'y' : 104.14435677530014},{ 'x' : 6.901132075471708, 'y' : 107.28123499142364},{ 'x' : 8.155883361921099, 'y' : 109.16336192109776},{ 'x' : 10.038010291595214, 'y' : 111.04548885077185},{ 'x' : 11.292761578044605, 'y' : 112.92761578044596},{ 'x' : 12.547512864493996, 'y' : 114.18236706689535},{ 'x' : 14.42963979416811, 'y' : 116.06449399656944},{ 'x' : 16.311766723842197, 'y' : 117.31924528301886},{ 'x' : 18.193893653516312, 'y' : 117.94662092624355},{ 'x' : 20.0760205831904, 'y' : 119.20137221269295},{ 'x' : 21.33077186963982, 'y' : 119.20137221269295},{ 'x' : 23.8402744425386, 'y' : 119.82874785591764},{ 'x' : 25.722401372212715, 'y' : 119.82874785591764},{ 'x' : 27.604528301886802, 'y' : 119.82874785591764},{ 'x' : 29.486655231560917, 'y' : 119.82874785591764},{ 'x' : 31.368782161235004, 'y' : 119.82874785591764},{ 'x' : 33.25090909090909, 'y' : 119.20137221269295},{ 'x' : 34.50566037735851, 'y' : 118.57399656946825},{ 'x' : 35.7604116638079, 'y' : 117.94662092624355},{ 'x' : 37.01516295025729, 'y' : 117.31924528301886},{ 'x' : 38.89728987993141, 'y' : 116.06449399656944},{ 'x' : 39.5246655231561, 'y' : 114.18236706689535},{ 'x' : 40.1520411663808, 'y' : 112.30024013722124},{ 'x' : 41.40679245283019, 'y' : 109.79073756432246},{ 'x' : 42.03416809605491, 'y' : 107.28123499142364},{ 'x' : 42.66154373927961, 'y' : 104.77173241852486},{ 'x' : 43.288919382504304, 'y' : 101.63485420240136},{ 'x' : 43.916295025729, 'y' : 99.12535162950255},{ 'x' : 44.543670668953695, 'y' : 95.98847341337904},{ 'x' : 45.17104631217839, 'y' : 91.59684391080614},{ 'x' : 45.17104631217839, 'y' : 88.45996569468267},{ 'x' : 45.17104631217839, 'y' : 85.32308747855916},{ 'x' : 45.17104631217839, 'y' : 81.55883361921096},{ 'x' : 45.17104631217839, 'y' : 78.42195540308745},{ 'x' : 45.17104631217839, 'y' : 73.40295025728986},{ 'x' : 44.543670668953695, 'y' : 70.26607204116635},{ 'x' : 44.543670668953695, 'y' : 66.50181818181815},{ 'x' : 44.543670668953695, 'y' : 63.99231560891937},{ 'x' : 44.543670668953695, 'y' : 60.855437392795864},{ 'x' : 43.916295025729, 'y' : 58.34593481989705},{ 'x' : 43.916295025729, 'y' : 54.58168096054885},{ 'x' : 43.916295025729, 'y' : 52.07217838765007},{ 'x' : 43.288919382504304, 'y' : 49.56267581475126},{ 'x' : 43.916295025729, 'y' : 47.05317324185248},{ 'x' : 43.916295025729, 'y' : 45.17104631217836},{ 'x' : 43.916295025729, 'y' : 43.288919382504275},{ 'x' : 43.288919382504304, 'y' : 41.40679245283016},{ 'x' : 43.288919382504304, 'y' : 38.89728987993138},{ 'x' : 43.288919382504304, 'y' : 37.01516295025726},{ 'x' : 43.288919382504304, 'y' : 34.50566037735845},{ 'x' : 43.288919382504304, 'y' : 32.623533447684366},{ 'x' : 42.66154373927961, 'y' : 30.74140651801028},{ 'x' : 42.66154373927961, 'y' : 29.48665523156086},{ 'x' : 43.288919382504304, 'y' : 26.977152658662078},{ 'x' : 43.288919382504304, 'y' : 25.72240137221266},{ 'x' : 43.288919382504304, 'y' : 23.840274442538572},{ 'x' : 43.288919382504304, 'y' : 22.58552315608918},{ 'x' : 43.288919382504304, 'y' : 21.33077186963976},{ 'x' : 43.288919382504304, 'y' : 20.07602058319037},{ 'x' : 43.288919382504304, 'y' : 18.82126929674098},{ 'x' : 43.288919382504304, 'y' : 17.56651801029156},{ 'x' : 43.288919382504304, 'y' : 15.057015437392778},{ 'x' : 43.916295025729, 'y' : 13.174888507718663},{ 'x' : 43.916295025729, 'y' : 11.920137221269272},{ 'x' : 44.543670668953695, 'y' : 10.66538593481988},{ 'x' : 45.17104631217839, 'y' : 9.410634648370461},{ 'x' : 46.42579759862781, 'y' : 7.528507718696375},{ 'x' : 48.307924528301896, 'y' : 6.901132075471679},{ 'x' : 49.56267581475129, 'y' : 6.273756432246984},{ 'x' : 51.4448027444254, 'y' : 5.64638078902226},{ 'x' : 53.954305317324184, 'y' : 5.019005145797564},{ 'x' : 55.209056603773604, 'y' : 5.019005145797564},{ 'x' : 57.09118353344769, 'y' : 5.019005145797564},{ 'x' : 58.973310463121805, 'y' : 5.64638078902226},{ 'x' : 60.2280617495712, 'y' : 5.64638078902226},{ 'x' : 62.73756432247001, 'y' : 6.901132075471679},{ 'x' : 63.9923156089194, 'y' : 8.15588336192107},{ 'x' : 65.87444253859348, 'y' : 9.410634648370461},{ 'x' : 68.3839451114923, 'y' : 11.920137221269272},{ 'x' : 69.63869639794169, 'y' : 14.429639794168082},{ 'x' : 70.8934476843911, 'y' : 16.939142367066864},{ 'x' : 72.1481989708405, 'y' : 20.07602058319037},{ 'x' : 73.40295025728989, 'y' : 23.212898799313876},{ 'x' : 74.03032590051458, 'y' : 26.349777015437354},{ 'x' : 74.65770154373931, 'y' : 30.114030874785556},{ 'x' : 75.285077186964, 'y' : 33.87828473413376},{ 'x' : 75.9124528301887, 'y' : 38.269914236706654},{ 'x' : 76.5398284734134, 'y' : 41.40679245283016},{ 'x' : 77.16720411663809, 'y' : 45.79842195540306},{ 'x' : 77.16720411663809, 'y' : 49.56267581475126},{ 'x' : 77.79457975986278, 'y' : 53.954305317324156},{ 'x' : 78.42195540308748, 'y' : 57.71855917667236},{ 'x' : 78.42195540308748, 'y' : 60.855437392795864},{ 'x' : 79.0493310463122, 'y' : 64.61969125214407},{ 'x' : 79.0493310463122, 'y' : 67.75656946826757},{ 'x' : 79.0493310463122, 'y' : 70.26607204116635},{ 'x' : 79.0493310463122, 'y' : 72.77557461406516},{ 'x' : 78.42195540308748, 'y' : 75.91245283018867},{ 'x' : 78.42195540308748, 'y' : 79.04933104631215},{ 'x' : 78.42195540308748, 'y' : 81.55883361921096},{ 'x' : 77.79457975986278, 'y' : 84.06833619210974},{ 'x' : 77.79457975986278, 'y' : 86.57783876500855},{ 'x' : 78.42195540308748, 'y' : 89.08734133790736},{ 'x' : 78.42195540308748, 'y' : 90.34209262435675},{ 'x' : 78.42195540308748, 'y' : 92.85159519725556},{ 'x' : 79.0493310463122, 'y' : 95.36109777015434},{ 'x' : 79.0493310463122, 'y' : 97.87060034305316},{ 'x' : 79.0493310463122, 'y' : 99.75272727272724},{ 'x' : 79.0493310463122, 'y' : 101.63485420240136},{ 'x' : 79.6767066895369, 'y' : 102.88960548885075},{ 'x' : 79.6767066895369, 'y' : 105.39910806174956},{ 'x' : 79.6767066895369, 'y' : 107.28123499142364},{ 'x' : 79.6767066895369, 'y' : 109.16336192109776},{ 'x' : 80.3040823327616, 'y' : 111.04548885077185},{ 'x' : 80.3040823327616, 'y' : 112.30024013722124},{ 'x' : 80.93145797598629, 'y' : 114.18236706689535},{ 'x' : 81.55883361921099, 'y' : 116.06449399656944},{ 'x' : 82.18620926243568, 'y' : 117.31924528301886},{ 'x' : 83.4409605488851, 'y' : 118.57399656946825},{ 'x' : 84.69571183533449, 'y' : 119.82874785591764},{ 'x' : 86.57783876500858, 'y' : 120.45612349914234},{ 'x' : 88.4599656946827, 'y' : 121.08349914236703},{ 'x' : 90.34209262435678, 'y' : 121.08349914236703},{ 'x' : 92.85159519725559, 'y' : 121.71087478559176},{ 'x' : 95.3610977701544, 'y' : 121.71087478559176},{ 'x' : 99.12535162950257, 'y' : 121.08349914236703},{ 'x' : 101.63485420240139, 'y' : 120.45612349914234},{ 'x' : 104.1443567753002, 'y' : 120.45612349914234},{ 'x' : 107.28123499142367, 'y' : 119.20137221269295},{ 'x' : 109.79073756432248, 'y' : 118.57399656946825},{ 'x' : 112.3002401372213, 'y' : 117.31924528301886},{ 'x' : 114.18236706689538, 'y' : 116.06449399656944},{ 'x' : 115.43711835334477, 'y' : 114.80974271012005},{ 'x' : 116.69186963979419, 'y' : 113.55499142367066},{ 'x' : 119.20137221269297, 'y' : 110.41811320754715},{ 'x' : 120.4561234991424, 'y' : 107.90861063464834},{ 'x' : 121.08349914236709, 'y' : 105.39910806174956},{ 'x' : 121.08349914236709, 'y' : 102.26222984562605},{ 'x' : 121.71087478559178, 'y' : 98.49797598627785},{ 'x' : 121.71087478559178, 'y' : 96.61584905660376},{ 'x' : 122.33825042881648, 'y' : 92.22421955403084},{ 'x' : 122.33825042881648, 'y' : 88.45996569468267},{ 'x' : 121.71087478559178, 'y' : 85.32308747855916},{ 'x' : 121.71087478559178, 'y' : 81.55883361921096},{ 'x' : 121.71087478559178, 'y' : 78.42195540308745},{ 'x' : 121.08349914236709, 'y' : 73.40295025728986},{ 'x' : 121.08349914236709, 'y' : 69.63869639794166},{ 'x' : 121.08349914236709, 'y' : 65.87444253859346},{ 'x' : 121.08349914236709, 'y' : 61.48281303602056},{ 'x' : 120.4561234991424, 'y' : 56.46380789022297},{ 'x' : 120.4561234991424, 'y' : 52.699554030874765},{ 'x' : 120.4561234991424, 'y' : 48.93530017152656},{ 'x' : 120.4561234991424, 'y' : 45.17104631217836},{ 'x' : 120.4561234991424, 'y' : 41.40679245283016},{ 'x' : 120.4561234991424, 'y' : 37.64253859348196},{ 'x' : 120.4561234991424, 'y' : 33.87828473413376},{ 'x' : 120.4561234991424, 'y' : 31.368782161234975},{ 'x' : 120.4561234991424, 'y' : 28.859279588336165},{ 'x' : 120.4561234991424, 'y' : 26.349777015437354},{ 'x' : 120.4561234991424, 'y' : 23.840274442538572},{ 'x' : 120.4561234991424, 'y' : 21.958147512864457},{ 'x' : 120.4561234991424, 'y' : 20.703396226415066},{ 'x' : 120.4561234991424, 'y' : 19.448644939965675},{ 'x' : 120.4561234991424, 'y' : 18.193893653516284},{ 'x' : 120.4561234991424, 'y' : 16.939142367066864},{ 'x' : 120.4561234991424, 'y' : 15.684391080617473},{ 'x' : 120.4561234991424, 'y' : 14.429639794168082},{ 'x' : 120.4561234991424, 'y' : 13.174888507718663},{ 'x' : 120.4561234991424, 'y' : 11.920137221269272},{ 'x' : 119.82874785591767, 'y' : 10.66538593481988},{ 'x' : 118.57399656946828, 'y' : 10.66538593481988},{ 'x' : 116.69186963979419, 'y' : 9.410634648370461},{ 'x' : 114.80974271012008, 'y' : 8.783259005145766},{ 'x' : 112.3002401372213, 'y' : 8.15588336192107},{ 'x' : 109.16336192109779, 'y' : 6.901132075471679},{ 'x' : 106.02648370497428, 'y' : 6.901132075471679},{ 'x' : 102.26222984562608, 'y' : 5.64638078902226},{ 'x' : 97.87060034305318, 'y' : 5.019005145797564},{ 'x' : 92.85159519725559, 'y' : 4.391629502572869},{ 'x' : 87.2052144082333, 'y' : 3.1368782161234776},{ 'x' : 82.18620926243568, 'y' : 3.1368782161234776},{ 'x' : 75.9124528301887, 'y' : 1.8821269296740866},{ 'x' : 70.26607204116638, 'y' : 1.254751286449391},{ 'x' : 64.6196912521441, 'y' : 1.254751286449391},{ 'x' : 58.34593481989711, 'y' : 0.6273756432246955},{ 'x' : 53.32692967409949, 'y' : 0.6273756432246955},{ 'x' : 48.307924528301896, 'y' : 0.6273756432246955},{ 'x' : 42.66154373927961, 'y' : 0.6273756432246955},{ 'x' : 37.01516295025729, 'y' : 0.6273756432246955},{ 'x' : 31.9961578044597, 'y' : 0.6273756432246955},{ 'x' : 29.486655231560917, 'y' : 0.6273756432246955},{ 'x' : 25.09502572898799, 'y' : 0.0},{ 'x' : 21.33077186963982, 'y' : 0.6273756432246955},{ 'x' : 16.939142367066893, 'y' : 0.6273756432246955},{ 'x' : 15.057015437392806, 'y' : 0.6273756432246955},{ 'x' : 11.9201372212693, 'y' : 0.6273756432246955},{ 'x' : 8.783259005145823, 'y' : 0.6273756432246955},{ 'x' : 5.6463807890223165, 'y' : 0.6273756432246955},{ 'x' : 3.136878216123506, 'y' : 0.0},{ 'x' : 1.882126929674115, 'y' : 0.0},]
straight_line = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : -0.6273756432246955, 'y' : 8.155883361921099},{ 'x' : -0.6273756432246955, 'y' : 15.057015437392778},{ 'x' : -1.2547512864494053, 'y' : 25.09502572898799},{ 'x' : -1.8821269296741008, 'y' : 36.38778730703257},{ 'x' : -3.136878216123506, 'y' : 46.42579759862778},{ 'x' : -3.136878216123506, 'y' : 55.209056603773575},{ 'x' : -3.7642538593482016, 'y' : 62.73756432246998},{ 'x' : -3.7642538593482016, 'y' : 69.01132075471696},{ 'x' : -3.136878216123506, 'y' : 73.40295025728986},]
mostyn = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : -0.63, 'y' : 4.39},{ 'x' : 0.0, 'y' : 10.04},{ 'x' : 0.0, 'y' : 13.8},{ 'x' : 0.0, 'y' : 20.08},{ 'x' : 0.63, 'y' : 32.0},{ 'x' : 1.25, 'y' : 42.66},{ 'x' : 1.88, 'y' : 53.95},{ 'x' : 1.88, 'y' : 63.36},{ 'x' : 1.88, 'y' : 72.78},{ 'x' : 1.88, 'y' : 80.3},{ 'x' : 1.25, 'y' : 87.21},{ 'x' : 0.63, 'y' : 93.48},{ 'x' : -0.63, 'y' : 99.13},{ 'x' : -1.88, 'y' : 104.77},{ 'x' : -2.51, 'y' : 110.42},{ 'x' : -3.76, 'y' : 116.06},{ 'x' : -4.39, 'y' : 122.34},{ 'x' : -5.02, 'y' : 127.98},{ 'x' : -5.02, 'y' : 134.26},{ 'x' : -5.02, 'y' : 140.53},{ 'x' : -4.39, 'y' : 146.18},{ 'x' : -4.39, 'y' : 151.2},{ 'x' : -3.76, 'y' : 156.84},{ 'x' : -3.14, 'y' : 161.24},{ 'x' : -3.14, 'y' : 162.49},{ 'x' : -3.14, 'y' : 163.75},{ 'x' : -3.14, 'y' : 165.0},{ 'x' : -3.14, 'y' : 166.25},{ 'x' : -3.14, 'y' : 167.51},{ 'x' : -3.14, 'y' : 168.76},{ 'x' : -1.25, 'y' : 170.02},{ 'x' : 0.63, 'y' : 171.27},{ 'x' : 3.76, 'y' : 171.9},{ 'x' : 6.9, 'y' : 173.16},{ 'x' : 11.29, 'y' : 173.78},{ 'x' : 16.31, 'y' : 175.04},{ 'x' : 21.33, 'y' : 176.29},{ 'x' : 26.98, 'y' : 176.92},{ 'x' : 32.62, 'y' : 178.17},{ 'x' : 42.66, 'y' : 178.8},{ 'x' : 50.19, 'y' : 180.06},{ 'x' : 60.23, 'y' : 180.06},{ 'x' : 63.99, 'y' : 180.06},{ 'x' : 72.15, 'y' : 180.06},{ 'x' : 79.68, 'y' : 178.8},{ 'x' : 85.95, 'y' : 176.92},{ 'x' : 92.85, 'y' : 174.41},{ 'x' : 98.5, 'y' : 171.27},{ 'x' : 101.01, 'y' : 169.39},{ 'x' : 106.03, 'y' : 165.63},{ 'x' : 111.05, 'y' : 161.24},{ 'x' : 115.44, 'y' : 156.84},{ 'x' : 119.83, 'y' : 151.82},{ 'x' : 124.22, 'y' : 146.81},{ 'x' : 127.98, 'y' : 141.79},{ 'x' : 131.12, 'y' : 136.77},{ 'x' : 134.26, 'y' : 131.12},{ 'x' : 136.77, 'y' : 125.48},{ 'x' : 139.28, 'y' : 119.83},{ 'x' : 141.79, 'y' : 114.18},{ 'x' : 143.67, 'y' : 108.54},{ 'x' : 145.55, 'y' : 102.26},{ 'x' : 146.81, 'y' : 95.99},{ 'x' : 147.43, 'y' : 90.34},{ 'x' : 147.43, 'y' : 82.81},{ 'x' : 146.81, 'y' : 75.91},{ 'x' : 146.18, 'y' : 67.76},{ 'x' : 143.67, 'y' : 59.6},{ 'x' : 139.9, 'y' : 51.44},{ 'x' : 135.51, 'y' : 42.66},{ 'x' : 131.12, 'y' : 34.51},{ 'x' : 126.1, 'y' : 26.98},{ 'x' : 120.46, 'y' : 20.08},{ 'x' : 114.81, 'y' : 13.8},{ 'x' : 109.16, 'y' : 8.78},{ 'x' : 106.65, 'y' : 6.27},{ 'x' : 100.38, 'y' : 1.88},{ 'x' : 94.11, 'y' : -1.25},{ 'x' : 88.46, 'y' : -3.76},{ 'x' : 82.19, 'y' : -5.65},{ 'x' : 75.29, 'y' : -6.27},{ 'x' : 72.15, 'y' : -6.9},{ 'x' : 65.25, 'y' : -6.27},{ 'x' : 58.97, 'y' : -5.65},{ 'x' : 52.07, 'y' : -4.39},{ 'x' : 45.8, 'y' : -3.76},{ 'x' : 39.52, 'y' : -1.88},{ 'x' : 33.25, 'y' : -0.63},{ 'x' : 26.98, 'y' : 0.63},{ 'x' : 21.33, 'y' : 2.51},{ 'x' : 16.31, 'y' : 4.39},{ 'x' : 11.29, 'y' : 5.65},{ 'x' : 6.9, 'y' : 6.27},]
revamping_everthing = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : 0.0, 'y' : 1.88},{ 'x' : -0.63, 'y' : 3.14},{ 'x' : -0.63, 'y' : 4.39},{ 'x' : -0.63, 'y' : 6.27},{ 'x' : -1.25, 'y' : 7.53},{ 'x' : -1.25, 'y' : 10.67},{ 'x' : -1.88, 'y' : 13.8},{ 'x' : -1.88, 'y' : 22.59},{ 'x' : -1.88, 'y' : 28.23},{ 'x' : -2.51, 'y' : 35.76},{ 'x' : -2.51, 'y' : 41.41},{ 'x' : -3.14, 'y' : 47.05},{ 'x' : -2.51, 'y' : 52.07},{ 'x' : -1.88, 'y' : 56.46},{ 'x' : -1.25, 'y' : 60.86},{ 'x' : -0.63, 'y' : 64.62},{ 'x' : 0.0, 'y' : 69.01},{ 'x' : 0.63, 'y' : 72.78},{ 'x' : 0.63, 'y' : 77.79},{ 'x' : 1.88, 'y' : 80.93},{ 'x' : 2.51, 'y' : 82.81},{ 'x' : 3.76, 'y' : 85.32},{ 'x' : 5.65, 'y' : 87.21},{ 'x' : 7.53, 'y' : 90.34},{ 'x' : 9.41, 'y' : 91.6},{ 'x' : 10.67, 'y' : 93.48},{ 'x' : 11.92, 'y' : 94.11},{ 'x' : 13.8, 'y' : 94.73},{ 'x' : 15.06, 'y' : 95.36},{ 'x' : 16.31, 'y' : 95.99},{ 'x' : 17.57, 'y' : 95.99},{ 'x' : 18.82, 'y' : 95.99},{ 'x' : 20.08, 'y' : 95.99},{ 'x' : 21.33, 'y' : 95.36},{ 'x' : 22.59, 'y' : 94.73},{ 'x' : 24.47, 'y' : 94.11},{ 'x' : 25.72, 'y' : 92.85},{ 'x' : 27.6, 'y' : 90.97},{ 'x' : 27.6, 'y' : 89.71},{ 'x' : 28.86, 'y' : 88.46},{ 'x' : 29.49, 'y' : 86.58},{ 'x' : 30.11, 'y' : 84.7},{ 'x' : 30.11, 'y' : 82.19},{ 'x' : 30.74, 'y' : 80.93},{ 'x' : 31.37, 'y' : 78.42},{ 'x' : 31.37, 'y' : 75.29},{ 'x' : 32.0, 'y' : 72.78},{ 'x' : 32.62, 'y' : 69.64},{ 'x' : 32.62, 'y' : 65.87},{ 'x' : 32.62, 'y' : 62.11},{ 'x' : 32.62, 'y' : 57.72},{ 'x' : 32.62, 'y' : 52.7},{ 'x' : 33.25, 'y' : 49.56},{ 'x' : 33.25, 'y' : 47.68},{ 'x' : 33.25, 'y' : 43.92},{ 'x' : 33.25, 'y' : 40.15},{ 'x' : 33.88, 'y' : 34.51},{ 'x' : 33.88, 'y' : 30.74},{ 'x' : 33.88, 'y' : 26.98},{ 'x' : 33.88, 'y' : 24.47},{ 'x' : 33.88, 'y' : 20.7},{ 'x' : 33.88, 'y' : 16.31},{ 'x' : 34.51, 'y' : 12.55},{ 'x' : 34.51, 'y' : 10.04},{ 'x' : 35.13, 'y' : 7.53},{ 'x' : 35.76, 'y' : 5.65},{ 'x' : 36.39, 'y' : 3.76},{ 'x' : 37.64, 'y' : 2.51},{ 'x' : 38.9, 'y' : 0.0},{ 'x' : 39.52, 'y' : -1.88},{ 'x' : 40.78, 'y' : -2.51},{ 'x' : 42.66, 'y' : -3.76},{ 'x' : 44.54, 'y' : -5.02},{ 'x' : 47.05, 'y' : -6.27},{ 'x' : 48.94, 'y' : -6.9},{ 'x' : 53.33, 'y' : -5.65},{ 'x' : 57.09, 'y' : -5.02},{ 'x' : 58.35, 'y' : -3.76},{ 'x' : 59.6, 'y' : -2.51},{ 'x' : 60.86, 'y' : -1.25},{ 'x' : 62.74, 'y' : 1.88},{ 'x' : 63.99, 'y' : 3.76},{ 'x' : 65.25, 'y' : 6.9},{ 'x' : 65.87, 'y' : 10.67},{ 'x' : 67.13, 'y' : 14.43},{ 'x' : 67.13, 'y' : 16.31},{ 'x' : 68.38, 'y' : 20.08},{ 'x' : 68.38, 'y' : 25.1},{ 'x' : 69.01, 'y' : 29.49},{ 'x' : 69.64, 'y' : 34.51},{ 'x' : 70.27, 'y' : 39.52},{ 'x' : 70.89, 'y' : 45.8},{ 'x' : 70.89, 'y' : 48.94},{ 'x' : 71.52, 'y' : 54.58},{ 'x' : 73.4, 'y' : 65.25},{ 'x' : 73.4, 'y' : 73.4},{ 'x' : 74.66, 'y' : 82.81},{ 'x' : 74.66, 'y' : 90.34},{ 'x' : 75.91, 'y' : 97.24},{ 'x' : 76.54, 'y' : 102.26},{ 'x' : 77.17, 'y' : 107.28},{ 'x' : 77.79, 'y' : 108.54},{ 'x' : 78.42, 'y' : 107.28},{ 'x' : 80.3, 'y' : 106.65},{ 'x' : 82.19, 'y' : 106.03},{ 'x' : 84.07, 'y' : 106.65},{ 'x' : 86.58, 'y' : 106.65},{ 'x' : 89.09, 'y' : 106.03},{ 'x' : 91.6, 'y' : 106.03},{ 'x' : 94.11, 'y' : 105.4},{ 'x' : 95.36, 'y' : 105.4},{ 'x' : 97.87, 'y' : 104.77},{ 'x' : 100.38, 'y' : 104.14},{ 'x' : 102.26, 'y' : 103.52},{ 'x' : 104.14, 'y' : 102.26},{ 'x' : 106.03, 'y' : 101.01},{ 'x' : 107.28, 'y' : 100.38},{ 'x' : 109.16, 'y' : 98.5},{ 'x' : 110.42, 'y' : 96.62},{ 'x' : 111.67, 'y' : 94.11},{ 'x' : 112.3, 'y' : 92.85},{ 'x' : 112.93, 'y' : 90.97},{ 'x' : 113.55, 'y' : 89.09},{ 'x' : 114.18, 'y' : 87.83},{ 'x' : 114.18, 'y' : 85.95},{ 'x' : 114.81, 'y' : 84.7},{ 'x' : 115.44, 'y' : 82.19},{ 'x' : 115.44, 'y' : 80.93},{ 'x' : 115.44, 'y' : 79.68},{ 'x' : 115.44, 'y' : 76.54},{ 'x' : 115.44, 'y' : 74.03},{ 'x' : 115.44, 'y' : 70.89},{ 'x' : 115.44, 'y' : 67.13},{ 'x' : 114.81, 'y' : 63.36},{ 'x' : 114.81, 'y' : 58.97},{ 'x' : 114.18, 'y' : 55.21},{ 'x' : 113.55, 'y' : 51.44},{ 'x' : 113.55, 'y' : 48.31},{ 'x' : 113.55, 'y' : 45.8},{ 'x' : 113.55, 'y' : 44.54},{ 'x' : 112.93, 'y' : 40.78},{ 'x' : 112.93, 'y' : 38.9},{ 'x' : 112.93, 'y' : 36.39},{ 'x' : 112.93, 'y' : 34.51},{ 'x' : 112.93, 'y' : 32.62},{ 'x' : 112.93, 'y' : 30.74},{ 'x' : 112.93, 'y' : 28.86},{ 'x' : 112.3, 'y' : 26.98},{ 'x' : 112.93, 'y' : 25.1},{ 'x' : 112.93, 'y' : 23.21},{ 'x' : 112.93, 'y' : 21.96},{ 'x' : 113.55, 'y' : 20.08},{ 'x' : 113.55, 'y' : 18.82},{ 'x' : 113.55, 'y' : 17.57},{ 'x' : 113.55, 'y' : 16.31},{ 'x' : 113.55, 'y' : 15.06},{ 'x' : 114.18, 'y' : 13.8},{ 'x' : 114.18, 'y' : 12.55},{ 'x' : 114.18, 'y' : 11.29},{ 'x' : 114.18, 'y' : 10.04},{ 'x' : 114.18, 'y' : 8.78},{ 'x' : 114.18, 'y' : 7.53},{ 'x' : 113.55, 'y' : 6.27},{ 'x' : 113.55, 'y' : 5.02},{ 'x' : 111.67, 'y' : 3.76},{ 'x' : 107.28, 'y' : 2.51},{ 'x' : 104.77, 'y' : 1.88},{ 'x' : 97.24, 'y' : 0.63},{ 'x' : 92.85, 'y' : 0.63},{ 'x' : 89.71, 'y' : 0.63},{ 'x' : 80.93, 'y' : 0.63},{ 'x' : 78.42, 'y' : 0.63},{ 'x' : 69.01, 'y' : 0.63},{ 'x' : 60.86, 'y' : 0.63},{ 'x' : 53.95, 'y' : 1.88},{ 'x' : 46.43, 'y' : 2.51},{ 'x' : 40.15, 'y' : 3.76},{ 'x' : 33.88, 'y' : 5.02},{ 'x' : 27.6, 'y' : 5.65},{ 'x' : 21.33, 'y' : 6.27},{ 'x' : 15.68, 'y' : 6.27},{ 'x' : 9.41, 'y' : 6.9},{ 'x' : 3.76, 'y' : 6.27},{ 'x' : -1.88, 'y' : 5.65},{ 'x' : -5.65, 'y' : 5.65},{ 'x' : -9.41, 'y' : 5.02},]

# END OF PATHS ######### END OF PATHS ######### END OF PATHS ######### END OF PATHS ######### END OF PATHS

def get_heading_to_object(gameobject_1, gameobject_2):
    '''
    RETURNS IN DEGREES
    TODO: if someone doesn't pass a gameobject then this will break everything
    '''

    # Checks to see if both objects are of the GameObject class
    assert isinstance(gameobject_1, GameObject)
    assert isinstance(gameobject_2, GameObject)

    if gameobject_1.x_pos == gameobject_2.x_pos:
        return 180

    if gameobject_1.x_pos < gameobject_2.x_pos:
        ang = math.atan(((gameobject_2.y_pos - gameobject_1.y_pos)) /
                        (gameobject_2.x_pos - gameobject_1.x_pos)) * RAD_TO_DEG
    else:
        ang = math.atan(((gameobject_2.y_pos - gameobject_1.y_pos)) /
                        (gameobject_2.x_pos - gameobject_1.x_pos)) * RAD_TO_DEG + 180
    
    # if gameobject_1.x_pos > gameobject_2.x_pos:
    #     ang -= 90 * sign(ang)

    # if gameobject_1.y_pos > gameobject_2.y_pos:
        
    ang = 90 - ang
    if ang > 180:
        ang -= 360
    if ang < -180:
        ang += 360

    return ang

######## COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ###

def autonomous():
    # init() # Init a second time (Hopefully to fix the problem with the motors breaking for some reason)
    r.run_autonomous(simple_path)
    r.stop_moving()

def driver_control():
    # Change this to be relative to the match time???
    timer = Timer()
    timer.reset()

    reset_theta_timer = Timer()
    reset_theta_timer.reset()

    r.driver_controlled_timer.reset()

    r.autonomous_timer.reset()

    while True:
        # Update the robot's information
        new_theta = controller_1.axis1.position() * 0.75 + r.theta
        if controller_1.axis1.position() == 0:
            new_theta = r.target_state["theta"]
            
        r.set_target_state(
            {
                "x_pos" : r.x_pos + controller_1.axis4.position() * 0.5,
                "y_pos" : r.y_pos + controller_1.axis3.position() * 0.5,
                "theta" : new_theta,
                "slow_mode" : controller_1.buttonR1.pressing(),
            }
        )
        if controller_1.buttonUp.pressing():
            r.set_target_state(
                {
                    "theta" : 0,
                }
            )
        if controller_1.buttonDown.pressing():
            r.set_target_state(
                {
                    "theta" : 180,
                }
            )
        if controller_1.buttonLeft.pressing():
            r.set_target_state(
                {
                    "theta" : 90,
                }
            )
        if controller_1.buttonRight.pressing():
            r.set_target_state(
                {
                    "theta" : -90,
                }
            )
        if controller_1.buttonY.pressing():
            r.set_target_state({
                "theta" : get_heading_to_object(r, r.goal)
            })
        r.update()

        # Timer to print things out to the terminal every x seconds
        if (timer.time() > 0.1 * 1000):
            # r.print(f("pos",r.x_pos,r.y_pos, "gps", r.x_from_gps, r.y_from_gps, "vel",r.x_vel,r.y_vel))
            
            r.print(f("thetas", r.theta, r.target_state["theta"]))
            r.print(f("pos", r.x_pos, r.y_pos, "gameobject", r.goal.x_pos, r.goal.y_pos, "theta", r.theta, get_heading_to_object(r, r.goal)))
            # r.print(f("x_enc", r.x_enc, "prev", r.previous_state["x_enc"], "theta", r.theta,r.theta_vel,  "time", r.state["time"], r.delta_time, r.previous_state["time"]))
            # print(r.driver_controlled_timer.value(), flywheel_motor_1.velocity(PERCENT), flywheel_motor_2.velocity(PERCENT), flywheel_motor_1.power(), flywheel_motor_2.power(), flywheel_motor_1.current(), flywheel_motor_2.current(), flywheel_motor_1.torque(), flywheel_motor_2.torque(), flywheel_motor_1.temperature(), flywheel_motor_2.temperature())
            timer.reset()

        # robot axis are based on x, y, and r vectors
        # r.drive(controller_1.axis4.position(), controller_1.axis3.position(),
        #         controller_1.axis1.position(), False, True)
            
        # r.slow_mode = controller_1.buttonR2.pressing()

        # Run the intake subsystem, set the desired speed
        # r.intake(controller_1.axis2.position())

        # r.set_flywheel_speed(controller_1.axis3.position())

        # if controller_1.buttonUp.pressing():
        #     r.set_flywheel_speed(100)
        # elif controller_1.buttonDown.pressing():
        #     r.set_flywheel_speed(0)
        # elif controller_1.buttonLeft.pressing():
        #     r.set_flywheel_speed(66)
        # elif controller_1.buttonRight.pressing():
        #     r.set_flywheel_speed(33)
        
        # if controller_1.buttonA.pressing():
        #     shooter_thread = Thread(r.shoot_disk)

        if controller_1.buttonR1.pressing() and not r.is_shooting:
            r.shoot_disk()

        if not controller_1.buttonX.pressing():
            reset_theta_timer.reset()

        # If someone has pressed button x for more than 1 second, reset the orientaiton
        if reset_theta_timer.value() > 1:
            print("Resetting the orientation of the robot!!")
            r.reset_theta()
            controller_1.rumble("...")
            reset_theta_timer.reset()

        if controller_1.buttonB.pressing():
            r.print_debug_info()

        wait(0.01, SECONDS)

def init():
    # Make it so that the motors stop instead of coast

    # HAVE DAVID TRY THIS OUT
    left_motor_a.set_stopping(BRAKE)
    right_motor_a.set_stopping(BRAKE)
    left_motor_b.set_stopping(BRAKE)
    right_motor_b.set_stopping(BRAKE)

    flywheel_motor_1.spin(FORWARD)
    flywheel_motor_2.spin(FORWARD)

    flywheel_motor_1.set_velocity(0)
    flywheel_motor_2.set_velocity(0)

    left_motor_a.set_velocity(0, PERCENT)
    right_motor_a.set_velocity(0, PERCENT)
    left_motor_b.set_velocity(0, PERCENT)
    right_motor_b.set_velocity(0, PERCENT)

    left_motor_a.spin(FORWARD)
    # These wheels are reversed so that they spin ccw instead of cw for forward
    right_motor_a.spin(REVERSE)
    left_motor_b.spin(FORWARD)
    # These wheels are reversed so that they spin ccw instead of cw for forward
    right_motor_b.spin(REVERSE)

    index_motor.set_velocity(100, PERCENT)
    index_motor.set_position(0, DEGREES)

    index_motor.spin_for(FORWARD, 0, TURNS, False)

    intake_motor.spin(FORWARD)
    intake_motor.set_velocity(0, PERCENT)

    roller_motor.spin(FORWARD)
    roller_motor.set_velocity(0, PERCENT)

    # Set the optical light power
    roller_optical.set_light_power(0)
    roller_optical.object_detect_threshold(100)


    # Wait for the gyro to settle, if it takes more then 10 seconds then close out of the loop
    t = Timer()

    t.reset()
    if gps.installed():
        gps.calibrate()
    while (inertial.gyro_rate(ZAXIS) != 0 and t.value() < 10):
        print("Waiting for gyro to init...")
        wait(0.1, SECONDS)
    
    r.init()
    controller_1.rumble("...")


red_goal = GameObject(0, 0)
blue_goal = GameObject(0, 0)

wheel_gear_ratio = 18
ticks_per_revolution = 50 * wheel_gear_ratio

field_length = 356  # CM

r = Robot(0, 0)
r.red_goal = red_goal
r.blue_goal = blue_goal

r.set_team("red")

# r.use_gps(gps.installed())
r.using_gps = False
r.drone_mode = True
r.slow_mode = False


r.flywheel_motor_1_PID.set_constants(0.5,0.01,10)
r.flywheel_motor_2_PID.set_constants(0.5,0.01,10)
r.x_vel_PID.set_constants(10,0,0)
r.y_vel_PID.set_constants(10,0,0)
r.theta_vel_PID.set_constants(10,0,1)

init()

# autonomous()
driver_control()

# competition = Competition(driver_control, autonomous)

##! PRIORITY
# TODO: Revamp the driving command to work with go_to_position and inputting position and rotation vectors 
# TODO: Theoretically, shouldn't max acceleration half if we're close to 0 for the motor speed? rework that function
# TODO: Make a spin to command heading
# TODO: Make a better pose/orienation initialization/resetting tool (initialize the position in the init() command, don't worry if we can't init yet)
# TODO: Use sensor fusion with Kalman filter for better position
# TODO: Make it os that the we init the robots orientation based off othe GPS (round it to the nearest 90 or 45 degrees), but make it so that drone mode still works
# TODO: Use the gps and figure out how precise it is
# TODO: Make pid for 
# TODO: Experiment not waiting in self.position_update()
# TODO: Make odometry consistent with real world measurements

# * IMPORTANT
# TODO: Instead of rotating the driver control vector based off of the current theta, why not move it based off of the predicted theta/halfway between the two? this might remove the drifting that happens when rotation while moving
# TODO: test out the custom robot print function
# TODO: Work with david and figure out how to make robot driving feel better (more responsive, smooth, etc.)
# TODO: Figure out variance in GPS sensor data and variance in model (Q and R)
# TODO: Test out the cool Robot.print command and see if it prints on brain and console and how well it does so
# TODO: make the offset of the gps so that it reports the position of the center of the robot
# TODO: When the robot has an x,y position close to the edge of the field, make it so that the robot physically cannot slam into the wall (or use sensors)
# TODO: Change this to be relative to the match time???

# Not important
# TODO: test to see if threads die when they lose scope
# TODO: Figure out the sig figs of the inbuilt Timer class, if its good lets use it instead of time.time_ms
# TODO: make an auto path class that has information like description, color, etc.
# TODO: Research what the motor.set_max_torque function does
# TODO: Research how to use motor.torqax_torque function does
# TODO: Research how to use motor.torque
# TODO: Research how to utilize motor.temperature
# TODO: Make an info screen that showue
# TODO: Make an info screen that shows if there are any status issues wrong with the robot
# TODO: If you know that motor could be against the greater resistance that could stall it and overheat, you could limit the current to avoid overheating and raise it again when there is no resistance.

# GPS
# Starting = -152.5, 8.4
# Ending = 133.8, 4.0

# Starting = -152, 7.8
# Ending 160, -10

# Starting = -152, 1
# Ending = 143, -5

# ENCODERS
# Starting = 0,0
# Ending = 6.2, 303.4

# Starting 0,0
# Ending 16, 331 (Real, 123 3/4)

# Starting 0,0
# Ending = 0.4, 315, 117.5


# Flywheel

# no pid, direct control
# 3.024 1.5, 2.23
# 5.184 61, 48 


# pid (1,0,0)
# 1.108 0,0
# 3.24 60, 50

# pid (2,0,0)
# 1.08, 0,0
# 3.348 60, 51