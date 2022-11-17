# region VEXcode Generated Robot Configuration
from math import sin, cos
import math
import time
from vex import *
import random

# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)

left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(
    left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
launching_motor_1 = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
launching_motor_2 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
launching_motor = MotorGroup(launching_motor_1, launching_motor_2)

led_a = Led(brain.three_wire_port.a)
inertial = Inertial(Ports.PORT9)
index_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
roller_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
intake_motor = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
roller_optical = Optical(Ports.PORT16)

gps = Gps(Ports.PORT3)


# wait for rotation sensor to fully initialize
wait(30, MSEC)

# For the inertial sensor to give us a reading
inertial.gyro_rate(ZAXIS)

wait(50, MSEC)
# endregion VEXcode Generated Robot Configuration

# ------------------------------------------
#
# 	Project:      VEXcode Project
#	Author:       VEX
# Created:
# Description:  VEXcode V5 Python Project
#
# ------------------------------------------

r2o2 = math.sqrt(2) / 2


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

    theta: float = 0

    x_velocity: float = 0
    y_velocity: float = 0

    previous_wheel_encoder_values = Vector([0, 0, 0, 0])
    previous_sum_of_wheel_encoders: int = 0

    previous_update_time: float = 0

    wheel_max_rpm: float = 200
    wheel_diameter_CM: float = 10.5
    wheel_distance_CM_to_DEG_coefficient: float = 360 / \
        (math.pi * wheel_diameter_CM)

    max_velocity: float
    max_acceleration: float

    previous_theta = 0
    total_theta = 0

    using_gps = False

    # PID controller constants
    motor_PID_kP = 10
    motor_PID_kI = 5
    motor_PID_kD = 0
    
    # Init the PID controllers for the robot
    left_motor_a_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    right_motor_a_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    left_motor_b_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    right_motor_b_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)

    previous_x_from_encoders = 0
    previous_y_from_encoders = 0

    # used to keep track of time in auto and driver mode respectively, use it for nicely logging data, can be used during either modes for end game/pathfinding rules
    autonomous_timer = Timer()
    driver_controlled_timer = Timer()

    previous_x_from_gps = 0
    previous_y_from_gps = 0

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

##### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ###
    def use_gps(self, value):
        self.using_gps = value
    
    def print_debug_info(self):
        print("left_motor_a_position:", left_motor_a.position(DEGREES), "left_motor_b_position:", left_motor_b.position(DEGREES), "right_motor_a_position:", right_motor_a.position(DEGREES), "right_motor_b_position:", right_motor_b.position(DEGREES))

    def set_time_to_top_speed(self, _time):
        '''
        Change the maximum acceleration of the robot so that it will reach max velocity in the given time
        '''
        self.max_acceleration = 2 * self.max_velocity / _time

    @staticmethod
    def reset_motor_positions():
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
        return self.x_velocity == 0 and self.y_velocity == 0

    def set_team(self, _color):
        '''
        Sets the team for the robot so that we can move the rollers to our team color, or shoot disks just into our goal, etc.
        '''
        _color = _color.lower()
        print("INPUT FROM SET_TEAM IS:\t", _color)
        if _color == "red":
            self.notBlue = True
            return
        elif _color == "blue":
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
        message = f" ({(self.autonomous_timer.time() / 1000):.2f}): {message}"
        if self.autonomous_timer.value() <= 15:
            message = "AUTO" + message
        elif self.driver_controlled_timer.value() < 105:
            message = "DRIVER" + message
        print(message)
        brain.screen.print(message)




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
        x_value = (encoders[0] - encoders[1] - encoders[2] + encoders[3]
                   ) / (self.wheel_distance_CM_to_DEG_coefficient * r2o2) / 4
        y_value = (encoders[0] + encoders[1] + encoders[2] + encoders[3]
                   ) / (self.wheel_distance_CM_to_DEG_coefficient * r2o2) / 4
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
                           * 20, 0, field_based_driving=True, square_input=False)
                wait(0.1, SECONDS)
            print("Done!") 

        else:
            left_motor_a_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            right_motor_a_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            left_motor_b_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            right_motor_b_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2


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
        t = Thread(self.forever_update)
        self.update()

        self.autonomous_timer.reset()

        for step in procedure:
            if "setX" in step:
                self.x_pos = step["setX"]
            if "setY" in step:
                self.y_pos = step["setY"]
            if "actions" in step.keys():
                # if "shootDisk" in step["actions"]:
                # r.shoot_disk()
                if "unload" in step["actions"]:
                    r.unload()
                if "load" in step["actions"]:
                    r.load()
                if "stop_intake" in step["actions"]:
                    r.stop_intake()

                if "spin_roller" in step["actions"]:
                    r.spin_roller()

            if "message" in step.keys():
                print(step["message"])
                brain.screen.print(step["message"])
                brain.screen.next_row()

            # TODO: add threading here
            if "x" in step.keys() and "y" in step.keys():
                print("Going to position:", step["x"], step["y"])
                r.go_to_position(step["x"], step["y"], False, 4)

        t.stop()

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

        self.print(f"Turning to heading {_heading}")

        pass

    # if you are readign this code and are not me (Matt Handzel) and don't know what this is, then look up python getters and setters

    @property
    def position(self):
        return (self.x_pos, self.y_pos)

    @position.setter
    def position(self, _x, _y):
        self.x_pos = _x
        self.y_pos = _y

    @staticmethod
    def unload():
        intake_motor.set_velocity(-50, PERCENT)

    @staticmethod
    def load():
        intake_motor.set_velocity(15, PERCENT)

    @staticmethod
    def stop_intake():
        intake_motor.set_velocity(0, PERCENT)

    def update(self):
        '''
        This is a VERY important function, it should be called once every [0.1 - 0.01] seconds (the faster the better, especially for controls)
        It updates the robot's position based off of the encoders and the gyro
        '''
        # TODO: Set position based off of the gps
        self.delta_time = getattr(time, "ticks_ms")() / \
            1000 - self.previous_update_time

        # prevent divide by zero
        if self.delta_time == 0:
            return
        # Check to see if the robot has moved AT ALL

        self.angular_velocity = inertial.gyro_rate(ZAXIS, VelocityUnits.DPS)
        # is a magic number that makes the gyro work better (experimentall I found that when the gyro reported that it spun 1 time, it actually overshot by about 3 degrees)
        self.total_theta += self.angular_velocity * self.delta_time / 0.99375
        self.theta = self.total_theta - (self.total_theta // 360 * 360)
        
        
        # Use encoders to get our x and y positions so that we can take the derivative and get our velocity
        self.x_from_encoders, self.y_from_encoders = self.get_position_from_encoders()

        delta_x_from_encoders = self.x_from_encoders - self.previous_x_from_encoders
        delta_y_from_encoders = self.y_from_encoders - self.previous_y_from_encoders 

        delta_x_from_encoders, delta_y_from_encoders = rotate_vector_2d(delta_x_from_encoders, delta_y_from_encoders, -self.theta * DEG_TO_RAD)

        # Get the velocity of the robot in deg/s for 4 wheels
        self.x_velocity = delta_x_from_encoders / self.delta_time
        self.y_velocity = delta_y_from_encoders / self.delta_time

        # Velocity of robot from gps perspective
        x_from_gps = 0
        y_from_gps = 0
        alpha = 0
        
        if self.using_gps and gps.quality() >= 100: 
            # Update alpha to value that uses gps
            alpha = 0.1
            x_from_gps = gps.x_position(DistanceUnits.CM)
            y_from_gps = gps.y_position(DistanceUnits.CM)

            delta_x_from_gps = gps.x_position(DistanceUnits.CM) - self.previous_x_from_gps 
            delta_y_from_gps = gps.y_position(DistanceUnits.CM) - self.previous_y_from_gps 

            self.previous_x_from_gps = gps.x_position(DistanceUnits.CM)
            self.previous_y_from_gps = gps.y_position(DistanceUnits.CM)

        # If we have not moved (from the encoders point of view), and the gps is not changing that much, then use the rolling average from the gps
        # If gps is enabled then the low and high pass filter will make the x and y position more stable, if gps is not enabled then the formula won't use gps data (alpha would equal 0)
        self.x_pos += delta_x_from_encoders * (1-alpha) + (self.x_pos - x_from_gps) * alpha
        self.y_pos += delta_y_from_encoders * (1-alpha) + (self.y_pos - y_from_gps) * alpha

        self.previous_wheel_encoder_values = self.get_current_wheel_encoder_values()
        self.previous_theta = self.theta

        self.previous_update_time = getattr(time, "ticks_ms")() / 1000
        self.previous_x_from_encoders = self.x_from_encoders
        self.previous_y_from_encoders = self.y_from_encoders


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
        delta_theta = (get_angle_to_object(
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

    @staticmethod
    def set_launcher_speed():
        pass

    def shoot_disk(self):
        '''
        This function will shoot a disk from the flywheel
        '''
        # Compute speed that the flywheel needs to speed

        # Set the motors to the speed

        # Index a disk out of the magazine
        index_motor.spin_for(FORWARD, 2, TURNS, True)

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

        delta_x = goal.x_pos - self.x_pos
        delta_y = goal.y_pos - self.y_pos

        # TODO: Factor in the position of the shooter
        # distance_to_goal = ((goal.x_pos - self.x_pos) ** 2 + (goal.y_pos - self.y_pos) ** 2) ** 0.5

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

        while t.time() < 500:  # 3 Decellerate
            self.drive(0, 0, 0)

        roller_motor.set_velocity(0, PERCENT)

    @staticmethod
    def spin_roller_backward():
        roller_motor.set_velocity(-100, PERCENT)

##### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ###
    def drive(self, x_vector, y_vector, r_vector, field_based_driving=False, constant_acceleration=True, square_input=True, slow_mode = False):
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
        if field_based_driving:
            x_vector, y_vector = rotate_vector_2d(x_vector, y_vector, self.theta * DEG_TO_RAD)


        # Square the input vectors and divide by 100 for better controls
        if square_input:
            x_vector = (x_vector ** 2) / 100 * sign(x_vector)
            y_vector = (y_vector ** 2) / 100 * sign(y_vector)
            r_vector = (r_vector ** 2) / 100 * sign(r_vector)

        if slow_mode:
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
# END OF PATHS ######### END OF PATHS ######### END OF PATHS ######### END OF PATHS ######### END OF PATHS


def get_angle_to_object(gameobject_1, gameobject_2):
    '''
    RETURNS IN DEGREES
    TODO: if someone doesn't pass a gameobject then this will break everything
    '''

    # Checks to see if both objects are of the GameObject class
    assert isinstance(gameobject_1, GameObject)
    assert isinstance(gameobject_2, GameObject)

    ang = math.atan(((gameobject_2.y_pos) - (gameobject_1.y_pos)) /
                    ((gameobject_2.x_pos) - (gameobject_1.x_pos))) * RAD_TO_DEG
    if gameobject_1.x_pos > gameobject_2.x_pos:
        ang += 180

    if ang > 180:
        ang -= 360

    return ang

######## COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ###


def autonomous():
    # init() # Init a second time (Hopefully to fix the problem with the motors breaking for some reason)
    r.run_autonomous(mostyn)
    r.stop_moving()


# 20 deg off 2 rotations for our method; 26
#

# 19 rotations - 54 seconds 305 deg
# slow, 4 rotations 85 seconds 346 degrees
# slow 6 rotations 101 seconds 342 degrees

# medium speed 20 rotations - 314 deg
# fast speed

def driver_control():
    # Change this to be relative to the match time???
    timer = Timer()
    timer.reset()

    reset_theta_timer = Timer()
    reset_theta_timer.reset()

    r.driver_controlled_timer.reset()


    while True:
        # Update the robot's information
        r.update()

        # Timer to print things out to the terminal every x seconds
        if (timer.time() > 0.1 * 1000):
            print("pos", r.x_pos * (138 - 21.5) / (312.5), r.y_pos* (138 - 21.5) / (312.5), "vel",r.x_velocity, r.y_velocity)
            # print("x:", gps.x_position(DistanceUnits.CM), "y:",gps.y_position(DistanceUnits.CM), "heading:", gps.heading(), "quality:", gps.quality())
            
            timer.reset()

        # launching_motor.set_velocity(controller_1.axis4.position(), PERCENT)
        # launching_motor_1.set_velocity(-100, PERCENT)
        # launching_motor_2.set_velocity(-100, PERCENT)
        
        # robot axis are based on x, y, and r vectors
        r.drive(controller_1.axis4.position(), controller_1.axis3.position(),
                controller_1.axis1.position(), True, True, False, controller_1.buttonR2.pressing())

        # Run the intake subsystem, set the desired speed
        r.intake(controller_1.axis2.position())

        if controller_1.buttonR1.pressing():
            r.shoot_disk()

        if controller_1.buttonL2.pressing():
            r.spin_roller_forward()

        if controller_1.buttonR2.pressing():
            r.spin_roller_backward()

        if not (controller_1.buttonR2.pressing() or controller_1.buttonL2.pressing()):
            roller_motor.set_velocity(0, PERCENT)

        if not controller_1.buttonX.pressing():
            reset_theta_timer.reset()

        # If someone has pressed button x for more than 1 second, reset the orientaiton
        if reset_theta_timer.value() > 1:
            print("Resetting the orientation of the robot!!")
            r.reset_theta()
            reset_theta_timer.reset()

        
        if controller_1.buttonB.pressing():
            r.print_debug_info()

        
        
            


        wait(0.01, SECONDS)


def init():
    # Make it so that the motors stop instead of coast
    # left_motor_a.set_stopping(HOLD)
    # right_motor_a.set_stopping(HOLD)
    # left_motor_b.set_stopping(HOLD)
    # right_motor_b.set_stopping(HOLD)

    # HAVE DAVID TRY THIS OUT
    left_motor_a.set_stopping(BRAKE)
    right_motor_a.set_stopping(BRAKE)
    left_motor_b.set_stopping(BRAKE)
    right_motor_b.set_stopping(BRAKE)

    launching_motor_1.spin(FORWARD)
    launching_motor_2.spin(FORWARD)

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
    while inertial.gyro_rate(ZAXIS) != 0 and t.value() < 10:
        print("Waiting for gyro to init...")
        wait(0.1, SECONDS)


red_goal = GameObject(100, 100)
blue_goal = GameObject(100, 100)

wheel_gear_ratio = 18
ticks_per_revolution = 50 * wheel_gear_ratio

RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

field_length = 356  # CM

r = Robot(0, 0)

r.use_gps(gps.installed())

init()
gps.calibrate()

# autonomous()
driver_control()

# competition = Competition(driver_control, autonomous)
# 138 - 


##! PRIORITY
# TODO: Use the gps and figure out how precise it is
# TODO: Figure out why 1 wheel is moving MUCH faster then the other wheels
# TODO: make a "switch" that can make the robot use the gps for its current position or motor encoder values
# TODO: use threads for auto mode to do multiple commands
# TODO: make autonomous mode use self.update
# TODO: Make a button that makes robot go slowly
# TODO: Work with david and figure out how to make robot driving feel better (more responsive, smooth, etc.)
# TODO: use low and high pass filter for position
# TODO: Revamp the driving command to work with go_to_position and inputting position and rotation vectors 
# TODO: test low pass and high pass filter

# * IMPORTANT
# TODO: Test out the cool Robot.print command and see if it prints on brain and console and how well it does so
# TODO: Make a spin to command
# TODO: test to see if threads die when they lose scope
# TODO: Change these to numpy arrays
# TODO: Figure out the sig figs of the inbuilt Timer class, if its good lets use it instead of time.time_ms
# TODO: Change this to be relative to the match time???
# TODO: When the robot has an x,y position close to the edge of the field, make it so that the robot physically cannot slam into the wall (or use sensors)
# TODO: make an auto path class that has information like description, color, etc.

# Not important
# TODO: Research what the motor.set_max_torque function does
# TODO: Research how to use motor.torque
# TODO: Research how to utilize motor.temperature
# TODO: Make an info screen that shows if there are any status issues wrong with the robot
