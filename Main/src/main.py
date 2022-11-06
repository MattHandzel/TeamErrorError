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
launching_motor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
led_a = Led(brain.three_wire_port.a)
inertial = Inertial(Ports.PORT9)
index_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
roller_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
intake_motor = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
roller_optical = Optical(Ports.PORT16)


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
        self.max_acceleration = 2 * self.max_velocity / 0.1

##### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ###
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
        new_sum_of_encoders = abs(left_motor_a.position(DEGREES)) + abs(right_motor_a.position(
            DEGREES)) + abs(left_motor_b.position(DEGREES)) + abs(right_motor_b.position(DEGREES))
        has_moved_boolean = new_sum_of_encoders != self.previous_sum_of_wheel_encoders
        self.previous_sum_of_wheel_encoders = new_sum_of_encoders
        return has_moved_boolean

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
        self.theta = 0

    def stop_moving(self):
        '''
        Stops all of the motors of the robot (this is good for when we want to switch from run_to_position mode to just spin mode (my nomaclature))
        '''
        left_motor_a.stop()
        left_motor_b.stop()
        right_motor_a.stop()
        right_motor_b.stop()


##### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ###


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

        # Reset the motor positions (this can be omitted)
        # self.reset_motor_positions()

        # self.x_pos = _target_x
        # self.y_pos =_target_y

        if not _pid:
            # These are the target position for each wheel, you find the target positions by finding the direction the wheels
            # are pointing in and then multiplying by the sin(theta) for the x and cos(theta) for theta
            left_motor_a_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            right_motor_a_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            left_motor_b_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            right_motor_b_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2

            x_pos, y_pos = self.get_position_from_encoders()

            while abs(x_pos - _target_x) > 5 or abs(y_pos - _target_y) > 5:
                x_pos, y_pos = self.get_position_from_encoders()

                # Speed towards target will be controlled by PID loop
                error_x = _target_x - x_pos
                error_y = _target_y - y_pos

                # 0.00001 is to prevent division by zero
                theta_to_target = math.atan(error_y / (error_x + 0.00001))

                if error_x < 0:  # expand domain of arctan
                    theta_to_target += math.pi
                
                self.drive(math.cos(theta_to_target) * 50, math.sin(theta_to_target) * 50, 0, square_input=False)



            print("Done!")

        else:
            # PID loop here, it exists once the wheel distance for each wheel is within _pidDistanceThreshold of the target distance
            x_pos, y_pos = self.get_position_from_encoders()
            x_direction_PID = PID(5, 0.1, 0)
            y_direction_PID = PID(5, 0.1, 0)
            magnitude_PID = PID(10, 0.1, 0)

            while abs(x_pos - _target_x) > _pidDistanceThreshold or abs(y_pos - _target_y) > _pidDistanceThreshold:
                x_pos, y_pos = self.get_position_from_encoders()

                # Speed towards target will be controlled by PID loop
                error_x = _target_x - x_pos
                error_y = _target_y - y_pos

                # 0.00001 is to prevent division by zero
                theta_to_target = math.atan(error_y / (error_x + 0.00001))

                if error_x < 0:  # expand domain of arctan
                    theta_to_target += math.pi

                # output_x = x_magnitude_PID.update(error_x, self.delta_time)
                # output_y = y_magnitude_PID.update(error_y, self.delta_time)

                output_magnitude = magnitude_PID.update(
                    (error_x ** 2 + error_y ** 2) ** (0.5), self.delta_time)

                self.drive(output_magnitude * cos(theta_to_target),
                           output_magnitude * sin(theta_to_target), 0, square_input=False)

                print("Target:")
                print("x:", _target_x)
                print("y:", _target_y)
                print("Current:")
                print("x:", x_pos)
                print("y:", y_pos)
                print("Encoders:")
                print((left_motor_a.position(DEGREES) / (self.wheel_distance_CM_to_DEG_coefficient * r2o2), right_motor_a.position(DEGREES) / (self.wheel_distance_CM_to_DEG_coefficient * r2o2),
                      left_motor_b.position(DEGREES) / (self.wheel_distance_CM_to_DEG_coefficient * r2o2), right_motor_b.position(DEGREES) / (self.wheel_distance_CM_to_DEG_coefficient * r2o2)))
                print("Output:")
                print("x:", cos(theta_to_target) * output_magnitude)
                print("y:", sin(theta_to_target) * output_magnitude)
                print("Waiting...")
                print("")

                # Gonne be sleeping based off of self.delta time so that this loop doesn't update more frequently then delta_time (to make sure the PID loop's I error term doesn't explode quickly)
                wait(self.delta_time, SECONDS)
            print("Exiting PID loop...")

    def forever_update(self, _delat=0.01):
        '''
        This is used in a Thread() so that we run the update function constantly (because its very freaking important)
        '''
        while True:
            self.update()
            wait(_delat, SECONDS)

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

        # TODO: Make change these to numpy arrays
        delta_encoders = (self.get_current_wheel_encoder_values(
        ) - self.previous_wheel_encoder_values)

        # Use encoders to get our x and y positions so that we can take the derivative and get our velocity
        x_from_encoders, y_from_encoders = self.get_position_from_encoders()

        # Get the velocity of the robot in deg/s for 4 wheels
        self.x_velocity = x_from_encoders / self.delta_time
        self.y_velocity = y_from_encoders / self.delta_time


        self.angular_velocity = inertial.gyro_rate(ZAXIS)
        self.total_theta += self.angular_velocity * self.delta_time * 0.992 # 0.992 is a magic number that makes the gyro work better (experimentall I found that when the gyro reported that it spun 1 time, it actually overshot by about 3 degrees)
        self.theta = self.total_theta - (self.total_theta // 360 * 360) 


        self.previous_wheel_encoder_values = self.get_current_wheel_encoder_values()
        self.previous_theta = self.theta

        self.previous_update_time = getattr(time, "ticks_ms")() / 1000


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

        delta_x = goal.x_pos - self.y_pos
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
    def drive(self, _x_vector, _y_vector, _r_vector, field_based_driving=False, constant_acceleration=True, square_input=True):
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
            x_vector = cos(self.theta * DEG_TO_RAD) * _x_vector - \
                sin(self.theta * DEG_TO_RAD) * _y_vector
            y_vector = cos(self.theta * DEG_TO_RAD) * _y_vector + \
                sin(self.theta * DEG_TO_RAD) * _x_vector
            r_vector = _r_vector
        else:
            x_vector = _x_vector
            y_vector = _y_vector
            r_vector = _r_vector

        # 10 spins, 9.49 seconds
        # 10 spins, 10.82 seconds
        # 10 spins, 9.6 seconds
        # 10 spins, 10.98 seconds
        # Square the input vectors and divide by 100 for better controls
        if square_input:
            x_vector = (x_vector ** 2) / 100 * sign(x_vector)
            y_vector = (y_vector ** 2) / 100 * sign(y_vector)
            r_vector = (r_vector ** 2) / 100 * sign(r_vector)

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

            # Accelerate the motors to the target velocity
            left_motor_a.set_velocity(left_motor_a_target_velocity, PERCENT)
            right_motor_a.set_velocity(right_motor_a_target_velocity, PERCENT)
            left_motor_b.set_velocity(left_motor_b_target_velocity, PERCENT)
            right_motor_b.set_velocity(right_motor_b_target_velocity, PERCENT)

        else:
            # Alpha controls how responsigve the driving is, higher alpha means more reponsibe
            alpha = 0.4

            if abs(_x_vector) < 3 and abs(_y_vector) < 3 and abs(_r_vector) < 3:
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


red_goal = GameObject(100, 100)
blue_goal = GameObject(100, 100)

wheel_gear_ratio = 18
ticks_per_revolution = 50 * wheel_gear_ratio

RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

field_length = 356  # CM

r = Robot(0, 0)


autonomousProcedureCloseToGoal = [
    {
        "setX": 0,
        "setY": 0,
    },
    {  # Go forward and score preloaded
        "x": 2 * 2.54,
        "y": 6 * 2.54,
        "actions": ["unload"]
    },
    {  # Go backward
        "x": 0,
        "y": -13 * 2.54,  # position is 2, -7
        "actions": ["stop_intake"]
    },
    {  # Move to the left # position -14, 0
        "message": "Moving to the left to pick up disk (x: -13 in, y: -7)",
        "x": -16 * 2.54,
        "y": 0,
    },
    {  # Go forward pick up disk # position will be -16 and ligned up against the barrier
        "message": "Picking up disk (x: 0 in, y: 3)",
        "x": 0 * 2.54,
        "y": 10 * 2.54,
        "actions": ["load"]
    },
    {
        # Go to original position
        "message": "Going to original position",
        "x": 20,
        "y": -4
    },
    {
        "message": "Dropping disk",
        "y": 10,
    },
    {
        "actions": ["unload"]
    },
    {
        "message": "DONE"
    }
]

autonomousSpinRollerClose = [
    {
        "setX": 0,
        "setY": 0,
    },
    {  # SPIN THE ROLLER
        "actions": ["spin_roller"],
        "message": "Spinning roller...",
    },
    {  # Move out of the way of the rollers
        "x": -7 * 2.54,
        "y": 0,
        "message": "Move out of the way of the rollers"
    },
    {  # MOVE TO EJECT DISKS
        "x": 6 * 2.54,
        "y": 58 * 2.54,
        "message": "Moving to eject disks",
    },
    {
        "x": 0,
        "y": 10 * 2.54,
        "actions": ["unload"]
    },
    {
        "actions": ["load"]
    }

]

autonomousCircleTest = [
    {
        "setX": cos(0) * 25,
        "setY": sin(0) * 25,
    }
]

for i in range(10):
    autonomousCircleTest.append({
        "x": cos(i * 0.1) * 25,
        "y": 0,
    })
for i in range(100):
    autonomousCircleTest.append({
        "x": round(cos(i / 10) * 25),
        "y": round(sin(i / 10) * 25),
    })

autonomousPathTest = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : -1.882126929674115, 'y' : -0.6273756432246955},{ 'x' : -5.019005145797621, 'y' : -0.6273756432246955},{ 'x' : -6.273756432247012, 'y' : -1.254751286449391},{ 'x' : -9.410634648370518, 'y' : -1.254751286449391},{ 'x' : -11.292761578044605, 'y' : -1.254751286449391},{ 'x' : -13.802264150943415, 'y' : -1.254751286449391},{ 'x' : -15.684391080617502, 'y' : -1.254751286449391},{ 'x' : -17.566518010291617, 'y' : -1.254751286449391},{ 'x' : -18.821269296741008, 'y' : -1.254751286449391},{ 'x' : -20.703396226415094, 'y' : -1.254751286449391},{ 'x' : -22.58552315608921, 'y' : -1.254751286449391},{ 'x' : -24.467650085763296, 'y' : -0.6273756432246955},{ 'x' : -25.722401372212715, 'y' : -0.6273756432246955},{ 'x' : -27.604528301886802, 'y' : -0.6273756432246955},{ 'x' : -29.486655231560917, 'y' : 0.0},{ 'x' : -31.9961578044597, 'y' : 0.627375643224724},{ 'x' : -33.878284734133814, 'y' : 0.627375643224724},{ 'x' : -35.133036020583205, 'y' : 1.2547512864494195},{ 'x' : -37.01516295025729, 'y' : 1.882126929674115},{ 'x' : -38.89728987993141, 'y' : 2.5095025728988105},{ 'x' : -40.77941680960549, 'y' : 3.7642538593482016},{ 'x' : -42.66154373927961, 'y' : 4.3916295025729255},{ 'x' : -44.543670668953695, 'y' : 5.6463807890223165},{ 'x' : -46.42579759862781, 'y' : 6.273756432247012},{ 'x' : -48.307924528301896, 'y' : 6.901132075471708},{ 'x' : -52.0721783876501, 'y' : 8.783259005145823},{ 'x' : -53.95430531732421, 'y' : 10.038010291595214},{ 'x' : -55.8364322469983, 'y' : 11.292761578044605},{ 'x' : -57.718559176672386, 'y' : 12.547512864494024},{ 'x' : -58.973310463121805, 'y' : 13.17488850771872},{ 'x' : -61.48281303602059, 'y' : 15.057015437392806},{ 'x' : -62.73756432247001, 'y' : 16.311766723842197},{ 'x' : -63.9923156089194, 'y' : 18.193893653516312},{ 'x' : -64.6196912521441, 'y' : 20.0760205831904},{ 'x' : -65.87444253859348, 'y' : 21.33077186963982},{ 'x' : -66.50181818181821, 'y' : 23.212898799313905},{ 'x' : -67.7565694682676, 'y' : 25.722401372212715},{ 'x' : -68.3839451114923, 'y' : 27.604528301886802},{ 'x' : -69.01132075471699, 'y' : 29.486655231560917},{ 'x' : -69.6386963979417, 'y' : 32.623533447684395},{ 'x' : -70.2660720411664, 'y' : 35.133036020583205},{ 'x' : -70.2660720411664, 'y' : 38.26991423670671},{ 'x' : -70.2660720411664, 'y' : 41.40679245283022},{ 'x' : -70.2660720411664, 'y' : 43.916295025729},{ 'x' : -70.2660720411664, 'y' : 47.053173241852505},{ 'x' : -70.2660720411664, 'y' : 50.19005145797601},{ 'x' : -69.01132075471699, 'y' : 54.58168096054891},{ 'x' : -68.3839451114923, 'y' : 58.34593481989711},{ 'x' : -67.1291938250429, 'y' : 62.11018867924531},{ 'x' : -65.24706689536879, 'y' : 65.87444253859348},{ 'x' : -63.3649399656947, 'y' : 69.63869639794169},{ 'x' : -61.48281303602059, 'y' : 72.77557461406519},{ 'x' : -58.973310463121805, 'y' : 75.9124528301887},{ 'x' : -57.718559176672386, 'y' : 77.16720411663809},{ 'x' : -55.8364322469983, 'y' : 79.6767066895369},{ 'x' : -53.32692967409949, 'y' : 81.55883361921099},{ 'x' : -50.19005145797601, 'y' : 84.0683361921098},{ 'x' : -46.42579759862781, 'y' : 86.57783876500858},{ 'x' : -42.03416809605491, 'y' : 88.4599656946827},{ 'x' : -40.1520411663808, 'y' : 90.34209262435678},{ 'x' : -35.7604116638079, 'y' : 92.85159519725559},{ 'x' : -28.231903945111497, 'y' : 96.61584905660379},{ 'x' : -22.58552315608921, 'y' : 99.1253516295026},{ 'x' : -15.057015437392806, 'y' : 102.26222984562608},{ 'x' : -9.410634648370518, 'y' : 104.1443567753002},{ 'x' : -3.136878216123506, 'y' : 105.39910806174959},{ 'x' : 1.8821269296740866, 'y' : 106.65385934819898},{ 'x' : 6.901132075471679, 'y' : 106.65385934819898},{ 'x' : 11.292761578044576, 'y' : 106.65385934819898},{ 'x' : 15.057015437392778, 'y' : 106.02648370497428},{ 'x' : 18.82126929674098, 'y' : 105.39910806174959},{ 'x' : 21.958147512864485, 'y' : 104.77173241852489},{ 'x' : 23.212898799313876, 'y' : 104.1443567753002},{ 'x' : 24.467650085763267, 'y' : 103.5169811320755},{ 'x' : 26.349777015437382, 'y' : 102.26222984562608},{ 'x' : 28.23190394511147, 'y' : 101.00747855917669},{ 'x' : 30.114030874785584, 'y' : 99.7527272727273},{ 'x' : 32.623533447684366, 'y' : 97.87060034305318},{ 'x' : 33.878284734133786, 'y' : 95.9884734133791},{ 'x' : 35.76041166380787, 'y' : 94.73372212692968},{ 'x' : 37.01516295025729, 'y' : 92.85159519725559},{ 'x' : 37.64253859348199, 'y' : 91.5968439108062},{ 'x' : 39.524665523156074, 'y' : 89.71471698113209},{ 'x' : 40.779416809605465, 'y' : 87.832590051458},{ 'x' : 42.034168096054884, 'y' : 85.95046312178388},{ 'x' : 43.288919382504275, 'y' : 84.0683361921098},{ 'x' : 43.91629502572897, 'y' : 82.18620926243568},{ 'x' : 45.17104631217836, 'y' : 79.6767066895369},{ 'x' : 45.798421955403086, 'y' : 77.79457975986278},{ 'x' : 46.42579759862778, 'y' : 75.285077186964},{ 'x' : 47.05317324185248, 'y' : 73.40295025728989},{ 'x' : 47.05317324185248, 'y' : 70.8934476843911},{ 'x' : 47.68054888507717, 'y' : 68.3839451114923},{ 'x' : 47.68054888507717, 'y' : 65.87444253859348},{ 'x' : 48.30792452830187, 'y' : 63.3649399656947},{ 'x' : 48.30792452830187, 'y' : 61.48281303602059},{ 'x' : 48.30792452830187, 'y' : 58.973310463121805},{ 'x' : 48.30792452830187, 'y' : 56.463807890222995},{ 'x' : 47.68054888507717, 'y' : 53.95430531732421},{ 'x' : 47.68054888507717, 'y' : 50.81742710120071},{ 'x' : 47.05317324185248, 'y' : 47.6805488850772},{ 'x' : 46.42579759862778, 'y' : 44.543670668953695},{ 'x' : 46.42579759862778, 'y' : 41.40679245283022},{ 'x' : 45.17104631217836, 'y' : 38.89728987993141},{ 'x' : 43.91629502572897, 'y' : 35.7604116638079},{ 'x' : 42.66154373927958, 'y' : 33.25090909090912},{ 'x' : 40.779416809605465, 'y' : 29.486655231560917},{ 'x' : 38.89728987993138, 'y' : 26.34977701543741},{ 'x' : 37.01516295025729, 'y' : 23.8402744425386},{ 'x' : 34.50566037735848, 'y' : 20.703396226415094},{ 'x' : 31.99615780445967, 'y' : 18.193893653516312},{ 'x' : 29.48665523156089, 'y' : 16.311766723842197},{ 'x' : 28.23190394511147, 'y' : 15.684391080617502},{ 'x' : 25.722401372212687, 'y' : 13.17488850771872},{ 'x' : 22.58552315608918, 'y' : 11.292761578044605},{ 'x' : 19.448644939965675, 'y' : 9.410634648370518},{ 'x' : 16.939142367066893, 'y' : 7.528507718696403},{ 'x' : 14.429639794168082, 'y' : 6.273756432247012},{ 'x' : 13.174888507718691, 'y' : 5.6463807890223165},{ 'x' : 11.292761578044576, 'y' : 4.3916295025729255},{ 'x' : 9.41063464837049, 'y' : 3.136878216123506},{ 'x' : 7.528507718696375, 'y' : 2.5095025728988105},{ 'x' : 5.646380789022288, 'y' : 1.882126929674115},{ 'x' : 3.764253859348173, 'y' : 0.627375643224724},{ 'x' : 1.8821269296740866, 'y' : 0.0},{ 'x' : 0.0, 'y' : -0.6273756432246955},{ 'x' : -1.2547512864494195, 'y' : -1.254751286449391},{ 'x' : -2.5095025728988105, 'y' : -1.254751286449391},]
autonomousPathTestZig = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : -0.6273756432246955, 'y' : 3.1368782161234776},{ 'x' : -1.254751286449391, 'y' : 6.901132075471708},{ 'x' : -1.8821269296741008, 'y' : 11.9201372212693},{ 'x' : -1.8821269296741008, 'y' : 16.311766723842197},{ 'x' : -1.8821269296741008, 'y' : 20.703396226415094},{ 'x' : -2.5095025728987963, 'y' : 25.722401372212687},{ 'x' : -2.5095025728987963, 'y' : 30.114030874785584},{ 'x' : -2.5095025728987963, 'y' : 35.133036020583205},{ 'x' : -2.5095025728987963, 'y' : 38.89728987993141},{ 'x' : -3.136878216123492, 'y' : 43.288919382504304},{ 'x' : -3.136878216123492, 'y' : 47.053173241852505},{ 'x' : -3.136878216123492, 'y' : 50.81742710120071},{ 'x' : -3.136878216123492, 'y' : 55.209056603773604},{ 'x' : -3.136878216123492, 'y' : 58.97331046312178},{ 'x' : -3.7642538593481873, 'y' : 63.9923156089194},{ 'x' : -3.7642538593481873, 'y' : 68.3839451114923},{ 'x' : -3.7642538593481873, 'y' : 70.26607204116638},{ 'x' : -3.7642538593481873, 'y' : 72.77557461406519},{ 'x' : -3.7642538593481873, 'y' : 75.9124528301887},{ 'x' : -3.136878216123492, 'y' : 79.04933104631218},{ 'x' : -3.136878216123492, 'y' : 81.55883361921099},{ 'x' : -2.5095025728987963, 'y' : 84.0683361921098},{ 'x' : -2.5095025728987963, 'y' : 85.32308747855919},{ 'x' : -1.8821269296741008, 'y' : 86.57783876500858},{ 'x' : -1.8821269296741008, 'y' : 87.83259005145797},{ 'x' : -1.8821269296741008, 'y' : 89.08734133790739},{ 'x' : -0.6273756432246955, 'y' : 87.83259005145797},{ 'x' : 0.0, 'y' : 84.69571183533449},{ 'x' : 0.6273756432247097, 'y' : 80.93145797598629},{ 'x' : 0.6273756432247097, 'y' : 77.16720411663809},{ 'x' : 1.2547512864494053, 'y' : 72.77557461406519},{ 'x' : 1.2547512864494053, 'y' : 68.3839451114923},{ 'x' : 1.2547512864494053, 'y' : 62.11018867924528},{ 'x' : 1.2547512864494053, 'y' : 57.09118353344769},{ 'x' : 1.2547512864494053, 'y' : 52.0721783876501},{ 'x' : 1.2547512864494053, 'y' : 47.6805488850772},{ 'x' : 1.2547512864494053, 'y' : 43.916295025729},{ 'x' : 1.2547512864494053, 'y' : 39.5246655231561},{ 'x' : 1.2547512864494053, 'y' : 37.64253859348199},{ 'x' : 1.2547512864494053, 'y' : 33.878284734133786},{ 'x' : 1.2547512864494053, 'y' : 29.48665523156089},{ 'x' : 1.8821269296741008, 'y' : 25.722401372212687},{ 'x' : 1.8821269296741008, 'y' : 22.58552315608921},{ 'x' : 1.8821269296741008, 'y' : 20.703396226415094},{ 'x' : 2.5095025728988105, 'y' : 16.939142367066893},{ 'x' : 3.136878216123506, 'y' : 13.174888507718691},{ 'x' : 3.136878216123506, 'y' : 10.038010291595214},{ 'x' : 3.136878216123506, 'y' : 6.901132075471708},{ 'x' : 3.136878216123506, 'y' : 5.6463807890223165},{ 'x' : 3.136878216123506, 'y' : 3.76425385934823},{ 'x' : 3.136878216123506, 'y' : 1.8821269296740866},{ 'x' : 3.136878216123506, 'y' : 0.6273756432246955},{ 'x' : 3.136878216123506, 'y' : -0.6273756432246955},{ 'x' : 2.5095025728988105, 'y' : 3.76425385934823},{ 'x' : 1.8821269296741008, 'y' : 9.41063464837049},{ 'x' : 1.2547512864494053, 'y' : 15.684391080617502},{ 'x' : 0.6273756432247097, 'y' : 23.212898799313905},{ 'x' : -0.6273756432246955, 'y' : 35.7604116638079},{ 'x' : -1.8821269296741008, 'y' : 45.798421955403086},{ 'x' : -1.8821269296741008, 'y' : 50.81742710120071},{ 'x' : -3.136878216123492, 'y' : 60.85543739279589},{ 'x' : -4.391629502572897, 'y' : 67.7565694682676},{ 'x' : -5.019005145797593, 'y' : 70.89344768439108},{ 'x' : -4.391629502572897, 'y' : 72.1481989708405},{ 'x' : -3.7642538593481873, 'y' : 74.03032590051458},{ 'x' : -2.5095025728987963, 'y' : 75.9124528301887},{ 'x' : -1.8821269296741008, 'y' : 77.79457975986278},{ 'x' : -1.254751286449391, 'y' : 79.04933104631218},{ 'x' : -0.6273756432246955, 'y' : 80.3040823327616},{ 'x' : -0.6273756432246955, 'y' : 81.55883361921099},{ 'x' : 0.0, 'y' : 82.81358490566038},{ 'x' : 1.2547512864494053, 'y' : 79.6767066895369},{ 'x' : 1.8821269296741008, 'y' : 75.9124528301887},{ 'x' : 1.8821269296741008, 'y' : 72.1481989708405},{ 'x' : 1.8821269296741008, 'y' : 70.26607204116638},{ 'x' : 2.5095025728988105, 'y' : 65.24706689536879},{ 'x' : 1.8821269296741008, 'y' : 59.6006861063465},{ 'x' : 1.8821269296741008, 'y' : 54.58168096054888},{ 'x' : 1.8821269296741008, 'y' : 48.93530017152659},{ 'x' : 1.2547512864494053, 'y' : 43.916295025729},{ 'x' : 1.2547512864494053, 'y' : 40.77941680960549},{ 'x' : 1.2547512864494053, 'y' : 36.387787307032596},{ 'x' : 0.6273756432247097, 'y' : 31.9961578044597},{ 'x' : 0.6273756432247097, 'y' : 28.231903945111497},{ 'x' : 0.6273756432247097, 'y' : 24.467650085763296},{ 'x' : 0.6273756432247097, 'y' : 20.703396226415094},{ 'x' : 0.6273756432247097, 'y' : 17.56651801029159},{ 'x' : 0.6273756432247097, 'y' : 15.057015437392806},{ 'x' : 0.6273756432247097, 'y' : 13.174888507718691},{ 'x' : 0.6273756432247097, 'y' : 11.9201372212693},{ 'x' : 0.6273756432247097, 'y' : 10.038010291595214},{ 'x' : 0.6273756432247097, 'y' : 8.783259005145794},{ 'x' : 0.6273756432247097, 'y' : 7.528507718696403},{ 'x' : 0.6273756432247097, 'y' : 6.273756432247012},{ 'x' : 0.6273756432247097, 'y' : 5.019005145797621},{ 'x' : 0.6273756432247097, 'y' : 3.76425385934823},{ 'x' : 1.2547512864494053, 'y' : 2.509502572898782},{ 'x' : 1.2547512864494053, 'y' : 4.3916295025729255},{ 'x' : 1.2547512864494053, 'y' : 7.528507718696403},{ 'x' : 0.6273756432247097, 'y' : 12.547512864493996},{ 'x' : 0.6273756432247097, 'y' : 17.56651801029159},{ 'x' : 0.6273756432247097, 'y' : 21.958147512864514},{ 'x' : 0.0, 'y' : 28.231903945111497},{ 'x' : 0.0, 'y' : 33.878284734133786},{ 'x' : 0.0, 'y' : 38.89728987993141},{ 'x' : 0.0, 'y' : 42.034168096054884},{ 'x' : 0.0, 'y' : 45.798421955403086},{ 'x' : 0.0, 'y' : 49.56267581475129},{ 'x' : 0.0, 'y' : 53.954305317324184},{ 'x' : 0.6273756432247097, 'y' : 57.718559176672386},{ 'x' : 0.6273756432247097, 'y' : 60.2280617495712},{ 'x' : 0.6273756432247097, 'y' : 62.73756432246998},{ 'x' : 1.2547512864494053, 'y' : 65.87444253859348},{ 'x' : 1.8821269296741008, 'y' : 68.3839451114923},{ 'x' : 1.8821269296741008, 'y' : 70.89344768439108},{ 'x' : 2.5095025728988105, 'y' : 72.77557461406519},{ 'x' : 2.5095025728988105, 'y' : 75.9124528301887},{ 'x' : 3.136878216123506, 'y' : 78.42195540308748},{ 'x' : 3.136878216123506, 'y' : 79.6767066895369},{ 'x' : 3.7642538593482016, 'y' : 81.55883361921099},{ 'x' : 4.391629502572911, 'y' : 82.81358490566038},{ 'x' : 4.391629502572911, 'y' : 84.0683361921098},{ 'x' : 4.391629502572911, 'y' : 85.32308747855919},]
autonomousPathTestReal = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : 1.2547512864494053, 'y' : 0.6273756432246955},{ 'x' : 1.2547512864494053, 'y' : 1.8821269296740866},{ 'x' : 1.2547512864494053, 'y' : 3.1368782161235345},{ 'x' : 1.2547512864494053, 'y' : 4.3916295025729255},{ 'x' : 1.8821269296741008, 'y' : 5.6463807890223165},{ 'x' : 1.8821269296741008, 'y' : 7.528507718696403},{ 'x' : 2.5095025728988105, 'y' : 8.783259005145794},{ 'x' : 2.5095025728988105, 'y' : 10.038010291595185},{ 'x' : 3.136878216123506, 'y' : 11.920137221269329},{ 'x' : 3.136878216123506, 'y' : 13.17488850771872},{ 'x' : 3.136878216123506, 'y' : 14.42963979416811},{ 'x' : 3.136878216123506, 'y' : 15.684391080617502},{ 'x' : 4.391629502572897, 'y' : 16.311766723842197},{ 'x' : 5.646380789022302, 'y' : 17.56651801029159},{ 'x' : 7.528507718696403, 'y' : 17.56651801029159},{ 'x' : 9.410634648370504, 'y' : 17.56651801029159},{ 'x' : 11.292761578044605, 'y' : 17.56651801029159},{ 'x' : 15.057015437392806, 'y' : 17.56651801029159},{ 'x' : 18.821269296740994, 'y' : 17.56651801029159},{ 'x' : 23.212898799313905, 'y' : 17.56651801029159},{ 'x' : 28.231903945111497, 'y' : 16.939142367066893},{ 'x' : 33.25090909090909, 'y' : 16.939142367066893},{ 'x' : 37.642538593482, 'y' : 16.939142367066893},{ 'x' : 43.28891938250429, 'y' : 16.311766723842197},{ 'x' : 48.935300171526606, 'y' : 16.311766723842197},{ 'x' : 53.3269296740995, 'y' : 15.684391080617502},{ 'x' : 58.345934819897096, 'y' : 15.684391080617502},{ 'x' : 63.36493996569469, 'y' : 15.684391080617502},{ 'x' : 68.38394511149228, 'y' : 15.057015437392806},{ 'x' : 70.2660720411664, 'y' : 15.057015437392806},{ 'x' : 74.6577015437393, 'y' : 14.42963979416811},{ 'x' : 79.67670668953689, 'y' : 13.802264150943415},{ 'x' : 84.06833619210978, 'y' : 13.17488850771872},{ 'x' : 86.5778387650086, 'y' : 13.17488850771872},{ 'x' : 90.96946826758149, 'y' : 12.547512864494024},{ 'x' : 95.98847341337908, 'y' : 11.920137221269329},{ 'x' : 101.00747855917668, 'y' : 11.920137221269329},{ 'x' : 104.77173241852488, 'y' : 11.292761578044633},{ 'x' : 109.79073756432247, 'y' : 10.665385934819938},{ 'x' : 114.80974271012009, 'y' : 10.665385934819938},{ 'x' : 119.20137221269299, 'y' : 10.038010291595185},{ 'x' : 124.22037735849058, 'y' : 10.038010291595185},{ 'x' : 129.2393825042882, 'y' : 10.038010291595185},{ 'x' : 133.63101200686106, 'y' : 10.038010291595185},{ 'x' : 138.65001715265868, 'y' : 10.038010291595185},{ 'x' : 143.04164665523155, 'y' : 10.665385934819938},{ 'x' : 147.43327615780447, 'y' : 10.665385934819938},{ 'x' : 151.8249056603774, 'y' : 10.665385934819938},{ 'x' : 155.58915951972557, 'y' : 10.665385934819938},{ 'x' : 159.35341337907374, 'y' : 10.665385934819938},{ 'x' : 162.49029159519728, 'y' : 10.665385934819938},{ 'x' : 165.62716981132075, 'y' : 10.038010291595185},{ 'x' : 168.7640480274443, 'y' : 10.038010291595185},{ 'x' : 171.27355060034307, 'y' : 9.41063464837049},{ 'x' : 173.15567753001716, 'y' : 9.41063464837049},{ 'x' : 174.41042881646655, 'y' : 9.41063464837049},{ 'x' : 175.66518010291594, 'y' : 9.41063464837049},{ 'x' : 176.91993138936533, 'y' : 9.41063464837049},{ 'x' : 178.17468267581472, 'y' : 9.41063464837049},{ 'x' : 179.42943396226417, 'y' : 9.41063464837049},{ 'x' : 178.17468267581472, 'y' : 10.038010291595185},{ 'x' : 176.29255574614064, 'y' : 10.665385934819938},{ 'x' : 175.03780445969124, 'y' : 10.665385934819938},{ 'x' : 173.15567753001716, 'y' : 11.292761578044633},{ 'x' : 171.27355060034307, 'y' : 11.920137221269329},{ 'x' : 168.7640480274443, 'y' : 12.547512864494024},{ 'x' : 166.25454545454545, 'y' : 13.17488850771872},{ 'x' : 163.11766723842197, 'y' : 13.802264150943415},{ 'x' : 161.2355403087479, 'y' : 14.42963979416811},{ 'x' : 159.35341337907374, 'y' : 14.42963979416811},{ 'x' : 158.09866209262435, 'y' : 15.057015437392806},{ 'x' : 156.84391080617496, 'y' : 15.057015437392806},{ 'x' : 155.58915951972557, 'y' : 15.684391080617502},{ 'x' : 154.33440823327618, 'y' : 15.684391080617502},{ 'x' : 153.0796569468268, 'y' : 15.684391080617502},{ 'x' : 151.8249056603774, 'y' : 16.311766723842197},{ 'x' : 150.57015437392795, 'y' : 17.56651801029159},{ 'x' : 149.94277873070325, 'y' : 18.82126929674098},{ 'x' : 149.31540308747856, 'y' : 20.076020583190427},{ 'x' : 148.68802744425386, 'y' : 21.958147512864514},{ 'x' : 148.06065180102917, 'y' : 24.467650085763296},{ 'x' : 147.43327615780447, 'y' : 26.977152658662078},{ 'x' : 146.80590051457978, 'y' : 30.114030874785612},{ 'x' : 146.17852487135508, 'y' : 33.878284734133786},{ 'x' : 145.55114922813038, 'y' : 38.26991423670671},{ 'x' : 144.9237735849057, 'y' : 42.66154373927958},{ 'x' : 144.296397941681, 'y' : 45.798421955403114},{ 'x' : 143.6690222984563, 'y' : 47.053173241852505},{ 'x' : 143.6690222984563, 'y' : 49.56267581475129},{ 'x' : 143.6690222984563, 'y' : 53.32692967409952},{ 'x' : 143.6690222984563, 'y' : 57.09118353344769},{ 'x' : 143.6690222984563, 'y' : 60.228061749571225},{ 'x' : 143.6690222984563, 'y' : 62.11018867924531},{ 'x' : 143.6690222984563, 'y' : 63.3649399656947},{ 'x' : 143.6690222984563, 'y' : 65.24706689536879},{ 'x' : 143.6690222984563, 'y' : 66.50181818181818},{ 'x' : 145.55114922813038, 'y' : 67.12919382504288},{ 'x' : 147.43327615780447, 'y' : 67.12919382504288},{ 'x' : 149.94277873070325, 'y' : 67.12919382504288},{ 'x' : 152.4522813036021, 'y' : 67.12919382504288},{ 'x' : 154.96178387650087, 'y' : 67.12919382504288},{ 'x' : 156.84391080617496, 'y' : 67.12919382504288},{ 'x' : 158.09866209262435, 'y' : 67.12919382504288},{ 'x' : 159.35341337907374, 'y' : 67.12919382504288},{ 'x' : 160.6081646655232, 'y' : 67.12919382504288},{ 'x' : 159.35341337907374, 'y' : 67.12919382504288},{ 'x' : 158.09866209262435, 'y' : 66.50181818181818},{ 'x' : 156.21653516295027, 'y' : 65.87444253859348},{ 'x' : 154.33440823327618, 'y' : 65.87444253859348},{ 'x' : 152.4522813036021, 'y' : 65.24706689536879},{ 'x' : 151.19753001715264, 'y' : 65.24706689536879},{ 'x' : 148.68802744425386, 'y' : 65.24706689536879},{ 'x' : 147.43327615780447, 'y' : 65.24706689536879},{ 'x' : 146.17852487135508, 'y' : 64.6196912521441},{ 'x' : 144.9237735849057, 'y' : 64.6196912521441},{ 'x' : 143.6690222984563, 'y' : 67.12919382504288},{ 'x' : 143.6690222984563, 'y' : 68.38394511149232},{ 'x' : 143.6690222984563, 'y' : 70.26607204116641},{ 'x' : 143.6690222984563, 'y' : 71.5208233276158},{ 'x' : 143.6690222984563, 'y' : 72.77557461406519},{ 'x' : 143.6690222984563, 'y' : 74.03032590051458},{ 'x' : 143.6690222984563, 'y' : 75.28507718696397},{ 'x' : 143.6690222984563, 'y' : 76.53982847341342},{ 'x' : 143.6690222984563, 'y' : 75.28507718696397},{ 'x' : 143.6690222984563, 'y' : 74.03032590051458},{ 'x' : 144.296397941681, 'y' : 72.77557461406519},{ 'x' : 144.296397941681, 'y' : 74.03032590051458},{ 'x' : 144.296397941681, 'y' : 75.28507718696397},{ 'x' : 144.296397941681, 'y' : 76.53982847341342},{ 'x' : 143.6690222984563, 'y' : 77.79457975986281},{ 'x' : 143.6690222984563, 'y' : 79.6767066895369},{ 'x' : 143.04164665523155, 'y' : 80.93145797598629},{ 'x' : 143.04164665523155, 'y' : 82.18620926243568},{ 'x' : 143.04164665523155, 'y' : 84.06833619210977},{ 'x' : 143.6690222984563, 'y' : 85.32308747855922},{ 'x' : 143.04164665523155, 'y' : 86.57783876500861},{ 'x' : 143.04164665523155, 'y' : 87.832590051458},{ 'x' : 144.296397941681, 'y' : 87.832590051458},{ 'x' : 146.80590051457978, 'y' : 87.832590051458},{ 'x' : 149.31540308747856, 'y' : 87.2052144082333},{ 'x' : 151.19753001715264, 'y' : 86.57783876500861},{ 'x' : 154.33440823327618, 'y' : 86.57783876500861},{ 'x' : 156.21653516295027, 'y' : 85.95046312178391},{ 'x' : 157.47128644939966, 'y' : 85.95046312178391},{ 'x' : 158.72603773584905, 'y' : 85.95046312178391},{ 'x' : 159.98078902229844, 'y' : 85.32308747855922},{ 'x' : 156.84391080617496, 'y' : 85.95046312178391},{ 'x' : 154.33440823327618, 'y' : 85.95046312178391},{ 'x' : 149.94277873070325, 'y' : 86.57783876500861},{ 'x' : 146.80590051457978, 'y' : 86.57783876500861},{ 'x' : 144.9237735849057, 'y' : 86.57783876500861},{ 'x' : 143.04164665523155, 'y' : 86.57783876500861},{ 'x' : 141.78689536878215, 'y' : 87.2052144082333},{ 'x' : 140.53214408233276, 'y' : 87.2052144082333},{ 'x' : 139.27739279588337, 'y' : 87.2052144082333},{ 'x' : 139.27739279588337, 'y' : 89.08734133790739},{ 'x' : 138.65001715265868, 'y' : 91.59684391080617},{ 'x' : 138.65001715265868, 'y' : 93.47897084048032},{ 'x' : 138.65001715265868, 'y' : 96.61584905660379},{ 'x' : 138.65001715265868, 'y' : 99.12535162950257},{ 'x' : 138.65001715265868, 'y' : 101.63485420240141},{ 'x' : 138.65001715265868, 'y' : 104.77173241852489},{ 'x' : 139.27739279588337, 'y' : 107.28123499142367},{ 'x' : 139.27739279588337, 'y' : 109.79073756432251},{ 'x' : 139.27739279588337, 'y' : 111.0454888507719},{ 'x' : 139.27739279588337, 'y' : 112.92761578044599},{ 'x' : 139.27739279588337, 'y' : 114.18236706689538},{ 'x' : 141.15951972555746, 'y' : 115.43711835334477},{ 'x' : 143.6690222984563, 'y' : 115.43711835334477},{ 'x' : 146.80590051457978, 'y' : 115.43711835334477},{ 'x' : 150.57015437392795, 'y' : 115.43711835334477},{ 'x' : 153.70703259005148, 'y' : 116.0644939965695},{ 'x' : 156.21653516295027, 'y' : 116.0644939965695},{ 'x' : 157.47128644939966, 'y' : 116.0644939965695},{ 'x' : 159.98078902229844, 'y' : 116.69186963979419},{ 'x' : 161.2355403087479, 'y' : 116.69186963979419},{ 'x' : 162.49029159519728, 'y' : 116.69186963979419},{ 'x' : 159.98078902229844, 'y' : 116.69186963979419},{ 'x' : 157.47128644939966, 'y' : 116.69186963979419},{ 'x' : 154.96178387650087, 'y' : 116.0644939965695},{ 'x' : 153.0796569468268, 'y' : 116.0644939965695},{ 'x' : 151.19753001715264, 'y' : 116.0644939965695},{ 'x' : 149.94277873070325, 'y' : 116.0644939965695},{ 'x' : 148.68802744425386, 'y' : 116.0644939965695},{ 'x' : 147.43327615780447, 'y' : 116.0644939965695},{ 'x' : 146.80590051457978, 'y' : 114.80974271012008},{ 'x' : 146.17852487135508, 'y' : 110.41811320754721},{ 'x' : 146.17852487135508, 'y' : 109.16336192109776},{ 'x' : 146.17852487135508, 'y' : 107.90861063464837},{ 'x' : 146.17852487135508, 'y' : 106.02648370497428},{ 'x' : 146.17852487135508, 'y' : 104.1443567753002},{ 'x' : 146.17852487135508, 'y' : 101.63485420240141},{ 'x' : 146.80590051457978, 'y' : 99.75272727272727},{ 'x' : 146.80590051457978, 'y' : 97.24322469982849},{ 'x' : 146.80590051457978, 'y' : 95.3610977701544},{ 'x' : 146.80590051457978, 'y' : 92.85159519725556},{ 'x' : 146.80590051457978, 'y' : 90.96946826758148},{ 'x' : 147.43327615780447, 'y' : 87.832590051458},{ 'x' : 147.43327615780447, 'y' : 85.32308747855922},{ 'x' : 147.43327615780447, 'y' : 82.81358490566038},{ 'x' : 148.06065180102917, 'y' : 79.0493310463122},{ 'x' : 148.06065180102917, 'y' : 74.65770154373928},{ 'x' : 147.43327615780447, 'y' : 72.1481989708405},{ 'x' : 147.43327615780447, 'y' : 69.01132075471702},{ 'x' : 148.06065180102917, 'y' : 65.24706689536879},{ 'x' : 148.06065180102917, 'y' : 62.73756432247001},{ 'x' : 148.68802744425386, 'y' : 58.97331046312178},{ 'x' : 148.68802744425386, 'y' : 56.463807890222995},{ 'x' : 148.68802744425386, 'y' : 53.95430531732421},{ 'x' : 149.31540308747856, 'y' : 52.072178387650126},{ 'x' : 149.31540308747856, 'y' : 50.81742710120068},{ 'x' : 149.94277873070325, 'y' : 47.6805488850772},{ 'x' : 150.57015437392795, 'y' : 45.798421955403114},{ 'x' : 151.19753001715264, 'y' : 43.288919382504275},{ 'x' : 151.8249056603774, 'y' : 40.1520411663808},{ 'x' : 151.8249056603774, 'y' : 38.26991423670671},{ 'x' : 151.8249056603774, 'y' : 37.01516295025732},{ 'x' : 151.8249056603774, 'y' : 34.50566037735848},{ 'x' : 151.8249056603774, 'y' : 33.25090909090909},{ 'x' : 151.8249056603774, 'y' : 31.368782161235004},{ 'x' : 152.4522813036021, 'y' : 29.486655231560917},{ 'x' : 152.4522813036021, 'y' : 28.231903945111526},{ 'x' : 152.4522813036021, 'y' : 26.977152658662078},{ 'x' : 152.4522813036021, 'y' : 25.722401372212687},{ 'x' : 152.4522813036021, 'y' : 24.467650085763296},{ 'x' : 152.4522813036021, 'y' : 23.212898799313905},{ 'x' : 153.0796569468268, 'y' : 21.958147512864514},{ 'x' : 154.96178387650087, 'y' : 20.703396226415123},{ 'x' : 156.84391080617496, 'y' : 19.44864493996573},{ 'x' : 159.98078902229844, 'y' : 18.193893653516284},{ 'x' : 162.49029159519728, 'y' : 17.56651801029159},{ 'x' : 166.88192109777015, 'y' : 16.939142367066893},{ 'x' : 171.90092624356777, 'y' : 16.311766723842197},{ 'x' : 175.66518010291594, 'y' : 16.311766723842197},{ 'x' : 178.80205831903947, 'y' : 16.311766723842197},{ 'x' : 181.31156089193826, 'y' : 16.311766723842197},{ 'x' : 183.82106346483704, 'y' : 16.311766723842197},{ 'x' : 185.07581475128643, 'y' : 16.311766723842197},{ 'x' : 186.33056603773582, 'y' : 16.311766723842197},{ 'x' : 187.58531732418527, 'y' : 16.311766723842197},{ 'x' : 183.82106346483704, 'y' : 18.193893653516284},{ 'x' : 180.05680960548887, 'y' : 18.82126929674098},{ 'x' : 176.29255574614064, 'y' : 19.44864493996573},{ 'x' : 172.52830188679246, 'y' : 20.076020583190427},{ 'x' : 168.7640480274443, 'y' : 20.703396226415123},{ 'x' : 164.99979416809606, 'y' : 20.703396226415123},{ 'x' : 161.2355403087479, 'y' : 21.33077186963982},{ 'x' : 157.47128644939966, 'y' : 21.33077186963982},{ 'x' : 154.33440823327618, 'y' : 21.33077186963982},{ 'x' : 152.4522813036021, 'y' : 21.958147512864514},{ 'x' : 150.57015437392795, 'y' : 21.958147512864514},{ 'x' : 148.68802744425386, 'y' : 21.958147512864514},{ 'x' : 147.43327615780447, 'y' : 21.958147512864514},{ 'x' : 146.17852487135508, 'y' : 21.958147512864514},{ 'x' : 144.9237735849057, 'y' : 20.703396226415123},{ 'x' : 145.55114922813038, 'y' : 21.958147512864514},{ 'x' : 145.55114922813038, 'y' : 23.212898799313905},{ 'x' : 145.55114922813038, 'y' : 25.09502572898799},{ 'x' : 145.55114922813038, 'y' : 27.60452830188683},{ 'x' : 144.9237735849057, 'y' : 30.741406518010308},{ 'x' : 144.9237735849057, 'y' : 33.878284734133786},{ 'x' : 144.9237735849057, 'y' : 35.76041166380793},{ 'x' : 144.9237735849057, 'y' : 37.642538593482016},{ 'x' : 144.296397941681, 'y' : 42.034168096054884},{ 'x' : 144.296397941681, 'y' : 46.42579759862781},{ 'x' : 143.6690222984563, 'y' : 50.19005145797598},{ 'x' : 143.6690222984563, 'y' : 52.69955403087482},{ 'x' : 143.6690222984563, 'y' : 54.58168096054891},{ 'x' : 143.6690222984563, 'y' : 55.8364322469983},{ 'x' : 143.6690222984563, 'y' : 57.09118353344769},{ 'x' : 143.6690222984563, 'y' : 58.34593481989708},{ 'x' : 143.6690222984563, 'y' : 59.60068610634647},{ 'x' : 143.04164665523155, 'y' : 60.85543739279592},{ 'x' : 143.04164665523155, 'y' : 62.11018867924531},{ 'x' : 142.41427101200685, 'y' : 63.9923156089194},{ 'x' : 141.78689536878215, 'y' : 65.24706689536879},{ 'x' : 141.15951972555746, 'y' : 66.50181818181818},{ 'x' : 139.90476843910807, 'y' : 67.12919382504288},{ 'x' : 138.65001715265868, 'y' : 67.12919382504288},{ 'x' : 136.7678902229846, 'y' : 67.12919382504288},{ 'x' : 133.63101200686106, 'y' : 67.12919382504288},{ 'x' : 131.74888507718697, 'y' : 67.12919382504288},{ 'x' : 129.2393825042882, 'y' : 67.12919382504288},{ 'x' : 127.35725557461409, 'y' : 67.12919382504288},{ 'x' : 123.59300171526588, 'y' : 67.12919382504288},{ 'x' : 119.20137221269299, 'y' : 67.12919382504288},{ 'x' : 115.43711835334479, 'y' : 67.75656946826757},{ 'x' : 110.41811320754717, 'y' : 68.38394511149232},{ 'x' : 104.77173241852488, 'y' : 68.38394511149232},{ 'x' : 98.4979759862779, 'y' : 69.01132075471702},{ 'x' : 88.45996569468268, 'y' : 69.63869639794171},{ 'x' : 80.30408233276158, 'y' : 70.26607204116641},{ 'x' : 75.91245283018868, 'y' : 70.8934476843911},{ 'x' : 65.8744425385935, 'y' : 71.5208233276158},{ 'x' : 58.345934819897096, 'y' : 72.1481989708405},{ 'x' : 49.5626758147513, 'y' : 73.40295025728989},{ 'x' : 42.661543739279594, 'y' : 73.40295025728989},{ 'x' : 35.7604116638079, 'y' : 74.03032590051458},{ 'x' : 30.1140308747856, 'y' : 74.03032590051458},{ 'x' : 26.349777015437397, 'y' : 74.03032590051458},{ 'x' : 20.0760205831904, 'y' : 74.65770154373928},{ 'x' : 14.429639794168097, 'y' : 74.65770154373928},{ 'x' : 8.155883361921099, 'y' : 74.65770154373928},{ 'x' : 2.5095025728988105, 'y' : 74.65770154373928},{ 'x' : -2.5095025728987963, 'y' : 74.65770154373928},{ 'x' : -7.528507718696389, 'y' : 74.65770154373928},{ 'x' : -11.920137221269286, 'y' : 74.65770154373928},{ 'x' : -15.684391080617488, 'y' : 74.65770154373928},{ 'x' : -19.44864493996569, 'y' : 74.65770154373928},{ 'x' : -23.21289879931389, 'y' : 75.28507718696397},{ 'x' : -26.34977701543739, 'y' : 74.65770154373928},{ 'x' : -27.604528301886788, 'y' : 74.65770154373928},{ 'x' : -30.11403087478559, 'y' : 74.65770154373928},{ 'x' : -32.62353344768439, 'y' : 74.65770154373928},{ 'x' : -35.760411663807886, 'y' : 74.03032590051458},{ 'x' : -38.26991423670669, 'y' : 74.03032590051458},{ 'x' : -40.779416809605486, 'y' : 74.03032590051458},{ 'x' : -43.916295025728985, 'y' : 74.03032590051458},{ 'x' : -47.053173241852484, 'y' : 74.03032590051458},{ 'x' : -49.56267581475129, 'y' : 74.03032590051458},{ 'x' : -51.44480274442539, 'y' : 73.40295025728989},{ 'x' : -52.69955403087478, 'y' : 73.40295025728989},{ 'x' : -53.954305317324184, 'y' : 73.40295025728989},{ 'x' : -55.836432246998285, 'y' : 74.03032590051458},{ 'x' : -57.09118353344768, 'y' : 74.03032590051458},{ 'x' : -58.34593481989708, 'y' : 74.03032590051458},{ 'x' : -59.60068610634649, 'y' : 74.03032590051458},{ 'x' : -60.85543739279588, 'y' : 74.03032590051458},]
def get_angle_to_object(gameobject_1, gameobject_2):
    '''
    RETURNS IN DEGREES
    TODO: if someone doesn't pass a gameobject then this will break everything
    '''
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
    r.run_autonomous(autonomousPathTestReal)
    r.stop_moving()


# 20 deg off 2 rotations for our method; 26 
# 

# 19 rotations - 54 seconds 305 deg
# slow, 4 rotations 85 seconds 346 degrees
# slow 6 rotations 101 seconds 342 degrees 

def driver_control():
    # Change this to be relative to the match time???
    timer = Timer()
    timer.reset()

    while True:
        # Update the robot's information
        r.update()
        # Timer to print things out to the terminal every x seconds
        if (timer.time() > 0.5 * 1000):
            print("theta:", r.theta, "num_rotations:", (r.total_theta // 360) + 1, " time:", brain.timer.time() / 1000)

            timer.reset()

        # print(r.get_position_from_encoders())
        # robot axis are based on x, y, and r vectors
        r.drive(controller_1.axis4.position(), controller_1.axis3.position(),
                controller_1.axis1.position(), True, True)

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

        wait(0.01, SECONDS)



def init():
    # Spin the launching motor (it has zero speed rn)

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

    launching_motor.set_velocity(0, PERCENT)
    launching_motor.spin(FORWARD)

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

# 11 turns -- 313 deg
# 23 turns - 250 deg
# 23 - 12 turns - 300 deg

# 5 turns - 340 deg
# 10 turns - 310 deg


init()
# autonomous()
driver_control()
print("Autonomous is done..")
# driver_control()
# competition = Competition(driver_control, autonomous)

# TODOS:
# DONE Smoother driving control
# DONE Make a maximum acceleration
# DONE Figure out how to make it so that motors go from go_to_position mode to drive mode
# DONE Make it so that the motors stop instead of coast
# HAVE DAVID TRY THIS OUT
# use threads for auto mode to do multiple commands
# test to see if threads die when they lose scope

# Change these to numpy arrays
# Change this to be relative to the match time???
# Research what the motor.set_max_torque function does
# Research how to utilize motor.set_timeout
# Research how to use motor.torque
# Research how to utilize motor.temperature
# Research what motor.stop does
# Make an info screen that shows the brain battery's capactiy

# Things to share w/rest of team
# ayoo mr mostyn you can wirelessly upload code
