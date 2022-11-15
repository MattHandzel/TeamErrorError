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

gps = Gps(Ports.PORT2)


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

    using_gps = False

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
    def use_gps(self, value):
        self.using_gps = value

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
    
    def get_position(self):
        if self.using_gps:
            return gps.x_position(DistanceUnits.CM), gps.y_position(DistanceUnits.CM)
        return self.get_position_from_encoders()

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
            left_motor_a_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            right_motor_a_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            left_motor_b_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
            right_motor_b_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * \
                r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2

            x_pos, y_pos = self.get_position()

            while abs(x_pos - _target_x) > 5 or abs(y_pos - _target_y) > 5:
                x_pos, y_pos = self.get_position()

                # Speed towards target will be controlled by PID loop
                error_x = _target_x - x_pos
                error_y = _target_y - y_pos

                # 0.00001 is to prevent division by zero
                theta_to_target = math.atan(error_y / (error_x + 0.00001))

                if error_x < 0:  # expand domain of arctan
                    theta_to_target += math.pi
                
                self.drive(math.cos(theta_to_target) * 50, math.sin(theta_to_target) * 50, 0, field_based_driving=True, square_input=False)



            print("Done!")

        else:
            # PID loop here, it exists once the wheel distance for each wheel is within _pidDistanceThreshold of the target distance
            x_pos, y_pos = self.get_position()
            magnitude_PID = PID(10, 0.1, 0)

            while abs(x_pos - _target_x) > _pidDistanceThreshold or abs(y_pos - _target_y) > _pidDistanceThreshold:
                x_pos, y_pos = self.get_position()

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
        self.total_theta += self.angular_velocity * self.delta_time / 0.99375  #is a magic number that makes the gyro work better (experimentall I found that when the gyro reported that it spun 1 time, it actually overshot by about 3 degrees)
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


########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ##########




######### END OF PATHS ######### END OF PATHS ######### END OF PATHS ######### END OF PATHS ######### END OF PATHS


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
    r.run_autonomous({})
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

    while True:
        # Update the robot's information
        r.update()

        # Timer to print things out to the terminal every x seconds
        if (timer.time() > 0.5 * 1000):
            print("theta:", r.theta, "num_rotations:", (r.total_theta // 360) + 1, "anuglar_velocity", inertial.gyro_rate(ZAXIS), " time:", brain.timer.time() / 1000)

            timer.reset()

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

    ### Wait for the gyro to settle, if it takes more then 10 seconds then close out of the loop
    t = Timer()

    t.reset()
    while inertial.gyro_rate(ZAXIS) != 0 and t.value() < 10:
        print("Waiting for gyro to init...")


red_goal = GameObject(100, 100)
blue_goal = GameObject(100, 100)

wheel_gear_ratio = 18
ticks_per_revolution = 50 * wheel_gear_ratio

RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

field_length = 356  # CM

r = Robot(0, 0)

r.use_gps(True)



init()


autonomous()


driver_control()

# competition = Competition(driver_control, autonomous)


##! PRIORITY
# TODO: Use the gps and figure out how precise it is
# TODO: make a "switch" that can make the robot use the gps for its current position or motor encoder values
# TODO: use threads for auto mode to do multiple commands
# TODO: make autonomous mode use self.update

## * IMPORTANT
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