#region VEXcode Generated Robot Configuration
from vex import *
import random

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)
left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)
launching_motor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
led_a = Led(brain.three_wire_port.a)
inertial = Inertial(Ports.PORT9)
index_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
roller_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
intake_motor = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
roller_optical = Optical(Ports.PORT16)


# wait for rotation sensor to fully initialize
wait(30, MSEC)
#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------
import time
import math
from math import sin, cos

r2o2 = math.sqrt(2) / 2

class Vector:
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
        return "Vector:\t"+ repr(self.data)
### UTIL FUNCTIONS (NOT SPECIFIC)

def sign(num):
    if num == 0:
        return 1.0
    return abs(num) / num

# class Timer:
    
#     def __init__(self):
#       pass

#     def start(self):
#       self.start_time = time.time()
    
#     def time(self):
#       return time.time() - self.start_time


class PID: 
    previous_value = None
    previous_time = None

    integral_error = 0
    derivative_error = 0
    proportional_error = 0
    
    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD
    
    def update(self, _value):
        self.integral_error += _value

        if self.previous_value != None:
            # Compute derivative term
            self.derivative_error = _value - self.previous_value
        
        return _value * self.kP + self.integral_error * self.kI + self.derivative_error * self.kD

    def set_constants(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD
    
class TargetPoint:
    def __init__(self, _x, _y, _total_tolerance_or_just_x_tolerance, _tolerance_y = -1):
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
    def __init__(self, x_pos, y_pos):
        self.x_pos = x_pos
        self.y_pos = y_pos

##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###
##### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ### START OF ROBOT CLASS ###

class Robot(GameObject):
    theta_shooter = 20
    theta_robot = 0
    length = 38.1
    
    theta = 0

    x_velocity = 0
    y_velocity = 0

    previous_wheel_encoder_values = Vector([0,0,0,0])
    previous_sum_of_wheel_encoders = 0

    previous_update_time = 0

    wheel_diameter_CM = 10.5
    wheel_distance_CM_to_DEG_coefficient = 360 / (math.pi * wheel_diameter_CM)

    def __init__(self, x_pos = 0, y_pos = 0, theta = 0):
        super().__init__(x_pos, y_pos)
        self.theta = theta

##### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ###
    @staticmethod
    def reset_motor_positions():
        left_motor_a.set_position(0, DEGREES)
        right_motor_a.set_position(0, DEGREES)
        left_motor_b.set_position(0, DEGREES)
        right_motor_b.set_position(0, DEGREES)

    def has_moved(self):
        '''
        Function returns true if the values of the encoders for the motors have
        changed AT ALL
        '''
        new_sum_of_encoders = abs(left_motor_a.position(DEGREES)) + abs(right_motor_a.position(DEGREES)) + abs(left_motor_b.position(DEGREES)) + abs(right_motor_b.position(DEGREES)) 
        has_moved_boolean = new_sum_of_encoders != self.previous_sum_of_wheel_encoders 
        self.previous_sum_of_wheel_encoders = new_sum_of_encoders
        return has_moved_boolean
    
    def set_team(self, _color):
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
        self.theta = 0


    def stop(self):
        left_motor_a.stop()
        left_motor_b.stop()
        right_motor_a.stop()
        right_motor_b.stop()
        

    
##### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### AUTO ### 

    def go_to_position(self, _x, _y, _pid=False): # Target position in cm
        # Offset the target position by the robot's current position
        delta_x = _x - self.x_pos
        delta_y = _y - self.y_pos

        # Reset the motor positions (this can be omitted)
        self.reset_motor_positions()    

        # self.x_pos = _x
        # self.y_pos = _y


        if not _pid:
          # These are the target position for each wheel, you find the target positions by finding the direction the wheels
          # are pointing in and then multiplying by the sin(theta) for the x and cos(theta) for theta
          left_motor_a_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
          right_motor_a_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
          left_motor_b_target_position = -delta_x * self.wheel_distance_CM_to_DEG_coefficient * r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2
          right_motor_b_target_position = delta_x * self.wheel_distance_CM_to_DEG_coefficient * r2o2 + delta_y * self.wheel_distance_CM_to_DEG_coefficient * r2o2

          # print("left_motor_a_target_position:", left_motor_a_target_position)
          # print("right_motor_a_target_position:", right_motor_a_target_position)
          # print("left_motor_b_target_position:", left_motor_b_target_position)
          # print("right_motor_b_target_position:", right_motor_b_target_position)

          left_motor_a.set_velocity(10,PERCENT)
          right_motor_a.set_velocity(10,PERCENT)
          left_motor_b.set_velocity(10,PERCENT)
          right_motor_b.set_velocity(10,PERCENT)

          # Move the motors the current target position
          left_motor_a.spin_for(FORWARD, left_motor_a_target_position, DEGREES, wait=False)
          right_motor_a.spin_for(FORWARD, right_motor_a_target_position, DEGREES, wait=False)
          left_motor_b.spin_for(FORWARD, left_motor_b_target_position, DEGREES, wait=False)
          right_motor_b.spin_for(FORWARD, right_motor_b_target_position, DEGREES, wait=False)

          while not (left_motor_a.is_done() and right_motor_a.is_done() and left_motor_b.is_done() and right_motor_b.is_done()):
            print("waiting...")
            wait(0.5, SECONDS)
          print("Done!")
          self.stop()
        
          

    
    def run_autonomous(self, procedure):
        # r.position = (autonomousProcedureLeft[0]["setX"],autonomousProcedureLeft[0]["setY"])
        
        for step in procedure:
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

            if "x" in step.keys() and "y" in step.keys():
                print("Going to position:", step["x"], step["y"])
                t = Thread(r.go_to_position, (step["x"], step["y"]))
            
            wait(2, SECONDS)

                
    
    def turn_to_heading(self, _heading, _wait = True):
        # There are two ways to approach this, one of them is to compute the positions 
        # that the motors need to go to in order to turn the robot (that number can be)
        # calculated by figuring out the circumference that the robot will rotate around
        # and then divide that by the number of degrees, or it can be found experimentally

        # Another way to approach this is to use a PID-esque loop where we check our current heading
        # at each timestep and drive the robot at a specific velocity until the desired heading is reached 

        pass

    

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
        intake_motor.set_velocity(0,PERCENT)
    
    def update(self):
        # TODO: Set position based off of the gps 
        delta_time = getattr(time, "ticks_ms")() - self.previous_update_time
        # funcs = ([(getattr(time,attr)) for attr in dir(time)[2:]])
        # print(time.ticks_ms() / 1000)
        if delta_time == 0:
            return
        # Check to see if the robot has moved AT ALL

        ### TODO: Make change these to numpy arrays
        delta_encoders = (self.get_current_wheel_encoder_values() - self.previous_wheel_encoder_values)

        ### Get the velocity of the robot in deg/s for 4 wheels
        self.x_velocity = (-delta_encoders[0] + delta_encoders[1] + delta_encoders[2] - delta_encoders[3]) / delta_time
        self.y_velocity = -1 * (delta_encoders[0] + delta_encoders[1] + delta_encoders[2] + delta_encoders[3]) / delta_time 

        ### Divide by 4 because we have it for 4 wheels rn
        self.x_velocity /= 4
        self.y_velocity /= 4

        self.theta = inertial.heading(DEGREES)
        
        self.previous_wheel_encoder_values = self.get_current_wheel_encoder_values()

        self.previous_update_time = getattr(time, "ticks_ms")()

    def get_current_wheel_encoder_values(self):
        return Vector([left_motor_a.position(DEGREES), right_motor_a.position(DEGREES), left_motor_b.position(DEGREES), right_motor_b.position(DEGREES)])



    def turn_to_object(self, gameobject):
        # Get the delta theta needed for the robot to turn
        delta_theta = (get_angle_to_object(r, red_goal) - inertial.heading(DEGREES))
        
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

    
        return delta_theta

##### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### SHOOTER ### 
    @staticmethod
    def set_launcher_speed():
        pass

    def shoot_disk(self):
        # Compute speed that the flywheel needs to speed

        # Set the motors to the speed
        
        # Index a disk out of the magazine
        index_motor.spin_for(FORWARD, 2, TURNS, True)
        
        # Launch disk
        
        pass
    

    def compute_speed_of_flywheel(self):
        # Equation to find the speed that the flywheel needs:

        self.update()
        goal = blue_goal
        if self.notBlue:
            goal = red_goal

        delta_x = goal.x_pos - self.y_pos
        delta_y = goal.y_pos - self.y_pos

        # TODO: Factor in the position of the shooter 
        # distance_to_goal = ((goal.x_pos - self.x_pos) ** 2 + (goal.y_pos - self.y_pos) ** 2) ** 0.5
        
        ### Compute the linear speed that the disk needs to be launched to
        # math.cos(theta_shooter * DEG_TO_RAD)
        
        # distance_to_goal * theta_shooter
        pass
    

##### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS ### ROLLERS 

    @staticmethod
    def spin_roller_forward():
        roller_motor.set_velocity(100, PERCENT)
    
    def auto_spin_roller(self):
        # If we are on the red team
        if self.notBlue:
            # Thresholds the roller being the "blue" color
            pass
    
    def spin_roller(self):
        # Set the velocity to an amount to spin the roller
        ## SPINING WITH POSITIVE POWER MAKES THE ROLLERS GO COUNTER CLOCKWISE
        roller_motor.set_velocity(-15, PERCENT)
        # Drive the robot to the right at half speed
        self.stop()
        t = Timer()

        t.reset()
        while t.time() < 500: ### Accellerate
          self.drive(40, 0, 0)

        while t.time() < 500: ##3 Decellerate
          self.drive(0, 0, 0)



        roller_motor.set_velocity(0, PERCENT)


    @staticmethod
    def spin_roller_backward():
        roller_motor.set_velocity(-100, PERCENT)

##### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### DRIVE ### 
    def drive(self, _x_vector, _y_vector, _r_vector, field_based_driving = False):

        left_motor_a.spin(FORWARD)
        left_motor_b.spin(FORWARD)
        right_motor_a.spin(FORWARD)
        right_motor_b.spin(FORWARD)
        
        # Cool driving toggle (basically you rotate the target direction vector based on)
        # the robots heading (https://stackoverflow.com/questions/14607640/rotating-a-vector-in-3d-space) 
        if field_based_driving:
            # print("Previous" + str(_x_vector) + "\t" + str(_y_vector))
            _x_vector = cos(self.theta * DEG_TO_RAD) * _x_vector - sin(self.theta * DEG_TO_RAD) * _y_vector
            _y_vector = cos(self.theta * DEG_TO_RAD) * _y_vector + sin(self.theta * DEG_TO_RAD) * _x_vector
            # print("After" + str(_x_vector) + "\t" + str(_y_vector))
            print("")

        # 10 spins, 9.49 seconds
        # 10 spins, 10.82 seconds
        # 10 spins, 9.6 seconds
        # 10 spins, 10.98 seconds
        # Square the input vectors and divide by 100 for better controls
        x_vector = (_x_vector ** 2) / 100 * sign(_x_vector)
        y_vector = (_y_vector ** 2) / 100 * sign(_y_vector)
        r_vector = (_r_vector ** 2) / 100 * sign(_r_vector)

        ### HAVE DAVID TRY THIS OUT (sqrt the input might be better for da vid)
        # x_vector = (abs(_x_vector) ** (0.5)) * 10 * sign(_x_vector)
        # y_vector = (abs(_y_vector) ** (0.5)) * 10 * sign(_y_vector)
        # r_vector = (abs(_r_vector) ** (0.5)) * 10 * sign(_r_vector)


        # Get the motor powers
        left_motor_a_target_velocity = x_vector + y_vector + r_vector
        right_motor_a_target_velocity =  -x_vector + y_vector - r_vector
        left_motor_b_target_velocity = -x_vector + y_vector + r_vector
        right_motor_b_target_velocity = x_vector + y_vector - r_vector

        # Alpha controls how responsigve the driving is, higher alpha means more reponsibe
        alpha = 0.4

        if abs(_x_vector) < 3 and abs(_y_vector) < 3 and abs(_r_vector) < 3:
            alpha = 0.8

        left_motor_a_target_velocity = left_motor_a.velocity(PERCENT) * (1 - alpha) + left_motor_a_target_velocity * alpha
        right_motor_a_target_velocity = right_motor_a.velocity(PERCENT) * (1 - alpha) + right_motor_a_target_velocity * alpha
        left_motor_b_target_velocity = left_motor_b.velocity(PERCENT) * (1 - alpha) + left_motor_b_target_velocity * alpha
        right_motor_b_target_velocity = right_motor_b.velocity(PERCENT) * (1 - alpha) + right_motor_b_target_velocity * alpha
        

        
        ## Display content
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
        ### There could be a sensor that detects if theres a disk and then turn on the intake
        # intake_motor.set_velocity(100, PERCENT)

        ### UNCOMMENT THIS LINE TO MAKE MOTOR INTAKE SPIN VARIABLE
        intake_motor.set_velocity(_intake_speed, PERCENT)
    
    
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########
########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ########## END OF ROBOT CLASS ##########


red_goal = GameObject(100, 100)
blue_goal = GameObject(100,100)

wheel_gear_ratio = 18
ticks_per_revolution = 50 * wheel_gear_ratio

RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

field_length = 356 # CM

r = Robot(0,0)


autonomousProcedureCloseToGoal = [
    {
        "setX" : 0,
        "setY" : 0,
    },
    { # Go forward and score preloaded
        "x" : 2 * 2.54,
        "y" : 6 * 2.54,
        "actions": ["unload"]
    }, 
    { # Go backward
        "x" : 0,
        "y" : -13 * 2.54, # position is 2, -7
        "actions" : ["stop_intake"]
    },
    { # Move to the left # position -14, 0
        "message" : "Moving to the left to pick up disk (x: -13 in, y: -7)",
        "x" : -16 * 2.54,
        "y" : 0, 
    },
    { # Go forward pick up disk # position will be -16 and ligned up against the barrier
        "message" : "Picking up disk (x: 0 in, y: 3)",
        "x" : 0 * 2.54,
        "y" : 10 * 2.54,
        "actions" : ["load"]
    },
    {
        # Go to original position
        "message" : "Going to original position",
        "x" : 20,
        "y" : -4
    },
    {
        "message" : "Dropping disk",
        "y" : 10,
    },
    {
        "actions" : ["unload"]
    },
    {
        "message" : "DONE"
    }
]

autonomousSpinRollerClose =  [  
    {
        "setX" : 0,
        "setY" : 0,
    },
    { ### SPIN THE ROLLER
        "actions" : ["spin_roller"],
        "message" : "Spinning roller...",
    },
    { # Move out of the way of the rollers
        "x" : -7 * 2.54,
        "y" : 0,
        "message" : "Move out of the way of the rollers"
    },
    { # MOVE TO EJECT DISKS
        "x" : 6 * 2.54,
        "y" : 58 * 2.54,
        "message" : "Moving to eject disks",
    },
    {
        "x" : 0,
        "y" : 10 *2.54,
        "actions" : ["unload"]
    },
    {
      "actions" : ["load"]
    }
    
]


def get_angle_to_object(gameobject_1, gameobject_2):
    '''
    RETURNS IN DEGREES
    TODO: if someone doesn't pass a gameobject then this will break everything
    '''
    ang = math.atan(((gameobject_2.y_pos) - (gameobject_1.y_pos)) / ((gameobject_2.x_pos) - (gameobject_1.x_pos))) * RAD_TO_DEG 
    if gameobject_1.x_pos > gameobject_2.x_pos:
        ang += 180

    if ang > 180:
        ang -= 360
        
    return ang

######## COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ###

def autonomous():
    # init() # Init a second time (Hopefully to fix the problem with the motors breaking for some reason)
    r.run_autonomous(autonomousSpinRollerClose)
    r.stop()

def driver_control():
    # Change this to be relative to the match time???
    while True:
        # robot axis are based on x, y, and r vectors
        r.drive(controller_1.axis4.position(), controller_1.axis3.position(), controller_1.axis1.position(), False)
        
        # Update the robot's information
        r.update()

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
        
        ### Remove this if we are not experining good riving
        wait(0.1, SECONDS)
    

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
    

    launching_motor.set_velocity(0,PERCENT)
    launching_motor.spin(FORWARD)

    left_motor_a.set_velocity(0,PERCENT)
    right_motor_a.set_velocity(0, PERCENT)
    left_motor_b.set_velocity(0,PERCENT)
    right_motor_b.set_velocity(0, PERCENT)

    left_motor_a.spin(FORWARD)
    right_motor_a.spin(REVERSE) # These wheels are reversed so that they spin ccw instead of cw for forward
    left_motor_b.spin(FORWARD)
    right_motor_b.spin(REVERSE) # These wheels are reversed so that they spin ccw instead of cw for forward

    index_motor.set_velocity(100, PERCENT)
    index_motor.set_position(0,DEGREES)
    
    index_motor.spin_for(FORWARD, 0, TURNS, False)

    intake_motor.spin(FORWARD)
    intake_motor.set_velocity(0,PERCENT)

    roller_motor.spin(FORWARD)
    roller_motor.set_velocity(0, PERCENT)


from vex import *

init()
autonomous()
driver_control()
print("Autonomous is done..")
# driver_control()
# competition = Competition(driver_control, autonomous)

### TODOS:
# Smoother driving control
# Make a maximum acceleration
# Figure out how to make it so that motors go from go_to_position mode to drive mode
# Make it so that the motors stop instead of coast
# HAVE DAVID TRY THIS OUT

# Change these to numpy arrays
# Change this to be relative to the match time???
# Research what the motor.set_max_torque function does
# Research how to utilize motor.set_timeout
# Research how to use motor.torque
# Research how to utilize motor.temperature
# Research what motor.stop does
# Make an info screen that shows the brain battery's capactiy

### Things to share w/rest of team
# ayoo mr mostyn you can wirelessly upload code
