import math
import time
from vex import *
from math import cos, sin, pi, sqrt

global g
g = -9.81

global RAD_TO_DEG
global DEG_TO_RAD
RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

global r2o2
r2o2 = math.sqrt(2) / 2

# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
controller_2 = Controller(PARTNER)
left_motor_a = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)

right_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)

flywheel_motor_1 = Motor(Ports.PORT17, GearSetting.RATIO_6_1, False)
flywheel_motor_2 = Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)

led_a = Led(brain.three_wire_port.a)
inertial = Inertial(Ports.PORT16)
index_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
roller_and_intake_motor_1 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
roller_and_intake_motor_2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
roller_and_intake_motor = MotorGroup(roller_and_intake_motor_1, roller_and_intake_motor_2)

roller_optical = Optical(Ports.PORT12)

indexer = Pneumatics(brain.three_wire_port.a)
expansion = Pneumatics(brain.three_wire_port.b)

gps = Gps(Ports.PORT13)
vision__DISC = Signature(1, 6911, 8133, 7522,-6787, -5937, -6362,1.3, 0)
vision__BRIGHT_DISK = Signature(2, 217, 491, 354,-7169, -6839, -7004,3, 0)
vision_15__SIG_1 = Signature(1, 1101, 2123, 1612,-5789, -4977, -5383,1.4, 0)
vision__DISC_4 = Signature(4, 0, 0, 0,0, 0, 0,3, 0)
vision__SIG_5 = Signature(5, 0, 0, 0,0, 0, 0,3, 0)
vision__SIG_6 = Signature(6, 0, 0, 0,0, 0, 0,3, 0)
vision__SIG_7 = Signature(7, 0, 0, 0,0, 0, 0,3, 0)
vision = Vision(Ports.PORT15, 50, vision__DISC, vision__BRIGHT_DISK, vision_15__SIG_1, vision__DISC_4, vision__SIG_5, vision__SIG_6, vision__SIG_7)

DISC_SIGNATURES = [vision__DISC, vision__BRIGHT_DISK, vision_15__SIG_1, vision__DISC_4, vision__SIG_5, vision__SIG_6, vision__SIG_7]


def init():
    # Make it so that the motors stop instead of coast

    # HAVE DAVID TRY THIS OUT
    left_motor_a.set_stopping(BRAKE)
    right_motor_a.set_stopping(BRAKE)
    left_motor_b.set_stopping(BRAKE)
    right_motor_b.set_stopping(BRAKE)

    flywheel_motor_1.spin(FORWARD, 0, VOLT)
    flywheel_motor_2.spin(FORWARD, 0, VOLT)

    # 
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

    roller_and_intake_motor_1.spin(FORWARD)
    roller_and_intake_motor_1.set_velocity(0, PERCENT)
    roller_and_intake_motor_2.spin(REVERSE)
    roller_and_intake_motor_2.set_velocity(0, PERCENT)

    # Set the optical light power
    roller_optical.set_light_power(100)

    # Wait for the gyro to settle, if it takes more then 10 seconds then close out of the loop
    t = Timer()

    expansion.close()

    t.reset()
    while (inertial.gyro_rate(ZAXIS) != 0 and t.value() < 10):
        print("Waiting for gyro to init...")
        wait(0.1, SECONDS)
    
    controller_1.rumble("...")


class GameObject:
    '''
    If we want to introduce game object (like say the goal or barriers), we have have to robot look up important information about the game object
    '''

    def __init__(self, x_pos, y_pos):
        self.x_pos = x_pos
        self.y_pos = y_pos


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

def getPathOnXYFunction(funcs, delta_t = 0.01):
  '''
    Funcs - An array of two functions, the first one will return the x component of an objects trajectory at time point t, and the second will return the y component of an objects trajectory at time point t. It will run these function until the object hits the ground.

    Returns an array of x positions, y positions, and the time it took to hit the ground
  '''

  # Get the x and y functions out of the functions array so that we can more intuitively refer to them
  xFunc = funcs[0]
  yFunc = funcs[1]

  # Start off time at delta time (no need to compute the x and y positions at t=0 because we know that it will start on the group)
  t = delta_t

  # Create an array to store the x and y positions, initialize the array with the x and y positions at the first timestep
  x = [xFunc(t)]
  y = [yFunc(t)]
  
  # Run the functions until the y x of the function is less than 0 (the object has hit the ground)
  while y[-1] > 0 and x[-1] > 0:
    t += delta_t
    x.append(xFunc(t))
    y.append(yFunc(t))
  return x, y, t

def returnXYFuncs(theta, v_i):
  '''This will return two funcions, for the x and y component of the objects path, depending upon the objects initial launch angle (theta) and initial velocity'''
  return returnXFunc(theta, v_i), returnYFunc(theta, v_i)

def returnXFunc(theta, v_i):
  '''Returns the x component of the objects trajecotry using the following formula'''
  return lambda t: cos(theta) * v_i * t

def returnYFunc(theta, v_i):
  ''' Returns the x component of the objects trajecotry using the following formula.'''
  # NOTE: This function is assuming that you live on earth and thus acceleration is gravity
  return lambda t: (1/2 * g * t * t + sin(theta) * v_i * t)

def calculateRequiredInitialVelocityToPassThroughAPoint(coords):
  '''
  This is a function that I derived in order to calculate the required initial velocity for an object to pass through a point.
  The formula for this equation is:

         ______________________________________________
         |         ____________________________________
   v_i = |-2gy + __|((2gy)^2 - (4 * -(g^2 * x^2)))
         |----------------------------------------------
       __|                      2

  '''
  x = coords[0]
  y = coords[1]

  w = 2 * g * y
  q = - g * g * x * x

  squareRoot = math.sqrt((w * w) - (4 * q))
  expression = (-w + squareRoot)/2

  return (math.sqrt(expression))

def getThetaForPathToHitPoint(v_i, point, sizeOfPoint = 0.05):
  '''This function, when given the initial velocity required, will output the angle needed to shoot at.
    point - point we want to hit
    sizeOfPoint - the tolerance at which we can hit the point, at extereme initial velocities, this needs to be very high
  '''
  theta = 0
  go = True
  iterations = 0
  minimum_distance = 0

  # How this works is that it plots the trajectory of the object at changing angles of being shot, and it returns the correct angle once it is hit. This can be optimized by a hell of a lot and there is probably a mathetmatical formula that you can use to get the correct point in like 2 milliseconds buuuuuuut I already made a very good formula before that used a lot of brain power and Winter break was almost over so I settled on this solution, if I need to run this formula on a system that actually shoots things and is very time sensitive, then I will fix this, but otherwise there isn't a need to fix it. 
  while go:
    new_theta_1 = theta + (pi / 4) * 2 ** (-iterations)
    new_theta_2 = theta - (pi / 4) * 2 ** (-iterations)

    # Run a simulation for both new theta angles

    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(new_theta_1, v_i))
    minimum_distance_theta_1 = float('inf')

    # Find the point that is the closest to the point we want to hit
    for x, y in zip(_x, _y):
      distance = math.sqrt((x - point[0])**2 + (y - point[1])**2)
      minimum_distance_theta_1 = min(minimum_distance_theta_1, distance)
    
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(new_theta_2, v_i))
    minimum_distance_theta_2 = float('inf')

    # Find the point that is the closest to the point we want to hit
    for x, y in zip(_x, _y):
      distance = math.sqrt((x - point[0])**2 + (y - point[1])**2)
      minimum_distance_theta_2 = min(minimum_distance_theta_2, distance)
    
    # If the new theta angles are closer to the point we want to hit, then we will use those angles
    if minimum_distance_theta_1 < minimum_distance_theta_2:
      minimum_distance = minimum_distance_theta_1
      theta = new_theta_1
    else:
      minimum_distance = minimum_distance_theta_2
      theta = new_theta_2

    # If the point we want to hit is within the tolerance, then we are done
    if minimum_distance < sizeOfPoint:
      go = False

    iterations += 1
    if iterations > 50:
      print("Reached maximimum iterations!", theta)
      go = False

  return theta

def getViForPathToHitPoint(theta, point, sizeOfPoint = 0.05):
  '''This function, when given the initial velocity required, will output the angle needed to shoot at.
    point - point we want to hit
    sizeOfPoint - the tolerance at which we can hit the point, at extereme initial velocities, this needs to be very high
  '''
  vi = 0
  go = True
  iterations = 0
  minimum_distance = 0
  max_vi = 8.65
  delta_time = 0.001
  hit_time = 0

  # How this works is that it plots the trajectory of the object at changing angles of being shot, and it returns the correct angle once it is hit. This can be optimized by a hell of a lot and there is probably a mathetmatical formula that you can use to get the correct point in like 2 milliseconds buuuuuuut I already made a very good formula before that used a lot of brain power and Winter break was almost over so I settled on this solution, if I need to run this formula on a system that actually shoots things and is very time sensitive, then I will fix this, but otherwise there isn't a need to fix it. 
  while go:
    new_vi_1 = vi + (max_vi / 2) * 2 ** (-iterations)
    new_vi_2 = vi - (max_vi / 2) * 2 ** (-iterations)

    # Run a simulation for both new vi angles

    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, new_vi_1), delta_time)
    minimum_distance_vi_1 = float('inf')
    
    hit_time_vi_1 = 0
    
    # Find the point that is the closest to the point we want to hit
    for x, y in zip(_x, _y):
      distance = math.sqrt((x - point[0])**2 + (y - point[1])**2)
      minimum_distance_vi_1 = min(minimum_distance_vi_1, distance)
      if minimum_distance_vi_1 == distance:
        hit_time_vi_1 = (_x.index(x) + 1) * delta_time
    
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, new_vi_2), delta_time)
    minimum_distance_vi_2 = float('inf')
    hit_time_vi_2 = 0
    # Find the point that is the closest to the point we want to hit
    for x, y in zip(_x, _y):
      distance = math.sqrt((x - point[0])**2 + (y - point[1])**2)
      minimum_distance_vi_2 = min(minimum_distance_vi_2, distance)
      if minimum_distance_vi_2 == distance:
        hit_time_vi_2 = (_x.index(x) + 1) * delta_time

    # If the new vi angles are closer to the point we want to hit, then we will use those angles
    if minimum_distance_vi_1 < minimum_distance_vi_2:
      minimum_distance = minimum_distance_vi_1
      vi = new_vi_1
      hit_time = hit_time_vi_1
    else:
      minimum_distance = minimum_distance_vi_2
      vi = new_vi_2
      hit_time = hit_time_vi_2

    # If the point we want to hit is within the tolerance, then we are done
    if minimum_distance < sizeOfPoint:
      go = False

    iterations += 1
    if iterations > 20:
      go = False

  return vi, hit_time


class Button:
  needs_to_render = True

  def __init__(self, name = "", x = 0, y = 0, w = 0, h = 0, color = 0, call_back = None, *args):
    self.name = name
    self.x = x
    self.y = y
    self.w = w
    self.h = h
    self.color = color
    self.call_back = call_back
    self.args = args

  def render(self):
    brain.screen.draw_rectangle(self.x, self.y, self.w, self.h, self.color)

    x_position_of_text = self.x + self.w / 2 - len(self.name) * 5
    y_position_of_text = self.y + self.h / 2 + 5

    x_position_of_text = max(x_position_of_text, self.x)
    y_position_of_text = max(y_position_of_text, self.y)

    brain.screen.print_at(self.name, x=x_position_of_text, y=y_position_of_text, opaque = False)
  
  def set_callback(self, function):
    self.call_back = function
  
  # If we do something like button() then it will run the callback function
  def __call__(self):
    if self.call_back != None:
        # If self is a variable in the call_back function then pass self as the button 
        self.call_back(*self.args)

class Text:
    def __init__(self, name = "", x = 0, y = 0, w = 0, h = 0, color = 0, call_back = None, *args):
        self.name = name
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.color = color
        self.call_back = call_back
        self.args = args

    def render(self):
        # If there is a call back function, then call it and set the name to the return value
        if self.call_back != None:
            self.name = self.call_back(*self.args)

        brain.screen.draw_rectangle(self.x, self.y, self.w, self.h, self.color)

        x_position_of_text = self.x + self.w / 2 - len(self.name) * 5
        y_position_of_text = (self.y + self.h / 2) + 5

        x_position_of_text = max(x_position_of_text, self.x)
        y_position_of_text = max(y_position_of_text, self.y)
        
        brain.screen.print_at(self.name, x=x_position_of_text, y=y_position_of_text, opaque = False)
    
    def __call__(self):
        # this does literllay nothing
        if self.call_back != None:
            self.name = self.call_back(*self.args)



class Switch:
    # A switch class is the same as a button class, but instead it has states and each state calls another function
    # so for example, the switch class changes it colors when it changes states

    needs_to_render = True

    def __init__(self, name = [], x = 0, y = 0, w = 0, h = 0, color = [], states = [], *args):
        self.name = name
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        if type(states) == list:
            self.states = states
        else:
            self.states = [states] * len(args[0])
        self.current_state = 0
        self.colors = color
        self.args = args

    def render(self):
        brain.screen.draw_rectangle(self.x, self.y, self.w, self.h, self.colors[self.current_state])

        x_position_of_text = self.x + self.w / 2 - len(self.name[self.current_state]) * 5
        y_position_of_text = self.y + self.h / 2 + 5

        x_position_of_text = max(x_position_of_text, self.x)
        y_position_of_text = max(y_position_of_text, self.y)

        brain.screen.print_at(self.name[self.current_state], x=x_position_of_text, y=y_position_of_text, opaque = False)

    def set_states(self, states):
        self.states = states

    def set_state(self, state):
        self.current_state = state

    def change_state(self):
        self.current_state = (self.current_state + 1) % len(self.states)
        print("changing state to...", self.current_state)

    def run_state(self):
        self.states[self.current_state](*[arg[self.current_state] for arg in self.args])

    def __call__(self):
        self.change_state()
        self.run_state()




class GUI:
  '''
  What I want this class to do it to make a way for the drivers to interact with the brain screen and see some status
  things like if the motors are too hot, or if there is any self-diagnosed problem. I also want people to be able to select
  from the brain what team we're on.

  Brain screen dimensions: 480 x 240 pizels. Top left is (0,0)

  Each character is 10 x 10 pixels
  '''

  elements = []

  previous_brain_screen_state = False

  def __init__(self):
    pass

  def add_element(self, element):
    self.elements.append(element)

  def update(self):
    # for element in self.elements:
    #   # If ANY element needs to re-render, then re-render everything
    #   if element.needs_to_render:
    #     break
    # # If the for loops exists without a break statemenet
    # else:
    #   self.render()

    # If the brain has been pressed ANYWHERE
    if brain.screen.pressing() and not self.previous_brain_screen_state:
      # X and y positions of where the finger pressed
      x, y = brain.screen.x_position(), brain.screen.y_position()
      # 37, 27; 45, 19; 

        # so if we're making a gui, the top left corner of the button gets drawn at its x and y position:

        # that means to see if something is inside of the button, the x position has to be greater than the buttons x position, and the x position of click can not minus the width cannot be larger than the button's width
        # same thing for y

        # bottom right 460, 213
        # bottom left 20, 213
      for element in self.elements:
        if (x - element.x) > 0 and (x - element.x) < element.w and (y-element.y) > 0 and (y - element.y) < element.h:
          element()
        
    self.previous_brain_screen_state = brain.screen.pressing()
  
  def render(self):
    brain.screen.clear_screen()
    for element in self.elements:
      element.render()
    
    brain.screen.render()

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


def get_heading_to_object(gameobject_1, gameobject_2):
    '''
    RETURNS IN DEGREES
    TODO: if someone doesn't pass a gameobject then this will break everything
    '''
    # Checks to see if both objects are of the GameObject class
    if gameobject_1.x_pos == gameobject_2.x_pos:
        return 180

    if gameobject_1.x_pos < gameobject_2.x_pos:
        ang = math.atan(((gameobject_2.y_pos - gameobject_1.y_pos)) /
                        (gameobject_2.x_pos - gameobject_1.x_pos)) * RAD_TO_DEG
    else:
        ang = math.atan(((gameobject_2.y_pos - gameobject_1.y_pos)) /
                        (gameobject_2.x_pos - gameobject_1.x_pos)) * RAD_TO_DEG + 180
    
    ang = 90 - ang
    if ang > 180:
        ang -= 360
    if ang < -180:
        ang += 360

    return ang


class Test:
  def __init__(self, name, args):
    name.lower()
    self.name = name
    self.args = args
  
  def __getitem__(self, key):
    return self.args[key]

  # overload equals operator
  def __eq__(self, name):
    name.lower()
    return self.name == name

class Robot:
    '''
    This is the big-boy class, this is the robot class, this is the class that controls the robot, there is a lot of stuff here
    '''
    flywheel_1_voltage_factor = 0
    flywheel_2_voltage_factor = 0

    previous_flywheel_speed = 0
    ###* ORIENTATION/POSITION VARIABLES
    total_theta = 0
    theta_offset = 0
    max_velocity: float
    max_acceleration: float

    previous_x_from_encoders = 0
    previous_y_from_encoders = 0

    previous_x_from_gps = 0
    previous_y_from_gps = 0

    total_x_from_encoders = 0
    total_y_from_encoders = 0

    ###* FLYWHEEL
    length: float = 38.1

    # Set the offset for the flywheel from the center of the robot
    flywheel_offset_x = 0   
    flywheel_offset_y = 0

    flywheel_angle = 45 * DEG_TO_RAD

    flywheel_1_avg_speed = 0
    flywheel_2_avg_speed = 0

    previous_flywheel_1_avg_speed = 0
    previous_flywheel_2_avg_speed = 0

    previous_flywheel_1_error = 0
    previous_flywheel_2_error = 0


    flywheel_speed = 0

    flywheel_height_from_ground_IN = -99999

    flywheel_motor_1_PID = PID(2, 0, 0)
    flywheel_motor_2_PID = PID(2, 0, 0)

    flywheel_motor_1_error = 0
    flywheel_motor_2_error = 0

    ###* DRIVETRAIN

    previous_update_time: float = 0

    drivetrain_gear_ratio = 18

    wheel_max_rpm: float = 200
    wheel_diameter_CM: float = 10.5

    # In order to get this, it is ticks for the specific gear ratio we're using divided by the circumeference of our wheel
    wheel_distance_CM_to_TICK_coefficient: float = (drivetrain_gear_ratio / 6 * 300) / \
        (math.pi * wheel_diameter_CM)

    ###* PID controllers

    flywheel_motor_1_average_output = 0
    flywheel_motor_2_average_output = 0 

    # PID controllers
    motor_PID_kP = 10
    motor_PID_kI = 5
    motor_PID_kD = 0

    pos_PID_kP = 0.1
    pos_PID_kI = 0
    pos_PID_kD = 0
    x_pos_PID = PID(pos_PID_kP,pos_PID_kI,pos_PID_kD)
    y_pos_PID = PID(pos_PID_kP,pos_PID_kI,pos_PID_kD)

    rotation_PID_kP = 0.1
    rotation_PID_kI = 0
    rotation_PID_kD = 0
    rotation_PID = PID(rotation_PID_kP,rotation_PID_kI,rotation_PID_kD)

    left_motor_a_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    right_motor_a_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    left_motor_b_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    right_motor_b_PID = PID(motor_PID_kP,motor_PID_kI,motor_PID_kD)
    
    x_vel_PID = PID(0.1, 0, 0)
    y_vel_PID = PID(0.1, 0, 0)
    theta_vel_PID = PID(0.1, 0, 0)

    ###* MISC
    update_loop_delay = 0.01  # 10 ms

    # Used to keep track of time in auto and driver mode respectively, use it for nicely logging data, can be used during either modes for end game/pathfinding rules
    autonomous_timer = Timer()
    driver_controlled_timer = Timer()

    target_reached = False
    position_tolerance = 3 # tolerance to target position in cm
    orientation_tolerance = 8 # tolerance to target orientation in degrees
    
    # From 0,0 (which is the center of the field). Dimensions were got from page 89 on: https://content.vexrobotics.com/docs/2022-2023/vrc-spin-up/VRC-SpinUp-Game-Manual-2.2.pdf
    red_goal = GameObject((122.63 - 70.2) * 2.54, (122.63 - 70.2) * 2.54)
    blue_goal = GameObject(-(122.63 - 70.2) * 2.54, -(122.63 - 70.2) * 2.54)

    flywheel_speed_levels = [
        20,
        25,
        30,
        35,
        40,
        45,
        50,
        55,
    ]
    
    # State dictionary will hold ALL information about the robot
    '''
        x_pos: X position of the robot
        y_pos: Y position of the robot

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

        "override_velocity_x" : None,
        "override_velocity_y" : None,
        "override_velocity_theta" : None,

        # Roller stuff
        "roller_state" : "none",
        "auto_roller" : False,
        "roller_speed" : 0,
        
        # Intake
        "auto_intake" : False,
        "disc_in_intake" : False,

        # Actuators
        "flywheel_speed" : 0,
        "intake_speed" : 0,

        # expansion
        "launch_expansion" : False,

        # Commands
        "using_gps" : False,
        "is_shooting": False,
        "slow_mode" : False,
        "drone_mode" : False,
        "autonomous" : False,
        
    }

    intake_timer = Timer()

    target_state = {}
    
    all_states = []
    delta_time = 0

    save_states = False

    path = []

    def __init__(self):
        '''
        Initializes the robot class, computes the max velocity and acceleration
        params -
            x_pos - set the robot's x position
            y_pos - set the robot's y position
            theta - set the robot's theta
        '''

        # what our max velocity "should" be (can go higher or lower)
        self.max_velocity = ((self.wheel_max_rpm / 60) *
                             math.pi * 2 * self.wheel_diameter_CM / math.sqrt(2))
        # This number in the divisor means it will speedup/slow down in that many seeconds
        self.max_acceleration = 2 * self.max_velocity / 0.05

        # Set origin of the gps
        gps.set_origin(0,0)

        self.initial_state = self.state.copy()
        self.set_target_state(self.state)
        self.previous_state = self.state
    
    # There are two init methods, this init initializes the class, the other init method we call to initlize the robot
    def init(self):
        
        # Set heading based on gps
        if self.using_gps:

            # For some reason, the gps returns nan, just wait until it doesn't return nan
            while str(gps.heading()) == "nan":
                print("Waiting for gps", gps.heading())
                time.sleep(0.1)

            self.theta_offset = gps.heading()
            self.total_theta = self.theta_offset

        self.set_target_state(self.state)
        self.previous_state = self.state
        Thread(self.update_loop)


    def update_loop(self):
      while True:
        self.update()
        wait(self.update_loop_delay, SECONDS)

    def update(self):
        '''
        This is a VERY important function, it should be called once every [0.1 - 0.01] seconds (the faster the better, especially for controls)
        It updates the robot's position based off of the encoders and the gyro
        '''
        self.delta_time = (getattr(time, "ticks_ms")() / 1000) - (self.previous_state["time"])

        # prevent divide by zero
        if self.delta_time == 0:
            return
        
        if self.save_states:
            self.all_states.append(self.state)

        # Update the previous state before doing state estimation
        self.previous_state = self.state.copy()

        # Estimate our current position
        self.estimate_state()

        # All controls for movement, pid control for robot 
        self.position_update()

        # All of the updates needed for the flywheel, including pid control
        self.flywheel_update()

        # All of code for roller, including auto roller
        self.roller_update()
        
        # All of the code and checks for the intake, allows for auto intake and manual control
        self.intake_update()

        # Run all the code needed for the expansion
        self.expansion_update()

    
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


        # Make sqrt the delta theta, so that the tolerance is not linear but a sqrt relationshup
        delta_theta = math.sqrt(abs(delta_theta)) * sign(delta_theta) 

        # The multiplying by 100 and dividing by self.position tolerance scales it so that at position tolerance the velocity is 100 percent, squaring the velocity makes it so that if we get closer than we go slower
        target_x_vel = ((clamp(delta_x, self.position_tolerance, -self.position_tolerance) * 100 / self.position_tolerance) ** 2) / 100 * sign(delta_x)
        target_y_vel = ((clamp(delta_y, self.position_tolerance, -self.position_tolerance) * 100 / self.position_tolerance) ** 2) / 100 * sign(delta_y)
        target_theta_vel = ((clamp(delta_theta, self.orientation_tolerance, -self.orientation_tolerance) * 100 / self.orientation_tolerance) ** 2) / 100 * sign(delta_theta)
        
        if self.target_state["override_velocity_x"] != None:
            target_x_vel = self.target_state["override_velocity_x"]
        if self.target_state["override_velocity_y"] != None:
            target_y_vel = self.target_state["override_velocity_y"]
        if self.target_state["override_velocity_theta"] != None:
            target_theta_vel = self.target_state["override_velocity_theta"]

        if abs(delta_theta) < 1:
            target_theta_vel = 0 

        # Spin the motors (in case they were stopped before, might be able to be ommitted later)
        left_motor_a.spin(FORWARD)
        left_motor_b.spin(FORWARD)
        right_motor_a.spin(FORWARD)
        right_motor_b.spin(FORWARD)

        # Cool driving toggle (basically you rotate the target direction vector based on)
        # the robots heading (https://stackoverflow.com/questions/14607640/rotating-a-vector-in-3d-space)
        if self.drone_mode:                                                         # Add self.theta add dt times angular velocity to get a better approximation of actual theta at timestamp
            target_x_vel, target_y_vel = rotate_vector_2d(target_x_vel, target_y_vel, (self.theta + self.delta_time / 2 * self.theta_vel) * DEG_TO_RAD)

        if self.slow_mode:
            target_x_vel = target_x_vel / 4
            target_y_vel = target_y_vel / 4
            target_theta_vel = target_theta_vel / 4

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
    
    def expansion_update(self):
        if self.target_state["launch_expansion"]:
            expansion.open()
        
            
    
    def estimate_state(self):
        '''
        Estimates the state of the robot
        '''
        self.state["time"] = getattr(time, "ticks_ms")() / 1000
        self.theta_vel = inertial.gyro_rate(ZAXIS)
        
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
            # self.total_theta = gps.heading()

            # Update alpha to value that uses gps
            alpha = 1
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
        self.x_pos += delta_x_from_encoders * (1-alpha) + (x_from_gps-self.x_pos) * alpha 
        self.y_pos += delta_y_from_encoders * (1-alpha) + (y_from_gps-self.y_pos) * alpha
        
        self.x_from_gps = x_from_gps
        self.y_from_gps = y_from_gps

        self.total_x_from_encoders += delta_x_from_encoders
        self.total_y_from_encoders += delta_y_from_encoders

        if abs(self.state["x_pos"] - self.target_state["x_pos"]) < self.position_tolerance and abs(self.state["y_pos"] - self.target_state["y_pos"]) < self.position_tolerance:
            # print("TARGET STATE REACHED")
            self.target_reached = True
        
        self.theta -= self.theta_offset
    
    def intake_update(self):
        # If we have auto_intake enbaled 
        if self["auto_intake"]:
            objs = [vision.take_snapshot(z) for z in range(8)]
            objs = [obj for obj in objs if obj]
            if len(objs) > 1:
                self.intake_speed = 100
                self["disc_in_intake"] = True
                self.intake_timer.reset()
            elif self.intake_timer.time(MSEC) > 2000: # If it has been more than 1 second since we last saw a disc, stop intake
                self.intake_speed = 0
                self["disc_in_intake"] = False

        roller_and_intake_motor_1.spin(REVERSE, self.intake_speed, VelocityUnits.PERCENT)
        roller_and_intake_motor_2.spin(FORWARD, self.intake_speed, VelocityUnits.PERCENT)
        
    def roller_update(self):
        # Only if we have auto_roller enabled will we change the roller_speed
        if self["auto_roller"]:
            if not roller_optical.is_near_object():
                self["roller_state"] = "none"
                self["roller_speed"] = 0
            else:
                # If the sensor detects and object and if that object is red, or blue then run the roller
                if roller_optical.color() == Color.RED:
                    self["roller_state"] = "red"
                elif roller_optical.is_near_object() and roller_optical.color() == Color.BLUE:
                    self["roller_state"] = "blue"
                else:
                    self["roller_state"] = "none"

                # If we detect that the roller is not our team's color, spin it
                self["roller_speed"] = 0
                if (self["roller_state"] == "red" and not self.notBlue) or (self["roller_state"] == "blue" and self.notBlue):
                    self["roller_speed"] = 100

                # This is a way to indicate to the drivers that we are done spinning the roller
                if (self["roller_state"] == "red" and self.notBlue) or (self["roller_state"] == "blue" and not self.notBlue):
                    controller_1.rumble("-")
                    if self.previous_state["roller_state"] != self["roller_state"]:
                        self["roller_speed"] = -50
        
        if not self["disc_in_intake"]:
            roller_and_intake_motor_1.spin(REVERSE, self["roller_speed"], VelocityUnits.PERCENT)
            roller_and_intake_motor_2.spin(REVERSE, self["roller_speed"], VelocityUnits.PERCENT)

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
                    print("setthing theta to", _state[key])
            
        self.update_constants(_state)
        self.target_reached = False
    
    def update_constants(self, _state):
        if "drone_mode" in _state:
            self.drone_mode = _state["drone_mode"]
        if "slow_mode" in _state:
            self.slow_mode = _state["slow_mode"]
        if "using_gps" in _state:
            self.using_gps = _state["using_gps"]
        if "intake_speed" in _state:  
            self.intake_speed = _state["intake_speed"]
        if "roller_speed" in _state:
            self.roller_speed = _state["roller_speed"]
        if "auto_intake" in _state:
            self.auto_intake = _state["auto_intake"]
        if "auto_roller" in _state:
            self.auto_roller = _state["auto_roller"]
        if "flywheel_speed" in _state:
            self.flywheel_speed = _state["flywheel_speed"]

    
    def __setitem__(self, key, value):
        self.state[key] = value
    
    def __getitem__(self, key):
        return self.state[key]

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
        print("SET_TEAM IS:\t", _color)
        _color.lower()
        print("INPUT FROM SET_TEAM IS:\t", _color)
        if _color == "red":
            self.notBlue = True
            self.goal = self.red_goal
            return
        elif _color == "blue":
            self.goal = self.blue_goal
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
        print("r.turn_to_roller: Current position is", x_pos, y_pos, "and setting heading to", roller_angle)
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
                unload - unload the disk
                load - load the disk
                shoot - shoot the disk
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
            
            for key in step.keys():
                if key in self.state.keys():
                    print("setting key of", key, "to", step[key])
                    target_state[key] = step[key]

            # if "actions" in step.keys():
            #     if "unload" in step["actions"]:
            #         r.unload()
            #     if "load" in step["actions"]:
            #         r.load()

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

            if "wait" in step.keys():
                wait(step["wait"], SECONDS)

            while not self.target_reached:
                self.update()
                wait(0.1, SECONDS)

        self.set_target_state(self.initial_state)

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
    def intake_speed(self):
        return self.state["intake_speed"]
    
    @intake_speed.setter
    def intake_speed(self, _intake_speed):
        self.state["intake_speed"] = _intake_speed
            
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
    
    def get_current_wheel_encoder_values(self):
        '''
        Returns a vector of all of the encoder values
        '''
        return Vector([left_motor_a.position(DEGREES), right_motor_a.position(DEGREES), left_motor_b.position(DEGREES), right_motor_b.position(DEGREES)])

    def launch_expansion(self):
        '''
        This function will launch the expansion
        '''
        expansion.open()


    def turn_to_object(self, _gameobject):
        '''
        This function will have the robot turn to face the object
        '''
        # Get the delta theta needed for the robot to turn
        delta_theta = (get_heading_to_object(
            self, _gameobject) - inertial.heading(DEGREES))

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

##### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ###
    def flywheel_update(self):
        self.flywheel_pid(self.flywheel_speed)        
    
    def flywheel_pid(self, speed):
        # TODO: Smoothing the output: The output of the PID controller may be noisy, which can cause instability in the system. To smooth the output, you can try using a low-pass filter to remove high-frequency noise.
        # TODO: Adding an anti-windup mechanism: An anti-windup mechanism can prevent the integral term from growing too large, which can cause the controller to lose control. This can be especially important if the system has a large dead-zone or if the output is saturated for long periods of time.

        # MAX_FLYWHEEL_SPEED = 100 / 60
        MAX_VOLTAGE = 10 # I don't think this is the true maximum voltage btw
        
        speed_alpha = 1
        self.flywheel_1_avg_speed = flywheel_motor_1.velocity(PERCENT) * speed_alpha + self.flywheel_1_avg_speed * (1 - speed_alpha)
        self.flywheel_2_avg_speed = flywheel_motor_2.velocity(PERCENT) * speed_alpha + self.flywheel_2_avg_speed * (1 - speed_alpha)

        error_1 = speed - flywheel_motor_1.velocity(PERCENT) # self.flywheel_1_avg_speed 
        error_2 = speed - flywheel_motor_2.velocity(PERCENT) # self.flywheel_2_avg_speed 

        self.flywheel_motor_1_average_output = min(self.flywheel_motor_1_average_output, MAX_VOLTAGE)
        self.flywheel_motor_2_average_output = min(self.flywheel_motor_2_average_output, MAX_VOLTAGE)

        self.flywheel_motor_1_average_output = max(self.flywheel_motor_1_average_output, -MAX_VOLTAGE)
        self.flywheel_motor_2_average_output = max(self.flywheel_motor_2_average_output, -MAX_VOLTAGE)

        if abs(error_1) > MAX_VOLTAGE / (self.flywheel_motor_1_PID.kP if self.flywheel_motor_1_PID.kP != 0 else 0.0001):
            self.flywheel_motor_1_average_output = max(self.flywheel_motor_1_average_output, MAX_VOLTAGE) if error_1 > 0 else min(self.flywheel_motor_1_average_output, -MAX_VOLTAGE)
        if abs(error_2) > MAX_VOLTAGE / (self.flywheel_motor_2_PID.kP if self.flywheel_motor_2_PID.kP != 0 else 0.0001):
            self.flywheel_motor_2_average_output = max(self.flywheel_motor_2_average_output, MAX_VOLTAGE) if error_2 > 0 else min(self.flywheel_motor_2_average_output, -MAX_VOLTAGE)
        
        # self.flywheel_motor_1_average_output = max(self.flywheel_motor_1_average_output, -MAX_VOLTAGE)
        # self.flywheel_motor_2_average_output = max(self.flywheel_motor_2_average_output, -MAX_VOLTAGE)

        output_alpha = 0.1
        self.flywheel_motor_1_average_output = self.flywheel_motor_1_average_output * (1-output_alpha) + self.flywheel_motor_1_PID.update(error_1) * (output_alpha)
        self.flywheel_motor_2_average_output = self.flywheel_motor_2_average_output * (1-output_alpha) + self.flywheel_motor_2_PID.update(error_2) * (output_alpha)

        if self.flywheel_speed == 0:
            self.flywheel_motor_1_average_output = 0
            self.flywheel_motor_2_average_output = 0

        kP = 0.01
        kD = 0.15

        proportional_term_flywheel_1 = kP * (self.flywheel_speed - self.flywheel_1_avg_speed)
        derivative_term_flywheel_1 = kD * (self.flywheel_1_avg_speed - self.previous_flywheel_1_avg_speed)
        # error_helper_term_flywheel_1 = 0.01 * ((self.flywheel_speed - self.flywheel_1_avg_speed) - self.previous_flywheel_1_error) 

        derivative_term_flywheel_1 = max(derivative_term_flywheel_1, -0.3)
        derivative_term_flywheel_1 = min(derivative_term_flywheel_1, 0.3)

        # derivative_term_flywheel_1 = 0

        proportional_term_flywheel_2 = kP * (self.flywheel_speed - self.flywheel_2_avg_speed)
        derivative_term_flywheel_2 = kD * (self.flywheel_2_avg_speed - self.previous_flywheel_2_avg_speed)
        # error_helper_term_flywheel_2 = 0.01 * ((self.flywheel_speed - self.flywheel_1_avg_speed) - self.previous_flywheel_1_error) 

        derivative_term_flywheel_2 = max(derivative_term_flywheel_2, -0.3)
        derivative_term_flywheel_2 = min(derivative_term_flywheel_2, 0.3)

        # derivative_term_flywheel_2 = 0

        # If we are more than 10 percent off of the target speed
        # if abs(self.flywheel_1_avg_speed - self.flywheel_speed) > 5:
        self.flywheel_1_voltage_factor += proportional_term_flywheel_1 - derivative_term_flywheel_1 + (self.flywheel_speed - self.previous_flywheel_speed) * 0.1
        # if abs(self.flywheel_2_avg_speed - self.flywheel_speed) > 5:
        self.flywheel_2_voltage_factor += proportional_term_flywheel_2 - derivative_term_flywheel_2 + (self.flywheel_speed - self.previous_flywheel_speed) * 0.1
        if self.flywheel_1_voltage_factor > MAX_VOLTAGE:
            self.flywheel_1_voltage_factor = MAX_VOLTAGE
        if self.flywheel_2_voltage_factor > MAX_VOLTAGE:
            self.flywheel_2_voltage_factor = MAX_VOLTAGE
        if self.flywheel_1_voltage_factor < -MAX_VOLTAGE:
            self.flywheel_1_voltage_factor = -MAX_VOLTAGE
        if self.flywheel_2_voltage_factor < -MAX_VOLTAGE:
            self.flywheel_2_voltage_factor = -MAX_VOLTAGE

        flywheel_motor_1.spin(FORWARD, self.flywheel_1_voltage_factor + proportional_term_flywheel_1 * 0, VOLT)
        flywheel_motor_2.spin(FORWARD, self.flywheel_2_voltage_factor + proportional_term_flywheel_2 * 0, VOLT)

        self.flywheel_motor_1_error = (self.flywheel_1_avg_speed - self.previous_flywheel_1_avg_speed)
        self.flywheel_motor_2_error = (self.flywheel_2_avg_speed - self.previous_flywheel_2_avg_speed)

        # Compute derivative of average flywheel speed
        self.previous_flywheel_1_avg_speed = self.flywheel_1_avg_speed
        self.previous_flywheel_2_avg_speed = self.flywheel_2_avg_speed

        # self.previous_flywheel_1_error = self.flywheel_speed - self.flywheel_1_avg_speed
        # self.previous_flywheel_2_error = self.flywheel_speed - self.flywheel_2_avg_speed
        self.previous_flywheel_speed = float(self.flywheel_speed)
        # print("Error", error_1, error_2, "Velocity",flywheel_motor_1.velocity(VelocityUnits.PERCENT), "Output", output_1, output_2)

    
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
        wait(1, SECONDS)
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

        delta_x = self.goal.x_pos - self.x_pos - self.flywheel_offset_x
        delta_y = self.goal.y_pos - self.y_pos - self.flywheel_offset_y

        point = (self.goal.x_pos * 2.54 / 100, (self.goal.y_pos - self.flywheel_height_from_ground_IN) * 2.54 / 100)
        v_i, hit_time = getViForPathToHitPoint(self.flywheel_angle, (point), sizeOfPoint = 0.001)
        print("SETTING THE FLYWHEEL TO x M/S")

        # Compute the linear speed that the disk needs to be launched to
        # math.cos(theta_flywheel * DEG_TO_RAD)

        # distance_to_goal * theta_flywheel
        pass    

class RobotDoctor:
  r = Robot()
  flywheel_speed_tolerance = 2
  
  # Initalize robot and robot peripherals
  def init(self):
    init()
    self.r.init()

  def run_tests(self, tests):
    for test in tests:
      self.run_test(test)
  
  def fail_test(self, message):
    self.r.print(message)
    controller_1.rumble("...---...")
  
  def pass_test(self, message):
    self.r.print(message)
    controller_1.rumble("-")

  def run_test(self, test):
    if test == "flywheel":
      self.test_flywheel(test["comprehensive"])
    elif test == "flywheel_temp":
      self.test_flywheel_temp()
    elif test == "gps":
      self.test_gps(test["comprehensive"])
    else:
      self.r.print("Test not found!")
      controller_1.rumble("...---...")
    
  def test_flywheel(self, comprehensive = False):
    '''
    Test the flywheel speed to see if it holds over time, if the test is comprehensive it will see if the flywheel speed holds over 5 seconds and do more minute changes in speed 
    '''
    test_time = 5 if comprehensive else 2
    failed = False

    for speed in range(0,101,(10 if comprehensive else 25)):
      self.r.set_target_state({
        "flywheel_speed" : speed,
      })
      self.r.print("Testing flywheel...")
      wait(2, SECONDS)
      timer = Timer()
      timer.reset()

      # Run the test for 2 seconds and if the average speed is not good
      while timer.time(SECONDS) < test_time:
        flywheel_1_difference = self.r.flywheel_1_avg_speed - self.r.target_state["flywheel_speed"]
        flywheel_2_difference = self.r.flywheel_2_avg_speed - self.r.target_state["flywheel_speed"]
        print(flywheel_1_difference, flywheel_2_difference)
        if flywheel_1_difference > self.flywheel_speed_tolerance or flywheel_2_difference > self.flywheel_speed_tolerance:
          self.r.print(f("Flywheel speed should be", self.r.target_state["flywheel_speed"], "1:", self.r.flywheel_1_avg_speed,"2:", self.r.flywheel_2_avg_speed))
          failed = True
        wait(0.25, SECONDS)
    
    # If the robot failed this test then do a little rumble rumble
    if not failed:
      self.pass_test("Flywheel test complete!")
    else:
      self.fail_test("Flywheel test failed!")
  
  def test_flywheel_temp(self):
    if flywheel_motor_1.temperature() > 45 or flywheel_motor_2.temperature() > 45:
      self.fail_test(f("Flywheel motor is too hot! 1:", flywheel_motor_1.temperature(), "2:", flywheel_motor_2.temperature()))
    else:
      self.pass_test("Flywheel test complete!")
  

  def test_gps(self, comprehensive):
    path = [
      {
        "x" : 0,
        "y" : 0,
        "theta" : 0,
      },
      {
        "x" : 100,
        "y" : 100,
        "theta" : 180,
      },
      {
        "x" : 100,
        "y" : -100,
        "theta" : 0,
      },
      {
        "x" : -100,
        "y" : -100,
        "theta" : -180,
      },
      {
        "x" : -100,
        "y" : 100,
        "theta" : 0,
      },
      {
        "x" : 0,
        "y" : 0,
        "theta" : 180,
      },
    ]


    for point in path:
      self.r.set_target_state(point)
      self.r.print(("Robot should be going to...", point))
      
      # Go to that position until the up button is pressed
      while not controller_1.buttonUp.pressing():
        self.r.print(f("X:", self.r.x_pos, "Y:", self.r.y_pos, "Theta:", self.r.theta))
        wait(1, SECONDS)
        
        
prematch_checks = [
  Test(
    "flywheel",
    args = {
      "comprehensive" : False
    }
  ),
  Test(
    "flywheel_temp",
    {}
  ),
]

during_match_checks = [
  Test(
    "flywheel_temp",
    {}
  ),
]

all_tests = [
  Test(
    "flywheel",
    args = {
      "comprehensive" : True
    }
  ),
  Test(
    "flywheel_temp",
    {}
  ),
  Test(
    "gps",
    {}
  )
]

wait(30, MSEC)

######## COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ###
def autonomous():
    r.run_autonomous(auto_test)
    r.stop_moving()

def driver_control():
    # Change this to be relative to the match time???
    timer = Timer()
    timer.reset()

    reset_theta_timer = Timer()
    reset_theta_timer.reset()

    r.driver_controlled_timer.reset()

    r.autonomous_timer.reset()
    
    previous_controller_states = {
        "buttonL1" : controller_1.buttonL1.pressing(),
        "buttonR1" : controller_1.buttonR1.pressing(),
        "buttonL2" : controller_1.buttonL2.pressing(),
        "buttonR2" : controller_1.buttonR2.pressing(),
        "buttonUp" : controller_1.buttonUp.pressing(),
        "buttonDown" : controller_1.buttonDown.pressing(),
        "buttonLeft" : controller_1.buttonLeft.pressing(),
        "buttonRight" : controller_1.buttonRight.pressing(),
        "buttonX" : controller_1.buttonX.pressing(),
        "buttonY" : controller_1.buttonY.pressing(),
        "buttonA" : controller_1.buttonA.pressing(),
        "buttonB" : controller_1.buttonB.pressing(),
    }

    r.auto_intake = True
    r.auto_roller = False

    gui.render()
    while True:

        gui.update()
        gui.render()
        # Update the robot's information


        new_theta = controller_1.axis1.position() * 0.5
        # if controller_1.axis1.position() == 0:
        #     new_theta = r.target_state["theta"] - r.theta

        new_x = (controller_1.axis4.position()) 
        new_y = (controller_1.axis3.position())
        r.set_target_state(
            {
                "override_velocity_x" : new_x,
                "override_velocity_y" : new_y,
                "theta" : r.theta + new_theta,
                "slow_mode" : controller_1.buttonL1.pressing(),
                "auto_intake": False,
                "intake_speed" : controller_1.buttonL1.pressing() * 100 - controller_1.buttonR1.pressing() * 100,
            }
        )

        if controller_1.buttonA.pressing():
            r.set_target_state({
                "theta" : 90,
            })
        
        if controller_1.buttonY.pressing():
            r.set_target_state({
                "theta" : -90,
            })

        # If only the x button is being pressed then rotate the robot to 0 deg, 
        # if the x and a button is pressed, rotate to 45, if a and y pressed
        # rotate to -45 deg
        if controller_1.buttonX.pressing():
            if controller_1.buttonA.pressing():
                r.set_target_state({
                    "theta" : 45,
                })
            elif controller_1.buttonY.pressing():
                r.set_target_state({
                    "theta" : -45,
                })
            else:
                r.set_target_state({
                    "theta" : 0,
                })

        # If only the b button is being pressed then rotate the robot to 180 deg,
        # if the b and a button is pressed rotate to 45, if b and y is presed
        # rotate to -135 deg
        if controller_1.buttonB.pressing():
            if controller_1.buttonA.pressing():
                r.set_target_state({
                    "theta" : 135,
                })
            elif controller_1.buttonY.pressing():
                r.set_target_state({
                    "theta" : -135,
                })
            else:
                r.set_target_state({
                    "theta" : 180,
                })
        # Timer to print things out to the terminal every x seconds
        if (timer.time() > 0.1 * 1000):
            # print(r.flywheel_speed, flywheel_motor_1.velocity(PERCENT),flywheel_motor_2.velocity(PERCENT), r.flywheel_motor_1_error, r.flywheel_motor_2_error)
            # print(r.state["override_velocity_x"], r.state["override_velocity_y"], r.state["theta"])
            # print(r.x_pos, r.y_pos, r.theta, r.target_state["theta"])

            # Print the flywheel torque
            print("flywheel_motor_torque", flywheel_motor_1.torque(PERCENT), flywheel_motor_2.torque(PERCENT), "intake", roller_and_intake_motor_1.torque(PERCENT), roller_and_intake_motor_2.torque(PERCENT))
            controller_1.screen.clear_row(3)
            controller_1.screen.print(f("FLY:", r.flywheel_speed, "ang:", round(r.theta)))
            
            timer.reset()

        if controller_1.buttonUp.pressing():
            r.set_target_state({
                "launch_expansion" : True
            })
        elif controller_1.buttonDown.pressing():
            expansion.close()
        
        if controller_1.buttonLeft.pressing() or controller_1.buttonRight.pressing():
            r.shoot_disk()

        # When the buttons are pressed to change the level of the flywheel speed, r2 decreases level, l2 increases level
        if controller_1.buttonR2.pressing() and not previous_controller_states["buttonR2"]:
            temp_copy = r.flywheel_speed_levels.copy()
            temp_copy.append(r.flywheel_speed)
            temp_copy.sort()
            new_flywheel_speed = temp_copy[min(temp_copy.index(r.flywheel_speed)+1,len(temp_copy)-1)]
            r.set_target_state({
                # "flywheel_speed" : min(r.flywheel_speed + 2, 100)
                "flywheel_speed" : new_flywheel_speed
            })

        elif controller_1.buttonL2.pressing() and not previous_controller_states["buttonL2"]:
            temp_copy = r.flywheel_speed_levels.copy()
            temp_copy.append(r.flywheel_speed)
            temp_copy.sort()
            new_flywheel_speed = temp_copy[max(temp_copy.index(r.flywheel_speed)-1,0)]
            r.set_target_state({                
                # "flywheel_speed": max(r.flywheel_speed - 2, 0)
                "flywheel_speed": new_flywheel_speed
            })

        # Update previous controller states so we can track what the controller did before, we use this so that we can see if a button was pressed, not held, etc.
        previous_controller_states = {
            "buttonL1" : controller_1.buttonL1.pressing(),
            "buttonR1" : controller_1.buttonR1.pressing(),
            "buttonL2" : controller_1.buttonL2.pressing(),
            "buttonR2" : controller_1.buttonR2.pressing(),
            "buttonUp" : controller_1.buttonUp.pressing(),
            "buttonDown" : controller_1.buttonDown.pressing(),
            "buttonLeft" : controller_1.buttonLeft.pressing(),
            "buttonRight" : controller_1.buttonRight.pressing(),
            "buttonX" : controller_1.buttonX.pressing(),
            "buttonY" : controller_1.buttonY.pressing(),
            "buttonA" : controller_1.buttonA.pressing(),
            "buttonB" : controller_1.buttonB.pressing(),
        }


        wait(0.01, SECONDS)


########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ##########
# simple_path = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : 0.0, 'y' : 1.254751286449391},{ 'x' : 0.0, 'y' : 3.1368782161234776},{ 'x' : 0.0, 'y' : 5.64638078902226},{ 'x' : 0.0, 'y' : 8.783259005145766},{ 'x' : 0.0, 'y' : 11.292761578044576},{ 'x' : -0.6273756432246955, 'y' : 15.057015437392778},{ 'x' : -0.6273756432246955, 'y' : 18.82126929674098},{ 'x' : -0.6273756432246955, 'y' : 23.212898799313876},{ 'x' : 0.0, 'y' : 26.977152658662078},{ 'x' : 0.0, 'y' : 28.23190394511147},{ 'x' : 0.0, 'y' : 31.99615780445967},{ 'x' : 0.0, 'y' : 35.76041166380787},{ 'x' : 0.0, 'y' : 40.15204116638077},{ 'x' : 0.0, 'y' : 44.543670668953666},{ 'x' : 0.627375643224724, 'y' : 47.05317324185248},{ 'x' : 0.627375643224724, 'y' : 50.81742710120065},{ 'x' : 1.2547512864494195, 'y' : 55.20905660377355},{ 'x' : 1.2547512864494195, 'y' : 58.97331046312175},{ 'x' : 1.2547512864494195, 'y' : 62.110188679245255},{ 'x' : 1.2547512864494195, 'y' : 66.50181818181815},{ 'x' : 1.882126929674115, 'y' : 70.26607204116635},{ 'x' : 1.882126929674115, 'y' : 74.65770154373925},{ 'x' : 1.882126929674115, 'y' : 78.42195540308745},{ 'x' : 1.882126929674115, 'y' : 82.18620926243565},{ 'x' : 2.5095025728988105, 'y' : 85.32308747855916},{ 'x' : 2.5095025728988105, 'y' : 89.08734133790736},{ 'x' : 3.7642538593482016, 'y' : 92.85159519725556},{ 'x' : 4.391629502572897, 'y' : 95.98847341337904},{ 'x' : 5.019005145797621, 'y' : 99.12535162950255},{ 'x' : 5.6463807890223165, 'y' : 101.63485420240136},{ 'x' : 6.273756432247012, 'y' : 104.14435677530014},{ 'x' : 6.901132075471708, 'y' : 107.28123499142364},{ 'x' : 8.155883361921099, 'y' : 109.16336192109776},{ 'x' : 10.038010291595214, 'y' : 111.04548885077185},{ 'x' : 11.292761578044605, 'y' : 112.92761578044596},{ 'x' : 12.547512864493996, 'y' : 114.18236706689535},{ 'x' : 14.42963979416811, 'y' : 116.06449399656944},{ 'x' : 16.311766723842197, 'y' : 117.31924528301886},{ 'x' : 18.193893653516312, 'y' : 117.94662092624355},{ 'x' : 20.0760205831904, 'y' : 119.20137221269295},{ 'x' : 21.33077186963982, 'y' : 119.20137221269295},{ 'x' : 23.8402744425386, 'y' : 119.82874785591764},{ 'x' : 25.722401372212715, 'y' : 119.82874785591764},{ 'x' : 27.604528301886802, 'y' : 119.82874785591764},{ 'x' : 29.486655231560917, 'y' : 119.82874785591764},{ 'x' : 31.368782161235004, 'y' : 119.82874785591764},{ 'x' : 33.25090909090909, 'y' : 119.20137221269295},{ 'x' : 34.50566037735851, 'y' : 118.57399656946825},{ 'x' : 35.7604116638079, 'y' : 117.94662092624355},{ 'x' : 37.01516295025729, 'y' : 117.31924528301886},{ 'x' : 38.89728987993141, 'y' : 116.06449399656944},{ 'x' : 39.5246655231561, 'y' : 114.18236706689535},{ 'x' : 40.1520411663808, 'y' : 112.30024013722124},{ 'x' : 41.40679245283019, 'y' : 109.79073756432246},{ 'x' : 42.03416809605491, 'y' : 107.28123499142364},{ 'x' : 42.66154373927961, 'y' : 104.77173241852486},{ 'x' : 43.288919382504304, 'y' : 101.63485420240136},{ 'x' : 43.916295025729, 'y' : 99.12535162950255},{ 'x' : 44.543670668953695, 'y' : 95.98847341337904},{ 'x' : 45.17104631217839, 'y' : 91.59684391080614},{ 'x' : 45.17104631217839, 'y' : 88.45996569468267},{ 'x' : 45.17104631217839, 'y' : 85.32308747855916},{ 'x' : 45.17104631217839, 'y' : 81.55883361921096},{ 'x' : 45.17104631217839, 'y' : 78.42195540308745},{ 'x' : 45.17104631217839, 'y' : 73.40295025728986},{ 'x' : 44.543670668953695, 'y' : 70.26607204116635},{ 'x' : 44.543670668953695, 'y' : 66.50181818181815},{ 'x' : 44.543670668953695, 'y' : 63.99231560891937},{ 'x' : 44.543670668953695, 'y' : 60.855437392795864},{ 'x' : 43.916295025729, 'y' : 58.34593481989705},{ 'x' : 43.916295025729, 'y' : 54.58168096054885},{ 'x' : 43.916295025729, 'y' : 52.07217838765007},{ 'x' : 43.288919382504304, 'y' : 49.56267581475126},{ 'x' : 43.916295025729, 'y' : 47.05317324185248},{ 'x' : 43.916295025729, 'y' : 45.17104631217836},{ 'x' : 43.916295025729, 'y' : 43.288919382504275},{ 'x' : 43.288919382504304, 'y' : 41.40679245283016},{ 'x' : 43.288919382504304, 'y' : 38.89728987993138},{ 'x' : 43.288919382504304, 'y' : 37.01516295025726},{ 'x' : 43.288919382504304, 'y' : 34.50566037735845},{ 'x' : 43.288919382504304, 'y' : 32.623533447684366},{ 'x' : 42.66154373927961, 'y' : 30.74140651801028},{ 'x' : 42.66154373927961, 'y' : 29.48665523156086},{ 'x' : 43.288919382504304, 'y' : 26.977152658662078},{ 'x' : 43.288919382504304, 'y' : 25.72240137221266},{ 'x' : 43.288919382504304, 'y' : 23.840274442538572},{ 'x' : 43.288919382504304, 'y' : 22.58552315608918},{ 'x' : 43.288919382504304, 'y' : 21.33077186963976},{ 'x' : 43.288919382504304, 'y' : 20.07602058319037},{ 'x' : 43.288919382504304, 'y' : 18.82126929674098},{ 'x' : 43.288919382504304, 'y' : 17.56651801029156},{ 'x' : 43.288919382504304, 'y' : 15.057015437392778},{ 'x' : 43.916295025729, 'y' : 13.174888507718663},{ 'x' : 43.916295025729, 'y' : 11.920137221269272},{ 'x' : 44.543670668953695, 'y' : 10.66538593481988},{ 'x' : 45.17104631217839, 'y' : 9.410634648370461},{ 'x' : 46.42579759862781, 'y' : 7.528507718696375},{ 'x' : 48.307924528301896, 'y' : 6.901132075471679},{ 'x' : 49.56267581475129, 'y' : 6.273756432246984},{ 'x' : 51.4448027444254, 'y' : 5.64638078902226},{ 'x' : 53.954305317324184, 'y' : 5.019005145797564},{ 'x' : 55.209056603773604, 'y' : 5.019005145797564},{ 'x' : 57.09118353344769, 'y' : 5.019005145797564},{ 'x' : 58.973310463121805, 'y' : 5.64638078902226},{ 'x' : 60.2280617495712, 'y' : 5.64638078902226},{ 'x' : 62.73756432247001, 'y' : 6.901132075471679},{ 'x' : 63.9923156089194, 'y' : 8.15588336192107},{ 'x' : 65.87444253859348, 'y' : 9.410634648370461},{ 'x' : 68.3839451114923, 'y' : 11.920137221269272},{ 'x' : 69.63869639794169, 'y' : 14.429639794168082},{ 'x' : 70.8934476843911, 'y' : 16.939142367066864},{ 'x' : 72.1481989708405, 'y' : 20.07602058319037},{ 'x' : 73.40295025728989, 'y' : 23.212898799313876},{ 'x' : 74.03032590051458, 'y' : 26.349777015437354},{ 'x' : 74.65770154373931, 'y' : 30.114030874785556},{ 'x' : 75.285077186964, 'y' : 33.87828473413376},{ 'x' : 75.9124528301887, 'y' : 38.269914236706654},{ 'x' : 76.5398284734134, 'y' : 41.40679245283016},{ 'x' : 77.16720411663809, 'y' : 45.79842195540306},{ 'x' : 77.16720411663809, 'y' : 49.56267581475126},{ 'x' : 77.79457975986278, 'y' : 53.954305317324156},{ 'x' : 78.42195540308748, 'y' : 57.71855917667236},{ 'x' : 78.42195540308748, 'y' : 60.855437392795864},{ 'x' : 79.0493310463122, 'y' : 64.61969125214407},{ 'x' : 79.0493310463122, 'y' : 67.75656946826757},{ 'x' : 79.0493310463122, 'y' : 70.26607204116635},{ 'x' : 79.0493310463122, 'y' : 72.77557461406516},{ 'x' : 78.42195540308748, 'y' : 75.91245283018867},{ 'x' : 78.42195540308748, 'y' : 79.04933104631215},{ 'x' : 78.42195540308748, 'y' : 81.55883361921096},{ 'x' : 77.79457975986278, 'y' : 84.06833619210974},{ 'x' : 77.79457975986278, 'y' : 86.57783876500855},{ 'x' : 78.42195540308748, 'y' : 89.08734133790736},{ 'x' : 78.42195540308748, 'y' : 90.34209262435675},{ 'x' : 78.42195540308748, 'y' : 92.85159519725556},{ 'x' : 79.0493310463122, 'y' : 95.36109777015434},{ 'x' : 79.0493310463122, 'y' : 97.87060034305316},{ 'x' : 79.0493310463122, 'y' : 99.75272727272724},{ 'x' : 79.0493310463122, 'y' : 101.63485420240136},{ 'x' : 79.6767066895369, 'y' : 102.88960548885075},{ 'x' : 79.6767066895369, 'y' : 105.39910806174956},{ 'x' : 79.6767066895369, 'y' : 107.28123499142364},{ 'x' : 79.6767066895369, 'y' : 109.16336192109776},{ 'x' : 80.3040823327616, 'y' : 111.04548885077185},{ 'x' : 80.3040823327616, 'y' : 112.30024013722124},{ 'x' : 80.93145797598629, 'y' : 114.18236706689535},{ 'x' : 81.55883361921099, 'y' : 116.06449399656944},{ 'x' : 82.18620926243568, 'y' : 117.31924528301886},{ 'x' : 83.4409605488851, 'y' : 118.57399656946825},{ 'x' : 84.69571183533449, 'y' : 119.82874785591764},{ 'x' : 86.57783876500858, 'y' : 120.45612349914234},{ 'x' : 88.4599656946827, 'y' : 121.08349914236703},{ 'x' : 90.34209262435678, 'y' : 121.08349914236703},{ 'x' : 92.85159519725559, 'y' : 121.71087478559176},{ 'x' : 95.3610977701544, 'y' : 121.71087478559176},{ 'x' : 99.12535162950257, 'y' : 121.08349914236703},{ 'x' : 101.63485420240139, 'y' : 120.45612349914234},{ 'x' : 104.1443567753002, 'y' : 120.45612349914234},{ 'x' : 107.28123499142367, 'y' : 119.20137221269295},{ 'x' : 109.79073756432248, 'y' : 118.57399656946825},{ 'x' : 112.3002401372213, 'y' : 117.31924528301886},{ 'x' : 114.18236706689538, 'y' : 116.06449399656944},{ 'x' : 115.43711835334477, 'y' : 114.80974271012005},{ 'x' : 116.69186963979419, 'y' : 113.55499142367066},{ 'x' : 119.20137221269297, 'y' : 110.41811320754715},{ 'x' : 120.4561234991424, 'y' : 107.90861063464834},{ 'x' : 121.08349914236709, 'y' : 105.39910806174956},{ 'x' : 121.08349914236709, 'y' : 102.26222984562605},{ 'x' : 121.71087478559178, 'y' : 98.49797598627785},{ 'x' : 121.71087478559178, 'y' : 96.61584905660376},{ 'x' : 122.33825042881648, 'y' : 92.22421955403084},{ 'x' : 122.33825042881648, 'y' : 88.45996569468267},{ 'x' : 121.71087478559178, 'y' : 85.32308747855916},{ 'x' : 121.71087478559178, 'y' : 81.55883361921096},{ 'x' : 121.71087478559178, 'y' : 78.42195540308745},{ 'x' : 121.08349914236709, 'y' : 73.40295025728986},{ 'x' : 121.08349914236709, 'y' : 69.63869639794166},{ 'x' : 121.08349914236709, 'y' : 65.87444253859346},{ 'x' : 121.08349914236709, 'y' : 61.48281303602056},{ 'x' : 120.4561234991424, 'y' : 56.46380789022297},{ 'x' : 120.4561234991424, 'y' : 52.699554030874765},{ 'x' : 120.4561234991424, 'y' : 48.93530017152656},{ 'x' : 120.4561234991424, 'y' : 45.17104631217836},{ 'x' : 120.4561234991424, 'y' : 41.40679245283016},{ 'x' : 120.4561234991424, 'y' : 37.64253859348196},{ 'x' : 120.4561234991424, 'y' : 33.87828473413376},{ 'x' : 120.4561234991424, 'y' : 31.368782161234975},{ 'x' : 120.4561234991424, 'y' : 28.859279588336165},{ 'x' : 120.4561234991424, 'y' : 26.349777015437354},{ 'x' : 120.4561234991424, 'y' : 23.840274442538572},{ 'x' : 120.4561234991424, 'y' : 21.958147512864457},{ 'x' : 120.4561234991424, 'y' : 20.703396226415066},{ 'x' : 120.4561234991424, 'y' : 19.448644939965675},{ 'x' : 120.4561234991424, 'y' : 18.193893653516284},{ 'x' : 120.4561234991424, 'y' : 16.939142367066864},{ 'x' : 120.4561234991424, 'y' : 15.684391080617473},{ 'x' : 120.4561234991424, 'y' : 14.429639794168082},{ 'x' : 120.4561234991424, 'y' : 13.174888507718663},{ 'x' : 120.4561234991424, 'y' : 11.920137221269272},{ 'x' : 119.82874785591767, 'y' : 10.66538593481988},{ 'x' : 118.57399656946828, 'y' : 10.66538593481988},{ 'x' : 116.69186963979419, 'y' : 9.410634648370461},{ 'x' : 114.80974271012008, 'y' : 8.783259005145766},{ 'x' : 112.3002401372213, 'y' : 8.15588336192107},{ 'x' : 109.16336192109779, 'y' : 6.901132075471679},{ 'x' : 106.02648370497428, 'y' : 6.901132075471679},{ 'x' : 102.26222984562608, 'y' : 5.64638078902226},{ 'x' : 97.87060034305318, 'y' : 5.019005145797564},{ 'x' : 92.85159519725559, 'y' : 4.391629502572869},{ 'x' : 87.2052144082333, 'y' : 3.1368782161234776},{ 'x' : 82.18620926243568, 'y' : 3.1368782161234776},{ 'x' : 75.9124528301887, 'y' : 1.8821269296740866},{ 'x' : 70.26607204116638, 'y' : 1.254751286449391},{ 'x' : 64.6196912521441, 'y' : 1.254751286449391},{ 'x' : 58.34593481989711, 'y' : 0.6273756432246955},{ 'x' : 53.32692967409949, 'y' : 0.6273756432246955},{ 'x' : 48.307924528301896, 'y' : 0.6273756432246955},{ 'x' : 42.66154373927961, 'y' : 0.6273756432246955},{ 'x' : 37.01516295025729, 'y' : 0.6273756432246955},{ 'x' : 31.9961578044597, 'y' : 0.6273756432246955},{ 'x' : 29.486655231560917, 'y' : 0.6273756432246955},{ 'x' : 25.09502572898799, 'y' : 0.0},{ 'x' : 21.33077186963982, 'y' : 0.6273756432246955},{ 'x' : 16.939142367066893, 'y' : 0.6273756432246955},{ 'x' : 15.057015437392806, 'y' : 0.6273756432246955},{ 'x' : 11.9201372212693, 'y' : 0.6273756432246955},{ 'x' : 8.783259005145823, 'y' : 0.6273756432246955},{ 'x' : 5.6463807890223165, 'y' : 0.6273756432246955},{ 'x' : 3.136878216123506, 'y' : 0.0},{ 'x' : 1.882126929674115, 'y' : 0.0},]
straight_line = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : -0.6273756432246955, 'y' : 8.155883361921099},{ 'x' : -0.6273756432246955, 'y' : 15.057015437392778},{ 'x' : -1.2547512864494053, 'y' : 25.09502572898799},{ 'x' : -1.8821269296741008, 'y' : 36.38778730703257},{ 'x' : -3.136878216123506, 'y' : 46.42579759862778},{ 'x' : -3.136878216123506, 'y' : 55.209056603773575},{ 'x' : -3.7642538593482016, 'y' : 62.73756432246998},{ 'x' : -3.7642538593482016, 'y' : 69.01132075471696},{ 'x' : -3.136878216123506, 'y' : 73.40295025728986},]
mostyn = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : -0.63, 'y' : 4.39},{ 'x' : 0.0, 'y' : 10.04},{ 'x' : 0.0, 'y' : 13.8},{ 'x' : 0.0, 'y' : 20.08},{ 'x' : 0.63, 'y' : 32.0},{ 'x' : 1.25, 'y' : 42.66},{ 'x' : 1.88, 'y' : 53.95},{ 'x' : 1.88, 'y' : 63.36},{ 'x' : 1.88, 'y' : 72.78},{ 'x' : 1.88, 'y' : 80.3},{ 'x' : 1.25, 'y' : 87.21},{ 'x' : 0.63, 'y' : 93.48},{ 'x' : -0.63, 'y' : 99.13},{ 'x' : -1.88, 'y' : 104.77},{ 'x' : -2.51, 'y' : 110.42},{ 'x' : -3.76, 'y' : 116.06},{ 'x' : -4.39, 'y' : 122.34},{ 'x' : -5.02, 'y' : 127.98},{ 'x' : -5.02, 'y' : 134.26},{ 'x' : -5.02, 'y' : 140.53},{ 'x' : -4.39, 'y' : 146.18},{ 'x' : -4.39, 'y' : 151.2},{ 'x' : -3.76, 'y' : 156.84},{ 'x' : -3.14, 'y' : 161.24},{ 'x' : -3.14, 'y' : 162.49},{ 'x' : -3.14, 'y' : 163.75},{ 'x' : -3.14, 'y' : 165.0},{ 'x' : -3.14, 'y' : 166.25},{ 'x' : -3.14, 'y' : 167.51},{ 'x' : -3.14, 'y' : 168.76},{ 'x' : -1.25, 'y' : 170.02},{ 'x' : 0.63, 'y' : 171.27},{ 'x' : 3.76, 'y' : 171.9},{ 'x' : 6.9, 'y' : 173.16},{ 'x' : 11.29, 'y' : 173.78},{ 'x' : 16.31, 'y' : 175.04},{ 'x' : 21.33, 'y' : 176.29},{ 'x' : 26.98, 'y' : 176.92},{ 'x' : 32.62, 'y' : 178.17},{ 'x' : 42.66, 'y' : 178.8},{ 'x' : 50.19, 'y' : 180.06},{ 'x' : 60.23, 'y' : 180.06},{ 'x' : 63.99, 'y' : 180.06},{ 'x' : 72.15, 'y' : 180.06},{ 'x' : 79.68, 'y' : 178.8},{ 'x' : 85.95, 'y' : 176.92},{ 'x' : 92.85, 'y' : 174.41},{ 'x' : 98.5, 'y' : 171.27},{ 'x' : 101.01, 'y' : 169.39},{ 'x' : 106.03, 'y' : 165.63},{ 'x' : 111.05, 'y' : 161.24},{ 'x' : 115.44, 'y' : 156.84},{ 'x' : 119.83, 'y' : 151.82},{ 'x' : 124.22, 'y' : 146.81},{ 'x' : 127.98, 'y' : 141.79},{ 'x' : 131.12, 'y' : 136.77},{ 'x' : 134.26, 'y' : 131.12},{ 'x' : 136.77, 'y' : 125.48},{ 'x' : 139.28, 'y' : 119.83},{ 'x' : 141.79, 'y' : 114.18},{ 'x' : 143.67, 'y' : 108.54},{ 'x' : 145.55, 'y' : 102.26},{ 'x' : 146.81, 'y' : 95.99},{ 'x' : 147.43, 'y' : 90.34},{ 'x' : 147.43, 'y' : 82.81},{ 'x' : 146.81, 'y' : 75.91},{ 'x' : 146.18, 'y' : 67.76},{ 'x' : 143.67, 'y' : 59.6},{ 'x' : 139.9, 'y' : 51.44},{ 'x' : 135.51, 'y' : 42.66},{ 'x' : 131.12, 'y' : 34.51},{ 'x' : 126.1, 'y' : 26.98},{ 'x' : 120.46, 'y' : 20.08},{ 'x' : 114.81, 'y' : 13.8},{ 'x' : 109.16, 'y' : 8.78},{ 'x' : 106.65, 'y' : 6.27},{ 'x' : 100.38, 'y' : 1.88},{ 'x' : 94.11, 'y' : -1.25},{ 'x' : 88.46, 'y' : -3.76},{ 'x' : 82.19, 'y' : -5.65},{ 'x' : 75.29, 'y' : -6.27},{ 'x' : 72.15, 'y' : -6.9},{ 'x' : 65.25, 'y' : -6.27},{ 'x' : 58.97, 'y' : -5.65},{ 'x' : 52.07, 'y' : -4.39},{ 'x' : 45.8, 'y' : -3.76},{ 'x' : 39.52, 'y' : -1.88},{ 'x' : 33.25, 'y' : -0.63},{ 'x' : 26.98, 'y' : 0.63},{ 'x' : 21.33, 'y' : 2.51},{ 'x' : 16.31, 'y' : 4.39},{ 'x' : 11.29, 'y' : 5.65},{ 'x' : 6.9, 'y' : 6.27},]
# revamping_everthing = [{ 'x' : 0.0, 'y' : 0.0},{ 'x' : 0.0, 'y' : 1.88},{ 'x' : -0.63, 'y' : 3.14},{ 'x' : -0.63, 'y' : 4.39},{ 'x' : -0.63, 'y' : 6.27},{ 'x' : -1.25, 'y' : 7.53},{ 'x' : -1.25, 'y' : 10.67},{ 'x' : -1.88, 'y' : 13.8},{ 'x' : -1.88, 'y' : 22.59},{ 'x' : -1.88, 'y' : 28.23},{ 'x' : -2.51, 'y' : 35.76},{ 'x' : -2.51, 'y' : 41.41},{ 'x' : -3.14, 'y' : 47.05},{ 'x' : -2.51, 'y' : 52.07},{ 'x' : -1.88, 'y' : 56.46},{ 'x' : -1.25, 'y' : 60.86},{ 'x' : -0.63, 'y' : 64.62},{ 'x' : 0.0, 'y' : 69.01},{ 'x' : 0.63, 'y' : 72.78},{ 'x' : 0.63, 'y' : 77.79},{ 'x' : 1.88, 'y' : 80.93},{ 'x' : 2.51, 'y' : 82.81},{ 'x' : 3.76, 'y' : 85.32},{ 'x' : 5.65, 'y' : 87.21},{ 'x' : 7.53, 'y' : 90.34},{ 'x' : 9.41, 'y' : 91.6},{ 'x' : 10.67, 'y' : 93.48},{ 'x' : 11.92, 'y' : 94.11},{ 'x' : 13.8, 'y' : 94.73},{ 'x' : 15.06, 'y' : 95.36},{ 'x' : 16.31, 'y' : 95.99},{ 'x' : 17.57, 'y' : 95.99},{ 'x' : 18.82, 'y' : 95.99},{ 'x' : 20.08, 'y' : 95.99},{ 'x' : 21.33, 'y' : 95.36},{ 'x' : 22.59, 'y' : 94.73},{ 'x' : 24.47, 'y' : 94.11},{ 'x' : 25.72, 'y' : 92.85},{ 'x' : 27.6, 'y' : 90.97},{ 'x' : 27.6, 'y' : 89.71},{ 'x' : 28.86, 'y' : 88.46},{ 'x' : 29.49, 'y' : 86.58},{ 'x' : 30.11, 'y' : 84.7},{ 'x' : 30.11, 'y' : 82.19},{ 'x' : 30.74, 'y' : 80.93},{ 'x' : 31.37, 'y' : 78.42},{ 'x' : 31.37, 'y' : 75.29},{ 'x' : 32.0, 'y' : 72.78},{ 'x' : 32.62, 'y' : 69.64},{ 'x' : 32.62, 'y' : 65.87},{ 'x' : 32.62, 'y' : 62.11},{ 'x' : 32.62, 'y' : 57.72},{ 'x' : 32.62, 'y' : 52.7},{ 'x' : 33.25, 'y' : 49.56},{ 'x' : 33.25, 'y' : 47.68},{ 'x' : 33.25, 'y' : 43.92},{ 'x' : 33.25, 'y' : 40.15},{ 'x' : 33.88, 'y' : 34.51},{ 'x' : 33.88, 'y' : 30.74},{ 'x' : 33.88, 'y' : 26.98},{ 'x' : 33.88, 'y' : 24.47},{ 'x' : 33.88, 'y' : 20.7},{ 'x' : 33.88, 'y' : 16.31},{ 'x' : 34.51, 'y' : 12.55},{ 'x' : 34.51, 'y' : 10.04},{ 'x' : 35.13, 'y' : 7.53},{ 'x' : 35.76, 'y' : 5.65},{ 'x' : 36.39, 'y' : 3.76},{ 'x' : 37.64, 'y' : 2.51},{ 'x' : 38.9, 'y' : 0.0},{ 'x' : 39.52, 'y' : -1.88},{ 'x' : 40.78, 'y' : -2.51},{ 'x' : 42.66, 'y' : -3.76},{ 'x' : 44.54, 'y' : -5.02},{ 'x' : 47.05, 'y' : -6.27},{ 'x' : 48.94, 'y' : -6.9},{ 'x' : 53.33, 'y' : -5.65},{ 'x' : 57.09, 'y' : -5.02},{ 'x' : 58.35, 'y' : -3.76},{ 'x' : 59.6, 'y' : -2.51},{ 'x' : 60.86, 'y' : -1.25},{ 'x' : 62.74, 'y' : 1.88},{ 'x' : 63.99, 'y' : 3.76},{ 'x' : 65.25, 'y' : 6.9},{ 'x' : 65.87, 'y' : 10.67},{ 'x' : 67.13, 'y' : 14.43},{ 'x' : 67.13, 'y' : 16.31},{ 'x' : 68.38, 'y' : 20.08},{ 'x' : 68.38, 'y' : 25.1},{ 'x' : 69.01, 'y' : 29.49},{ 'x' : 69.64, 'y' : 34.51},{ 'x' : 70.27, 'y' : 39.52},{ 'x' : 70.89, 'y' : 45.8},{ 'x' : 70.89, 'y' : 48.94},{ 'x' : 71.52, 'y' : 54.58},{ 'x' : 73.4, 'y' : 65.25},{ 'x' : 73.4, 'y' : 73.4},{ 'x' : 74.66, 'y' : 82.81},{ 'x' : 74.66, 'y' : 90.34},{ 'x' : 75.91, 'y' : 97.24},{ 'x' : 76.54, 'y' : 102.26},{ 'x' : 77.17, 'y' : 107.28},{ 'x' : 77.79, 'y' : 108.54},{ 'x' : 78.42, 'y' : 107.28},{ 'x' : 80.3, 'y' : 106.65},{ 'x' : 82.19, 'y' : 106.03},{ 'x' : 84.07, 'y' : 106.65},{ 'x' : 86.58, 'y' : 106.65},{ 'x' : 89.09, 'y' : 106.03},{ 'x' : 91.6, 'y' : 106.03},{ 'x' : 94.11, 'y' : 105.4},{ 'x' : 95.36, 'y' : 105.4},{ 'x' : 97.87, 'y' : 104.77},{ 'x' : 100.38, 'y' : 104.14},{ 'x' : 102.26, 'y' : 103.52},{ 'x' : 104.14, 'y' : 102.26},{ 'x' : 106.03, 'y' : 101.01},{ 'x' : 107.28, 'y' : 100.38},{ 'x' : 109.16, 'y' : 98.5},{ 'x' : 110.42, 'y' : 96.62},{ 'x' : 111.67, 'y' : 94.11},{ 'x' : 112.3, 'y' : 92.85},{ 'x' : 112.93, 'y' : 90.97},{ 'x' : 113.55, 'y' : 89.09},{ 'x' : 114.18, 'y' : 87.83},{ 'x' : 114.18, 'y' : 85.95},{ 'x' : 114.81, 'y' : 84.7},{ 'x' : 115.44, 'y' : 82.19},{ 'x' : 115.44, 'y' : 80.93},{ 'x' : 115.44, 'y' : 79.68},{ 'x' : 115.44, 'y' : 76.54},{ 'x' : 115.44, 'y' : 74.03},{ 'x' : 115.44, 'y' : 70.89},{ 'x' : 115.44, 'y' : 67.13},{ 'x' : 114.81, 'y' : 63.36},{ 'x' : 114.81, 'y' : 58.97},{ 'x' : 114.18, 'y' : 55.21},{ 'x' : 113.55, 'y' : 51.44},{ 'x' : 113.55, 'y' : 48.31},{ 'x' : 113.55, 'y' : 45.8},{ 'x' : 113.55, 'y' : 44.54},{ 'x' : 112.93, 'y' : 40.78},{ 'x' : 112.93, 'y' : 38.9},{ 'x' : 112.93, 'y' : 36.39},{ 'x' : 112.93, 'y' : 34.51},{ 'x' : 112.93, 'y' : 32.62},{ 'x' : 112.93, 'y' : 30.74},{ 'x' : 112.93, 'y' : 28.86},{ 'x' : 112.3, 'y' : 26.98},{ 'x' : 112.93, 'y' : 25.1},{ 'x' : 112.93, 'y' : 23.21},{ 'x' : 112.93, 'y' : 21.96},{ 'x' : 113.55, 'y' : 20.08},{ 'x' : 113.55, 'y' : 18.82},{ 'x' : 113.55, 'y' : 17.57},{ 'x' : 113.55, 'y' : 16.31},{ 'x' : 113.55, 'y' : 15.06},{ 'x' : 114.18, 'y' : 13.8},{ 'x' : 114.18, 'y' : 12.55},{ 'x' : 114.18, 'y' : 11.29},{ 'x' : 114.18, 'y' : 10.04},{ 'x' : 114.18, 'y' : 8.78},{ 'x' : 114.18, 'y' : 7.53},{ 'x' : 113.55, 'y' : 6.27},{ 'x' : 113.55, 'y' : 5.02},{ 'x' : 111.67, 'y' : 3.76},{ 'x' : 107.28, 'y' : 2.51},{ 'x' : 104.77, 'y' : 1.88},{ 'x' : 97.24, 'y' : 0.63},{ 'x' : 92.85, 'y' : 0.63},{ 'x' : 89.71, 'y' : 0.63},{ 'x' : 80.93, 'y' : 0.63},{ 'x' : 78.42, 'y' : 0.63},{ 'x' : 69.01, 'y' : 0.63},{ 'x' : 60.86, 'y' : 0.63},{ 'x' : 53.95, 'y' : 1.88},{ 'x' : 46.43, 'y' : 2.51},{ 'x' : 40.15, 'y' : 3.76},{ 'x' : 33.88, 'y' : 5.02},{ 'x' : 27.6, 'y' : 5.65},{ 'x' : 21.33, 'y' : 6.27},{ 'x' : 15.68, 'y' : 6.27},{ 'x' : 9.41, 'y' : 6.9},{ 'x' : 3.76, 'y' : 6.27},{ 'x' : -1.88, 'y' : 5.65},{ 'x' : -5.65, 'y' : 5.65},{ 'x' : -9.41, 'y' : 5.02},]

auto_test = [
    {
        "auto_intake" : False,
        "auto_roller" : False,
        "intake_speed" : 100,
        "wait" : 2,
        "message" : "THIS IS A TEST"
    }
]

field_length = 356  # CM

r = Robot()

r.using_gps = gps.installed()
r.drone_mode = True
r.slow_mode = False
r["auto_roller"] = False

r.save_states = False

# kP = 0.5
# kI = 0.25
# kD = 0
# r.flywheel_motor_1_PID.set_constants(kP,kI,kD)
# r.flywheel_motor_2_PID.set_constants(kP,kI,kD)


r.x_vel_PID.set_constants(10,0,0)
r.y_vel_PID.set_constants(10,0,0)
r.theta_vel_PID.set_constants(10,0,1)

########## GUI ########## GUI ##########

# See if this can be moved in the gui.py file to make it make a little bit more sense

gui = GUI()

gui.add_element(Switch(
    ["Team Blue", "Team Red"],
    0,
    0,
    120,
    40,
    [Color.BLUE, Color.RED],
    [r.set_team] * 2,
    ["blue", "red"]
))

def change_drone_mode(value):
    r.set_target_state({
        "drone_mode" : value
    })
    print("r.drone_mode =",r.drone_mode)

gui.add_element(Switch(
    ["Drone Mode", "Robot Mode"],
    120,
    0,
    120,
    40,
    [Color(0x888888), Color.BLACK],
    [change_drone_mode] * 2,
    (True, False),
))

# Create a grid that increments the flywheel speed
gui.add_element(Button(
    "Flywheel +2",
    0,
    40,
    120,
    40,
    (0x88AA88),
    lambda: r.set_target_state({
        "flywheel_speed" : r.flywheel_speed + 2,
    }),
))

gui.add_element(Button(
    "Flywheel -2",
    0,
    80,
    120,
    40,
    (0xAA8888),
    lambda: r.set_target_state({
        "flywheel_speed" : r.flywheel_speed - 2,
    }),
))

# Reset theta function
def reset_robot_theta():
    r.set_target_state({
        "theta" : 0
    })
    r.total_theta = 0

gui.add_element(Button(
    "Reset ??",
    0,
    200,
    120,
    40,
    (0xAAAAFA),
    reset_robot_theta
))

# Append the current state to a path
gui.add_element(Button(
    "Add to Path",
    120,
    200,
    120,
    40,
    (0xAAFAAA),
    r.path.append(r.state.copy())
))

# Save the path by printing the representation (in the future it can be saved to sd card) 
gui.add_element(Button(
    "Save Path",
    240,
    200,
    120,
    40,
    (0xAAAAAA),
    lambda: print(repr(r.path))
))


# IMPORTANT NUMBERS
gui.add_element(Text(
    "",
    240,
    0,
    120,
    40,
    (0x110000),
    lambda: f("SP:", r.flywheel_speed),
))

gui.add_element(Text(
    "",
    360,
    0,
    120,
    40,
    (0x110000),
    lambda: f("Real:", round(r.flywheel_1_avg_speed), round(r.flywheel_2_avg_speed))
))

gui.add_element(Text(
    "",
    240,
    40,
    120,
    40,
    Color.BLACK,
    lambda: "x: {:.2f}".format(r.x_pos),
))
gui.add_element(Text(
    "",
    240,
    80,
    120,
    40,
    Color.BLACK,
    lambda: "y: {:.2f}".format(r.y_pos),
))
gui.add_element(Text(
    "",
    240,
    120,
    120,
    40,
    Color.BLACK,
    lambda: f("??:", str(r.theta).split(".")[0]),
))
gui.add_element(Text(
    "",
    360,
    0,
    120,
    40,
    Color.BLACK,
    lambda: f("temp:", flywheel_motor_1.temperature(), flywheel_motor_2.temperature())
))

######### GUI ######### GUI ######### GUI ######### GUI ######### GUI ######### GUI #########

init()
r.init()

rd = RobotDoctor()
# rd.init()
# rd.run_tests(all_tests)

r.using_gps = False

gui.render()

# autonomous()
driver_control()
# competition = Competition(driver_control, autonomous)

##! PRIORITY
# TODO: Make it so that the robot movement is based off of the gps for now AND THEN add it using the encoders (kinda backwards, ik)
# TODO: Figure out if serial communication is 2 ways, if we can use input(), and then if we can run that command with os.sys()
# TODO: Instead of making the new target position based off of the current state, make it based off of the target state
# TODO: Make robot doctor program
# TODO: test robot doctor program and use it in the init method if successful
# TODO: Theoretically, shouldn't max acceleration half if we're close to 0 for the motor speed? rework that function
# TODO: Make a better pose/orienation initialization/resetting tool (initialize the position in the init() command, don't worry if we can't init yet)
# TODO: Use sensor fusion with Kalman filter for better position
# TODO: Make it os that the we init the robots orientation based off othe GPS (round it to the nearest 90 or 45 degrees), but make it so that drone mode still works
# TODO: Make odometry consistent with real world measurements
# TODO: Use trapazoids instead of rectangles when integrating the velocity
# TODO: See if we can register callback functions instead of checking things in the updates and see how much that will improve performance

# * IMPORTANT
# TODO: Instead of rotating the driver control vector based off of the current theta, why not move it based off of the predicted theta/halfway between the two? this might remove the drifting that happens when rotation while moving
# TODO: Figure out variance in GPS sensor data and variance in model (Q and R)
# TODO: make the offset of the gps so that it reports the position of the center of the robot
# TODO: When the robot has an x,y position close to the edge of the field, make it so that the robot physically cannot slam into the wall (or use sensors)
# TODO: Change this to be relative to the match time???

# Not important
# TODO: Work with david and figure out how to make robot driving feel better (more responsive, smooth, etc.)
# TODO: test to see if threads die when they lose scope
# TODO: make an auto path class that has information like description, color, etc.
# TODO: Research how to utilize motor.temperature
# TODO: Make an info screen that shows if there are any status issues wrong with the robot