import math
import time
from vex import *
from math import cos, sin, pi, sqrt, atan2


# Constants initialization
global g
g = -9.81  # Use for flywheel speed calculator

global RAD_TO_DEG
global DEG_TO_RAD
RAD_TO_DEG = 180 / math.pi
DEG_TO_RAD = math.pi / 180

global robot_debug_mode
robot_debug_mode = False

global recording_autonomous
recording_autonomous = False

global r2o2
r2o2 = math.sqrt(2) / 2

# *###### INITIATLIZATION OF PERIPHERALS
brain = Brain()
controller_1 = Controller(PRIMARY)
controller_2 = Controller(PARTNER)
left_motor_a = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)

right_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)

flywheel_motor_1 = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
flywheel_motor_2 = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)

indexer_limit_switch = DigitalIn(brain.three_wire_port.c)

inertial = Inertial(Ports.PORT8)

roller_and_intake_motor_1 = Motor(Ports.PORT4, GearSetting.RATIO_36_1, False)
roller_and_intake_motor_2 = Motor(Ports.PORT5, GearSetting.RATIO_36_1, False)
roller_and_intake_motor = MotorGroup(
    roller_and_intake_motor_1, roller_and_intake_motor_2)

roller_optical = Optical(Ports.PORT6)

indexer = Pneumatics(brain.three_wire_port.a)
expansion = Pneumatics(brain.three_wire_port.b)

flywheel_status_light = Led(brain.three_wire_port.h)

gps = Gps(Ports.PORT12)

# Vision signatures
vision__DISC = Signature(1, 6911, 8133, 7522, -6787, -5937, -6362, 1.3, 0)
vision__BRIGHT_DISK = Signature(2, 217, 491, 354, -7169, -6839, -7004, 3, 0)
vision__RED = Signature(3, 7243, 8689, 7966,-701, 107, -297,3, 0)
vision__BLUE = Signature(4, -1985, 1, -992,1981, 6665, 4323,1.4, 0)
vision__DISK = Signature(5, 1905, 2299, 2102,-4017, -3641, -3829,2.5, 0)
vision = Vision(Ports.PORT7, 50, vision__DISC, vision__BRIGHT_DISK, vision__RED,
                vision__BLUE, vision__DISK)

DISC_SIGNATURES = [vision__DISC, vision__BRIGHT_DISK]

def set_debug_value(_value):
    one_controller_mode = _value

def start_recording_mode_for_autonomous(_value):
    print("I HAVE BEEN PRESSED AND MY VAL IS", _value)
    global recording_autonomous 
    recording_autonomous = _value

def print_state_nicely(state):
  # print white space so we can see easier when coping and pasting
  print("[\n")

  # nicely format the state dictionary
  for s in state:
    _str = ""
    _str += "\n    {"
    for key in s:
      _str += "\n        " + "\"" + key + "\"" + ": " + str(s[key]) + ","
    _str += "\n    },"
    print(_str)

  print("]\n")

def get_angle_to_object(pos_1, pos_2):
    '''
    RETURNS IN DEGREES
    '''

    # If the passed in objects are GameObjects then change the pos's into a tuple of x,y values
    if type(pos_1) == GameObject:
        pos_1 = (pos_1.x, pos_1.y)
    if type(pos_2) == GameObject:
        pos_2 = (pos_2.x, pos_2.y)

    ang = atan2(pos_2[0] - pos_1[0], pos_2[1] - pos_1[1]) * RAD_TO_DEG
    if ang > 180:
        ang -= 360
    if ang < -180:
        ang += 360

    return ang

# 53
def init():
    '''
    This function will initialize every subsystem with constants that wont change throughout the competition,
    it will set driver controlled motors to break mode, start spinning motors that use the "set_velocity()" function,
    and initialize our gyroscope

    '''
    left_motor_a.set_stopping(BRAKE)
    right_motor_a.set_stopping(BRAKE)
    left_motor_b.set_stopping(BRAKE)
    right_motor_b.set_stopping(BRAKE)

    flywheel_motor_1.spin(FORWARD, 0, VOLT)
    flywheel_motor_2.spin(FORWARD, 0, VOLT)

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

    # Set the optical light power
    # roller_optical.set_light_power(100)
    # roller_optical.object_detect_threshold(0)

    expansion.close()

    t = Timer()
    t.reset()

    # Set our target states (this initializes drone_mode on and gusing gps is determined if the gps is plugged in)
    r.set_target_state({
        "drone_mode": False,
        "using_gps": gps.installed(),
    })

    gps.set_origin(84, 200, MM)

    # Wait for the gyro to settle, if it takes more then 10 seconds then close out of the loop
    # When the gyro sensor inits, it reads some value for the Z rotation, this is less than a few degrees, but i don't like it
    while (inertial.gyro_rate(ZAXIS) != 0 and t.value() < 10):
        print("Waiting for gyro to init...")
        wait(0.1, SECONDS)

    # Rumlbed the control to indicate to the driver (and me) that the robot is ready to run
    controller_1.rumble("...")


class GameObject:
    '''
    If we want to introduce game object (like say the goal or barriers), we have have to robot look up important information about the game object
    '''

    def __init__(self, x_pos, y_pos):
        self.x_pos = x_pos
        self.y_pos = y_pos


def f(*args):
    '''
    This function replaces the f-strings that are in python 3.8 (i think) and above, but aren't in python 3.6, which is what the brain uses
    '''
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
    if _max < _min:
        _max, _min = _min, _max
    return max(min(num, _max), _min)


def rotate_vector_2d(x, y, theta):
    '''
    Rotates a vector by theta degrees
    '''
    x_old = x
    x = x * math.cos(theta) - y * math.sin(theta)
    y = x_old * math.sin(theta) + y * math.cos(theta)

    return x, y


def getPathOnXYFunction(funcs, delta_t=0.01):
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

def getThetaForPathToHitPoint(v_i, point, sizeOfPoint=0.05):
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


def getViForPathToHitPoint(theta, point, sizeOfPoint=0.05):
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

    _x, _y, _t = getPathOnXYFunction(
        returnXYFuncs(theta, new_vi_1), delta_time)
    minimum_distance_vi_1 = float('inf')

    hit_time_vi_1 = 0

    # Find the point that is the closest to the point we want to hit
    for x, y in zip(_x, _y):
      distance = math.sqrt((x - point[0])**2 + (y - point[1])**2)
      minimum_distance_vi_1 = min(minimum_distance_vi_1, distance)
      if minimum_distance_vi_1 == distance:
        hit_time_vi_1 = (_x.index(x) + 1) * delta_time

    _x, _y, _t = getPathOnXYFunction(
        returnXYFuncs(theta, new_vi_2), delta_time)
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
    '''
    Basic button class, params:
      -  name: Name of button (gets displayed in the middle of the button)
      -  x: x position of the top-left corner of the button on the screen
      -  y: y position of the top-left corner of the button on the screen
      -  w: width of the button (px)
      -  h: height of the button (px)
      -  color: color of the button
      -  call_back: function that gets called when the button is pressed
      -  args: arguments that get passed to the functions
    '''
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

        # Draw a rectangle on the screen
        brain.screen.draw_rectangle(self.x, self.y, self.w, self.h, self.color)   

        # Figure out the x and y position of the text so that it gets centered, the max() function prevents the text from going outside the left edge of the box
        x_position_of_text = max((self.x + self.w / 2) - (len(self.name) * 5), self.x)
        y_position_of_text = max(self.y + self.h / 2 + 5 , self.y)

        brain.screen.print_at(self.name, x=x_position_of_text, y=y_position_of_text, opaque = False)

    def set_callback(self, function):
        self.call_back = function

    # If we do something like button() then it will run the callback function
    def __call__(self):
        if self.call_back != None:
            # If self is a variable in the call_back function then pass self as the button 
            self.call_back(*self.args)

class Text:
    '''
    Basic text class, params:
      -  name: Name of text (gets displayed in the middle of the text)
      -  x: x position of the top-left corner of the text-box on the screen
      -  y: y position of the top-left corner of the text-box on the screen
      -  w: width that the text-box occupates 
      -  h: height that the text-box occupates 
      -  color: color of the text fill
      -  call_back: function that gets called periodically to update the text name (function must return a string)
      -  args: arguments that get passed to the functions
    '''
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
        # If there is a call back function, then call it and set the name to the return value, else the name will never change
        if self.call_back != None:
            self.name = self.call_back(*self.args)

        brain.screen.draw_rectangle(self.x, self.y, self.w, self.h, self.color)

        # Figure out the x and y position of the text so that it gets centered, the max() function prevents the text from going outside the left edge of the box
        x_position_of_text = max((self.x + self.w / 2) - (len(self.name) * 5), self.x)
        y_position_of_text = max(self.y + self.h / 2 + 5 , self.y)
        
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

        # Figure out the x and y position of the text so that it gets centered, the max() function prevents the text from going outside the left edge of the box
        x_position_of_text = max(self.x + self.w / 2 - len(self.name[self.current_state]) * 5, self.x)
        y_position_of_text = max(self.y + self.h / 2 + 5 , self.y)

        brain.screen.print_at(self.name[self.current_state], x=x_position_of_text, y=y_position_of_text, opaque = False)

    def set_states(self, states):
        self.states = states

    def set_state(self, state):
        self.current_state = state

    def change_state(self):
        self.current_state = (self.current_state + 1) % len(self.name)

    def run_state(self):
        try:
            self.states[self.current_state](*[arg[self.current_state] for arg in self.args])
        except IndexError:
            print("index error lmao")

    def __call__(self):
        '''
        Whenever the switch gets pressed, change the sate of the switch (which changes the name and the color), and run the new call-back function
        '''
        self.change_state()
        self.run_state()


class GUI:
  '''
  What I want this class to do it to make a way for the drivers to interact with the brain screen and see some status
  things like if the motors are too hot, or if there is any self-diagnosed problem. I also want people to be able to select
  from the brain what team we're on.

  Brain screen dimensions: 480 x 240 pizels. Top left is (0,0)

  Each character in a string is 10 x 10 pixels
  '''

  elements = []

  pages = []

  page_num = 0

  previous_brain_screen_state = False

  def __init__(self):
    Thread(self.update_forever)

  def add_page(self, elements = []):
    self.pages.append(elements)

  def add_element(self, element, page_num = None):
    if page_num == None:
        self.elements.append(element)
        return
    self.pages[page_num].append()


  def update(self):
    # If the brain has been pressed ANYWHERE
    if brain.screen.pressing() and not self.previous_brain_screen_state:
        # X and y positions of where the finger pressed
        x, y = brain.screen.x_position(), brain.screen.y_position()
        for element in self.pages[self.page_num]:
            # Figure out if the finger press was inside the bound-box of an element
            if (x - element.x) > 0 and (x - element.x) < element.w and (y-element.y) > 0 and (y - element.y) < element.h:
                # Run the callback function of the element as a thread (so the rest of the code DOES NOT stop)
                Thread(element.__call__)

        
    self.previous_brain_screen_state = brain.screen.pressing()
  
  def render(self):
    '''
    Renders each element of the gui
    '''
    brain.screen.clear_screen()
    if len(self.pages) > 0:
        for element in self.pages[self.page_num]:
            element.render()
    
    brain.screen.render()

  def set_page(self, page_num):
    self.page_num = page_num
    self.elements = self.pages[page_num - 1]

  def update_forever(self):
    while True:
        self.update()
        self.render()
        wait(0.05, SECONDS)

class Vector:
    '''
    Vector class I wrote because basic python lists are lame, this is as slow as normal python, should be ideally replaced with numpy arrays
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

class Robot:
    '''
    This is the big-boy robot class, this is the class that controls the robot, there is a lot of stuff here
    '''
    # * ORIENTATION/POSITION VARIABLES
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

    gps_theta_on_robot = 90 # gps is 90 deg to the right from the center of the robot

    # * FLYWHEEL
    length: float = 38.1

    x_from_gps = 0
    y_from_gps = 0
    
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

    integral_term_flywheel_1 = 0
    integral_term_flywheel_2 = 0

    previous_flywheel_speed = 0

    flywheel_speed = 0

    flywheel_height_from_ground_IN = -99999

    flywheel_motor_1_PID = PID(2, 0, 0)
    flywheel_motor_2_PID = PID(2, 0, 0)

    flywheel_motor_1_error = 0
    flywheel_motor_2_error = 0

    running_autonomous = False

    # * DRIVETRAIN

    previous_update_time: float = 0

    drivetrain_gear_ratio = 18

    wheel_max_rpm: float = 200
    wheel_diameter_CM: float = 8.255

    # In order to get this, it is ticks for the specific gear ratio we're using divided by the circumeference of our wheel
    wheel_distance_CM_to_TICK_coefficient: float = (drivetrain_gear_ratio / 6 * 300) / (math.pi * wheel_diameter_CM) * 0.47

    # * PID controllers

    flywheel_motor_1_average_output = 0
    flywheel_motor_2_average_output = 0 

    autonomous_speed: float = 0.48
    
    # * MISC
    update_loop_delay = 0.00  # 10 ms

    # Used to keep track of time in auto and driver mode respectively, use it for nicely logging data, can be used during either modes for end game/pathfinding rules
    autonomous_timer = Timer()
    driver_controlled_timer = Timer()

    target_reached = False
    position_tolerance = 3 # tolerance to target position in cm
    orientation_tolerance = 8 # tolerance to target orientation in degrees
    
    # From 0,0 (which is the center of the field). Dimensions were got from page 89 on: https://content.vexrobotics.com/docs/2022-2023/vrc-spin-up/VRC-SpinUp-Game-Manual-2.2.pdf
    red_goal = GameObject(143, 143)
    blue_goal = GameObject(-143, -143)

    flywheel_speed_levels = [
        0,
        33,
        34,
        round(35.0000000),
        36,
        37,
        38,
        39,
        40,
        41,
        42,
        43,
        45,
        46,
        100,
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
        "x_gps" : 0,
        "y_gps" : 0,
        "x_enc" : 0,
        "y_enc" : 0,
        "theta" : 0,
        "theta_vel" : 0,
        "time" : 0,

        "autonomous_speed" : 0,

        # override velocities
        "override_velocity_x" : 0,
        "override_velocity_y" : 0,
        "override_velocity_theta" : 0,

        # Roller stuff
        "roller_state" : "none",
        "auto_roller" : False,
        "roller_speed" : 0,
        "roller_spin_for" : 0,
        "roller_and_intake_motor_1_done" : True,
        "roller_and_intake_motor_2_done" : True, 
        
        # Intake
        "auto_intake" : False,
        "disc_in_intake" : False,

        # Actuators
        "flywheel_speed" : 0,
        "intake_speed" : 0,
        "shoot_disc" : 0,

        # expansion
        "launch_expansion" : False,

        # Commands
        "using_gps" : False,
        "is_shooting": False,
        "slow_mode" : False,
        "drone_mode" : False,
        "autonomous" : False,

        "flywheel_torque" : 0,
        "disc_shot" : False,

        "flywheel_1_torque" : 0,
        "flywheel_2_torque" : 0,

        "is_shooting" : False,
    }

    intake_timer = Timer()

    target_state = {
        "x_pos" : None,
        "y_pos" : None,
        "theta" : None,
    }
    
    all_states = []
    delta_time = 0

    is_red_team = False

    save_states = False

    path = [
        # Initial state of the robot
        {
        "set_x" : 0,
        "set_y" : 0,
        "set_theta" : 0,
        "override_velocity_x" : None,
        "override_velocity_y" : None,
        "override_velocity_theta" : None,
        "drone_mode" : True,
        },
    ]

    total_updates = 0

    flywheel_recovery_timer = Timer()

    average_target_flywheel_output_1 = 0
    average_target_flywheel_output_2 = 0

    def __init__(self):

        # what our max velocity "should" be (can go higher or lower)
        self.max_velocity = ((self.wheel_max_rpm / 60) *
                             math.pi * 2 * self.wheel_diameter_CM / math.sqrt(2))
        # This number in the divisor means it will speedup/slow down in that many seeconds
        self.max_acceleration = 2 * self.max_velocity / 0.00001

        # Set origin of the gps
        gps.set_origin(0,0)

        # Thsee next few lines just make sure that the state system works, and saves and initial state that we return to when the autonomous mode ends
        self.initial_state = self.state.copy()
        self.set_target_state(self.state)
        self.previous_state = self.state
    
    # There are two init methods, this init initializes the class, the other init method we call to initlize the robot
    def init(self):
        '''
        Different than the Robot()__init__ dunder that gets called when the robot is made, this is a manual initialization that 
        starts the update loop and turns on the robot
        '''
        
        # Set heading based on gps
        if self.using_gps:

            # For some reason, the gps returns nan, just wait until it doesn't return nan
            while str(gps.heading()) == "nan":
                print("Waiting for gps", gps.heading())
                time.sleep(0.1)

            # Actual theta of the robot from the field is the gps heading minus the angle of the gps on the robot (90 deg in this instance)
            self.theta_offset = gps.heading() - self.gps_theta_on_robot # The + 90 is because the gps is 90 deg off of the robot

        self.set_target_state(self.state)
        self.previous_state = self.state
        Thread(self.update_loop)

        self.flywheel_recovery_timer.reset()

    def update_loop(self):
        '''
        Runs the self.update command every self.update_loop_delay seconds forever
        '''
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

        
        # If we want to save the states of the the robot (which will allow us to do cool things such as recording our matches and replaying our momvements, this must be turned to true)
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

        # Update all status indicators
        self.status_update()


    
    def position_update(self):
        '''
        Updates the position of the robot
        '''
        
        # Find out how much the robot needs to move in each direction
        delta_x = 0
        delta_y = 0
        delta_theta = 0

        # If the target state does not exist then we don't need to move in that direction
        if self.target_state["x_pos"] != None:
            delta_x = self.target_state["x_pos"] - self.x_pos
        
        if self.target_state["y_pos"] != None:
            delta_y = self.target_state["y_pos"] - self.y_pos

        if self.target_state["theta"] != None:
            delta_theta = self.target_state["theta"] - self.theta

        # Turn via the shortest path
        if delta_theta > 180:
            delta_theta -= 360
        elif delta_theta < -180:
            delta_theta += 360

        orientation_tolerance = 9
        position_tolerance = 15

        # Make sqrt the delta theta, so that the tolerance is not linear but a sqrt relationshup
        delta_theta = math.sqrt(abs(delta_theta)) * sign(delta_theta) 

        # The multiplying by 100 and dividing by self.position tolerance scales it so that at position tolerance the velocity is 100 percent, squaring the velocity makes it so that if we get closer than we go slower
        # target_x_vel = ((clamp(delta_x, position_tolerance, -position_tolerance) * 100 / position_tolerance) ** 2) / 100 * sign(delta_x) * (0.3 if self.running_autonomous else 1)
        # target_y_vel = ((clamp(delta_y, position_tolerance, -position_tolerance) * 100 / position_tolerance) ** 2) / 100 * sign(delta_y) * (0.1 if self.running_autonomous else 1)
        
        # target_x_vel = ((clamp(delta_x, position_tolerance, -position_tolerance) * 100 / position_tolerance) ** 2) / 100 * sign(delta_x) * (0.5 if self.running_autonomous else 1)
        # target_y_vel = ((clamp(delta_y, position_tolerance, -position_tolerance) * 100 / position_tolerance) ** 2) / 100 * sign(delta_y) * (0.5 if self.running_autonomous else 1)
        
        target_y_vel = clamp(delta_y, position_tolerance, -position_tolerance) * 100/position_tolerance * (self.autonomous_speed if self.running_autonomous else 1)
        target_x_vel = clamp(delta_x, position_tolerance, -position_tolerance) * 100/position_tolerance * (self.autonomous_speed if self.running_autonomous else 1)

        target_theta_vel = ((clamp(delta_theta, orientation_tolerance, -orientation_tolerance) * 100 / (orientation_tolerance - 3)) ** 2) / 100 * sign(delta_theta) * (1 if self.running_autonomous else 1)
        
        target_theta_vel = clamp(target_theta_vel, 100, -100)

        if self.target_state["override_velocity_x"] != None:
            target_x_vel = self.target_state["override_velocity_x"]
        if self.target_state["override_velocity_y"] != None:
            target_y_vel = self.target_state["override_velocity_y"]
        if self.target_state["override_velocity_theta"] != None:
            target_theta_vel = self.target_state["override_velocity_theta"]

        # Cool driving toggle (basically you rotate the target direction vector based on)
        # the robots heading (https://stackoverflow.com/questions/14607640/rotating-a-vector-in-3d-space)
        if self.drone_mode:                                                         # Add self.theta add dt times angular velocity to get a better approximation of actual theta at timestamp
            target_x_vel, target_y_vel = rotate_vector_2d(target_x_vel, target_y_vel, (self.theta + self.delta_time / 2 * self.theta_vel) * DEG_TO_RAD)

        if self.slow_mode:
            target_x_vel = target_x_vel / 4
            target_y_vel = target_y_vel / 4
            target_theta_vel = target_theta_vel / 4

        max_speed_percentage = 100
        if max_speed_percentage - abs(target_theta_vel) < abs(target_x_vel) + abs(target_y_vel):
            # print("previous X, y, theta", target_x_vel, target_y_vel, target_theta_vel)
            target_x_vel = (max_speed_percentage - abs(target_theta_vel)) * target_x_vel / (abs(target_x_vel) + abs(target_y_vel))
            target_y_vel = (max_speed_percentage - abs(target_theta_vel)) * target_y_vel / (abs(target_x_vel) + abs(target_y_vel))
            # print("after X, y, theta", target_x_vel, target_y_vel, target_theta_vel)

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
            max_acceleration_left_motor_a = self.max_acceleration * 0.5

        if abs(right_motor_a.velocity(PERCENT)) < slow_down_speed_threshold:
            max_acceleration_right_motor_a = self.max_acceleration * 0.5

        if abs(left_motor_b.velocity(PERCENT)) < slow_down_speed_threshold:
            max_acceleration_left_motor_b = self.max_acceleration * 0.5

        if abs(right_motor_b.velocity(PERCENT)) < slow_down_speed_threshold:
            max_acceleration_right_motor_b = self.max_acceleration * 0.5
        
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
        left_motor_a.spin(FORWARD, left_motor_a_target_velocity, PERCENT)
        right_motor_a.spin(FORWARD, right_motor_a_target_velocity, PERCENT)
        left_motor_b.spin(FORWARD, left_motor_b_target_velocity, PERCENT)
        right_motor_b.spin(FORWARD, right_motor_b_target_velocity, PERCENT)

        # left_motor_a.spin(FORWARD, left_motor_a_target_velocity / 100 * 10.9, VOLT)
        # right_motor_a.spin(FORWARD, right_motor_a_target_velocity / 100 * 10.9, VOLT)
        # left_motor_b.spin(FORWARD, left_motor_b_target_velocity / 100 * 10.9, VOLT)
        # right_motor_b.spin(FORWARD, right_motor_b_target_velocity / 100 * 10.9, VOLT)


    def expansion_update(self):
        '''
        Launch the expansion
        '''
        if self.target_state["launch_expansion"]:
            expansion.open()
        else:
            expansion.close()
        
    
    def estimate_state(self):
        '''
        Estimates the state of the robot
        '''

        # Get the time the program was running for 
        self.state["time"] = getattr(time, "ticks_ms")() / 1000
        
        # Find our angular velocity
        self.theta_vel = inertial.gyro_rate(ZAXIS)
        
        # Find the total amount the robot has turned by integrating the velocity, then dividing it by a number
        # I found that makes the gyro work better (experimentall I found that when the gyro reported that it spun 1 time, it actually overshot by about 3 degrees)
        self.total_theta += self.theta_vel * self.delta_time / 2 / 0.999426111  

        self.theta = self.total_theta - (self.total_theta // 360 * 360)
        
        # Use encoders to get our x and y positions so that we can take the derivative and get our velocity
        self.x_enc, self.y_enc = self.get_position_from_encoders()

        # Find our delta encoders amount
        delta_x_from_encoders = self.x_enc - self.previous_state["x_enc"]
        delta_y_from_encoders = self.y_enc - self.previous_state["y_enc"]

        # Rotate the numbers based on our orientation relative to the robot
        delta_x_from_encoders, delta_y_from_encoders = rotate_vector_2d(delta_x_from_encoders, delta_y_from_encoders, -self.theta * DEG_TO_RAD)

        # Get the velocity of the robot in deg/s for 4 wheels
        self.x_vel = delta_x_from_encoders / self.delta_time
        self.y_vel = delta_y_from_encoders / self.delta_time

        # Velocity of robot from gps perspective
        x_from_gps = 0
        y_from_gps = 0
        theta_field = 0
        alpha = 0

        delta_x_from_gps = 0
        delta_y_from_gps = 0

        # If the gps is being used and not being obstructed
        if self.using_gps and gps.quality() >= 100: 
            
            # We can choose to use the gps or the gyro for our heading
            # self.total_theta = gps.heading()

            # Update alpha to value that uses gps
            alpha = 1
            x_from_gps = gps.x_position(DistanceUnits.CM)
            y_from_gps = gps.y_position(DistanceUnits.CM)
            theta_field = gps.rotation() - self.gps_theta_on_robot

            # If we want to figure out our velocity from the gps
            delta_x_from_gps = gps.x_position(DistanceUnits.CM) - self.previous_x_from_gps 
            delta_y_from_gps = gps.y_position(DistanceUnits.CM) - self.previous_y_from_gps 

            self.previous_x_from_gps = gps.x_position(DistanceUnits.CM)
            self.previous_y_from_gps = gps.y_position(DistanceUnits.CM)

            # print("Position from gps is", x_from_gps, y_from_gps, theta_from_gps)

            # self.field_theta = self.field_theta * 0.9 + 0.1 * gps.heading()
            # print("We have moved x_enc, y_enc", delta_x_from_encoders, delta_y_from_encoders, "from gps", (x_from_gps - self.x_pos), (y_from_gps - self.y_pos))


        alpha = 0
        # If we have not moved (from the encoders point of view), and the gps is not changing that much, then use the rolling average from the gps
        # If gps is enabled then the low and high pass filter will make the x and y position more stable, if gps is not enabled then the formula won't use gps data (alpha would equal 0)
        self.x_pos += delta_x_from_encoders * (1-alpha) + (delta_x_from_gps-self.x_pos) * alpha 
        self.y_pos += delta_y_from_encoders * (1-alpha) + (delta_y_from_gps-self.y_pos) * alpha
        
        gps_alpha = 0.2
        self.x_from_gps = self.x_from_gps * (1-gps_alpha) + x_from_gps * gps_alpha
        self.y_from_gps = y_from_gps
        self.theta_field = theta_field

        self.total_x_from_encoders += delta_x_from_encoders
        self.total_y_from_encoders += delta_y_from_encoders

        # If we are close to our target position then say that we reached our target state (useful for autonomous mode)
        # if ((abs(self.state["x_pos"] - self.target_state["x_pos"]) < self.position_tolerance and abs(self.state["y_pos"] - self.target_state["y_pos"]) < self.position_tolerance) \
        #     or (self.state["override_velocity_y"] != None or self.state["override_velocity_x"] != None)):
        
        target_state_position_tolerance = 0.5
        target_state_theta_tolerance = 0.7

        if self.shoot_disc <= 0 and self["roller_and_intake_motor_1_done"] and self["roller_and_intake_motor_2_done"] and abs(self["x_pos"] - self.target_state["x_pos"]) < target_state_position_tolerance and abs(self["y_pos"] - self.target_state["y_pos"]) < target_state_position_tolerance and abs(self["theta"] - self.target_state["theta"]) < target_state_theta_tolerance:
            self.target_reached = True
        

        # Get the torque on the flywheel
        self["flywheel_1_torque"] = flywheel_motor_1.torque()
        self["flywheel_2_torque"] = flywheel_motor_2.torque()
        
        # Figure out if the disc has been shot by seeing if there is a large change in the flywheel torque
        if (((self["flywheel_1_torque"] - self.previous_state["flywheel_1_torque"]) + (self["flywheel_2_torque"] - self.previous_state["flywheel_2_torque"])) / 2 > 0.009) and self.is_shooting and indexer_limit_switch.value() == 1:
            if self.flywheel_recovery_timer.value() > 0.45:
                self.flywheel_recovery_timer.reset()
            self.state["disc_shot"] = True
        else:
            self.state["disc_shot"] = False


        # If the roller was sent to false BEFORE
        self["roller_and_intake_motor_1_done"] = roller_and_intake_motor_1.is_done()
        self["roller_and_intake_motor_2_done"] = roller_and_intake_motor_2.is_done()
        
        if self["roller_and_intake_motor_1_done"] and self["roller_and_intake_motor_2_done"] and not self.previous_state["roller_and_intake_motor_1_done"] and not self.previous_state["roller_and_intake_motor_2_done"]:
            print("values", roller_and_intake_motor_1.is_done(), roller_and_intake_motor_2.is_done())
            self["roller_spin_for"] = 0

        

        
        # if self.previous_state["roller_is_done"] == True and self["roller_is_done"] == False:
        #     self["roller_is_done"] = (roller_and_intake_motor_1.is_done() and roller_and_intake_motor_2.is_done())

        # if not self.previous_state["roller_is_done"] and self["roller_is_done"]:
        #     # setting the roller_spin_for to zero
        #     print("setting toller-spin_for to zero")
        #     self["roller_spin_for"] = 0

    
    def intake_update(self):
        # If we have auto_intake enbaled then use the camera and try to look for the disc signatures
        if self["auto_intake"]:
            objs = [vision.take_snapshot(z) for z in range(8)]
            objs = [obj for obj in objs if obj]
            if len(objs) > 1:
                self.intake_speed = 100

                # Assume that there's a disc in the intake 
                self["disc_in_intake"] = True
                self.intake_timer.reset()
            elif self.intake_timer.time(MSEC) > 1500: # If it has been more than 1 second since we last saw a disc, stop intake
                self.intake_speed = 0
                self["disc_in_intake"] = False
        
        if self["roller_spin_for"] == 0 and self["roller_and_intake_motor_1_done"] and self["roller_and_intake_motor_2_done"]:
            roller_and_intake_motor_1.spin(REVERSE, self.intake_speed, VelocityUnits.PERCENT)
            roller_and_intake_motor_2.spin(FORWARD, self.intake_speed, VelocityUnits.PERCENT)
        
        # If there is too much torque in the system, then pulse the motors
        # max_torque_before_stopping = 2.1
        # if roller_and_intake_motor_1.torque() > max_torque_before_stopping and roller_and_intake_motor_2.torque() > max_torque_before_stopping:
        #     print("WE ARE PULSING RIGHT NOW")
        #     roller_and_intake_motor_1.spin(REVERSE, -self.intake_speed, VelocityUnits.PERCENT)
        #     roller_and_intake_motor_2.spin(FORWARD, -self.intake_speed, VelocityUnits.PERCENT)
            
        
    def roller_update(self):
        if self["auto_roller"]:
            # If the topical sensor does not detect an object then turn off the roller
            if not roller_optical.is_near_object():
                # print("object not detected", roller_optical.color())
                self["roller_state"] = "none"
                self["roller_speed"] = 0
            else:
                # If the sensor detects and object and if that object is red, or blue then run the roller
                if roller_optical.color() == Color.RED:
                    self["roller_state"] = "red"
                elif roller_optical.color() == Color.BLUE:
                    self["roller_state"] = "blue"
                else:
                    self["roller_state"] = "none"

                # If we detect that the roller is not our team's color, spin it
                self["roller_speed"] = 0
                if (self["roller_state"] != "none" and not self.is_red_team):
                    self["roller_speed"] = 80

                # This is a way to indicate to the drivers that we are done spinning the roller
                if (self["roller_state"] == "red" and self.is_red_team) or (self["roller_state"] == "blue" and not self.is_red_team):
                    controller_1.rumble("-")
                    if self.previous_state["roller_state"] != self["roller_state"]:
                        self["roller_speed"] = -50
        
            # If the roller sees the red or blue roller then spin the roller/intake system
            if self["roller_state"] != "none":
                roller_and_intake_motor_1.spin(REVERSE, self["roller_speed"], VelocityUnits.PERCENT)
                roller_and_intake_motor_2.spin(FORWARD, self["roller_speed"], VelocityUnits.PERCENT)

        if self["roller_spin_for"] != 0 and self["roller_and_intake_motor_1_done"] and self["roller_and_intake_motor_2_done"]:
            
            roller_and_intake_motor_1.set_velocity(-100)
            roller_and_intake_motor_2.set_velocity(100)
            
            
            # When we rotate our roller, for every 285 degrees it rotates for, the game field roller will rotate 360 degrees (3'' roller wheels compared to 2.375'' roller)
            roller_and_intake_motor_1.spin_for(REVERSE, self["roller_spin_for"] * 285.0, DEGREES, False)
            roller_and_intake_motor_2.spin_for(FORWARD, self["roller_spin_for"] * 285.0, DEGREES, False)

            roller_and_intake_motor_1.set_velocity(-100)
            roller_and_intake_motor_2.set_velocity(100)

            self["roller_and_intake_motor_1_done"] = False
            self["roller_and_intake_motor_2_done"] = False

            self["roller_spin_for"] = 0
            # self.previous_state["roller_and_intake_motor_1_done"] = True
            # self.previous_state["roller_and_intake_motor_2_done"] = True

            # print("done status", roller_and_intake_motor_1.is_done(), roller_and_intake_motor_2.is_done())


##### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ### MISC/UTIL ###
    def set_target_state(self, _state):
        '''
        Updates the state of the robot
        '''

        # This makes it so that in case the new target state doesn't have a value, instead of just deleting that value, we don't change it
        for key in _state.keys():
            if key in self.state:
                self.target_state[key] = _state[key]
                
                if key == "theta" and self.target_state[key] != None:
                    self.target_state[key] = _state[key] % 360        
       
                
        # This fucntion updates any constants that are not targets
        self.update_constants(_state)

        # Make is so that we haven't reached our target state because we just set our target state
        self.target_reached = False
    
    def update_constants(self, _state):
        # This fucntion updates any constants that are not targets
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
        if "shoot_disc" in _state:
            self.shoot_disc = _state["shoot_disc"]
        if "roller_spin_for" in _state:
            self["roller_spin_for"] = _state["roller_spin_for"]
        if "autonomous_speed" in _state:
            self["autonomous_speed"] = _state["autonomous_speed"]


    def __setitem__(self, key, value):
        '''
        Whenever you try to set an item from the robot with self["key"] = SOMETHING or robot["key"] = SOMETHING then access the robot's state
        '''
        self.state[key] = value
    
    def __getitem__(self, key):
        '''
        Whenever you try to get an item from the robot with self["key"] or robot["key"] then access the robot's state
        '''
        return self.state[key]

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
        Function returns true if the values of the encoders for the motors have changed AT ALL
        '''
        return self.x_vel == 0 and self.y_vel == 0

    def set_team(self, _color):
        '''
        Sets the team for the robot so that we can move the rollers to our team color, or shoot discs just into our goal, etc.
        '''
        _color.lower()
        if _color == "red":
            self.is_red_team = True
            self.goal = self.red_goal
            return
        elif _color == "blue":
            self.goal = self.blue_goal
            self.is_red_team = False
            return
        raise Exception("broo you dodn't set a proper team color")

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

        # Thread(self.,(roller_angle,))

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

    def run_autonomous(self):
        '''
        Given an autonomous procedure, this function will run the procedure
        procedure has very important requirements optional params:
            set_x - sets the x position of the robot
            set_y - sets the y position of the robot
            set_theta - sets the theta of the robot
            other params - any other param that is passed that is a key in the robot's state sets the target state of the robot to that value

        '''
        if self.running_autonomous:
            # In case we accidentally run 2 autos at the same time lmmao
            return
        # Update loop
        self.running_autonomous = True
        self.update()
        self.autonomous_timer.reset()
        # timeout_timer = Timer()
        # timeout_time = 0
        self["override_velocity_x"] = None
        self["override_velocity_y"] = None
        self["override_velocity_theta"] = None
        

        for step in self.autonomous_procedure:
            target_state = {}
            self["override_velocity_x"] = None
            self["override_velocity_y"] = None
            self["override_velocity_theta"] = None

            # if "timeout" in step.keys():
            #     timeout_timer.reset()
            #     timeout_time = step["timeout"]

            if "set_x" in step:
                self.x_pos = step["set_x"]
            if "set_y" in step:
                self.y_pos = step["set_y"]
            if "set_theta" in step:
                self.total_theta = step["set_theta"]
            
            for key in step.keys():
                if key in self.state.keys():
                    print("setting key of", key, "to", step[key])
                    target_state[key] = step[key]

            if "message" in step.keys():
                self.print(step["message"])

            if "funcs" in step.keys():
                for func in step["funcs"]:
                    func()
            print("target state is", target_state)
            self.set_target_state(target_state)

            if "wait" in step.keys():
                print("Waiting for", step["wait"], "seconds")
                wait(step["wait"], SECONDS)

            # While we haven't reached the target state then just wait
            while not self.target_reached:
                # print("WE ARE WAITING...", self.target_state["x_pos"], self.x_pos, self.target_state["y_pos"], self.y_pos, self.target_state["theta"], self.total_theta, self["override_velocity_x"])
                wait(0.05, SECONDS)
        print("DONE WITH AUTONOMOUS")

        # After the autonomous mode is over then set the target state to the robot's initial state (so we don't move and turn everything off)
        self.set_target_state(self.initial_state)
        self.running_autonomous = False

    def set_autonomous_procedure(self, procedure):
        self.autonomous_procedure = procedure

##### PROPERTIES ##### PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES #####  PROPERTIES ##### 

    # if you are reading this code and are not me (Matt Handzel) and don't know what this is, then look up python getters and setters
    @property
    def position(self):
        return (self.x_pos, self.y_pos)

    @position.setter
    def position(self, _x, _y):
        self.x_pos = _x
        self.y_pos = _y

    @property
    def is_shooting(self):
        return self.state["is_shooting"]
    
    @is_shooting.setter
    def is_shooting(self, _is_shooting):
        self.state["is_shooting"] = _is_shooting
    
    @property
    def disc_shot(self):
        return self.state["disc_shot"]
    
    @disc_shot.setter
    def disc_shot(self, _disc_shot):
        self.state["disc_shot"] = _disc_shot
    
    @property
    def shoot_disc(self):
        return self.state["shoot_disc"]
    
    @shoot_disc.setter
    def shoot_disc(self, _shoot_disc):
        self.state["shoot_disc"] = _shoot_disc

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

    @property
    def autonomous_speed(self):
        return self.state["autonomous_speed"]
    
    @autonomous_speed.setter
    def autonomous_speed(self, _autonomous_speed):
        self.state["autonomous_speed"] = _autonomous_speed
    
    
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

##### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ### FLYWHEEL ###
    def flywheel_update(self):
        self.flywheel_pid(self.flywheel_speed) 
        if self["shoot_disc"] > 0 and not self.is_shooting:
            Thread(self.index_disc)       
    
    def flywheel_pid(self, speed):
        # TODO: Smoothing the output: The output of the PID controller may be noisy, which can cause instability in the system. To smooth the output, you can try using a low-pass filter to remove high-frequency noise.
        # TODO: Adding an anti-windup mechanism: An anti-windup mechanism can prevent the integral term from growing too large, which can cause the controller to lose control. This can be especially important if the system has a large dead-zone or if the output is saturated for long periods of time.

        # MAX_FLYWHEEL_SPEED = 100 / 60
        MAX_VOLTAGE = 10.9 # I don't think this is the true maximum voltage btw
        
        # This so for the derivative term so that it isn't working with the chaotic value of flywheel speed that fluctuates rapidly, but the averaged flywheel speed
        speed_alpha = 0.2 # was 0.25
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

        kP = 0.4 # before aws 0.03
        kI = 0.0005 / 0.022 # before was 0.0004 (this worked pretty well)
        kD = 0.03 * 0.022

        # Boolean to determine if the flywheel is recovering from a disc being shot
        flywheel_recovery_mode = (self.flywheel_recovery_timer.value()) < (0.3 * (sqrt(self.flywheel_speed / 35))) and self.is_shooting

        # If we are shooting then do NOT increment the integral term and greatly increase the proportional term so that we can increase our speed much faster
        if flywheel_recovery_mode:
            # self.flywheel_recovery_timer.reset()
            kI = 0
            kD = 0
            kP = 2.4 

        proportional_term_flywheel_1 = kP * (self.flywheel_speed - flywheel_motor_1.velocity(PERCENT))
        derivative_term_flywheel_1 = kD * (self.flywheel_1_avg_speed - self.previous_flywheel_1_avg_speed) / self.delta_time
        # error_helper_term_flywheel_1 = 0.01 * ((self.flywheel_speed - self.flywheel_1_avg_speed) - self.previous_flywheel_1_error) 

        derivative_term_flywheel_1 = max(derivative_term_flywheel_1, -0.5)
        derivative_term_flywheel_1 = min(derivative_term_flywheel_1, 0.5)

        # derivative_term_flywheel_1 = 0

        proportional_term_flywheel_2 = kP * (self.flywheel_speed - flywheel_motor_2.velocity(PERCENT))
        derivative_term_flywheel_2 = kD * (self.flywheel_2_avg_speed - self.previous_flywheel_2_avg_speed)  / self.delta_time
        # error_helper_term_flywheel_2 = 0.01 * ((self.flywheel_speed - self.flywheel_1_avg_speed) - self.previous_flywheel_1_error) 

        derivative_term_flywheel_2 = max(derivative_term_flywheel_2, -0.5)
        derivative_term_flywheel_2 = min(derivative_term_flywheel_2, 0.5)
        
        self.integral_term_flywheel_1 += self.delta_time * kI * (self.flywheel_speed - self.flywheel_1_avg_speed) + (self.flywheel_speed - self.previous_flywheel_speed) * 0.108
        self.integral_term_flywheel_2 += self.delta_time * kI * (self.flywheel_speed - self.flywheel_2_avg_speed) + (self.flywheel_speed - self.previous_flywheel_speed) * 0.11 

        # Clamp the voltage so we don't send more than the maximum I stated 
        self.integral_term_flywheel_1 = clamp(self.integral_term_flywheel_1, MAX_VOLTAGE, -MAX_VOLTAGE) 
        self.integral_term_flywheel_2 = clamp(self.integral_term_flywheel_2, MAX_VOLTAGE, -MAX_VOLTAGE) 


        if flywheel_recovery_mode:
            target_flywheel_output_1 = self.integral_term_flywheel_1 + proportional_term_flywheel_1 - derivative_term_flywheel_1
            target_flywheel_output_2 = self.integral_term_flywheel_2 + proportional_term_flywheel_2 - derivative_term_flywheel_2
        else:
            alpha = 0.1
            self.average_target_flywheel_output_1 = self.average_target_flywheel_output_1 * (1-alpha) + (self.integral_term_flywheel_1 + proportional_term_flywheel_1 - derivative_term_flywheel_1) * (alpha)  
            self.average_target_flywheel_output_2 = self.average_target_flywheel_output_2 * (1-alpha) + (self.integral_term_flywheel_2 + proportional_term_flywheel_2 - derivative_term_flywheel_2) * (alpha)  
        
            target_flywheel_output_1 = self.average_target_flywheel_output_1
            target_flywheel_output_2 = self.average_target_flywheel_output_2

        flywheel_motor_1.spin(FORWARD, clamp(target_flywheel_output_1, MAX_VOLTAGE, -MAX_VOLTAGE), VOLT)
        flywheel_motor_2.spin(FORWARD, clamp(target_flywheel_output_2, MAX_VOLTAGE, -MAX_VOLTAGE), VOLT)
        # print(brain.timer.value(), flywheel_motor_1.velocity(PERCENT), flywheel_motor_2.velocity(PERCENT), target_flywheel_output_1, target_flywheel_output_2, int(self.is_shooting) * 35)

        self.flywheel_motor_1_error = (self.flywheel_1_avg_speed - self.previous_flywheel_1_avg_speed)
        self.flywheel_motor_2_error = (self.flywheel_2_avg_speed - self.previous_flywheel_2_avg_speed)

        # Compute derivative of average flywheel speed
        self.previous_flywheel_1_avg_speed = self.flywheel_1_avg_speed
        self.previous_flywheel_2_avg_speed = self.flywheel_2_avg_speed

        self.previous_flywheel_speed = float(self.flywheel_speed)

        self.derivative_term_flywheel_1 = derivative_term_flywheel_1
        self.derivative_term_flywheel_2 = derivative_term_flywheel_2

        self.proportional_term_flywheel_1 = proportional_term_flywheel_1
        self.proportional_term_flywheel_2 = proportional_term_flywheel_2

    
    def index_disc(self):
        '''
        This function will shoot a disc from the flywheel
        '''
        if self.is_shooting:
            return

        self.is_shooting = True

        indexer.open()
        
        t = Timer()
        t.reset()
        
        while not self["disc_shot"] and t.time() < 1250:
            wait(0.01, SECONDS)

        indexer.close()
        
        # wait for the limit switch to be swticehd
        while not (indexer_limit_switch.value() == 0):
            wait(0.01, SECONDS)

        self.shoot_disc -= 1
        self.is_shooting = False

    
    def status_update(self):
        
        # Indicate to the drivers that the flywheel is ready to be shot
        if self.flywheel_speed > 0 and abs(flywheel_motor_1.velocity(PERCENT) - self.flywheel_speed) < 1.5 and abs(flywheel_motor_2.velocity(PERCENT) - self.flywheel_speed) < 1.5 and self.flywheel_recovery_timer.value() > 0.3:
            flywheel_status_light.on()
        else:
            flywheel_status_light.off()

    def compute_speed_of_flywheel(self):
        '''
        Because we know our distance to the goal, we can compute the speed that the flywheel needs to launch in order to make the disc in
        '''
        # Equation to find the speed that the flywheel needs:

        self.update()

        delta_x = self.goal.x_pos - self.x_pos - self.flywheel_offset_x
        delta_y = self.goal.y_pos - self.y_pos - self.flywheel_offset_y

        point = (self.goal.x_pos * 2.54 / 100, (self.goal.y_pos - self.flywheel_height_from_ground_IN) * 2.54 / 100)
        v_i, hit_time = getViForPathToHitPoint(self.flywheel_angle, (point), sizeOfPoint = 0.001)
        print("SETTING THE FLYWHEEL TO x M/S")

        # Compute the linear speed that the disc needs to be launched to
        # math.cos(theta_flywheel * DEG_TO_RAD)

        # distance_to_goal * theta_flywheel
        pass    

    def return_state_of_auto(self):
        # Return all of the states that we're going to need during autonomous to set
        state_names = ["x_pos", "y_pos", "theta", "flywheel_speed", "intake_speed", "launch_expansion"]
        state = {state : self[state] for state in state_names}
        state["message"] = "\"This is state #" + str(len(r.path)) + "!\""

        state["x_pos"] = round(state["x_pos"], 1)
        state["y_pos"] = round(state["y_pos"], 1)
        state["theta"] = round(state["theta"], 1)

        return state
    
    def save_state(self):
        self.path.append(self.return_state_of_auto())

class Test:
    '''
    Tests that the robot doctor program uses
    '''
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


class RobotDoctor:
    '''
    Class that runs tests on the robot to make sure it is working as intended
    '''
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

######## COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ###
def autonomous():
    r.run_autonomous()
    r.stop_moving()

def driver_control():
    timer = Timer()
    timer.reset()

    reset_theta_timer = Timer()
    reset_theta_timer.reset()

    controller_output_timer = Timer()
    controller_output_timer.reset()

    r.driver_controlled_timer.reset()

    r.autonomous_timer.reset()
    
    # A dictionary that stores the previous button states so that we can reference previous states without having to make a new variable every time 
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

    previous_controller_states_2 = {
            "buttonL1" : controller_2.buttonL1.pressing(),
            "buttonR1" : controller_2.buttonR1.pressing(),
            "buttonL2" : controller_2.buttonL2.pressing(),
            "buttonR2" : controller_2.buttonR2.pressing(),
            "buttonUp" : controller_2.buttonUp.pressing(),
            "buttonDown" : controller_2.buttonDown.pressing(),
            "buttonLeft" : controller_2.buttonLeft.pressing(),
            "buttonRight" : controller_2.buttonRight.pressing(),
            "buttonX" : controller_2.buttonX.pressing(),
            "buttonY" : controller_2.buttonY.pressing(),
            "buttonA" : controller_2.buttonA.pressing(),
            "buttonB" : controller_2.buttonB.pressing(),
        }

    expansion_timer = Timer()
    expansion_timer.reset()

    # Timer that allows us to keep track for how long we have been pressing the aimbot button for to make the robot move more as time goes on
    aimbot_timer = Timer() 

    gui.set_page(0)
    while True:

        if r.running_autonomous:
            continue

        auto_orientate_dictionary = {
          0 : controller_1.buttonX.pressing(),
          -90 : controller_1.buttonY.pressing(),
          90 : controller_1.buttonA.pressing(),
          180 : controller_1.buttonB.pressing(),
        }

        auto_orientate_total_button_pressed_count = int(controller_1.buttonX.pressing()) + int(controller_1.buttonY.pressing()) + int(controller_1.buttonA.pressing()) + int(controller_2.buttonB.pressing()) 
        auto_orientate_target_theta = None
        
        if auto_orientate_total_button_pressed_count > 0:
            auto_orientate_target_theta = 0
            for key in auto_orientate_dictionary:
                if auto_orientate_dictionary[key]:
                    auto_orientate_target_theta += key
            
            auto_orientate_target_theta /= auto_orientate_total_button_pressed_count

        # Render and update the gui before everything else
        # If the joystick hasn't been pressed 
        new_theta = ((controller_1.axis1.position())) ** 2 * sign(controller_1.axis1.position()) / 100 * (0.7 if recording_autonomous else 1)

        # if abs(new_theta) < 2:
        #     new_theta = None

        new_x = ((controller_1.axis4.position())) ** 2 * sign(controller_1.axis4.position()) / 100 * (0.25 if recording_autonomous else 1)
        new_y = ((controller_1.axis3.position())) ** 2 * sign(controller_1.axis3.position()) / 100 * (0.25 if recording_autonomous else 1)
        
        new_intake = clamp(controller_2.buttonL1.pressing() * 100 - controller_2.buttonR1.pressing() * 100 + controller_1.buttonLeft.pressing() * 100, 100, -100)

        if controller_2.buttonLeft.pressing():
            new_intake = None
            r.set_target_state({
                "roller_spin_for" : 0.5,
                }
            )

        if controller_2.buttonRight.pressing():
            new_intake = None
            r.set_target_state({
                "roller_spin_for" : 0.25,
                }
            )

        # ### AUTO-ORIENTATE FEATURE
        # if controller_1.buttonA.pressing():
        #     new_theta = None
        #     r.set_target_state({
        #         "theta" : 90,
        #     })
        
        # if controller_1.buttonY.pressing():
        #     new_theta = None
        #     r.set_target_state({
        #         "theta" : -90,
        #     })

        # # If only the x button is being pressed then rotate the robot to 0 deg, 
        # # if the x and a button is pressed, rotate to 45, if a and y pressed
        # # rotate to -45 deg
        # if controller_1.buttonX.pressing():
        #     new_theta = None
        #     if controller_1.buttonA.pressing():
        #         r.set_target_state({
        #             "theta" : 45,
        #         })
        #     elif controller_1.buttonY.pressing():
        #         r.set_target_state({
        #             "theta" : -45,
        #         })
        #     else:
        #         r.set_target_state({
        #             "theta" : 0,
        #         })

        if controller_2.buttonA.pressing() and not previous_controller_states_2["buttonA"]:
            new_intake = None
            r.set_target_state(
                {
                    "roller_spin_for" : 0.5,
                }
            )
        
        if controller_1.buttonRight.pressing():
            new_theta = None
            reset_robot_theta()

        # # If only the b button is being pressed then rotate the robot to 180 deg,
        # # if the b and a button is pressed rotate to 45, if b and y is presed
        # # rotate to -135 deg
        # if controller_1.buttonB.pressing():
        #     new_theta = None
        #     if controller_1.buttonA.pressing():
        #         r.set_target_state({
        #             "theta" : 135,
        #         })
        #     elif controller_1.buttonY.pressing():
        #         r.set_target_state({
        #             "theta" : -135,
        #         })
        #     else:
        #         r.set_target_state({
        #             "theta" : 180,
        #         })
        
        if controller_1.buttonDown.pressing() and r.using_gps and gps.quality() >= 100:
            new_theta = None
            
            # If the robot is closer to one goal or the other 
            goal = r.red_goal if r.x_from_gps + r.y_from_gps > 0 else r.blue_goal

            r.set_target_state(
                {
                    # In order to get the angle to an object take the atan2 of the d_x, d_y, then subtract the theta offset to convert it back to the robots local coordinate frame
                    "theta" : get_angle_to_object((r.x_from_gps, r.y_from_gps), (goal.x_pos, goal.y_pos)) - r.theta_offset,
                }
            )

        # If the driver is pressing the left button, then turn on aimbot
        if controller_1.buttonLeft.pressing():
            if not previous_controller_states["buttonLeft"]:
                aimbot_timer.reset()

            # Aimbot for robot
            disc = vision.take_snapshot(5)
            
            # Vision sensor objects
            blue_goal = vision.take_snapshot(3)
            
            # Red goal goes not exist yet
            red_goal = vision.take_snapshot(4)

            # print(blue_goal, red_goal)

            if disc:
                goal = disc
                #! THIS MEANS WE WILL SHOOT AT ANY TEAM'S COLOR
                
                # Size of vision sensor screen is 212 vertical and 316 horizontal
                kP_for_aimbot = 20
                scale_time_value = 1/5

                # scale the values from [-1,1] and multiply by 30 as the kP, and then multiply by the sqrt of how long we've been holding the button for (to simulate a kI)
                delta_theta = kP_for_aimbot * ((float(goal[0].centerX) - 316/2) / (316/2)) * sqrt(1 + scale_time_value * aimbot_timer.value())
                print("delta theta", delta_theta)
                new_theta += delta_theta
            

        # if controller_2.buttonA.pressing():
        #     new_theta = None
        #     r.set_target_state({
        #         "theta" : 90,
        #     })
        
        # if controller_2.buttonY.pressing():
        #     new_theta = None
        #     r.set_target_state({
        #         "theta" : -90,
        #     })

        # If only the x button is being pressed then rotate the robot to 0 deg, 
        # if the x and a button is pressed, rotate to 45, if a and y pressed
        # rotate to -45 deg
        # if controller_2.buttonX.pressing():
        #     new_theta = None
        #     if controller_2.buttonA.pressing():
        #         r.set_target_state({
        #             "theta" : 45,
        #         })
        #     elif controller_2.buttonY.pressing():
        #         r.set_target_state({
        #             "theta" : -45,
        #         })
        #     else:
        #         r.set_target_state({
        #             "theta" : 0,
        #         })
        
        if controller_2.buttonDown.pressing():
            reset_robot_theta()

        # If only the b button is being pressed then rotate the robot to 180 deg,
        # if the b and a button is pressed rotate to 45, if b and y is presed
        # rotate to -135 deg
        # if controller_2.buttonB.pressing():
        #     new_theta = None
        #     if controller_2.buttonA.pressing():
        #         r.set_target_state({
        #             "theta" : 135,
        #         })
        #     elif controller_2.buttonY.pressing():
        #         r.set_target_state({
        #             "theta" : -135,
        #         })
        #     else:
        #         r.set_target_state({
        #             "theta" : 180,
        #         })

        if recording_autonomous:
            new_theta = None

        # Update the target state of the robot so that in its update() function it does everything there
        r.set_target_state(
            {
                "override_velocity_x" : new_x,
                "override_velocity_y" : new_y,
                "override_velocity_theta" : new_theta,
                "slow_mode" : controller_1.buttonL1.pressing(),
                "intake_speed" : new_intake,
            }
        )

        if controller_1.buttonL1.pressing() and not previous_controller_states["buttonL1"]:
            r.path.append(r.return_state_of_auto())

        # Timer to print things out to the terminal every x seconds
        if (timer.value() > 0.02):
            Thread(output_data)
            timer.reset()
        
        # Display data to the controllers
        if controller_output_timer.value() > 0.2:
            controller_1.screen.clear_row(3)
            controller_1.screen.print(f("FLY:", r.flywheel_speed, "ang:", round(r.theta, 1), len(r.path)))
            controller_2.screen.clear_row(3)
            controller_2.screen.print(f("FLY:", r.flywheel_speed, "ang:", round(r.theta)))
            controller_output_timer.reset()


        if controller_2.buttonUp.pressing() and not previous_controller_states_2["buttonUp"] and expansion_timer.value() < (60 + 45 - 11):
            r.set_target_state({
                "launch_expansion" : True

            })
        elif controller_2.buttonDown.pressing():
            r.set_target_state({
                "launch_expansion" : False
            })
            expansion.close()
        
        if controller_2.buttonUp.pressing():
            r.set_target_state({
                "launch_expansion" : True
            })
        elif controller_2.buttonDown.pressing():
            r.set_target_state({
                "launch_expansion" : False
            })
            expansion.close()

        # Shoot a disc if either of these are pressing
        if controller_1.buttonR1.pressing():
            r.set_target_state({
                "shoot_disc": 1,
            })

        # When the buttons are pressed to change the level of the flywheel speed, r2 decreases level, l2 increases level
        if controller_1.buttonR2.pressing() and not previous_controller_states["buttonR2"]:
            temp_copy = r.flywheel_speed_levels.copy()
            if r.flywheel_speed not in temp_copy:
                temp_copy.append(r.flywheel_speed)
            temp_copy.sort()
            new_flywheel_speed = temp_copy[min(temp_copy.index(r.flywheel_speed)+1,len(temp_copy)-1)]
            r.set_target_state({
                # "flywheel_speed" : min(r.flywheel_speed + 2, 100)
                "flywheel_speed" : new_flywheel_speed
            })

        elif controller_1.buttonL2.pressing() and not previous_controller_states["buttonL2"]:
            temp_copy = r.flywheel_speed_levels.copy()
            if r.flywheel_speed not in temp_copy:
                temp_copy.append(r.flywheel_speed)
            temp_copy.sort()
            new_flywheel_speed = temp_copy[max(temp_copy.index(r.flywheel_speed)-1,0)]
            r.set_target_state({                
                # "flywheel_speed": max(r.flywheel_speed - 2, 0)
                "flywheel_speed": new_flywheel_speed
            })

        if controller_2.buttonR2.pressing() and not previous_controller_states_2["buttonR2"]:
            temp_copy = r.flywheel_speed_levels.copy()
            if r.flywheel_speed not in temp_copy:
                temp_copy.append(r.flywheel_speed)
            temp_copy.sort()
            new_flywheel_speed = temp_copy[min(temp_copy.index(r.flywheel_speed)+1,len(temp_copy)-1)]
            r.set_target_state({
                # "flywheel_speed" : min(r.flywheel_speed + 2, 100)
                "flywheel_speed" : new_flywheel_speed
            })

        elif controller_2.buttonL2.pressing() and not previous_controller_states_2["buttonL2"]:
            temp_copy = r.flywheel_speed_levels.copy()
            if r.flywheel_speed not in temp_copy:
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

        previous_controller_states_2 = {
            "buttonL1" : controller_2.buttonL1.pressing(),
            "buttonR1" : controller_2.buttonR1.pressing(),
            "buttonL2" : controller_2.buttonL2.pressing(),
            "buttonR2" : controller_2.buttonR2.pressing(),
            "buttonUp" : controller_2.buttonUp.pressing(),
            "buttonDown" : controller_2.buttonDown.pressing(),
            "buttonLeft" : controller_2.buttonLeft.pressing(),
            "buttonRight" : controller_2.buttonRight.pressing(),
            "buttonX" : controller_2.buttonX.pressing(),
            "buttonY" : controller_2.buttonY.pressing(),
            "buttonA" : controller_2.buttonA.pressing(),
            "buttonB" : controller_2.buttonB.pressing(),
        }
        wait(0.01, SECONDS)

########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ########## PATHS ##########

# Reset theta function
def reset_robot_theta():
    r.set_target_state({
        "theta" : 0
    })
    r.total_theta = 0

shoot_discs_auto_program = [
    {
        "wait" : 8,
    },
    {
        "flywheel_speed" : 50,
        "wait" : 2.
    },
    {
        "shoot_disc" : 4,
    },
]

match_auto_three_squares = [
    {
        # Set the target x, y, and theta positions to None
        "x_pos" : None,
        "y_pos" : None,
        "theta" : None,
        "auto_intake" : False,
        "auto_roller" : True,
        "drone_mode" : True,
        "override_velocity_theta" : None,
        "override_velocity_y" : -40,
        "message" : "Doing rollers...",
        "wait" : 0.5,
        "flywheel_speed" : 36,
    },
    {
        "roller_spin_for" : -0.35,
    },
    {
        "override_velocity_y" : 10,
        "wait" : 1,
    },
    {   
        "theta" : 95, 
        "override_velocity_y" : 0,
        "override_velocity_x" : 0,
        "wait" : 2,
        "flywheel_speed" : 30,
    },
    {
        "shoot_disc" : 3,
    },
]

match_auto_two_squares = [
    {
        # Set the target x, y, and theta positions to None
        "x_pos" : None,
        "y_pos" : None,
        "theta" : None,
        "auto_intake" : False,
        "auto_roller" : False,
        "drone_mode" : True,
        "override_velocity_theta" : None,
        # POSITIVE INTAKE SPEED IS BLUE, NEGATIVE IS RED
        "flywheel_speed" : 30,
    },
    {
        "override_velocity_y": 38,
        "wait" : 0.4
    },
    {
        # Shoot discs here
        "wait" : 2,
        "override_velocity_y": 0,
        "override_velocity_x": 0,
        "theta" : -100 # 90 - 69
    },
    {
        "shoot_disc" : 1,
    },
    {
        "wait" : 2.1,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "theta" : 0,
    },
    {
        "override_velocity_x": 40,
        "override_velocity_y": 0,
        "wait" : 1.2,
    },
    {
        "override_velocity_x": 0,
        "override_velocity_y": -50,
        "wait" : 1,
    },
    {
        "roller_spin_for" : -0.35,
        "flywheel_speed" : 32,
    },
    {
        # At this point we have just done the roller and are moving forward
        "wait" : 0.4,
        "override_velocity_x" : -5,
        "override_velocity_y" : 45,
    },
    {
        "override_velocity_x" : 0,
        "override_velocity_y" : 0,
        "shoot_disc" : 0,
        "theta" : 90,
        "wait" : 6
    },
    {
        "funcs" : [reset_robot_theta],
    }
    
]

skills_auto = [
    {
        # Set the target x, y, and theta positions to None
        "x_pos" : None,
        "y_pos" : None,
        "theta" : None,
        "auto_intake" : False,
        "auto_roller" : True,
        "drone_mode" : True,
        "override_velocity_theta" : None,
        "override_velocity_y" : -35,
        "message" : "Doing rollers...",
        "wait" : 0.7,
        "flywheel_speed" : 35.5,
    },
    {
        "roller_spin_for" : 0.5,
    },
    {
        "wait" : 1,
        "override_velocity_x" : 0,
        "override_velocity_y" : 37,
        "override_velocity_theta" : None,
    },
    {
        "override_velocity_y" : 0,
        "wait" : 0.5,
    },
    {
        "override_velocity_theta" : None,
        "override_velocity_y" : 0,
        "wait" : 0.5,
        "message" : "setting theta to...",
        "theta" : 90,
    },
    {
        "override_velocity_x" : 35,
        "wait" : 1.2,
    },
    {
        "override_velocity_x" : 0,
    },
    {
        "theta" : 99,
        "wait" : 2,
    },
    # Shoot flywheel
    {
        "shoot_disc" : 1,
    },
    {
        "wait" : 2.5
    },
    {
        "intake_speed" : 100,
        "shoot_disc" : 1,
    },
    {
        "theta" : 90,
        "wait" : 1.5,
    },
    {
        "flywheel_speed" : 0,
        "override_velocity_x" : -35,
        "wait" : 2,
    },
    {
        "override_velocity_x" : 0,
        "wait" : 1,
        "message" : "your dad",
    },
    {
        "override_velocity_x" : -40,
        "wait" : 0.5,
        "message" : "your mom",
        "intake_speed" : 0,
    },
    {
        "override_velocity_x" : -30,
        "override_velocity_y" : -5,
        "wait" : 0.7,
        "message" : "Doing othe>r rollers",
    },
    {
        "override_velocity_x" : -30,
        "override_velocity_y" : -5,
        "wait" : 0.5,
    },
    {
        "override_velocity_x" : -20,
        "roller_spin_for" : 0.5,
        "message" : "message 1"
    },
    # Moving away from other roller
    {
        "override_velocity_x" : 30,
        "override_velocity_y" : 7,
        "wait" : 1.3,
        "message" : "MESSAGE 2",
        "intake_speed" : 0,
    },
    {
        "override_velocity_x" : 0,
        "override_velocity_y" : 0,
        "theta" : 270 - 45,
        "wait" : 1.5,
        "message" : "MESSAGE 3"
    },
    {
        "flywheel_speed" : 34,
        "message" : "moving towards other rollers",
        "override_velocity_x" : 40,
        "override_velocity_y" : 40,
        "wait" : 2.4, # was 5.8
    },
    {
        "wait" : 0.1,
        "intake_speed" : -0,
    },
    {
        "wait" : 0.1,
        "intake_speed" : 0,
    },
    {
        "override_velocity_x" : 0,
        "override_velocity_y" : 0,
        "theta" : 145,
        "wait" : 2,
    },
    {
        "shoot_disc" : 3,
    },
    {
        "theta" : 270 - 45,
        "wait" : 1
    },
    {
        "override_velocity_x" : 40,
        "override_velocity_y" : 40,
        "wait" : 1,
    },
    {
        # SPIN AROUND AND STUFF
        "override_velocity_x" : 20,
        "override_velocity_y" : 20,
        "wait" : 4,
        "theta" : 0,
        "intake_speed" : 0,
    },
    {   
        "override_velocity_x" : 0,
        "override_velocity_y" : 40,
        "wait" : 1.4,
    },
    {
        "override_velocity_y" : 20,
        "roller_spin_for" : 0.7,
        "wait" : 5,
    },
    {
        "roller_spin_for" : 0.65,
    },
    {
        "override_velocity_x" : 0,
        "override_velocity_y" : -50,
        "wait" : 1.5,
    },
    {
        "override_velocity_x" : 0,
        "override_velocity_y" : 0,
        "wait" : 1.5,
    },
    {
        "theta" : -135,
        "wait" : 2
    },
    {
        "launch_expansion" : True,
        "wait" : 2,
    }
]

skills_auto = [
    {
        # Set the target x, y, and theta positions to None
        "x_pos" : None,
        "y_pos" : None,
        "theta" : None,
        "auto_intake" : False,
        "auto_roller" : True,
        "drone_mode" : True,
        "override_velocity_theta" : None,
        "override_velocity_y" : -35,
        "message" : "Doing rollers...",
        "wait" : 0.7,
        "flywheel_speed" : 35.5,
    },
    {
        "roller_spin_for" : 0.5,
    },
    {
        "wait" : 1,
        "override_velocity_x" : 0,
        "override_velocity_y" : 37,
        "override_velocity_theta" : None,
    },
    {
        "override_velocity_y" : 0,
        "wait" : 0.5,
    },
    {
        "override_velocity_theta" : None,
        "override_velocity_y" : 0,
        "wait" : 0.5,
        "message" : "setting theta to...",
        "theta" : 90,
    },
    {
        "override_velocity_x" : 35,
        "wait" : 1.2,
    },
    {
        "override_velocity_x" : 0,
    },
    {
        "theta" : 99,
        "wait" : 2,
    },
    # Shoot flywheel
    {
        "shoot_disc" : 1,
    },
    {
        "wait" : 2
    },
    {
        "intake_speed" : 100,
        "shoot_disc" : 1,
    },
    {
        "theta" : 90,
        "wait" : 1.5,
    },
    {
        "flywheel_speed" : 0,
        "override_velocity_x" : -35,
        "wait" : 2,
    },
    {
        "override_velocity_x" : 0,
        "wait" : 1,
        "message" : "your dad",
    },
    {
        "override_velocity_x" : -40,
        "wait" : 0.5,
        "message" : "your mom",
        "intake_speed" : 0,
    },
    {
        "override_velocity_x" : -30,
        "override_velocity_y" : -5,
        "wait" : 0.7,
        "message" : "Doing othe>r rollers",
    },
    {
        "override_velocity_x" : -30,
        "override_velocity_y" : -5,
        "wait" : 0.5,
    },
    {
        "override_velocity_x" : -20,
        "roller_spin_for" : 0.5,
        "message" : "message 1"
    },
    # Moving away from other roller
    {
        "override_velocity_x" : 30,
        "override_velocity_y" : 7,
        "wait" : 1.3,
        "message" : "MESSAGE 2",
        "intake_speed" : 0,
    },
    {
        "override_velocity_x" : 0,
        "override_velocity_y" : 0,
        "theta" : 90 - 45,
        "wait" : 1.5,
        "message" : "MESSAGE 3"
    },
    {
        "override_velocity_x" : -15,
        "override_velocity_y" : -20,
        "wait" : 1.2,
    },
    {
        "override_velocity_x" : 0,
        "override_velocity_y" : 0,
    },
    {
        "launch_expansion" : True,
        "wait" : 2,
    }
]

skills_auto_w_expansion = skills_auto.copy()
skills_auto_w_expansion.extend([
    {
        "message" : "turning to expand",
        "theta" : -135,
        "wait" : 2
    },
    {
        "message" : "expanding",
        "launch_expansion" : True
    }
]
)

auto_intake_on_forever = [
    {
        "intake_speed" : 100,
        "wait" : 9999,
    },
    {
        "intake_speed" : 100,
        "wait" : 999
    }
]
auto_test_odometry = [
    {
        "override_velocity_theta": None,
        "drone_mode": True,
        "override_velocity_x": None,
        "override_velocity_y": None,
        "set_x": 0,
        "set_y": 0,
        "set_theta": 0,
        "autonomous_speed" : 0.5,
    },
    {
        "launch_expansion": False,
        "theta": 360,
        "intake_speed": 0,
        "message": "Move towards the rollers",
        "flywheel_speed": 0,
        "x_pos": -7,
        "y_pos": -16.5,
    },
    {
        "roller_spin_for" : 0.5,
        "message" : "Doing the rollers!"
    },
    {
        "x_pos": 7,
        "y_pos": 3,
    },
    {
        "launch_expansion": False,
        "theta": 90,
        "intake_speed": 0,
        "message": "Rotating the robot!",
        "flywheel_speed": 0,
    },
    {
        "launch_expansion": False,
        "theta": 90,
        "intake_speed": 100,
        "message": "This is state #4!",
        "flywheel_speed": 0,
        "x_pos": 5,
        "y_pos": 15,
    },
    {
        "launch_expansion": False,
        "theta": 90,
        "intake_speed": 100,
        "message": "About to do the other roller!",
        "flywheel_speed": 0,
        "x_pos": -50,
        "y_pos": 13,
    },
    {
        "intake_speed": 0,
        "x_pos": -62,
    },
    {
        "roller_spin_for" : 0.5,
    },
    {
        "flywheel_speed": 34,
    },
    {
        "launch_expansion": False,
        "theta": 90,
        "intake_speed": 0,
        "message": "This is state #3!",
        "x_pos": 100,
        "y_pos": 13,
    },
    {
        "theta" : 93,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "wait" : 0.5,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "wait" : 0.5,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "launch_expansion": False,
        "theta": 360,
        "intake_speed": 0,
        "message": "This is state #1!",
        "flywheel_speed": 0,
        "x_pos": 100,
        "y_pos": 4.5,
    },
    {
        "launch_expansion": False,
        "intake_speed": 0,
        "message": "This is state #2!",
        "flywheel_speed": 0,
        "x_pos": 80,
        "y_pos": 80,
    },
    {
        "theta": 225.0,
    },
    {
        "autonomous_speed" : 0.2,
        "launch_expansion": False,
        "theta": 225,
        "intake_speed": 100,
        "message": "This is state #4!",
        "flywheel_speed": 0,
        "x_pos": 164,
        "y_pos": 160,
    },
    {
        "autonomous_speed" : 0.48,
    },
    {
        "theta" : 180
    },
    {
        "launch_expansion": False,
        "intake_speed": 0,
        "message": "This is state #1!",
        "flywheel_speed": 0,
        "x_pos": 162.0,
        "y_pos": 163.8,
    },

    {
        "launch_expansion": False,
        "intake_speed": 0,
        "message": "This is state #2!",
        "flywheel_speed": 32,
        "x_pos": 200,
        "y_pos": 164.8,
    },
    {
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 0,
        "message": "This is state #3!",
        "flywheel_speed": 32,
        "x_pos": 200,
        "y_pos": 164.1,
    },
    {
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 0,
        "message": "This is state #4!",
        "flywheel_speed": 32,
        "x_pos": 200,
        "y_pos": 120,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "wait" : 0.7,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "wait" : 0.7,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "launch_expansion": False,
        "theta": 270,
        "intake_speed": 0,
        "message": "This is state #1!",
        "flywheel_speed": 0,
        "x_pos": 200,
        "y_pos": 120.9,
    },
    {
        "launch_expansion": False,
        "theta": 270,
        "intake_speed": 0,
        "message": "This is state #1!",
        "flywheel_speed": 0,
        "x_pos": 200,
        "y_pos": 210,
    },

    {
        "launch_expansion": False,
        "theta": 270,
        "intake_speed": 0,
        "message": "This is state #2!",
        "flywheel_speed": 0,
        "x_pos": 236,
        "y_pos": 210,
    },
    {
        "roller_spin_for" : 0.5,
        "message" : "Doing the third roller!",
    },
    {
        "launch_expansion": False,
        "theta": 270,
        "intake_speed": 0,
        "message": "This is state #3!",
        "flywheel_speed": 0,
        "x_pos": 176,
        "y_pos": 210,
    },

    {
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 100,
        "message": "This is state #4!",
        "flywheel_speed": 0,
        "x_pos": 176,
        "y_pos": 210,
    },

    {
        "launch_expansion": False,
        "theta": 180.1,
        "intake_speed": 100,
        "message": "This is state #5!",
        "flywheel_speed": 0,
        "x_pos": 176,
        "y_pos": 244.4,
    },

    {
        "launch_expansion": False,
        "theta": 180.5,
        "intake_speed": 0,
        "message": "This is state #6!",
        "flywheel_speed": 0,
        "x_pos": 176,
        "y_pos": 276.5,
    },
    {
        "roller_spin_for" : 0.5,
        "message" : "Doing the 4th roller",
    },
    {
        "x_pos": 200,
        "y_pos": 240,
    },
    {
        "launch_expansion": False,
        "theta": 225,
        "intake_speed": 0,
        "message": "This is state #2!",
        "flywheel_speed": 0,
        "x_pos": 200,
        "y_pos": 240,
    },
    {
        "launch_expansion": True,
        "wait" : 1,
    },
    {
        "wait" : 1,
    }
]

auto_test = [
    {
        "override_velocity_theta": None,
        "drone_mode": True,
        "override_velocity_x": None,
        "override_velocity_y": None,
        "set_x": 0,
        "set_y": 0,
        "set_theta": 0,
    },
    {
        "autonomous_speed" : 0.25,
        "y_pos" : 20,
    },
]



field_length = 356  # CM

# Initializes a robot object
r = Robot()

init()
r.init()
######### GUI ######### GUI ######### GUI ######### GUI ######### GUI ######### GUI #########

# Initialize the gui
gui = GUI()

# Add all of the gui elements
gui.add_page([
    Text("Competition Information", 120, 0, 240, 20, Color.BLACK),
    Switch(["Team Red", "Team Blue"],0,40,120,40,[Color.RED, Color.BLUE],[r.set_team] * 2,["red", "blue"],),
    Switch(["Robot Mode", "Drone Mode"], 120, 40, 120, 40, [Color(0x888888), Color.BLACK], [lambda value: r.set_target_state({"drone_mode" : value})] * 2, (False, True),),
    Switch(["Auto Intake", "Manual Intake"], 120, 120, 120, 40, [Color(0x88AA88), Color(0xAA8888)], [r.set_target_state] * 2, [{"auto_intake" : True}, {"auto_intake" : False}],),
    
    # Changes the roller mode from manual to auto
    Switch(["Auto Roller", "Manual Roller"], 120, 80, 120, 40, [Color(0x88AA88), Color(0xAA8888)], [r.set_target_state] * 2, [{"auto_roller" : True}, {"auto_roller" : False}],),
    Button("Reset θ", 0, 200, 120, 40, (0xAAAAFA), reset_robot_theta), 
    Switch(["GPS", "No GPS"], 120, 160, 120, 40, [Color(0x88AA88), Color(0xAA8888)], [r.set_target_state] * 2, [{"use_gps" : True}, {"use_gps" : False}]),
    Switch(["Expansion On", "Expansion Off"], 120, 200, 120, 40, [Color(0x88AA88), Color(0xAA8888)], [r.set_target_state] * 2, [{"launch_expansion" : True}, {"launch_expansion" : False}]),

    # Save the path by printing the representation (in the future it can be saved to sd car), 
    Text("", 240, 40, 120, 40, Color.BLACK, lambda: "x: {:.2f}".format(r.x_pos),), 
    Text("", 240, 80, 120, 40, Color.BLACK,lambda: "y: {:.2f}".format(r.y_pos),), 
    Text("", 240, 120, 120, 40, Color.BLACK, lambda: f("θ:", str(r.theta).split(".")[0]),),
     
    Text("", 360, 40, 120, 40, Color.BLACK, lambda: "t_x: {:.2f}".format(r.target_state["x_pos"]),), 
    Text("", 360, 80, 120, 40, Color.BLACK,lambda: "t_y: {:.2f}".format(r.target_state["y_pos"]),), 
    Text("", 360, 120, 120, 40, Color.BLACK, lambda: f("t_θ:", str(r.target_state["theta"]).split(".")[0]),),

    Button("Go To Debug", 360, 200, 120, 40, (0xAAAAAA), lambda: gui.set_page(1)),

    # Auto Selector
    Switch(["3-Square", "2-Square", "SKILLS", "ShootDiscs", "Intake Forever", "Temp", "Test"], 0, 120, 120, 40, [0x33FF33, 0x33CC33, 0x33AA33, 0x339933, 0x337733, 0x335533,0x333333], lambda auto_mode: r.set_autonomous_procedure(auto_mode), [match_auto_three_squares, match_auto_two_squares, skills_auto, shoot_discs_auto_program, auto_intake_on_forever, auto_test_odometry, auto_test]),
    Button("Run auto", 0, 160, 120, 40, (0xAAAAAA), lambda: r.run_autonomous()),
    Switch(["2-Controller", "1-Controller"], 240, 160, 120, 80, [0xCCAAAA, 0xCCAACC], lambda value: set_debug_value(value), (False, True))
])

gui.add_page([
    Text("Debug Information!", 120, 0, 240, 20, Color.BLACK),
    Switch(["Team Red", "Team Blue"],0,0,120,40,[Color.RED, Color.BLUE],[r.set_team] * 2,["red", "blue"],),
    Switch(["Drone Mode", "Robot Mode"], 120, 0, 120, 40, [Color(0x888888), Color.BLACK], [lambda value: r.set_target_state({"drone_mode" : value})] * 2, (True, False),),
    Switch(["Auto Intake", "Manual Intake"], 120, 120, 120, 40, [Color(0x88AA88), Color(0xAA8888)], [r.set_target_state] * 2, [{"auto_intake" : True}, {"auto_intake" : False}],),
    
    # Changes the roller mode from manual to auto
    Switch(["Auto Roller", "Manual Roller"], 120, 80, 120, 40, [Color(0x88AA88), Color(0xAA8888)], [r.set_target_state] * 2, [{"auto_roller" : True}, {"auto_roller" : False}],),
    Button("Flywheel +2", 0, 40, 120, 40, (0x88AA88), lambda: r.set_target_state({"flywheel_speed" : r.flywheel_speed + 2,}),), 
    Button("Flywheel -2", 0, 80, 120, 40, (0xAA8888), lambda: r.set_target_state({"flywheel_speed" : r.flywheel_speed - 2,}),),
    Button("Shoot Disc", 0, 120, 120, 40, (0x8888AA), lambda: r.set_target_state({"shoot_disc" : 1,}),),
    
    Button("Reset Theta", 0, 160, 120, 40, (0xAAAAFA), reset_robot_theta), 
    Switch(["GPS", "No GPS"], 120, 160, 120, 40, [Color(0x88AA88), Color(0xAA8888)], [r.set_target_state] * 2, [{"use_gps" : True}, {"use_gps" : False}]),
   
    Text("", 0, 200, 120, 40, (Color.BLACK), lambda: f("State #", len(r.path)),), 
    
    
    Button("Add to Path", 120, 200, 120, 40, (0xAAFAAA), lambda: r.save_state),
    # Save the path by printing the representation (in the future it can be saved to sd car), 
    Button("Save Path", 240, 200, 120, 40, (0xAAAAAA), lambda: print_state_nicely(r.path)),
    
    Text("", 240, 0, 120, 40, (0x110000), lambda: f("SP:", r.flywheel_speed),), 
    Text("", 360, 0, 120, 40, (0x110000), lambda: f("Real:", round(flywheel_motor_1.velocity(PERCENT)), round(flywheel_motor_2.velocity(PERCENT)))), 
    Text("", 240, 40, 120, 40, Color.BLACK, lambda: "x: {:.2f}".format(r.x_pos),), 
    Text("", 240, 80, 120, 40, Color.BLACK,lambda: "y: {:.2f}".format(r.y_pos),), 
    Text("", 240, 120, 120, 40, Color.BLACK, lambda: f("θ:", str(r.theta).split(".")[0]),), 
    Text("", 360, 40, 120, 40, Color.BLACK, lambda: f("T", round(flywheel_motor_1.temperature()), round(flywheel_motor_2.temperature()))),
    Switch(["Rec. Off", "Rec. On"], 360, 160, 120, 40, [Color.RED, Color.GREEN], lambda v: start_recording_mode_for_autonomous(v), (False, True)),
    Button("Go To Comp", 360, 200, 120, 40, (0xAAAAAA), lambda: gui.set_page(0)),
    Text("", 120, 40, 120, 40, Color.BLACK, lambda: f(round(left_motor_a.temperature()), round(left_motor_b.temperature()), round(right_motor_a.temperature()), round(right_motor_b.temperature()))),
])

gui.add_page([
    Text("Prematch Checklist!", 120, 0, 240, 20, Color.BLACK),
    Text("", 50, 20, 430, 30, Color.BLACK, lambda: f("Flywheel temperature < 45 deg? (", round(flywheel_motor_1.temperature()), round(flywheel_motor_2.temperature()),")")),
    Text("", 50, 30, 430, 30, Color.BLACK, lambda: f("Drivetrain good? (", round(left_motor_a.temperature()), round(right_motor_a.temperature()), round(left_motor_b.temperature()), round(right_motor_b.temperature()), ")")),
    Text("Flywheel lubed up recently?", 50, 80, 430, 30, Color.BLACK),
    Text("Expansion set?", 50, 110, 430, 30, Color.BLACK),
    Text("Preloads?", 50, 130, 430, 30, Color.BLACK),

    Button("Go To Comp", 360, 120, 120, 40, (0xAAAAAAA), lambda: gui.set_page(0))
])

gui.pages[2].extend(
    [Switch(["", "X"], 20, y, 30, 30, [Color.RED, Color.GREEN]) for y in range(20, 220, 30)]
)
gui.set_page(0)

# Use the robot doctor so that we can diagnose any problems
# rd = RobotDoctor()
# rd.init()
# rd.run_tests()

# 30 millisecond wait for the gyroscope to initialize
wait(30, MSEC)

r.set_autonomous_procedure(auto_test_odometry)

def test_drivetrain():
    ### Tests to see how long it takes for the drivetrain to overheat (reach 55 deg.)

    t = Timer()
    t.reset()
    val = 0
    while True:
        r.set_target_state({
            # Make it so that the robot is constantly rotating
            "override_theta_velocity" : 100,
        })

        if t.value() > 5 + val:
            # Display the temperature
            print(t.value(), left_motor_a.temperature(), right_motor_a.temperature(), left_motor_b.temperature(), right_motor_b.temperature())
            val = t.value()

        if left_motor_a.temperature() >= 55 or right_motor_a.temperature() >= 55 or left_motor_b.temperature() >= 55 or right_motor_b.temperature() >= 55:
            # When any of the motors overheat (reach 55 deg. C) then stop the program and display how long it took to overheat
            print("It took ", t.value(), "seconds to overheat!")
            
            # Stop the robot from rotating
            r.set_target_state({
                "theta" : r.theta
            })
            return

# test_drivetrain()

def output_data():
    # Print the flywheel torque
    # Prints the efficiency of the motors:
    # print(brain.timer.value(), flywheel_motor_1.velocity(PERCENT), flywheel_motor_2.velocity(PERCENT))
    # Display the flywheel speed and the robot theta from [-180 to 180]
    # print(brain.timer.value(), left_motor_a.position(DEGREES), right_motor_a.position(DEGREES), left_motor_b.position(DEGREES), right_motor_b.position(DEGREES))
    # print(brain.timer.value(), flywheel_motor_1.velocity(PERCENT), flywheel_motor_2.velocity(PERCENT))
    pass

competition = Competition(driver_control, r.run_autonomous)

# ! PRIORITY
# TODO: Figure out why the robot's encoders are reading a greater value than they actually are

# TODO: Make all pid controllers be based on delta time
# TODO: Make loop sleep time 0 AND then retune EVERYTHING

# TODO: Make flywheel integral term be better (maybe make it higher when we are winding up???)
# TODO: Do std evaluation on output from flywheels

# TODO: Use accel/encoders to figure out when the robot is stopped?!?!?
# TODO: Create some sort of timeout
# TODO: Make thingy that detects when robot stops and if that happens then continue the autonomous mode.
# TODO: For competition, be sure to turn on HOLD mode
# TODO: Make autonomous mode follow the same velocity vector as during the recording???

# TODO: Add a "neutral" team color, also make sure that the auto orientate towards high goal worksi n a regular match
# TODO: Clean up the code for auto-orientate
# TODO: For the aimbot, make it so that if we are on a specific team, the robot won't accidentally try to shoot at it
# TODO: Test to see if the aimbot gets tricked if there are different colors in the background (like a blue or something)

# TODO: Test removing the derivative term from the flywheel
# TODO: Make driving not care about turning speed (so turning speed takes precedence)
# TODO: Find position of gps in relation to center of robot
# TODO: Make it so that the robot movement is based off of the gps for now AND THEN add it using the encoders (kinda backwards, ik)

# TODO: Theoretically, shouldn't max acceleration half if we're close to 0 for the motor speed? rework that function
# TODO: Make a better pose/orienation initialization/resetting tool (initialize the position in the init() command, don't worry if we can't init yet)

# TODO: When the robot initializes, make it rotate towards 90 or 180 degrees exactly (if GPS is on)
# TODO: Use sensor fusion with Kalman filter for better position
# TODO: Make odometry consistent with real world measurements

# TODO: See if we can register callback functions instead of checking things in the updates and see how much that will improve performance
# TODO: Use trapazoids instead of rectangles when integrating the velocity

# * IMPORTANT
# TODO: Pid controller for motor movement?!?!?!?!?!?!?!?
# TODO: Figure out variance in GPS sensor data and variance in model (Q and R)
# TODO: make the offset of the gps so that it reports the position of the center of the robot
# TODO: When the robot has an x,y position close to the edge of the field, make it so that the robot physically cannot slam into the wall (or use sensors)
# TODO: Change this to be relative to the match time???

# Not important
# TODO: make an auto path class that has information like description, color, etc, maybe draw it out???.
