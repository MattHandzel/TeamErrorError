
import math
import time
from vex import *
from math import cos, sin, pi, sqrt, atan2

#region Initialization of Constants/Global Variables
global g
g = -9.81  # Use for flywheel speed calculator

global field_length
field_length = 356  # CM

global turret_theta_vel
turret_theta_vel = 0

global RAD_TO_DEG
RAD_TO_DEG = 180 / math.pi

global DEG_TO_RAD
DEG_TO_RAD = math.pi / 180

global robot_debug_mode
robot_debug_mode = False

global recording_autonomous
recording_autonomous = False

global r2o2
r2o2 = math.sqrt(2) / 2

# INITIATLIZATION OF PERIPHERALS
brain = Brain()
controller_1 = Controller(PRIMARY)
controller_2 = Controller(PARTNER)

global programming_chassis
programming_chassis = False

if not programming_chassis:
    left_motor_a = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
    left_motor_b = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)

    right_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
    right_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)

    inertial = Inertial(Ports.PORT8)

    flywheel_motor = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
    gps = Gps(Ports.PORT3)

else:
    gps = Gps(Ports.PORT12)
    # left_motor_a = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
    # left_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)

    # 7.5, 2.75
    # right_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
    # right_motor_b = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
    inertial = Inertial(Ports.PORT18)
    
    left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
    left_motor_b = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)

    right_motor_a = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
    right_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
    
    flywheel_motor = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)


indexer_limit_switch = DigitalIn(brain.three_wire_port.c)

turret_limit_switch = DigitalIn(brain.three_wire_port.d)

turret_motor = Motor(Ports.PORT14, GearSetting.RATIO_36_1, False)

roller_and_intake_motor_1 = Motor(Ports.PORT4, GearSetting.RATIO_36_1, False)
roller_and_intake_motor_2 = Motor(Ports.PORT5, GearSetting.RATIO_36_1, False)
roller_and_intake_motor = MotorGroup(roller_and_intake_motor_1, roller_and_intake_motor_2)

roller_optical = Optical(Ports.PORT6)

indexer = Pneumatics(brain.three_wire_port.a)
expansion = Pneumatics(brain.three_wire_port.b)

flywheel_status_light = Led(brain.three_wire_port.h)


# Vision signatures (disc signatures if we want to autoamtically sense discs, blue and red signatures for the red ang blue goals)
vision__DISC = Signature(1, 6911, 8133, 7522, -6787, -5937, -6362, 1.3, 0)
vision__BRIGHT_DISK = Signature(2, 217, 491, 354, -7169, -6839, -7004, 3, 0)
vision__RED_GOAL = Signature(3, 7243, 8689, 7966,-701, 107, -297,3, 0)
vision__BLUE_GOAL = Signature(4, -1985, 1, -992,1981, 6665, 4323,1.4, 0)
vision__DISK = Signature(5, 1905, 2299, 2102,-4017, -3641, -3829,2.5, 0)
vision = Vision(Ports.PORT7, 50, vision__DISC, vision__BRIGHT_DISK, vision__RED_GOAL,
                vision__BLUE_GOAL, vision__DISK)

DISC_SIGNATURES = [vision__DISC, vision__BRIGHT_DISK]

#endregion

#region Misc/Helper Functions/Classes

def multiply_probability_distribution(mean1, var1, mean2, var2):
  new_mean = (var1 * mean2 + var2 * mean1) / (var1 + var2)
  new_var = var1 * var2 / (var1 + var2)
  return [new_mean, new_var]

class CircularArray:

    def __init__(self, length):
        self.length = length
        self.arr = []
        self.index = 0

    def append(self, item):
        if len(self.arr) < self.length:
            self.arr.append(item)
            self.index = len(self.arr) % self.length
        else:
            self.arr[self.index] = item
            self.index = (self.index + 1) % self.length
    
    def pop_newest(self):
        self.index -= 1
        item = self.arr.pop(self.index)
        if self.index < 0:
            self.index = len(self.arr)
        return item

    def pop_oldest(self):
        # Removes the older
        item = self.arr.pop(self.index)
        if self.index >= len(self.arr):
            self.index = 0
        return item
class Vector:
    '''
    Vector class I wrote because basic python lists are lame, this is as slow as normal python, should be ideally replaced with numpy arrays
    '''
    def __init__(self, data):
        self.data = data

    def __add__(self, other):
        assert len(other) == len(self.data)
        return Vector([other[i] + self.data[i] for i in range(len(self.data))])

    def __sub__(self, other):
        assert len(other) == len(self.data)
        return Vector([self.data[i] - other[i] for i in range(len(self.data))])
  
    def __mul__(self, other):
        return Vector([x * other for x in self.data])
  
    def __rmul__(self, other):
        return Vector([x * other for x in self.data])
  
    def __rdev__(self, other):
        return Vector([other / x for x in self.data])
    
    def __truediv__(self, other):
        return Vector([x / other for x in self.data])

    def __getitem__(self, key):
        return self.data[key]

    def __len__(self):
        return len(self.data)

    def __repr__(self):
        return "Vector:	" + repr(self.data)

class numpy:
    @staticmethod
    def abs(vector):
        return Vector([abs(x) for x in vector])

    @staticmethod
    def dot(v1, v2):
        assert len(v1) == len(v2)
        return Vector([v1[i] * v2[i] for i in range(len(v1))])

def magnitude(vector):
    return sum([x**2 for x in vector])**0.5

def argmax(arr):
    return arr.index(max(arr))

def argmin(arr):
    return arr.index(min(arr))

def interpolate(x, x1, x2, y1, y2):
    return (x - x1) * (y2 - y1) / (x2 - x1) + y1

def closest_angle(ang):
        # Turn via the shortest path
    while ang > 180:
        ang -= 360
    
    while ang < -180:
        ang += 360

    return ang

def std(_list):
    if len(_list) == 0:
        return 0
    # Standard deviation
    _mean = sum(_list) / len(_list)
    _sum = 0
    for i in _list:
        _sum += (i - _mean)**2
    return math.sqrt(_sum / len(_list))

def set_debug_value(_value):
    one_controller_mode = _value

def start_recording_mode_for_autonomous(_value):
    print("I HAVE BEEN PRESSED AND MY VAL IS", _value)
    global recording_autonomous 
    recording_autonomous = _value
    
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
# Reset theta function
def reset_robot_theta():
    r.set_target_state({
        "theta" : 0
    })
    r.total_theta = 0


def print_state_nicely(state):
  # print white space so we can see easier when coping and pasting
  print("[\n")

  # nicely format the state dictionary
  for s in state:
    _str = ""
    _str += "\n{"

    for key in s:
      _str += "\n" + "\"" + key + "\"" + ": " + str(s[key]) + ","
    _str += "\n},"
    print(_str)

  print("]\n")

def get_angle_to_object(pos_1, pos_2):
    '''
    RETURNS IN DEGREES
    '''

    # If the passed in objects are GameObjects then change the pos's into a tuple of x,y values
    ang = atan2(pos_2[1] - pos_1[1], pos_2[0] - pos_1[0]) * RAD_TO_DEG
    
    # converts angle from rotation to heading
    return closest_angle(90 - ang)

def check_intersection(line1, line2):

    # line1 and line2 are tuples of 4 points (x1, y1, x2, y2)
    # returns True if they intersect, False otherwise
    
    # Prevents divide by 0 error
    if (line1[2] - line1[0]) == 0:
        line1[2] += 0.0001
        line1[0] -= 0.0001
    if (line2[2] - line2[0]) == 0:
        line2[2] += 0.0001
        line2[0] -= 0.0001

    # get the slope and y-intercept of each line
    m1 = (line1[3] - line1[1]) / (line1[2] - line1[0])
    m2 = (line2[3] - line2[1]) / (line2[2] - line2[0])
    b1 = line1[1] - m1 * line1[0]
    b2 = line2[1] - m2 * line2[0]
    
    # if the slopes are equal, they are parallel and don't intersect
    if m1 == m2:
        return False
    
    # find the point of intersection
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    
    # check if the point of intersection is within the line segments
    if x > max(line1[0], line1[2]) or x < min(line1[0], line1[2]):
        return False
    if x > max(line2[0], line2[2]) or x < min(line2[0], line2[2]):
        return False
    if y > max(line1[1], line1[3]) or y < min(line1[1], line1[3]):
        return False
    if y > max(line2[1], line2[3]) or y < min(line2[1], line2[3]):
        return False
    
    return True


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


class GameObject:
    '''
    If we want to introduce game object (like say the goal or barriers), we have have to robot look up important information about the game object
    '''

    def __init__(self, x_pos, y_pos):
        self.x_pos = x_pos
        self.y_pos = y_pos

#endregion

def calibrate_gps():
    initial_x_positions = []
    initial_y_positions = []
    initial_theta_positions = []

    if gps.installed():

        init_timer = Timer()
        init_timer.reset()
        init_time = 2
        while init_timer.value() < init_time:
            initial_x_positions.append(gps.x_position(DistanceUnits.CM))
            initial_y_positions.append(gps.y_position(DistanceUnits.CM))
            initial_theta_positions.append(gps.heading())
            wait(0.02, SECONDS)

        r.initial_x_field = float(sum(initial_x_positions) / len(initial_x_positions))
        r.initial_y_field = float(sum(initial_y_positions) / len(initial_y_positions))

        # self.initial_x_field, self.initial_y_field = rotate_vector_2d(self.initial_x_field, self.initial_y_field, self.gps_theta_on_robot * DEG_TO_RAD)
        r.initial_theta_field = float(sum(initial_theta_positions) / len(initial_theta_positions)) - r.gps_theta_on_robot # The + 90 is because the gps is 90 deg off of the robot
    
    print("From calibrating, average positions are: ", r.initial_x_field, r.initial_y_field, r.initial_theta_field)
    
    print("Stds are: ", std(initial_x_positions), std(initial_y_positions), std(initial_theta_positions))


def init():
    '''
    This function will initialize every subsystem with constants that wont change throughout the competition,
    it will set driver controlled motors to break mode, start spinning motors that use the "set_velocity()" function,
    and initialize our gyroscope. It should be called before every other function for the robot

    '''
    left_motor_a.set_stopping(BRAKE)
    right_motor_a.set_stopping(BRAKE)
    left_motor_b.set_stopping(BRAKE)
    right_motor_b.set_stopping(BRAKE)

    flywheel_motor.spin(FORWARD, 0, VOLT)
    flywheel_motor.set_stopping(BRAKE)

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

    turret_motor.set_stopping(HOLD)

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

    brain.screen.draw_rectangle(10, 10, 10, 10, Color.WHITE)

    if programming_chassis:
        # gps.set_origin(190, 60, MM)
        gps.set_origin(180, 35, MM)
    else: 
        gps.set_origin(150, 70, MM)

    inertial.calibrate()

    Thread(calibrate_gps)
    while inertial.is_calibrating():
        wait(0.05, SECONDS)

    # Wait for the gyro to settle, if it takes more then 10 seconds then close out of the loop
    # When the gyro sensor inits, it reads some value for the Z rotation, this is less than a few degrees, but i don't like it
    # while (inertial.gyro_rate(ZAXIS) != 0 and t.value() < 10):
    #     print("Waiting for gyro to init...")
    #     wait(0.1, SECONDS)

    # Rumlbed the control to indicate to the driver (and me) that the robot is ready to run
    # controller_1.rumble("...")

#region Object Trajectory Prediction

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

#endregion

#region GUI elements for brain screen
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

#endregion

class Robot:
    '''
    This is the big-boy robot class, this is the class that controls the robot, there is a lot of stuff here
    '''
    initialized = False
    #region Variables relating to orientation/positions/etc
    total_theta = 0
    initial_theta_field = 0
    max_velocity: float
    max_acceleration: float

    previous_x_encoders_relative = 0
    previous_y_encoders_relative = 0

    previous_x_gps_field = 0
    previous_y_gps_field = 0

    x_encoders_relative = 0
    y_encoders_relative = 0

    gps_theta_on_robot = 0 # gps is 90 deg to the right from the center of the robot
    gps_x_on_robot = 15 # 188
    gps_y_on_robot = 7 # 30

    delta_x_encoders_field = 0
    delta_y_encoders_field = 0

    delta_x_encoders_relative = 0
    delta_y_encoders_relative = 0

    x_encoders_field = 0
    y_encoders_field = 0

    delta_x_gps_relative = 0
    delta_y_gps_relative = 0
    
    delta_x_gps_field = 0
    delta_y_gps_field = 0

    theta_field = 0

    x_gps_relative = 0
    y_gps_relative = 0

    initial_x_field = 0
    initial_y_field = 0
    initial_theta_field = 0
    x_from_gps = 0
    y_from_gps = 0

    # variance of gps, we got this number through experimentation (sted ^ 2)
    gps_variance = 8

    x_position_variance = 8
    y_position_variance = 8

    predicted_position = 0

    #endregion

    #region Flywheel Variables    
    turret_initial_angle = 0

    turret_centered_angle = 22 

    turret_far_angle = 39
    
    # Set the offset for the flywheel from the center of the robot
    flywheel_offset_x = 0   
    flywheel_offset_y = 0

    flywheel_angle = 45 * DEG_TO_RAD

    flywheel_avg_speed = 0

    previous_flywheel_avg_speed = 0

    previous_flywheel_error = 0

    integral_term_flywheel = 0

    previous_flywheel_speed = 0

    flywheel_speed = 0

    flywheel_height_from_ground_IN = -99999

    flywheel_motor_error = 0

    limit_switch_active = True

    flywheel_motor_average_output = 0

    average_target_flywheel_output = 0
    
    flywheel_angle_offset = 9.5

    target_goal = None 

    flywheel_kP = 0.6 # before aws 0.03
    flywheel_kI = 0.002 / 0.022 # before was 0.0004 (this worked pretty well)
    flywheel_kD = 0.03 * 0.022
    flywheel_kF = 0.105
    
    flywheel_speed_levels = [
        0,
        50,
        51,
        52,
        53,
        54,
        55,
        56,
        57,
        58,
        59,
        60,
        61,
        62,
        63,
        64,
        65,
        66,
        67,
        68,
        69,
        70,
        71,
        72,
        73,
        74,
        100,
    ]

    flywheel_instantaneous_speed = 0
    
    # distance_speed_maps = {
    #     0: 0,
    #     100 : 53.5,
    #     122 : 55,
    #     158 : 57,
    #     183 : 58.5,
    #     219 : 62.5,
    #     290 : 71.5,
    # }
    distance_speed_maps = {
        0: 0,
        78: 49 + 0.25,
        98 : 50 + 0.25,
        152 : 52.75 + 0.25,
        183 : 55 + 0.25,
        267 : 64.5,
        315 : 68,
    }

    # A map that maps the speed of the flywheel to what direction the turret needs to turn
    speed_angle_maps = {

    }

    turret_theta_range = 40

    turret_limit_switch_timer = Timer()
    
    max_turret_torque = 0
    #endregion
    
    #region Drivetrain Variables
    drivetrain_gear_ratio = 18
    wheel_max_rpm: float = 200
    wheel_diameter_CM: float = 8.255

    # In order to get this, it is ticks for the specific gear ratio we're using divided by the circumeference of our wheel
    wheel_distance_CM_to_TICK_coefficient: float = (drivetrain_gear_ratio / 6 * 300) / (math.pi * wheel_diameter_CM) * 0.47
    #endregion

    #region Timers    
    flywheel_recovery_timer = Timer()
    intake_timer = Timer()

    # Used to keep track of time in auto and driver mode respectively, use it for nicely logging data, can be used during either modes for end game/pathfinding rules
    autonomous_timer = Timer()
    driver_controlled_timer = Timer()

    # endregion

    #region States/State Trackers
    previous_update_time: float = 0

    target_reached = False
    update_loop_delay = 0.006 # 10 ms
    
    autonomous_speed: float = 48
    running_autonomous = False

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

        "flywheel_instantaneous_speed" : 0,

        "min_velocity" : 0,
        "slow_down_distance" : 1,
        "tolerance" : 0.5,

        "shoot_at_theta" : None,

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
        "aimbot" : False,

        "flywheel_torque" : 0,
        "disc_shot" : False,

        "is_shooting" : False,
        "turret_torque" : 0,

        "auto_aim" : False,
        "auto_orientate" : False,
        "turret_theta" : 0,
    }

    target_state = {
        "x_pos" : None,
        "y_pos" : None,
        "theta" : None,
    }
    
    all_states = []

    delta_time = 0

    team_color = "neutral"

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

    turret_initialized = False

    #endregion
    
    #region MISC
    # From 0,0 (which is the center of the field). Dimensions were got from page 89 on: https://content.vexrobotics.com/docs/2022-2023/vrc-spin-up/VRC-SpinUp-Game-Manual-2.2.pdf
    red_goal = GameObject(133, 133)
    blue_goal = GameObject(-133, -133)
    
    #endregion

    def __init__(self):

        # what our max velocity "should" be (can go higher or lower)
        self.max_velocity = ((self.wheel_max_rpm / 60) *
                             math.pi * 2 * self.wheel_diameter_CM / math.sqrt(2))
        # This number in the divisor means it will speedup/slow down in that many seeconds
        self.max_acceleration = 2 * self.max_velocity / 0.033

        # Set origin of the gps
        gps.set_origin(0,0)

        # Thsee next few lines just make sure that the state system works, and saves and initial state that we return to when the autonomous mode ends
        self.initial_state = self.state.copy()
        self.set_target_state(self.state)
        self.previous_state = self.state
        
    
    # There are two init methods, this init initializes the class, the other init method we call to initlize the robot
    def init(self, _init_time = 1):
        '''
        Different than the Robot.__init__ dunder that gets called when the robot is made, this is a manual initialization that 
        starts the update loop and turns on the robot
        '''

        self.set_target_state(self.state)
        self.previous_state = self.state
        Thread(self.update_loop)

        self.flywheel_recovery_timer.reset()
        self.turret_limit_switch_timer.reset()

        self.initialized = True

    #region UPDATE FUNCTIONS
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

        # update the automatic stuff
        self.automatic_update()

        # All controls for movement, pid control for robot 
        self.position_update()

        # Update the turret
        self.turret_update()

        # All of the updates needed for the flywheel, including pid control
        self.flywheel_update()

        # All of code for roller, including auto roller=
        self.roller_update()
        
        # All of the code and checks for the intake, allows for auto intake and manual control
        self.intake_update()

        # Run all the code needed for the expansion
        self.expansion_update()

        # Update all status indicators
        self.status_update()

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
        self.delta_x_encoders_relative = self.x_enc - self.previous_state["x_enc"]
        self.delta_y_encoders_relative = self.y_enc - self.previous_state["y_enc"]

        # Rotate the numbers based on our orientation relative to the robot
        self.delta_x_encoders_relative, self.delta_y_encoders_relative = rotate_vector_2d(self.delta_x_encoders_relative, self.delta_y_encoders_relative, -self.theta * DEG_TO_RAD)

        # Get the velocity of the robot in deg/s for 4 wheels
        self.x_vel = self.delta_x_encoders_relative / self.delta_time
        self.y_vel = self.delta_y_encoders_relative / self.delta_time


        # Velocity of robot from gps perspective
        x_from_gps = 0
        y_from_gps = 0
        theta_field = 0
        gps_encoder_blend_alpha = 0

        self.delta_x_gps_field = 0
        self.delta_y_gps_field = 0

        x_vel_from_gps = None # 
        y_vel_from_gps = None

        # If the gps is being used and not being obstructed
        if self.using_gps and gps.quality() >= 100: 
            
            # We can choose to use the gps or the gyro for our heading
            # self.total_theta = gps.heading()

            # Update alpha to value that uses gps
            gps_encoder_blend_alpha = 1
            x_from_gps = gps.x_position(DistanceUnits.CM)
            y_from_gps = gps.y_position(DistanceUnits.CM)

            x_vel_from_gps = (x_from_gps - self.previous_x_gps_field) / self.delta_time
            y_vel_from_gps = (y_from_gps - self.previous_y_gps_field) / self.delta_time

            # x_from_gps, y_from_gps = rotate_vector_2d(x_from_gps, y_from_gps, self.gps_theta_on_robot * DEG_TO_RAD)

            theta_field = gps.heading() - self.gps_theta_on_robot

            # If we want to figure out our velocity from the gps
            self.delta_x_gps_field = gps.x_position(DistanceUnits.CM) - self.previous_x_gps_field 
            self.delta_y_gps_field = gps.y_position(DistanceUnits.CM) - self.previous_y_gps_field 

            self.previous_x_gps_field = gps.x_position(DistanceUnits.CM)
            self.previous_y_gps_field = gps.y_position(DistanceUnits.CM)

            # Rotate the values from the gps so that they are towards the center of the robot

            x_offset, y_offset = rotate_vector_2d(self.gps_x_on_robot, self.gps_y_on_robot, (self.theta + self.initial_theta_field) * DEG_TO_RAD)

            # print("x_from_gps, y_from_gps", x_from_gps, y_from_gps)
            # print("x_offset, y_offset", x_offset, y_offset)
            # x_from_gps -= x_offset
            # y_from_gps -= y_offset
            # print("x_from_gps, y_from_gps", x_from_gps, y_from_gps)

            # print("Position from gps is", x_from_gps, y_from_gps, theta_from_gps)

            # self.field_theta = self.field_theta * 0.9 + 0.1 * gps.heading()
            # print("We have moved x_enc, y_enc", delta_x_from_encoders, delta_y_from_encoders, "from gps", (x_from_gps - self.x_pos), (y_from_gps - self.y_pos))
        elif self.using_gps:
            # If we are using the gps but it is obstructed, then we want to use the encoders
            
            x_from_gps = self.x_from_gps
            y_from_gps = self.y_from_gps
            theta_field = self.theta_field

        gps_encoder_blend_alpha = 0
        gps_encoder_blend_alpha = clamp(gps_encoder_blend_alpha * self.delta_time, 1, 0)

        # If we have not moved (from the encoders point of view), and the gps is not changing that much, then use the rolling average from the gps
        # If gps is enabled then the low and high pass filter will make the x and y position more stable, if gps is not enabled then the formula won't use gps data (alpha would equal 0)
        self.x_pos += self.delta_x_encoders_relative * (1-gps_encoder_blend_alpha) + (self.delta_x_gps_field-self.x_pos) * gps_encoder_blend_alpha 
        self.y_pos += self.delta_y_encoders_relative * (1-gps_encoder_blend_alpha) + (self.delta_y_gps_field-self.y_pos) * gps_encoder_blend_alpha
        
        gps_alpha = 1
        gps_alpha = clamp(gps_alpha * self.delta_time, 1, 0)
        
        self.x_from_gps = self.x_from_gps * (1-gps_alpha) + x_from_gps * gps_alpha
        self.y_from_gps = self.y_from_gps * (1-gps_alpha) + y_from_gps * gps_alpha

        self.x_pos = self.x_from_gps
        self.y_pos = self.y_from_gps

        self.theta_field = theta_field * (1-gps_alpha) + self.theta_field * gps_alpha
        
        self.x_encoders_relative += self.delta_x_encoders_relative
        self.y_encoders_relative += self.delta_y_encoders_relative

        velocity_alpha = 0.2
        if x_vel_from_gps is not None:
            self.x_vel = self.x_vel * (1 - velocity_alpha) + x_vel_from_gps * velocity_alpha
            self.y_vet = self.y_vel * (1 - velocity_alpha) + y_vel_from_gps * velocity_alpha

        # Rotate the delta_x from the encoders to be relative to the field instead of relative to the robot
        self.delta_x_encoders_field, self.delta_y_encoders_field = rotate_vector_2d(self.delta_x_encoders_relative, self.delta_y_encoders_relative, -self.theta_field  * DEG_TO_RAD)
        
        self.x_encoders_field += self.delta_x_encoders_field
        self.y_encoders_field += self.delta_y_encoders_field
    
        self.delta_x_gps_relative, self.delta_y_gps_relative = rotate_vector_2d(self.delta_x_gps_field, self.delta_y_gps_field, self.initial_theta_field  * DEG_TO_RAD) 
        
        self.x_gps_relative, self.y_gps_relative = self.convert_field_to_local_coordinates(self.x_from_gps, self.y_from_gps)

        ### KALMAN FILTER

        self.aimbot_trust = gps.quality()

        # self.x_pos, self.y_pos = self.x_gps_relative, self.y_gps_relative
        
        # print(brain.timer.value(), self.delta_x_from_encoders, self.delta_y_from_encoders)
        # encoders_gps_fusion_alpha = 0.1

        # If we are close to our target position then say that we reached our target state (useful for autonomous mode)
        # if ((abs(self.state["x_pos"] - self.target_state["x_pos"]) < self.position_tolerance and abs(self.state["y_pos"] - self.target_state["y_pos"]) < self.position_tolerance)         #     or (self.state["override_velocity_y"] != None or self.state["override_velocity_x"] != None)):
        
        tolerance = self["tolerance"]
        target_state_theta_tolerance = 1.8

        # if self.shoot_disc <= 0 and self["roller_and_intake_motor_1_done"] aznd self["roller_and_intake_motor_2_done"] and ((abs(self["x_pos"] - self.target_state["x_pos"]) < target_state_position_tolerance and abs(self["y_pos"] - self.target_state["y_pos"]) < target_state_position_tolerance and abs(self["theta"] - self.target_state["theta"]) < target_state_theta_tolerance) or (False)):
        # distance_to_target_point = sqrt((self["x_pos"] - self.target_state["x_pos"]) ** 2 + (self["y_pos"] - self.target_state["y_pos"]) ** 2)
        # if self.shoot_disc <= 0 and self["roller_and_intake_motor_1_done"] and self["roller_and_intake_motor_2_done"] and distance_to_target_point < target_state_position_tolerance and abs(self["theta"] - self.target_state["theta"]) < target_state_theta_tolerance:

        # Get the torque on the flywheel
        self["flywheel_torque"] = flywheel_motor.torque()
        
        # Figure out if the disc has been shot by seeing if there is a large change in the flywheel torque
        if (self["flywheel_torque"] - self.previous_state["flywheel_torque"]) > 0.022 and self.is_shooting and self.limit_switch_active:
            if self.flywheel_recovery_timer.value() > 0.35:
                self.flywheel_recovery_timer.reset()
            self.state["disc_shot"] = True
        else:
            self.state["disc_shot"] = False

        # If the roller was sent to false BEFORE
        self["roller_and_intake_motor_1_done"] = roller_and_intake_motor_1.is_done()
        self["roller_and_intake_motor_2_done"] = roller_and_intake_motor_2.is_done()
        
        if self["roller_and_intake_motor_1_done"] and self["roller_and_intake_motor_2_done"] and not self.previous_state["roller_and_intake_motor_1_done"] and not self.previous_state["roller_and_intake_motor_2_done"]:
            self["roller_spin_for"] = 0
        
        # Turret 
        self["turret_theta"] = self.turret_centered_angle + self.compute_turret_ticks_to_angle(turret_motor.position())
        self.turret_theta = self["turret_theta"]

        # Calculate the target goal if we're on the neutral team, the goal thats the closest
        goals = []

        if self.team_color == "neutral":
            
            goals.append(self.blue_goal)
            goals.append(self.red_goal)
                    
            # Compute the distance to the goal
            distances_to_goal = [self.compute_distance_to_goal(goal) for goal in goals]

            if goals[argmin(distances_to_goal)] != self.target_goal:
                print("Distance to red goal", distances_to_goal[1], "Distance to blue goal", distances_to_goal[0])
                print("WE ARE SWITCHING GOALS NEW GOAL IS", "RED" if self.target_goal == self.blue_goal else "BLUE")
            

            # Make target goal the closest goal
            self.target_goal = goals[argmin(distances_to_goal)]

        elif self.team_color == "blue":
            self.target_goal = self.blue_goal
        
        elif self.team_color == "red":
            self.target_goal = self.red_goal
        
        self.turret_torque = turret_motor.torque()
        self.state["turret_torque"] = self.turret_torque

        # if (not self.turret_initialized) and turret_limit_switch.value() == 1:
        #     self.turret_limit_switch_timer.reset()
        # print("turret initialized" if self.turret_initialized else "not initialized")
        # If the turret is not initialized and we clicked the limit switch then set that to 0
        if (not self.turret_initialized) and (turret_limit_switch.value() == 0):
            turret_motor.set_position(0, DEGREES)

            self.turret_initialized = True

        self.max_turret_torque = max(self.max_turret_torque, self.turret_torque) 
        # print((self.turret_torque - self.previous_state["turret_torque"]), self.previous_state["turret_torque"], self.turret_torque, self.max_turret_torque)
        # Using the check_if_intersect function, see if the robot passes the line segment that makes up the target position
        line1 = [self.previous_state["x_pos"], self.previous_state["y_pos"], self.state["x_pos"], (self.state["y_pos"] )]
        line1b = [self.previous_state["x_pos"], self.previous_state["y_pos"], 2 * (self.state["x_pos"] - self.previous_state["x_pos"]), 2 * (self.state["y_pos"] - self.previous_state["y_pos"])]
        line1c = [self.previous_state["x_pos"] - 0.2 * cos(self.theta * DEG_TO_RAD), self.previous_state["y_pos"] - 0.2, 2 * (self.state["x_pos"] - self.previous_state["x_pos"]) + 0.2, 2 * (self.state["y_pos"] - self.previous_state["y_pos"]) + 0.2]
        line1d = [self.previous_state["x_pos"] + 0.2, self.previous_state["y_pos"] - 0.2, 2 * self.state["x_pos"] - self.previous_state["x_pos"] - 0.2, 2 * self.state["y_pos"] - self.previous_state["y_pos"] + 0.2]

        line2 = [self.target_state["x_pos"] - cos(self.theta * DEG_TO_RAD) * tolerance, self.target_state["y_pos"] - sin(self.theta * DEG_TO_RAD) * tolerance, self.target_state["x_pos"] + cos(self.theta * DEG_TO_RAD) * tolerance, self.target_state["y_pos"] + sin(self.theta * DEG_TO_RAD) * tolerance]
        is_intersecting = check_intersection(line1, line2) or check_intersection(line1b, line2) or check_intersection(line1d, line2) or check_intersection(line1c, line2) 
        is_close_enough = sqrt((self.x_pos - self.target_state["x_pos"]) ** 2 + (self.y_pos - self.target_state["y_pos"]) ** 2) < self.compute_tolerance_for_min_velocity(self["min_velocity"])
        rollers_done = self["roller_and_intake_motor_1_done"] and self["roller_and_intake_motor_2_done"]
        # passed_target_rotation = (self.previous_state["theta"] < self.target_state["theta"] and self.theta > self.target_state["theta"]) or (self.previous_state["theta"] > self.target_state["theta"] and self.theta < self.target_state["theta"])
        passed_target_rotation = False
        self["flywheel_instantaneous_speed"] = flywheel_motor.velocity(PERCENT)
        if (is_intersecting or is_close_enough) and (abs(self.theta - self.target_state["theta"]) < target_state_theta_tolerance or passed_target_rotation) and rollers_done and self.shoot_disc <= 0 and (turret_motor.is_done() or abs(self["turret_theta"] - self.target_state["turret_theta"]) < 0.5):
            self.target_reached = True

    def position_update(self):
        '''
        Updates the position of the robot
        '''
        
        # Find out how much the robot needs to move in each direction
        delta_x = 0
        delta_y = 0
        delta_theta = 0

        # # If the target state does not exist then we don't need to move in that direction
        if self.target_state["x_pos"] != None:
            delta_x = self.target_state["x_pos"] - self.x_pos
        
        if self.target_state["y_pos"] != None:
            delta_y = self.target_state["y_pos"] - self.y_pos

        if self.target_state["shoot_at_theta"] != None:        
            # turret_theta_range is the entire theta that the turret can sweep, half of that is the amount that the turret can turn right or left, and we multiply that by a constant so that there is some room for error
            
            if abs(self.theta - self.target_state["shoot_at_theta"]) < self.turret_theta_range / 2 * 0.5: 
                delta_theta = self.target_state["shoot_at_theta"] - self.theta
            else:
                delta_theta = 0.0

        if self.target_state["theta"] != None and self.target_state["shoot_at_theta"] == None:
            delta_theta = self.target_state["theta"] - self.theta

        # if self.target_state["x_pos"] != None:
        #     delta_x = self.target_state["x_pos"] - self.x_gps_relative
        
        # if self.target_state["y_pos"] != None:
        #     delta_y = self.target_state["y_pos"] - self.y_gps_relative

        # if self.target_state["theta"] != None:
        #     delta_theta = self.target_state["theta"] - self.theta
        
        # Turn via the shortest path
        delta_theta = closest_angle(delta_theta)
        
        orientation_tolerance = 8
        position_tolerance = 14

        max_velocity = 100

        # Make sqrt the delta theta, so that the tolerance is not linear but a sqrt relationshup
        delta_theta = math.sqrt(abs(delta_theta)) * sign(delta_theta) 

        # angle to target
        ang = atan2(delta_y, delta_x)
        distance = sqrt(delta_x**2 + delta_y**2)

        # target_total_vel = (max_velocity + (k_p * ((self["tolerance"]/k_p) - (min(abs(distance), self["tolerance"]/k_p))) / self["tolerance"]) * (self["min_velocity"] - max_velocity)) * (self.autonomous_speed / 100 if self.running_autonomous else 1)

        target_total_vel = (max_velocity + (((self["slow_down_distance"]) - (min(abs(distance), self["slow_down_distance"]))) / self["slow_down_distance"]) * (self["min_velocity"] - max_velocity)) * (self.autonomous_speed / 100 if self.running_autonomous else 1)

        # This line will make it so that when we tell the robot to go diagonally, it will acutally go faster in the diagonal direction to componsate for our drivetrain being shifter. Without this line, driving the robot at 50 in the x and y direction won't make it dravel 50 percent of the maximum velocity, but only 50 / sqrt(2) times the maximum velocity. this code corrects for that
        # target_total_vel *= (sin(ang % (pi/2)) + cos(ang % (pi/2)))
        
        target_x_vel = target_total_vel * cos(ang)
        target_y_vel = target_total_vel * sin(ang)
        
        target_x_vel = clamp(target_x_vel, max_velocity, -max_velocity) * (self.autonomous_speed / 100 if self.running_autonomous else 1)
        target_y_vel = clamp(target_y_vel, max_velocity, -max_velocity) * (self.autonomous_speed / 100 if self.running_autonomous else 1)

        target_theta_vel = ((clamp(delta_theta, orientation_tolerance, -orientation_tolerance) * max_velocity / (orientation_tolerance - 2.25)) ** 2) / max_velocity * sign(delta_theta) * (1 if self.running_autonomous else (-1 if programming_chassis else 1))
        
        target_theta_vel = clamp(target_theta_vel, max_velocity, -max_velocity)
        # print("estimate_state", delta_theta ** 2 * sign(delta_theta), r.theta, self.target_state["theta"],  target_theta_vel)
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
            target_theta_vel = target_theta_vel / 3
        
        if max_velocity - abs(target_theta_vel) < abs(target_x_vel) + abs(target_y_vel) and self.running_autonomous:
            # print("previous X, y, theta", target_x_vel, target_y_vel, target_theta_vel)
            target_x_vel = (max_velocity - abs(target_theta_vel)) * target_x_vel / (abs(target_x_vel) + abs(target_y_vel))
            target_y_vel = (max_velocity - abs(target_theta_vel)) * target_y_vel / (abs(target_x_vel) + abs(target_y_vel))
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
        # (self.max_acceleration / self.max_velocity * max_velocity) means the maximum acceleration of our robot IN A PERCENTAGE (i.e. 10 percent per second, 50 percent per second, -10 percent, per second)
        # (self.max_acceleration / self.max_velocity * max_velocity) * self.delta_time is the maximum change in velocity of the robot FOR THIS RUN THROUGH THE LOOP (by multipling by delta time we get how long its been since the last loop, which means we can change our velocity by that change in time time acceleration), Ex. if its been 0.1 seconds since the last loop, then our change in velocity can be 10 percent, but if it was 1 second then our change in velocity can be max_velocity percent beceause its been such a long time (if you still don't understand then learn what integration is)
        # the goal of the following lines is to clamp the change in velocity to not go over some number (max_acceleration),
        # so in order to do that we clamp motor_target_velocity between the motor_current_velocity plus the maximum change in velocity for this run of the loop, repeat for each wheel
        left_motor_a_target_velocity = clamp(left_motor_a_target_velocity, (max_acceleration_left_motor_a / self.max_velocity * max_velocity) * self.delta_time + left_motor_a.velocity(
            PERCENT), -(max_acceleration_left_motor_a / self.max_velocity * max_velocity) * self.delta_time + left_motor_a.velocity(PERCENT))
        right_motor_a_target_velocity = clamp(right_motor_a_target_velocity, (max_acceleration_right_motor_a / self.max_velocity * max_velocity) * self.delta_time + right_motor_a.velocity(
            PERCENT), -(max_acceleration_right_motor_a / self.max_velocity * max_velocity) * self.delta_time + right_motor_a.velocity(PERCENT))
        left_motor_b_target_velocity = clamp(left_motor_b_target_velocity, (max_acceleration_left_motor_b / self.max_velocity * max_velocity) * self.delta_time + left_motor_b.velocity(
            PERCENT), -(max_acceleration_left_motor_b / self.max_velocity * max_velocity) * self.delta_time + left_motor_b.velocity(PERCENT))
        right_motor_b_target_velocity = clamp(right_motor_b_target_velocity, (max_acceleration_right_motor_b / self.max_velocity * max_velocity) * self.delta_time + right_motor_b.velocity(
            PERCENT), -(max_acceleration_right_motor_b / self.max_velocity * max_velocity) * self.delta_time + right_motor_b.velocity(PERCENT))
        
        # Accelerate the motors to the target velocity.
        left_motor_a.spin(FORWARD, left_motor_a_target_velocity, PERCENT)
        right_motor_a.spin(FORWARD, right_motor_a_target_velocity, PERCENT)
        left_motor_b.spin(FORWARD, left_motor_b_target_velocity, PERCENT)
        right_motor_b.spin(FORWARD, right_motor_b_target_velocity, PERCENT)

    def flywheel_update(self):
        self.flywheel_speed = self["flywheel_speed"] # This is the target flywheel speed
        # See if we need to shoot a disc
        if self["shoot_disc"] > 0 and not self.is_shooting:
            Thread(self.index_disc)

        MAX_VOLTAGE = 10.9 # I don't think this is the true maximum voltage btw
        
        # This so for the derivative term so that it isn't working with the chaotic value of flywheel speed that fluctuates rapidly, but the averaged flywheel speed
        speed_alpha = 5
        speed_alpha = clamp(speed_alpha * self.delta_time, 1, 0)

        self.flywheel_avg_speed = flywheel_motor.velocity(PERCENT) * speed_alpha + self.flywheel_avg_speed * (1 - speed_alpha)
        self.flywheel_instantaneous_speed = flywheel_motor.velocity(PERCENT)

        if self.flywheel_speed == 0:
            self.flywheel_motor_average_output = 0

        self.flywheel_kP = 0.6 # before aws 0.03
        self.flywheel_kI = 0.0004 / 0.022 # before was 0.0004 (this worked pretty well)
        self.flywheel_kD = 0.03 * 0.022

        flywheel_recovery = 0.4

        # Boolean to determine if the flywheel is recovering from a disc being shot
        flywheel_recovery_mode = (self.flywheel_recovery_timer.value()) < (flywheel_recovery * (sqrt((self.flywheel_speed if self.flywheel_speed > 0 else 0) / 60))) and self.is_shooting

        # If we are shooting then do NOT increment the integral term and greatly increase the proportional term so that we can increase our speed much faster
        if flywheel_recovery_mode:
            # self.flywheel_recovery_timer.reset()
            self.flywheel_kP = 2.5 # before aws 0.03
            self.flywheel_kI = 0 # before was 0.0004 (this worked pretty well)
            self.flywheel_kD = 0

        proportional_term_flywheel = self.flywheel_kP * (self.flywheel_speed - flywheel_motor.velocity(PERCENT))
        derivative_term_flywheel = self.flywheel_kD * (self.flywheel_avg_speed - self.previous_flywheel_avg_speed) / self.delta_time
        # derivative_term_flywheel = clamp(derivative_term_flywheel, 1, -1)

        self.integral_term_flywheel += (self.flywheel_speed - self.previous_flywheel_speed) * self.flywheel_kF

        # If the flywheel is between 10 percent of its target speed then start integrating
        if abs(self.flywheel_speed - flywheel_motor.velocity(PERCENT)) < 20:
            self.integral_term_flywheel += self.delta_time * self.flywheel_kI * (self.flywheel_speed - self.flywheel_avg_speed)

            # Clamp the intergral term so it doesn't grow to be too extreme
            self.integral_term_flywheel = clamp(self.integral_term_flywheel, MAX_VOLTAGE, -MAX_VOLTAGE) 

        if flywheel_recovery_mode:
            target_flywheel_output = self.integral_term_flywheel + proportional_term_flywheel - derivative_term_flywheel
        else:
            output_alpha = 1 * 10 # CHANGEDI
            output_alpha = clamp(output_alpha * self.delta_time, 1, 0)

            self.average_target_flywheel_output = self.average_target_flywheel_output * (1-output_alpha) + (self.integral_term_flywheel + proportional_term_flywheel - derivative_term_flywheel) * output_alpha  
        
            target_flywheel_output = self.average_target_flywheel_output
        
        if self.flywheel_speed == 0:
            target_flywheel_output = 0
        

        flywheel_motor.spin(FORWARD, clamp(target_flywheel_output, MAX_VOLTAGE, -MAX_VOLTAGE), VOLT)

        # print(brain.timer.value(), self.flywheel_speed, flywheel_motor.velocity(PERCENT), self.average_target_flywheel_output, proportional_term_flywheel, self.integral_term_flywheel, derivative_term_flywheel)
        self.flywheel_motor_error = (self.flywheel_avg_speed - self.previous_flywheel_avg_speed)

        # Compute derivative of average flywheel speed
        self.previous_flywheel_avg_speed = self.flywheel_avg_speed

        self.previous_flywheel_speed = float(self.flywheel_speed)

        self.derivative_term_flywheel = derivative_term_flywheel

        self.proportional_term_flywheel = proportional_term_flywheel

    def automatic_update(self):
        # if True: # self["auto_flywheel_speed"]:
        #     distance_to_goal = self.compute_distance_to_goal(self.target_goal)
        #     self["flywheel_speed"] = self.compute_flywheel_speed(distance_to_goal)

        if self["auto_orientate"]:            
            pass

        if self["aimbot"]: # and self.aimbot_trust >= 100:
            x_pos_of_goal = self.target_goal.x_pos
            y_pos_of_goal = self.target_goal.y_pos

            x_pos_of_robot = self.x_from_gps
            y_pos_of_robot = self.y_from_gps

            # Forsee the position of the goals based on the current velocity of the robot
            if False:
                time_until_disc_gets_shot = 0.1
                y_pos_of_robot += self.x_vel * time_until_disc_gets_shot
                x_pos_of_robot += self.y_vet * time_until_disc_gets_shot

            distance_to_goal = self.compute_distance_to_goal((x_pos_of_goal, y_pos_of_goal))
            self["flywheel_speed"] = self.compute_flywheel_speed(distance_to_goal)
            
            # In order to get the angle to an object take the atan2 of the d_x, d_y, then subtract the theta offset to convert it back to the robots local coordinate frame
            angle = (get_angle_to_object((self.x_from_gps, self.y_from_gps), (x_pos_of_goal, y_pos_of_goal)) - self.initial_theta_field)
            # print("we aimbotting distance", distance_to_goal, "angle", angle)
            
            new_turret_angle = closest_angle(self.theta - angle)

            # The higher the speed the more the disc turns in its trajectory, so compensate for that
            # some code here

            self.target_state["turret_theta"] = -(new_turret_angle - self.flywheel_angle_offset)
        else:
            # if we are not doing aimbot, just make the turret shoot the disc straight
            self.target_state["turret_theta"] = self.flywheel_angle_offset

    def turret_update(self):
        
        # turret_motor.spin(FORWARD, turret_theta_vel, PERCENT)
        # return
        # print("turret_initialized", self.turret_initialized)
        
        if not self.turret_initialized:
            # print("not intitialized turret update")
            turret_motor.spin(REVERSE, -5, PERCENT)
            return

        turret_target_theta = (self.target_state["turret_theta"])

        # since the turret centered position is the turret when it is centered, if we want the turret to go to its 0 position (all the way to the right), it would be set to 81.6 degrees)
        turret_target_theta = self.turret_centered_angle - turret_target_theta
        
        turret_target_theta = clamp(turret_target_theta, self.turret_far_angle, 0) 
        
        slow_down_distance = 10 # degrees
        min_turret_speed = 20 # percent
        max_speed = 50

        turret_speed = ((max_speed - min_turret_speed) / slow_down_distance * clamp(abs(self.compute_turret_ticks_to_angle(turret_motor.position(DEGREES)) - turret_target_theta), slow_down_distance, 0) + min_turret_speed) * max_speed / 100
        
        turret_motor.spin_to_position(-self.compute_angle_to_turret_ticks(turret_target_theta), RotationUnits.DEG, turret_speed, PERCENT, False)

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
                if (self["roller_state"] != "none" and self.team_color == "blue"):
                    self["roller_speed"] = 80

                # This is a way to indicate to the drivers that we are done spinning the roller
                if (self["roller_state"] == "red" and self.team_color == "red") or (self["roller_state"] == "blue" and self.team_color == "blue"):
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
            roller_and_intake_motor_1.spin_for(FORWARD, self["roller_spin_for"] * 285.0, DEGREES, False)
            roller_and_intake_motor_2.spin_for(REVERSE, self["roller_spin_for"] * 285.0, DEGREES, False)

            roller_and_intake_motor_1.set_velocity(-100)
            roller_and_intake_motor_2.set_velocity(100)

            self["roller_and_intake_motor_1_done"] = False
            self["roller_and_intake_motor_2_done"] = False

            self["roller_spin_for"] = 0

    def expansion_update(self):
        '''
        Launch the expansion
        '''
        if self.target_state["launch_expansion"]:
            expansion.open()
        else:
            expansion.close()

    def status_update(self):
        
        # Indicate to the drivers that the flywheel is ready to be shot
        if abs(flywheel_motor.velocity(PERCENT) - self.flywheel_speed) < 2: 
            flywheel_status_light.on()
        else:
            flywheel_status_light.off()

    
    #endregion 

    #region AUTO
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

        timeout_timer = Timer()
        timeout_time = 999
        self["override_velocity_x"] = None
        self["override_velocity_y"] = None
        self["override_velocity_theta"] = None
        # print(type(self.autonomous_procedure))
        # Modify the states
        for step in self.autonomous_procedure:
            if "position_type" in step:
                if step["position_type"] == "field":
                    # Convert the position to be from the fields perspective to the robots perspective
                    if "x_pos" and "y_pos" in step:
                        print("Position is", self.initial_x_field, self.initial_y_field, self.initial_theta_field)
                        print("Converting ", step["x_pos"], step["y_pos"], " to ", self.convert_field_to_local_coordinates(step["x_pos"], step["y_pos"]))

                        step["x_pos"], step["y_pos"] = self.convert_field_to_local_coordinates(step["x_pos"], step["y_pos"])
                    else:
                        raise Exception("Robot->run_autonomous(): x_pos and y_pos is not in step")
            
            if "min_velocity" not in step:
                step["min_velocity"] = 0
            
            if "tolerance" not in step:
                step["tolerance"] = self.compute_tolerance_for_min_velocity(step["min_velocity"])
            
            if "slow_down_distance" not in step:
                step["slow_down_distance"] = self.compute_slow_down_distance_for_min_velocity(step["min_velocity"])

        try:

            # Run the autonomous procedure
            for step in self.autonomous_procedure:
                target_state = {}
                self["override_velocity_x"] = None
                self["override_velocity_y"] = None
                self["override_velocity_theta"] = None

                if "timeout" in step.keys():
                    timeout_timer.reset()
                    timeout_time = step["timeout"]
                else:
                    timeout_time = 999

                if "set_x" in step:
                    self.x_pos = step["set_x"]
                if "set_y" in step:
                    self.y_pos = step["set_y"]
                if "set_theta" in step:
                    self.total_theta = step["set_theta"]
                
                for key in step.keys():
                    if key in self.state.keys():
                        target_state[key] = step[key]

                if "message" in step.keys():
                    self.print(step["message"])

                if "funcs" in step.keys():
                    for func in step["funcs"]:
                        func()

                self.set_target_state(target_state)

                if "wait" in step.keys():
                    print("Waiting for", step["wait"], "seconds")
                    wait(step["wait"], SECONDS)

                # While we haven't reached the target state then just wait
                while (not self.target_reached) and (timeout_timer.value() < timeout_time):
                    # print("WE ARE WAITING...", self.target_state["x_pos"], self.x_pos, self.target_state["y_pos"], self.y_pos, self.target_state["theta"], self.total_theta, self["override_velocity_x"])
                    wait(0.05, SECONDS)
        except Exception as e:
            # show that there was an exception and draw something on the screen
            brain.screen.draw_rectangle(0,0,100,100, Color.PURPLE)
            print("\n\nThere was an exception in the auto!\n\n", e)
        print("DONE WITH AUTONOMOUS")

        # After the autonomous mode is over then set the target state to the robot's initial state (so we don't move and turn everything off)
        self.set_target_state(self.initial_state)
        self.running_autonomous = False

    def set_autonomous_procedure(self, procedure):
        self.autonomous_procedure = procedure

    
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

    def convert_field_to_local_coordinates(self, _x, _y):
        return rotate_vector_2d(_x - self.initial_x_field, _y - self.initial_y_field, self.initial_theta_field * DEG_TO_RAD)
    
    def compute_tolerance_for_min_velocity(self, _min_velocity):
        max_tolerance = 7
        min_tolerance = 2
        return (max_tolerance - min_tolerance) * (_min_velocity / 100) + min_tolerance

    def compute_slow_down_distance_for_min_velocity(self, _min_velocity):
        max_slow_down_distance = 20
        min_slow_down_distance = 10
        return (max_slow_down_distance - min_slow_down_distance) * (_min_velocity / 100) + min_slow_down_distance
    
    def compute_distance_to_goal(self, goal):
        if type(goal) == tuple or type(goal) == list:
            delta_x = goal[0] - self.x_pos
            delta_y = goal[1] - self.y_pos
        else:
            delta_x = goal.x_pos - self.x_pos
            delta_y = goal.y_pos - self.y_pos
    
        return sqrt(delta_x**2 + delta_y**2)

    def compute_flywheel_speed(self, distance):
        # Linearly interpolate between two closest distances

        # If the distance is less than the minimum distance then just return the minimum speed
        # print("compute_flywheel_speed", self.distance_speed_maps.keys())

        if distance < min((self.distance_speed_maps.keys())):
            return self.distance_speed_maps.get(min(self.distance_speed_maps.keys()))

        # If the distance is greater than the maximum distance then use the last 2 distances to 
        if distance > max((self.distance_speed_maps.keys())):
            return interpolate(distance, (list(self.distance_speed_maps.keys())[-1]), (list(self.distance_speed_maps.keys())[-2]), self.distance_speed_maps.get(list(self.distance_speed_maps.keys())[-2]), self.distance_speed_maps.get(list(self.distance_speed_maps.keys())[-2]))

        # Otherwise, find the two closest distances and interpolate between them
        smaller_distance = max([key for key in self.distance_speed_maps.keys() if key < distance])
        larger_distance = min([key for key in self.distance_speed_maps.keys() if key > distance])

        return interpolate(distance, smaller_distance, larger_distance, self.distance_speed_maps.get(smaller_distance), self.distance_speed_maps.get(larger_distance))

    def compute_angle_to_turret_ticks(self, angle):
        # 1200 ticks per rotation of the gear, so 1200 / 360 is the ticks per degree
        turret_gear_ratio = 84 / 36
        return angle * turret_gear_ratio

    def compute_turret_ticks_to_angle(self, turret_ticks):
        turret_gear_ratio = 84 / 36
        return turret_ticks / turret_gear_ratio
        

    #region Properties getters and setters (dunders included)
    
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
    
    #endregion

    #region MISC/UTIL
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
        
        while not self["disc_shot"] and t.time() < 1000:
            wait(0.01, SECONDS)

        indexer.close()

        t.reset()
        
        self.limit_switch_active = False

        # wait for the limit switch to be swticehd
        while not (indexer_limit_switch.value() == 0) and t.time() < 1000:
            wait(0.01, SECONDS)
        
        self.limit_switch_active = True

        self.shoot_disc -= 1
        self.is_shooting = False
  

    def get_current_wheel_encoder_values(self):
        '''
        Returns a vector of all of the encoder values
        '''
        return Vector([left_motor_a.position(DEGREES), right_motor_a.position(DEGREES), left_motor_b.position(DEGREES), right_motor_b.position(DEGREES)])

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

        constants = [
            "drone_mode",
            "slow_mode",
            "using_gps",
            "intake_speed",
            "roller_speed",
            "auto_intake",
            "auto_roller",
            "flywheel_speed",
            "shoot_disc",
            "roller_spin_for",
            "autonomous_speed",
            "min_velocity",
            "tolerance",
            "slow_down_distance",
            "aimbot",     
        ]

        for constant in constants:
            if constant in _state:
                self[constant] = _state[constant]

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
            self.team_color = "red"
            return
        elif _color == "blue":
            self.team_color = "blue"
            return
        else:
            self.team_color = "neutral"
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


#endregion

######## COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ### COMPETITION FUNCTIONS ###
def autonomous():
    if not r.initialized:
        r.init()
    r.run_autonomous()
    r.stop_moving()

def driver_control():
    
    # Initialize the robot
    if not r.initialized:
        r.init()

    #region Timers
    global serial_output_delay

    output_data_timer = Timer()
    output_data_timer.reset()

    reset_theta_timer = Timer()
    reset_theta_timer.reset()

    controller_output_timer = Timer()
    controller_output_timer.reset()

    # Reset the drivercontrolled and auto timers so that we can keep track and know how long the driver_controlled as been running for
    r.driver_controlled_timer.reset()
    
    r.autonomous_timer.reset()

    # Timer for expansion so we don't expand too early
    expansion_timer = Timer()
    expansion_timer.reset()
    #endregion
    
    #region Previous state declaration
    # A dictionary that stores the previous button states so that we can reference previous states without having to make a new variable every time 
    previous_controller_1_states = {
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
    
    left_motor_a.set_stopping(HOLD)
    left_motor_b.set_stopping(HOLD)
    right_motor_a.set_stopping(HOLD)
    right_motor_b.set_stopping(HOLD)
    
    #endregion

    previous_flywheel_speed = 0

    # Timer that allows us to keep track for how long we have been pressing the aimbot button for to make the robot move more as time goes on
    aimbot_timer = Timer() 

    gui.set_page(0)

    while True:

        if r.running_autonomous:
            continue
        
        #region Output data to console/controllers
        # Timer to print things out to the terminal every x seconds
        if (output_data_timer.value() > serial_output_delay):
            # print("outputting")
            Thread(output_data)
            output_data_timer.reset()
        
        # Display data to the controllers
        if controller_output_timer.value() > 0.2:
            controller_1.screen.clear_row(3)
            controller_1.screen.print(f("FLY:", round(r.flywheel_speed,1), round(r.theta, 1), r.turret_theta))
            controller_2.screen.clear_row(3)
            controller_2.screen.print(f("FLY:", r.flywheel_speed, "ang:", round(r.theta)))
            controller_output_timer.reset()
        #endregion
        
        #region Drive robot variables
        new_theta = ((controller_1.axis1.position())) ** 2 * sign(controller_1.axis1.position()) / 100 * (0.7 if recording_autonomous else 1)
        new_theta *= (-1 if programming_chassis else 1)
        global turret_theta_vel

        new_x = ((controller_1.axis4.position())) ** 2 * sign(controller_1.axis4.position()) / 100 * (0.25 if recording_autonomous else 1)
        new_y = ((controller_1.axis3.position())) ** 2 * sign(controller_1.axis3.position()) / 100 * (0.25 if recording_autonomous else 1)
        
        mag_of_velocity = sqrt(new_x ** 2 + new_y ** 2)
        ang_of_velocity_vector = atan2(new_y, new_x)

        # Comenstae x andd y velocity vectors for wierd robot shape
        new_x = cos(ang_of_velocity_vector) * mag_of_velocity * (sin(ang_of_velocity_vector % (pi/2)) + cos(ang_of_velocity_vector % (pi/2)))
        new_y = sin(ang_of_velocity_vector) * mag_of_velocity * (sin(ang_of_velocity_vector % (pi/2)) + cos(ang_of_velocity_vector % (pi/2)))

        if new_x + new_y > 100:
            new_x = new_x / (new_x + new_y) * 100
            new_y = new_y / (new_x + new_y) * 100

        new_intake = clamp(controller_2.buttonL1.pressing() * 100 - controller_2.buttonR1.pressing() * 100 + controller_1.buttonLeft.pressing() * 100, 100, -100)
        
        # turret_theta = r.target_state["turret_theta"] + controller_1.axis2.position() * 0.005
        # turret_theta = closest_angle(-r["theta"])

        #endregion
        
        #region Record autonomous
        if recording_autonomous:
            new_theta = None
        
        
        if controller_1.buttonL1.pressing() and not previous_controller_1_states["buttonL1"]:
            r.path.append(r.return_state_of_auto())


        if controller_1.buttonRight.pressing():
            new_theta = None
            reset_robot_theta()

        #endregion
        #region Auto orient feature
        auto_orientate_dictionary = {
          0 : controller_1.buttonX.pressing(),
        #   -90 : controller_1.buttonY.pressing(),
        #   90 : controller_1.buttonA.pressing(),
          180 : controller_1.buttonB.pressing(),
        }

        auto_orientate_total_button_pressed_count = int(controller_1.buttonX.pressing()) + int(controller_1.buttonY.pressing()) + int(controller_1.buttonA.pressing()) + int(controller_1.buttonB.pressing())
        auto_orientate_target_theta = None
        
        if auto_orientate_total_button_pressed_count > 0:
            auto_orientate_target_theta = 0
            for key in auto_orientate_dictionary:
                if auto_orientate_dictionary[key]:
                    auto_orientate_target_theta += key
            
            auto_orientate_target_theta /= auto_orientate_total_button_pressed_count

        if controller_1.buttonB.pressing():
            auto_orientate_target_theta = -135
        
        if auto_orientate_target_theta != None:
            new_theta = None
            r.set_target_state({
                "theta" : auto_orientate_target_theta
            })

        #endregion

        #region Roller spin for specific distance
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
        
        #endregion

        #region Aimbot

        # if controller_1.buttonUp.pressing() and r.using_gps and gps.quality() >= 100:

        # if we hit the button toglge this thing
        if controller_1.buttonUp.pressing() and not previous_controller_1_states["buttonUp"]:
            r.set_target_state({
                "aimbot" : not r.target_state["aimbot"]
            })
            print("setting aimbot to", r.target_state["aimbot"])        
        ### CAMERA AIMBOT
        # If the driver is pressing the left button, then turn on aimbot
        # if controller_1.buttonLeft.pressing():
        #     if not previous_controller_states["buttonLeft"]:
        #         aimbot_timer.reset()

        #     # Aimbot for robot
        #     disc = vision.take_snapshot(5)
            
        #     # Vision sensor objects
        #     blue_goal = vision.take_snapshot(3)
            
        #     # Red goal goes not exist yet
        #     red_goal = vision.take_snapshot(4)

        #     # print(blue_goal, red_goal)

        #     if disc:
        #         goal = disc
        #         #! THIS MEANS WE WILL SHOOT AT ANY TEAM'S COLOR
                
        #         # Size of vision sensor screen is 212 vertical and 316 horizontal
        #         kP_for_aimbot = 20
        #         scale_time_value = 1/5

        #         # scale the values from [-1,1] and multiply by 30 as the kP, and then multiply by the sqrt of how long we've been holding the button for (to simulate a kI)
        #         delta_theta = kP_for_aimbot * ((float(goal[0].centerX) - 316/2) / (316/2)) * sqrt(1 + scale_time_value * aimbot_timer.value())
        #         print("delta theta", delta_theta)
        #         new_theta += delta_theta

        #endregion

        #region Reset flywheel speed
        if (controller_1.buttonDown.pressing() and not previous_controller_1_states["buttonDown"]):
            r.set_target_state({
                "flywheel_speed": previous_flywheel_speed
            })

        if controller_2.buttonX.pressing() and not previous_controller_states_2["buttonX"]:
            previous_flywheel_speed = r.flywheel_speed
            r.set_target_state(
                {
                    "flywheel_speed" : 0,
                }
            )
        
        #endregion
        
        #region Update robot target state
        # Update the target state of the robot so that in its update() function it does everything there
        r.set_target_state(
            {
                "override_velocity_x" : new_x,
                "override_velocity_y" : new_y,
                "override_velocity_theta" : new_theta,
                "slow_mode" : controller_1.buttonL1.pressing(),
                "intake_speed" : new_intake,
                # "turret_theta" : turret_theta,
            }
        )
        #endregion

        #region Control expansion
        
        if controller_2.buttonUp.pressing(): # and expansion_timer.value() < (60 + 45 - 11):
            r.set_target_state({
                "launch_expansion" : True
            })
        elif controller_2.buttonDown.pressing():
            r.set_target_state({
                "launch_expansion" : False
            })
            expansion.close()

        if controller_1.buttonR1.pressing():
            r.set_target_state({
                "shoot_disc": 1,
            })

        #endregion
        
        #region Update Flywheel Speed
        # When the buttons are pressed to change the level of the flywheel speed, r2 decreases level, l2 increases level
        if (controller_1.buttonR2.pressing() and not previous_controller_1_states["buttonR2"]) or (controller_2.buttonR2.pressing() and not previous_controller_states_2["buttonR2"]):
            temp_copy = r.flywheel_speed_levels.copy()
            if r.flywheel_speed not in temp_copy:
                temp_copy.append(r.flywheel_speed)
            temp_copy.sort()
            new_flywheel_speed = temp_copy[min(temp_copy.index(r.flywheel_speed)+1,len(temp_copy)-1)]
            r.set_target_state({
                # "flywheel_speed" : min(r.flywheel_speed + 2, 100)
                "flywheel_speed" : new_flywheel_speed
            })
            print("setting flywheel speed to", new_flywheel_speed)

        elif (controller_1.buttonL2.pressing() and not previous_controller_1_states["buttonL2"]) or (controller_2.buttonL2.pressing() and not previous_controller_states_2["buttonL2"]):
            temp_copy = r.flywheel_speed_levels.copy()
            if r.flywheel_speed not in temp_copy:
                temp_copy.append(r.flywheel_speed)
            temp_copy.sort()
            new_flywheel_speed = temp_copy[max(temp_copy.index(r.flywheel_speed)-1,0)]
            r.set_target_state({                
                "flywheel_speed": new_flywheel_speed
            })
            print("setting flywheel speed to", new_flywheel_speed)
        #endregion
        
        #region end of match rumbles
        # if r.driver_controlled_timer.value() > 50 and r.driver_controlled_timer.value() < 50.1:
        #     controller_1.rumble(".")
        #     controller_2.rumble(".")
        # if r.driver_controlled_timer.value() > 57 and r.driver_controlled_timer.value() < 57.1:
        #     controller_1.rumble(".")
        #     controller_2.rumble(".")
        # if r.driver_controlled_timer.value() > 58 and r.driver_controlled_timer.value() < 58.1:
        #     controller_1.rumble(".")
        #     controller_2.rumble(".")
        # if r.driver_controlled_timer.value() > 59 and r.driver_controlled_timer.value() < 59.1:
        #     controller_1.rumble(".")
        #     controller_2.rumble(".")
        #endregion
        
        #region Update Previous Controller States
        # Update previous controller states so we can track what the controller did before, we use this so that we can see if a button was pressed, not held, etc.
        previous_controller_1_states = {
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
        #endregion
        wait(0.01, SECONDS)

#region Autonomous Programs
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

match_auto_two_squares = [
    {
        "override_velocity_theta": None,
        "drone_mode": True,
        "override_velocity_x": None,
        "override_velocity_y": None,
        "set_x": 0,
        "set_y": 0,
        "set_theta": 0,
        "autonomous_speed" : 65,
        "flywheel_speed": 65,
        "intake_speed": 100,
    },
    {
        "launch_expansion": False,
        "theta": 360.0,
        "message": "This is state #2!",
        "intake_speed": 0,
        "x_pos" : 54,
        "y_pos" : 0,
        "min_velocity" : 30,
    },
    {
        "launch_expansion": False,
        "theta": 360.0,
        "intake_speed": 0,
        "message": "This is state #2!",
        "x_pos" : 54,
        "y_pos" : -7  - 12.25,
        "timeout" : 1.75,
    },
    {
        "roller_spin_for" : 0.35,
        "timeout" : 2,
    },
    {
        "theta": 0,
        "intake_speed": 0,
        "message": "This is state #4!",
        "x_pos": 54,
        "y_pos": 21  - 13.00,
    },
    {
        "autonomous_speed" : 70,
        "theta": 13,
        "intake_speed": 0,
        "message": "This is state #4!",
        "x_pos": 54,
        "y_pos": 21  - 13.00 - 0.5,
    },
    {
        "autonomous_speed" : 65,
        'wait' : 0.75,
    },
    {
        "shoot_disc" : 1,
        "wait" : 2,
    },
    
    {
        "shoot_disc" : 1,
    },
    {
        "autonomous_speed" : 80,
        "launch_expansion": False,
        "theta": 0,
        "intake_speed": 0,
        "message": "This is state #2!",
        "x_pos": 52.0,
        "y_pos": 21.0  - 13.00,
    },

    {
        "launch_expansion": False,
        "theta": 0,
        "intake_speed": 0,
        "flywheel_speed" : 61.4,
        "message": "This is state #3!",
        "x_pos": 18,
        "y_pos": 27  - 13.00,
    },
    {
        "launch_expansion": False,
        "theta": 135,
        "intake_speed": 0,
        "message": "This is state #4!",
        "x_pos": 18,
        "y_pos": 27  - 13.00,
    },
    {
        "launch_expansion": False,
        "intake_speed": 100,
        "message": "This is state #5!",
        "x_pos": -11 + -70.0 + 43.00,
        "y_pos": 11 + 120  - 15.00 - 43.00,
    },
    {
        "theta" : 35,
    },
    {
        "wait" : 0.5,
    },
    {
        "shoot_disc" : 1,
        "wait" : 1.75,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "message" : "DONE WITH AUTONOMOUS"
    }
]

match_auto_rollers_only_easy = [

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

match_auto_three_squares = match_auto_two_squares

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

# auto_test_odometry = [
#     {
#         "override_velocity_theta": None,
#         "drone_mode": True,
#         "override_velocity_x": None,
#         "override_velocity_y": None,
#         "set_x": 0,
#         "set_y": 0,
#         "set_theta": 0,
#         "autonomous_speed" : 54,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 360,
#         "intake_speed": 0,
#         "message": "Move towards the rollers",
#         "flywheel_speed": 0,
#         "x_pos": -11,
#         "y_pos": -13,
#     },
#     {
#         "roller_spin_for" : 0.6,
#         "message" : "Doing the rollers!",
#         "wait" : 0.1,
#     },
#     {
#         "x_pos": 5,
#         "y_pos": 3,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 90,
#         "intake_speed": 0,
#         "message": "Rotating the robot!",
#         "flywheel_speed": 0,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 90,
#         "intake_speed": 100,
#         "message": "This is state #4!",
#         "flywheel_speed": 0,
#         "x_pos": 5,
#         "y_pos": 16,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 90,
#         "intake_speed": 100,
#         "message": "About to do the other roller!",
#         "flywheel_speed": 0,
#         "x_pos": -56,
#         "y_pos": 9.4,
#     },
#     {
#         "intake_speed": 0,
#         "x_pos": -67,
#     },
#     {
#         "theta" : 91,
#     },
#     {
#         "roller_spin_for" : 0.62,
#         "wait" : 0.1,
#     },
#     {
#         "theta" : 90,
#         "flywheel_speed": 53.5,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 90,
#         "intake_speed": 0,
#         "message": "This is state #3!",
#         "x_pos": 101,
#         "y_pos": 15 + 10.000,
#     },
#     {
#         "theta" : 100.5,
#     },
#     {
#         "shoot_disc" : 1,
#         "wait" : 1,
#     },
#     {
#         "shoot_disc" : 1,
#         "wait" : 0.7,
#     },
#     {
#         "shoot_disc" : 1,
#         "wait" : 0.7,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 360,
#         "intake_speed": 0,
#         "message": "This is state #1!",
#         "flywheel_speed": 0,
#         "x_pos": 100,
#         "y_pos": 6 + 10.000,
#     },
#     {
#         "launch_expansion": False,
#         "intake_speed": 0,
#         "message": "This is state #2!",
#         "flywheel_speed": 0,
#         "x_pos": 78,
#         "y_pos": 87 + 12.000000099,
#     },
#     {
#         "theta": 225.0,
#     },
#     {
#         "autonomous_speed" : 20,
#         "launch_expansion": False,
#         "theta": 225,
#         "intake_speed": 100,
#         "message": "This is state #4!",
#         "flywheel_speed": 0,
#         "x_pos": 161 + 17.000000099222,
#         "y_pos": 168 + 10.000 + 12.000000099,
#     },
#     {
#         "autonomous_speed" : 48,
#     },
#     {
#         "theta" : 180
#     },
#     {
#         "launch_expansion": False,
#         "intake_speed": 0,
#         "message": "This is state #1!",
#         "flywheel_speed": 0,
#         "x_pos": 161.0 + 17.000000099222,
#         "y_pos": 168 + 10.000,
#     },

#     {
#         "launch_expansion": False,
#         "intake_speed": 0,
#         "message": "This is state #2!",
#         "flywheel_speed": 54,
#         "x_pos": 200 + 17.000000099222,
#         "y_pos": 168 + 10.000,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 180,
#         "intake_speed": 0,
#         "message": "This is state #3!",
#         "x_pos": 200 + 17.000000099222,
#         "y_pos": 168 + 10.000,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 180,
#         "intake_speed": 0,
#         "message": "This is state #4!",
#         "x_pos": 208 + 17.000000099222,
#         "y_pos": 125 + 10.000,
#     },
#     {
#         "theta" : 192,
#     },
#     {
#         "shoot_disc" : 1,
#         "wait" : 1,
#     },
#     {
#         "shoot_disc" : 1,
#         "wait" : 1,
#     },
#     {
#         "shoot_disc" : 1,
#         "wait" : 1,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 270,
#         "intake_speed": 0,
#         "message": "This is state #1!",
#         "flywheel_speed": 0,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 270,
#         "intake_speed": 0,
#         "message": "This is state #1!",
#         "flywheel_speed": 0,
#         "x_pos": 208 + 17.000000099222,
#         "y_pos": 229,
#     },

#     {
#         "launch_expansion": False,
#         "theta": 270,
#         "intake_speed": 0,
#         "message": "This is state #2!",
#         "flywheel_speed": 0,
#         "x_pos": 238,
#         "y_pos": 229,
#     },{
#         "x_pos": 238,
#         "y_pos": 229,
#     },
#     {
#         "roller_spin_for" : 0.6,
#         "message" : "Doing the third roller!",
#         "wait" : 0.1,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 270,
#         "intake_speed": 0,
#         "message": "This is state #3!",
#         "flywheel_speed": 0,
#         "x_pos": 218,
#         "y_pos": 235,
#     },
#     {
#         "x_pos": 173,
#         "y_pos": 235,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 180,
#         "intake_speed": 100,
#         "message": "This is state #4!",
#         "flywheel_speed": 0,
#         "x_pos": 173,
#         "y_pos": 235,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 180,
#         "intake_speed": 100,
#         "message": "This is state #5!",
#         "flywheel_speed": 0,
#         "x_pos": 173,
#         "y_pos": 253,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 180,
#         "intake_speed": 0,
#         "message": "This is state #6!",
#         "flywheel_speed": 0,
#         "x_pos": 173,
#         "y_pos": 299,
#     },
#     {
#         "roller_spin_for" : 0.6,
#         "message" : "Doing the 4th roller",
#         "wait" : 0.1,
#     },
#     {
#         "x_pos": 215,
#         "y_pos": 265,
#     },
#     {
#         "launch_expansion": False,
#         "theta": 225,
#         "intake_speed": 0,
#         "message": "This is state #2!",
#         "flywheel_speed": 0,
#         "x_pos": 215,
#         "y_pos": 265,
#     },
#     {
#         "launch_expansion": True,
#         "wait" : 1,
#     },
#     {
#         "wait" : 1,
#     }
# ]
skills_auto = [
    {
        "override_velocity_theta": None,
        "drone_mode": True,
        "override_velocity_x": None,
        "override_velocity_y": None,
        "set_x": 0,
        "set_y": 0,
        "set_theta": 0,
        "autonomous_speed" : 75,
    },
    {
        "launch_expansion": False,
        "theta": 360,
        "intake_speed": 0,
        "message": "Move towards the rollers",
        "flywheel_speed": 0,
        "x_pos": -11,
        "y_pos": -10,
        "timeout" : 3,
    },
    {
        "roller_spin_for" : 0.6,
        "message" : "Doing the rollers!",
        "wait" : 0.1,
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
        "min_velocity" : 0,

    },
    {
        "launch_expansion": False,
        "theta": 90,
        "intake_speed": 100,
        "message": "This is state #4!",
        "flywheel_speed": 0,
        "x_pos": 5,
        "y_pos": 16,
    },
    {
        "launch_expansion": False,
        "theta": 90,
        "intake_speed": 100,
        "message": "About to do the other roller!",
        "flywheel_speed": 0,
        "x_pos": -60,
        "y_pos": 5,
    },
    {
        "intake_speed": 0,
        "x_pos": -72.5,
        "y_pos": 5 + 4.01231456556765674560009 - 1.5,
        "min_velocity" : 0,
        "flywheel_speed": 53,
        "timeout" : 3,
    },
    {
        "theta" : 90,
        "timeout" : 2,
    },
    {
        "roller_spin_for" : 0.62,
        "message" : "Doing the second roller",
        "wait" : 0.7,
    },
    {
        "theta" : 88,
    },
    {
        "launch_expansion": False,
        "theta": 90,
        "intake_speed": 0,
        "message": "This is state #3!",
        "x_pos": 100,
        "y_pos": 15 + 4.01231456556765674560009,
        "min_velocity" : 0,
    },
    {
        "intake_speed" : 100,
        "theta" : 105,

    },
    {
        "shoot_disc" : 1,
        "wait" : 2,
    },
    {
        "shoot_disc" : 1,
        "wait" : 2,
    },
    {
        "shoot_disc" : 1,
        "intake_speed" : 0,
    },
    # {
    #     "shoot_disc" : 1,
    # },
    {
        "launch_expansion": False,
        "theta": 360,
        "intake_speed": 0,
        "message": "This is state #1!",
        "flywheel_speed": 0,
        "x_pos": 100,
        "y_pos": 15 + 4.01231456556765674560009,
    },
    {
        "launch_expansion": False,
        "intake_speed": 0,
        "message": "This is state #2!",
        "flywheel_speed": 0,
        "x_pos": 78,
        "y_pos": 82 + 4.01231456556765674560009,
        "min_velocity" : 65,
    },
    {
        "theta": 225.0,
        "shoot_disc" : 1,
    },
    {
        "autonomous_speed" : 60,
        "launch_expansion": False,
        "theta": 225,
        "intake_speed": 100,
        "message": "This is state #4!",
        "flywheel_speed": 0,
        "x_pos": 161,
        "y_pos": 173 + 4.00000 + 3.754567355,
    },
    {
        "autonomous_speed" : 75,
    },
    {
        "theta" : 180
    },
    {
        "launch_expansion": False,
        "intake_speed": 0,
        "message": "This is state #2!",
        "flywheel_speed": 53,
        "x_pos": 213,
        "y_pos": 173 + 4.00000,
    },
    {
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 0,
        "message": "This is state #3!",
        "x_pos": 213,
        "y_pos": 173 + 4.00000,
    },
    {
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 0,
        "message": "This is state #4!",
        "x_pos": 213,
        "y_pos": 130,
        "min_velocity" : 0,
    },
    {
        "theta" : 191,
    },
    {
        "shoot_disc" : 1,
        "wait" : 1,
    },
    {
        "shoot_disc" : 1,
        "wait" : 1,
    },
    {
        "shoot_disc" : 1,
        "wait" : 1,
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
    },
    {
        "launch_expansion": False,
        "theta": 270,
        "intake_speed": 0,
        "message": "This is state #1!",
        "flywheel_speed": 0,
        "x_pos": 213,
        "y_pos": 220 + 13.0000123123,
        "shoot_disc" : 1,
    },

    {
        "shoot_disc" : 1,
        "launch_expansion": False,
        "theta": 270,
        "intake_speed": 0,
        "message": "This is state #2!",
        "flywheel_speed": 0,
        "x_pos": 251,
        "y_pos": 217 + 13.0000123123 + 3,
        "min_velocity" : 0,
        "timeout" : 5,
    },
    {
        "roller_spin_for" : 0.62,
        "message" : "Doing the third roller!",
        "wait" : 0.1,
    },
    {
        "x_pos": 225,
        "y_pos": 217 + 13.0000123123 + 1,
        "min_velocity" : 0,
    },
    {
        "theta" : 90,
        "intake_speed" : 100,
    },
    {
        "autonomous_speed" : 40,
        "launch_expansion": False,
        "theta": 90,
        "message": "This is state #3!",
        "flywheel_speed": 65.6,
        "x_pos": 176,
        "y_pos": 217 + 13.0000123123,
    },

    {
        "autonomous_speed" : 75,
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 100,
        "message": "This is state #4!",
        "x_pos": 176,
        "y_pos": 217 + 13.0000123123,
    },


    {
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 100,
        "message": "This is state #5!",
        "x_pos": 176,
        "y_pos": 250,
    },

    {
        "launch_expansion": False,
        "theta": 180,
        "intake_speed": 0,
        "message": "This is state #6!",
        "x_pos": 176,
        "y_pos": 293 + 20.000012312635636,
        "timeout" : 5,
    },
    {
        "roller_spin_for" : 0.6,
        "message" : "Doing the 4th roller",
        "wait" : 0.1,
    },
    {
        "x_pos": 215,
        "y_pos": 263 + 12.0000009191919191911191919,
        "theta" : 180,
    },
    {
        "x_pos": 215,
        "y_pos": 263 + 12.0000009191919191911191919,
        "theta" : 185,
    },
    ### Shoot 3 final discs
    {
        "shoot_disc" : 1,
        "wait" : 1,
    },
    {
        "shoot_disc" : 1,
        "wait" : 1,
    },
    {
        "shoot_disc" : 1,
        "wait" : 1,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "launch_expansion": False,
        "theta": 225,
        "intake_speed": 0,
        "message": "This is state #2!",
        "x_pos": 215,
        "y_pos": 263 + 12.0000009191919191911191919,
    },
    {
        "flywheel_speed" : 0,
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
        "roller_spin_for" : 20,
    },
    {
        "roller_spin_for" : 20,
    },
    {
        "roller_spin_for" : 20,
    },
    {
        "roller_spin_for" : 20,
    },
    {
        "roller_spin_for" : 20,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },
    {
        "roller_spin_for" : 2,
    },

]

turret_test = [
    {
        "override_velocity_theta": None,
        "drone_mode": True,
        "override_velocity_x": None,
        "override_velocity_y": None,
        "set_x": 0,
        "set_y": 0,
        "set_theta": 0,
        "autonomous_speed" : 75,
        "flywheel_speed" : 25,
    },
    {
        "wait" : 3,
    },
    {
        "shoot_at_theta" : 90,
    },
    {
        "wait" : 0.7,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "shoot_at_theta" : 0,
    },
    {
        "wait" : 0.7,
    },
    {
        "shoot_disc" : 1,
    },
    {
        "shoot_at_theta" : -90,
    },
    {
        "wait" : 0.7,
    },
    {
        "shoot_disc" : 1,
    },
]

#endregion
# Initializes a robot object
r = Robot()
wait(30, MSEC)

r.set_team("neutral")
init()
r.init()

r.set_autonomous_procedure(turret_test)
# r.run_autonomous()

#region GUI

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
    Switch(["3-Square", "2-Square", "SKILLS", "ShootDiscs", "Intake Forever", "Temp", "Test"], 0, 120, 120, 40, [0x33FF33, 0x33CC33, 0x33AA33, 0x339933, 0x337733, 0x335533,0x333333], lambda auto_mode: r.set_autonomous_procedure(auto_mode), [match_auto_three_squares, match_auto_two_squares, skills_auto, skills_auto, auto_intake_on_forever, skills_auto, auto_test]),
    Button("Run auto", 0, 160, 120, 40, (0xAAAAAA), lambda: r.run_autonomous()),
    Switch(["2-Controller", "1-Controller"], 240, 160, 120, 80, [0xCCAAAA, 0xCCAACC], lambda value: set_debug_value(value), (False, True))
])
global_turret_theta = 0
def turret_change_theta(change):
    global global_turret_theta
    r.set_target_state({
        "turret_theta" : r.target_state["turret_theta"] + change
    })
    print("global turret",global_turret_theta)

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
    
    Text("", 360, 0, 120, 40, (0x110000), lambda: f("Real:", round(flywheel_motor.velocity(PERCENT)))), 
    Text("", 360, 40, 120, 40, (0x110000), lambda: f("T:", round(flywheel_motor.temperature()))), 
    Button("T+", 360, 80, 120, 40, (0x110000), turret_change_theta, 1 ), 
    Button("T-", 360, 120, 120, 40, (0x110000), turret_change_theta, -1,), 
    Text("", 240, 0, 120, 40, (0x110000), lambda: f("SP:", r.flywheel_speed),), 
    Text("", 240, 40, 120, 40, Color.BLACK, lambda: "x: {:.2f}".format(r.x_pos),), 
    Text("", 240, 80, 120, 40, Color.BLACK,lambda: "y: {:.2f}".format(r.y_pos),), 
    Text("", 240, 120, 120, 40, Color.BLACK, lambda: f("θ:", str(r.theta).split(".")[0]),), 
    Switch(["Rec. Off", "Rec. On"], 360, 160, 120, 40, [Color.RED, Color.GREEN], lambda v: start_recording_mode_for_autonomous(v), (False, True)),
    Button("Go To Comp", 360, 200, 120, 40, (0xAAAAAA), lambda: gui.set_page(0)),
    Text("", 120, 40, 120, 40, Color.BLACK, lambda: f(round(left_motor_a.temperature()), round(left_motor_b.temperature()), round(right_motor_a.temperature()), round(right_motor_b.temperature()))),
])

gui.add_page([
    Text("Prematch Checklist!", 120, 0, 240, 20, Color.BLACK),
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

#endregion
r.update()
# 30 millisecond wait for the gyroscope to initialize

def output_data():
    global serial_output_delay
    serial_output_delay = 0.2

    # print("T", r.state["turret_theta"], r.target_state["turret_theta"], "rs", r.x_vel, r.y_vel, "fs", r.flywheel_speed, "pos", r.x_from_gps, r.y_from_gps, "distance", r.compute_distance_to_goal(goal=r.blue_goal))
    
    d_x, d_y = r.x_from_gps - r.initial_x_field, r.y_from_gps - r.initial_y_field

    print(d_x, d_y, sqrt(d_x**2 + d_y**2))

    # print(brain.timer.value(), r.x_from_gps, r.y_from_gps, r.theta, r.theta_field, closest_angle(get_angle_to_object((r.x_from_gps, r.y_from_gps), (r.target_goal.x_pos, r.target_goal.y_pos)) - r.initial_theta_field), r.initial_theta_field)
    # print(brain.timer.value(), r.flywheel_speed, r.flywheel_kP, r.flywheel_motor_average_output)
    # print(brain.timer.value(), r.theta, r["turret_theta"], r.compute_turret_ticks_to_angle(turret_motor.position(DEGREES)))
    # print(brain.timer.value(), flywheel_motor.velocity(PERCENT), r.flywheel_speed)
    
    # print(r.compute_angle_to_turret_ticks(100), r.compute_distance_to_goal(r.red_goal), r.compute_distance_to_goal(r.blue_goal))

    # print(r["flywheel_speed"])
    # print(brain.timer.value(), r.x_gps_relative, r.y_gps_relative)
    # print(brain.timer.value(), r.x_pos, r.y_pos, r.theta)
    # print("real", r.x_from_gps, r.y_from_gps, "delta", r.x_from_gps_robots_reference_frame, r.y_from_gps_robots_reference_frame)
    # print(brain.timer.value(), flywheel_motor.velocity(PERCENT), r.flywheel_avg_speed)
    # print(brain.timer.value(), r.initial_theta_field)

    # print(roller_and_intake_motor_1.position(DEGREES), roller_and_intake_motor_2.position(DEGREES))
    # Double check to see if this goes in the right direction(s)
    # print(brain.timer.value(), r.x_from_gps, r.y_from_gps, r.x_encoders_field + r.initial_x_field, r.y_encoders_field + r.initial_y_field)
    
    # print(brain.timer.value(), r.x_encoders_relative, r.y_encoders_relative, r.x_encoders_field, r.y_encoders_field)

    # These values should be very similar
    # print(brain.timer.value(), r.delta_x_from_encoders, r.delta_y_from_encoders, r.delta_x_from_gps_robots_reference_frame, r.delta_y_from_gps_robots_reference_frame)
    pass

def reset_flywheel_during_tuning():
    r.integral_term_flywheel = 0
    r.flywheel_motor_average_output = 0
    
    r.set_target_state({
        "flywheel_speed" : 0,
    })

    while abs(r.flywheel_instantaneous_speed) > 0.1:
        r.set_target_state({
            "flywheel_speed" : 0,
        })
        wait(0.1, SECONDS)

    r.integral_term_flywheel = 0
    r.flywheel_motor_average_output = 0

def see_how_long_it_takes_for_flywheel_to_reach_target_speed(position):
    if type(position) == Vector:
        r.flywheel_kP, r.flywheel_kI, r.flywheel_kD = position.data
    else:
        r.flywheel_kP, r.flywheel_kI, r.flywheel_kD = position

    flywheel_target_speed = 20

    reset_flywheel_during_tuning()
    wait(1, SECONDS)

    # print("Flywheel speed is at 0 percent!", r.flywheel_instantaneous_speed, r.flywheel_avg_speed)

    r.set_target_state({
        "flywheel_speed" : flywheel_target_speed
    })
    
    # Just in case theres some wacky thing going on with this
    while r.flywheel_instantaneous_speed < 1:
        wait(0.01, SECONDS)
    
    start_time = brain.timer.value()

    while r.flywheel_instantaneous_speed < flywheel_target_speed:
        # If we are taking a long time to converge, then increment kP 
        if (start_time - brain.timer.value() > 2):
            r.flywheel_kP += 0.001
            print(" we adding to kP!")
        wait(0.01, SECONDS)

    time_until_hit = brain.timer.value()

    # print("It took ", time_until_hit - start_time, " ms to hit the target speed!")

    reset_flywheel_during_tuning()

    return (time_until_hit - start_time)


def see_how_long_it_takes_for_flywheel_to_get_converge(position, flywheel_target_speed = 40):
    if len(position) == 3:
        if type(position) == Vector:
            r.flywheel_kP, r.flywheel_kI, r.flywheel_kF = position
        else:
            r.flywheel_kP, r.flywheel_kI, r.flywheel_kF = position
        # if type(position) == Vector:
        #     r.flywheel_kP, r.flywheel_kI, r.flywheel_kD = position
        # else:
        #     r.flywheel_kP, r.flywheel_kI, r.flywheel_kD = position
    elif len(position) == 4:
        if type(position) == Vector:
            r.flywheel_kP, r.flywheel_kI, r.flywheel_kD, r.flywheel_kF = position
        else:
            r.flywheel_kP, r.flywheel_kI, r.flywheel_kD, r.flywheel_kF = position

    max_time_until_convergence = 9

    reset_flywheel_during_tuning()
    wait(1, SECONDS)

    # print("Flywheel speed is at 0 percent!", r.flywheel_instantaneous_speed, r.flywheel_avg_speed)

    r.set_target_state({
        "flywheel_speed" : flywheel_target_speed
    })
    while r.flywheel_instantaneous_speed < 1:
        r.set_target_state({
            "flywheel_speed" : flywheel_target_speed
        })
        wait(0.01, SECONDS)
    
    start_time = brain.timer.value()

    previous_speeds_1 = CircularArray(20)

    stdev = std(previous_speeds_1.arr)
    average = 0
    accel = (r.state["flywheel_instantaneous_speed"] - r.previous_state["flywheel_instantaneous_speed"]) / r.delta_time
    # curvature = (delta_time * (r.previous_state["flywheel_speed"] - r.state["flywheel_speed"]))

    delta_time = 0.01
    # Wait for the flywheel to get to the target s;speed
    while r.flywheel_instantaneous_speed < flywheel_target_speed or abs(average - r.flywheel_speed) > 2:
        print("Average:\t", average, "Std:\t", stdev, "Accel:\t", accel)
        # If we are taking a long time to converge, then increment kP 
        if (brain.timer.value()-start_time > max_time_until_convergence):
            break
        previous_speeds_1.append(r.flywheel_instantaneous_speed)
        
        accel = (r.state["flywheel_instantaneous_speed"] - r.previous_state["flywheel_instantaneous_speed"]) / r.delta_time
        stdev = std(previous_speeds_1.arr)
        average = sum(previous_speeds_1.arr) / len(previous_speeds_1.arr)
        # print("Average:\t", average, "Std:\t", stdev, "Accel:\t", accel)

        
        wait(delta_time, SECONDS)

    time_until_hit = brain.timer.value()
    copy_of_previous_speeds = previous_speeds_1.arr.copy()
    
    del previous_speeds_1
    # [previous_speeds.pop_oldest() for b in range(round(previous_speeds.length * 0.8))]
    previous_speeds_1 = CircularArray(150)
    [previous_speeds_1.append(b) for b in copy_of_previous_speeds]
    
    previous_speeds_2 = CircularArray(20)
    average_2 = -100
    
    # USE CURIVATURE 
    while abs(average - r.flywheel_speed) > 0.1 or abs(average_2 - r.flywheel_speed) > 0.5 or stdev > 2.5:
        print("Average:\t", average, average_2, "Std:\t", stdev, "Accel:\t", accel)
        # If we are taking a long time to converge, then increment kP 
        if (brain.timer.value()-start_time > max_time_until_convergence):
            break
        previous_speeds_1.append(r.flywheel_instantaneous_speed)
        previous_speeds_2.append(r.flywheel_instantaneous_speed)
        stdev = std(previous_speeds_1.arr)
        average = sum(previous_speeds_1.arr) / len(previous_speeds_1.arr)
        average_2 = sum(previous_speeds_2.arr) / len(previous_speeds_2.arr)
        accel = (r.state["flywheel_instantaneous_speed"] - r.previous_state["flywheel_instantaneous_speed"]) / r.delta_time
        wait(0.01, SECONDS)

    time_until_converge = brain.timer.value()
    delta_time = (time_until_converge - start_time)

    if delta_time > max_time_until_convergence:
        score = delta_time + sqrt(abs(average - r.flywheel_speed)) + sqrt(stdev) 
        reset_flywheel_during_tuning()
        print("It took ", delta_time, " seconds to hit the target speed!", "score is: ", score)
        return score
    print("It took ", delta_time, " seconds to hit the target speed!")

    reset_flywheel_during_tuning()

    return (time_until_converge - start_time)


def compute_gradient(position):
  grads = []
  vectors = []

  for i, pos in enumerate(position):
    vectors.append(Vector([0] * i + [1] + [0] * (len(position) - i - 1)))


  for vector in vectors:
    epsilon = 1e-2
    h = vector * epsilon
    print("h is", h)
    print("vector plus h is", vector + h)
    print("vector minus h is", vector - h)
    print("f of vector + h: ", f(vector + h))
    print("f of vector - h: ", f(vector - h))


    print("dervative is", (f(vector + h) - f(vector - h)) * (1 / (2 * (epsilon))))

    grads.append((f(vector + h) - f(vector - h)) * (1 / (2 * (epsilon))))
  
  return grads

def compute_gradient_of_flywheel(position):
    f = see_how_long_it_takes_for_flywheel_to_get_converge
    multiplication_constants = [0.1, 0.005, 0.00001, 0.1]
    grads = []
    vectors = []

    # create identity matrix for position vector
    for i in range(len(position)):
        vectors.append(Vector([0] * i + [1] + [0] * (len(position) - i - 1)))

    for i in range(len(position)):
        vectors[i].data[i] *= multiplication_constants[i]
    
    for vector in vectors:
        epsilon = 1e-1
        h = vector * epsilon
        f_plus_h = 1000
        f_minus_h = -1000
        
        # All of the constants cannot be negative
        if position[len(grads)]-h[len(grads)] < 0:
            print("h[",len(grads),"] is too small, making it equal to", position[len(grads)])
            h.data[len(grads)] = position[len(grads)]

        while f_plus_h < 0.100 or f_minus_h < 0.100: # abs(f_plus_h - f_minus_h) > 0.500 or :
            f_plus_h = f(Vector(position) + h)
            f_minus_h = f(Vector(position) - h)

        # print("h is", h)
        # print("vector plus h is", Vector(position) + h)
        # print("vector minus h is", Vector(position) - h)
        # print("f of vector + h: ", f_plus_h)
        # print("f of vector - h: ", f_minus_h)

        # print("dervative is", (f_plus_h - f_minus_h) * (1 / (2 * (epsilon))))

        grads.append((f_plus_h - f_minus_h) * (1 / (2 * (epsilon))))

    return grads

def tune_flywheel(initial_pos):
    f = see_how_long_it_takes_for_flywheel_to_get_converge
    guess_position = initial_pos
    grad = Vector([0 for i in (guess_position)])
    gradients = []
    max_epsilons = Vector([0.01, 0.01, 0.01]) * 2
    max_attempts = 10
    epsilon = max_epsilons
    attempts = 0
    gradient_ascent_or_descend_constant = -1
    positions = []
    while attempts < max_attempts:
        print("Attempt ", attempts, ": | t =", brain.timer.value(), "| r =", guess_position, " | f(r) =", f(guess_position))
        grad = compute_gradient_of_flywheel(guess_position)
        delta_position = numpy.dot(Vector(grad), epsilon)
        guess_position = Vector(guess_position) + delta_position * gradient_ascent_or_descend_constant

        # Make it so that the minimum value for kP, kI, and kF is zero
        guess_position.data = [max(guess_position[i], 0) for i in range(len(guess_position))]

        print("\tGrad\t", grad)
        print("\tD_R\t", delta_position.data)
        print("\tR:\t", guess_position.data)
        positions.append(guess_position)
        gradients.append(grad)
        attempts += 1
        epsilon = max_epsilons  

        # wait for flywheel to cool down a little
        wait(20, SECONDS)
    print("it took ", attempts, "attempts to find the minimum, final position is", guess_position, "with height of ", f(guess_position))

    print("positions = ", [pos.data for pos in positions])
    print("gradients = ", [pos.data for pos in gradients])

# [-5350.0, -1400.0, 950.0]

        # Grad     [0.1548767, -0.7749939, 0.2749634, 0.4650879]
        # D_R      [0.0007743835, -0.003874969, 0.001374817, 0.000232544]
        # R:       [0.3367254, 0.2119748, 0, 0.10918]
        # R:       [0.1827017, 0.1534541, 0.1499479]

# Output one pass of the data to init the serial_output_delay
output_data()


competition = Competition(driver_control, autonomous)

# initial_pos = (0.6, 0.2, 0.11)
# initial_pos = [0.2588518, 0.1403032, 0.0950996]
# initial_pos = [0.217002, 0.1525032, 0.09129846]
# initial_pos = [0.1885019, 0.146754, 0.1340479]
# initial_pos = [0.3109977, 0.2244707, 0.2310476]
# initial_pos = [0.6, 0.0191, 0.105]

# It took  2.491005  seconds to hit the target speed!
#         Grad     [4.67495, -1.539993, -0.7650375]
#         D_R      [0.09349899, -0.03079987, -0.01530075]
#         R:       [0.485001, 0.03280014, 0.1370003]

# It took  2.365997  seconds to hit the target speed!
#         Grad     [1.075001, 0.3549862, -0.83498]
#         D_R      [0.02150002, 0.007099723, -0.0166996]
#         R:       [0.5785, 0.002000277, 0.1216996]

#         Grad     [0.1545048, 4.342556, -6.592007]
#         D_R      [0.003090096, 0.08685112, -0.1318401]
#         R:       [1.447774, 0, 0.1318401]

# print(see_how_long_it_takes_for_flywheel_to_get_converge(initial_pos))

# tune_flywheel(initial_pos)

# grads = []R:       [0.1827017, 0.1534541, 0.1499479]
# for i in range(5):
#     grads.append(compute_gradient_of_flywheel(initial_pos))

# after_5 = grads.copy()
# for i in range(5):
#     grads.append(compute_gradient_of_flywheel(initial_pos))

# after_10 = grads.copy()

# print(after_5)
# print(after_10)
# print(grads)

#region TODO LIST
# ! PRIORITY
# TEST: make default position of turret so that were shooting straight

# TODO: make gps calibration a button in case things go cray cray

# TODO: Make robot smoother
# TODO: Make turret slower

# TODO: test passing theta if it goes to the next state
# TODO: smooth the movemnet of the robot
# TODO: test slow down speed and tolerance variables

# TODO: Test pulsing feature

# TODO: Figure out why the robot's encoders are reading a greater value than they actually are

# TODO: apply fiter for encoders values so theyre cleaner
# TODO: Implement fast autonomous

# TODO: Use accel/encoders to figure out when the robot is stopped?!?!?

# TODO: Test removing the derivative term from the flywheel

# TODO: When the robot initializes, make it rotate towards 90 or 180 degrees exactly (if GPS is on)

# TODO: Use sensor fusion with Kalman filter for better position
# TODO: Make odometry consistent with real world measurements

# TODO: See if we can register callback functions instead of checking things in the updates and see how much that will improve performance
# TODO: Use trapazoids instead of rectangles when integrating the velocity

# * IMPORTANT
# TODO: Pid controller for motor movement?!?!?!?!?!?!?!?
# TODO: When the robot has an x,y position close to the edge of the field, make it so that the robot physically cannot slam into the wall (or use sensors)
# TODO: Change this to be relative to the match time???

# Not important
# TODO: make an auto path class that has information like description, color, etc, maybe draw it out???.

#endregion