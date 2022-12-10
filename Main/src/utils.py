import math
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
