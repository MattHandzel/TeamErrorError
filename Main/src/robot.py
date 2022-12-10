from utils import *
from peripherals import *

class Robot:
    '''
    This is the big-boy class, this is the robot class, this is the class that controls the robot, there is a lot of stuff here
    '''

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

    ###* 

    # Used to keep track of time in auto and driver mode respectively, use it for nicely logging data, can be used during either modes for end game/pathfinding rules
    autonomous_timer = Timer()
    driver_controlled_timer = Timer()

    target_reached = False
    position_tolerance = 3 # tolerance to target position in cm
    orientation_tolerance = 8 # tolerance to target orientation in degrees
    
    # From 0,0 (which is the center of the field). Dimensions were got from page 89 on: https://content.vexrobotics.com/docs/2022-2023/vrc-spin-up/VRC-SpinUp-Game-Manual-2.2.pdf
    red_goal = GameObject((122.63 - 70.2) * 2.54, (122.63 - 70.2) * 2.54)
    blue_goal = GameObject(-(122.63 - 70.2) * 2.54, -(122.63 - 70.2) * 2.54)

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

        # Roller stuff
        "roller_state" : "none",
        "auto_roller" : False,
        "roller_speed" : 0,
        
        # Intake
        "auto_intake" : False,

        # Actuators
        "flywheel_speed" : 0,
        "intake_speed" : 0,

        # Commands
        "using_gps" : False,
        "is_shooting": False,
        "slow_mode" : False,
        "drone_mode" : False,
        "autonomous" : False,
        
    }

    target_state = {}
    
    all_states = []
    delta_time = 0

    save_states = False

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

        self.set_target_state(self.state)
        self.previous_state = self.state
    
    # There are two init methods, this init initializes the class, the other init method we call to initlize the robot
    def init(self):
        
        # Set heading based on gps
        if self.using_gps:
            while str(gps.heading()) == "nan":
                print("waiting for gps", gps.heading())
                time.sleep(0.1)
            # print("gps.heading at init:", gps.heading())
            self.theta_offset = gps.heading()
            self.total_theta = self.theta_offset
            # self.theta_offset = gps.heading() / 90
            # self.theta_offset = round(self.theta_offset) * 90
            # print("gps heading roudned", self.theta_offset)

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
        
        if self.save_states:
            self.all_states.append(self.state)

        # Update the previous state before doing state estimation
        self.previous_state = self.state.copy()
        self.estimate_state()

        self.position_update()
        self.flywheel_update()
        self.intake_update()
        self.roller_update()

    
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
        # The multiplying by 100 and dividing by self.position tolerance scales it so that at position tolerance the velocity is 100 percent, squaring the velocity makes it so that if we get closer than we go slower
        target_x_vel = ((clamp(delta_x, self.position_tolerance, -self.position_tolerance) * 100 / self.position_tolerance) ** 2) / 100 * sign(delta_x)
        target_y_vel = ((clamp(delta_y, self.position_tolerance, -self.position_tolerance) * 100 / self.position_tolerance) ** 2) / 100 * sign(delta_y)
        target_theta_vel = ((clamp(delta_theta, self.orientation_tolerance, -self.orientation_tolerance) * 100 / self.orientation_tolerance) ** 2) / 100 * sign(delta_theta)
        
        if abs(delta_theta) < 1:
            target_theta_vel = 0 

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
        if self.drone_mode:                                                         # Add self.theta add dt times angular velocity to get a better approximation of actual theta at timestamp
            target_x_vel, target_y_vel = rotate_vector_2d(target_x_vel, target_y_vel, (self.theta + self.delta_time / 2 * self.theta_vel) * DEG_TO_RAD)

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
        # self.theta_vel = inertial.gyro_rate(ZAXIS)
        
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
            
            self["disk_near_intake"]
    
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
                        
        roller_motor.spin(FORWARD, self["roller_speed"], PERCENT)



    


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
        _color = _color.lower()
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
    def set_flywheel_speed(self, speed):
        self.flywheel_speed = speed
        
    
    def flywheel_update(self):
        self.flywheel_pid(self.flywheel_speed)        
    
    def flywheel_pid(self, speed):
        # MAX_FLYWHEEL_SPEED = 100 / 60
        MAX_VOLTAGE = 12 # I don't think this is the true maximum voltage btw
        
        alpha = 0.1
        self.flywheel_1_avg_speed = flywheel_motor_1.velocity(PERCENT) * alpha + self.flywheel_1_avg_speed * (1 - alpha)
        self.flywheel_2_avg_speed = flywheel_motor_2.velocity(PERCENT) * alpha + self.flywheel_2_avg_speed * (1 - alpha)

        # flywheel_motor_1.spin(FORWARD, speed, PERCENT)
        # flywheel_motor_2.spin(FORWARD, speed, PERCENT)

        error_1 = speed - self.flywheel_1_avg_speed 
        error_2 = speed - self.flywheel_2_avg_speed 

        # error_1 = speed - flywheel_motor_1.velocity(VelocityUnits.PERCENT) 
        # error_2 = speed - flywheel_motor_2.velocity(VelocityUnits.PERCENT) 
        
        output_1 = self.flywheel_motor_1_PID.update(error_1)
        output_2 = self.flywheel_motor_2_PID.update(error_2)

        output_1 = min(output_1, MAX_VOLTAGE)
        output_2 = min(output_2, MAX_VOLTAGE)

        output_1 = max(output_1, 0)
        output_2 = max(output_2, 0)

        # output_1 = max(output_1, -MAX_VOLTAGE)
        # output_2 = max(output_2, -MAX_VOLTAGE)

        flywheel_motor_1.spin(FORWARD, output_2, VOLT)
        flywheel_motor_2.spin(FORWARD, output_2, VOLT)

        self.flywheel_motor_1_error = error_1
        self.flywheel_motor_2_error = error_2

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

        delta_x = self.goal.x_pos - self.x_pos - self.flywheel_offset_x
        delta_y = self.goal.y_pos - self.y_pos - self.flywheel_offset_y

        point = (self.goal.x_pos * 2.54 / 100, (self.goal.y_pos - self.flywheel_height_from_ground_IN) * 2.54 / 100)
        v_i, hit_time = getViForPathToHitPoint(self.flywheel_angle, (point), sizeOfPoint = 0.001)
        print("SETTING THE FLYWHEEL TO x M/S")

        # Compute the linear speed that the disk needs to be launched to
        # math.cos(theta_flywheel * DEG_TO_RAD)

        # distance_to_goal * theta_flywheel
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
