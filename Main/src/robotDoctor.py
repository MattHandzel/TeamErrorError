from robot import *


class Test:
  def __init__(self, name, **kwargs):
    name.lower()
    self.name = name
    self.args = kwargs
  
  def __getitem__(self, key):
    return self.args[key]

  # overload equals operator
  def __eq__(self, name):
    name.lower()
    return self.name == name
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
    "flywheel_temp"
  ),
]

during_match_checks = [
  Test(
    "flywheel_temp"
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
    "flywheel_temp"
  ),
  Test(
    "gps"
  )
]

rd = RobotDoctor()
rd.init()
rd.run_tests(all_tests)