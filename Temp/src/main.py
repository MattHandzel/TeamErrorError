# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       matthandzel                                                  #
# 	Created:      11/21/2022, 3:15:25 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

motor1 = Motor(Ports.PORT4, GearSetting.RATIO_36_1, False)
motor2 = Motor(Ports.PORT5, GearSetting.RATIO_36_1, False)
controller1 = Controller(PRIMARY)

indexer = Pneumatics(brain.three_wire_port.a)

indexer.close()

motor1.spin(REVERSE)
motor2.spin(FORWARD)

motor1.set_velocity(0, PERCENT)
motor2.set_velocity(0, PERCENT)

while True:
  if controller1.buttonA.pressing():
    motor1.set_velocity(100, PERCENT)
    motor2.set_velocity(100, PERCENT)
    print("100", motor1.velocity(PERCENT), motor2.velocity(PERCENT))
  elif controller1.buttonB.pressing():
    motor1.set_velocity(0, PERCENT)
    motor2.set_velocity(0, PERCENT)
  elif controller1.buttonX.pressing():
    motor1.set_velocity(50, PERCENT)
    motor2.set_velocity(50, PERCENT)
    print("50", motor1.velocity(PERCENT), motor2.velocity(PERCENT))
  elif controller1.buttonY.pressing():
    motor1.set_velocity(25, PERCENT)
    motor2.set_velocity(25, PERCENT)
    print("25", motor1.velocity(PERCENT), motor2.velocity(PERCENT))
  print("TEMP", motor1.temperature(),motor2.temperature() )
  print("SPEED", motor1.velocity(PERCENT), motor2.velocity(PERCENT))
  motor1.set_velocity(-controller1.axis3.position(), PERCENT)
  motor2.set_velocity(controller1.axis2.position(), PERCENT)

  # launch  disc
  if controller1.buttonDown.pressing():
    indexer.open()
  elif controller1.buttonUp.pressing():
    indexer.close()
  wait(0.1, SECONDS)
