from utils import *
# Brain should be defined by default
brain = Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
controller_2 = Controller(PARTNER)
left_motor_a = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)

left_drive_smart = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
right_drive_smart = MotorGroup(right_motor_a, right_motor_b)

drivetrain = DriveTrain(
    left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1)

flywheel_motor_1 = Motor(Ports.PORT17, GearSetting.RATIO_6_1, False)
flywheel_motor_2 = Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)

led_a = Led(brain.three_wire_port.a)
inertial = Inertial(Ports.PORT16)
index_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
roller_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
intake_motor = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
roller_optical = Optical(Ports.PORT12)

indexer = Pneumatics(brain.three_wire_port.a)
expansion = Pneumatics(brain.three_wire_port.b)

gps = Gps(Ports.PORT13)