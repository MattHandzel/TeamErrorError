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

    intake_motor.spin(FORWARD)
    intake_motor.set_velocity(0, PERCENT)

    # 
    roller_motor.spin(FORWARD)
    roller_motor.set_velocity(0, PERCENT)
    roller_motor.set_stopping(BRAKE)

    # Set the optical light power
    roller_optical.set_light_power(100)

    # Wait for the gyro to settle, if it takes more then 10 seconds then close out of the loop
    t = Timer()

    t.reset()
    while (inertial.gyro_rate(ZAXIS) != 0 and t.value() < 10):
        print("Waiting for gyro to init...")
        wait(0.1, SECONDS)
    
    controller_1.rumble("...")
