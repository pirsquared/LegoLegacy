from pybricks.hubs import InventorHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase

# Set up all devices.
inventor_hub = InventorHub()
left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_wheel = Motor(Port.B, Direction.CLOCKWISE)
drive_base = DriveBase(left_wheel, right_wheel, 56, 195)
drive_base.use_gyro(True)


# The main program starts here.
print('''\
################################################
### Hello Pybricks! ############################
################################################
''')
drive_base.turn(1080)
