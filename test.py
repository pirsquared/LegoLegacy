from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase

# Set up all devices.
# The top side is the side with the light matrix
# The front side is the side with the usb port
# Setting up the hub appropriately allows for you to use the gyro for turning
# Axes are:
# `Axis.X` is forward, `-Axis.X` is backward
# `Axis.Y` is right, `-Axis.Y` is left
# `Axis.Z` is up, `-Axis.Z` is down
# Currently, the bot is built with the 'top_side' facing backward
# therefore we set the `top_side` argument to `-Axis.X`
# Currently, the bot is built with the 'front_side' facing up
# therefore we set the `front_side` argument to `Axis.Z`
hub = PrimeHub(top_side=-Axis.X, front_side=Axis.Z)
left = Motor(Port.E, Direction.COUNTERCLOCKWISE)
rite = Motor(Port.F, Direction.CLOCKWISE)
base = DriveBase(left, rite, 56, 114)


# The main program starts here.
print('''\
################################################
### Hello Pybricks! ############################
################################################
''')
base.use_gyro(True)
base.straight(100)
base.turn(180)
