from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task

hub = InventorHub()
left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
rite_wheel = Motor(Port.F, Direction.CLOCKWISE)
drive_base = DriveBase(left_wheel, rite_wheel, 56, 100)
drive_base.use_gyro(True)

octo_wheel = Motor(Port.C, Direction.CLOCKWISE)
fork_lift = Motor(Port.E, Direction.COUNTERCLOCKWISE)


# drive_base.turn(180, wait=False)
# octo_wheel.run_angle(720, 720)

# drive_base.turn(-180, wait=False)
# octo_wheel.run_angle(720, -720)

y = 180
fork_lift.run_angle(180, y)
fork_lift.run_angle(180, -y)
