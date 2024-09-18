from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task
from umath import pi


_CONFIG = {
    'ring_drive_teeth': 36,
    'ring_outer_teeth': 140,  # 35 per quarter ring * 4
    'lift_deg_teeth_ratio': 4.5,  # 45 degree turn on the lift motor results in
                                  # 10 teeth on the lift rack which is one full
                                  # length four piece.  The entire rack is six
                                  # pieces or 60 teeth in which the range only
                                  # goes to approximately 55 teeth. That trans-
                                  # lates into approximately 225 degrees of
                                  # motion on the drive motor.
                                  # This is intended for speed but will run into
                                  # issues if we need more torque/lift power.
}

def lift_unit_to_deg(number_of_units):
    teeth_per_unit = 2.5  # From the ev3 rack pieces, 10 teeth/length 4
    total_teeth = teeth_per_unit * number_of_units
    return total_teeth * _CONFIG['lift_deg_teeth_ratio']

hub = InventorHub()

class Lift(Motor):
    teeth_per_unit = 2.5

    def ratio(self):
        return self.teeth_per_unit * _CONFIG['lift_deg_teeth_ratio']

    def lift_units(self, **kwargs):
        kwargs['rotation_angle'] *= self.ratio()
        self.run_angle(**kwargs)

class Ring(Motor):
    def ratio(self):
        return _CONFIG['ring_outer_teeth'] / _CONFIG['ring_drive_teeth']

    def twist(self, **kwargs):
        kwargs['rotation_angle'] *= self.ratio()
        kwargs['speed'] *= self.ratio()
        self.run_angle(**kwargs)
    
    def ring_angle(self):
        return self.angle() / self.ratio()

left_wheel = Motor(Port.A, Direction.COUNTERCLOCKWISE)
rite_wheel = Motor(Port.F, Direction.CLOCKWISE)
drive_base = DriveBase(left_wheel, rite_wheel, 56, 100)
drive_base.use_gyro(True)

ring = Ring(Port.C, Direction.CLOCKWISE)
lift = Lift(Port.B, Direction.COUNTERCLOCKWISE)

# print(full_rotation_motor_angle)

# y = 180
# o = 360
# fork_lift.control.limits(2000, 2000, 1000)
# fork_lift.run_angle(180, y, wait=False)
# octo_wheel.run_angle(2000, o, wait=False)
# drive_base.straight(30, wait=True)
# wait(1000)

# fork_lift.run_angle(2000, -y, wait=True)
# fork_lift.run_angle(2000, y, wait=True)

# octo_wheel.run_angle(2000, -o, wait=True)
# fork_lift.run_angle(2000, -y, wait=False)
# drive_base.straight(-30, wait=True)
# wait(1000)

# print(fork_lift.control.limits())

# ring.run_angle(1000, full_rotation_motor_angle)
# ring.run_angle(1000, -full_rotation_motor_angle)

# sw = StopWatch()
# print(f'{"Angle":^7}{"Speed":^7}{"UP":>7}{"DOWN":>7}')
# fork_lift.control.limits(1000, 4000, 540)
# for angle in [225]:
#     for i in range(1, 3):
#         speed = 360 * i
#         print(f'{angle:<7}{speed:<7}', end='')
#         stamp = sw.time()
#         fork_lift.run_angle(speed, angle)
#         new_stamp = sw.time()
#         seconds = (new_stamp - stamp) / 1000
#         stamp = new_stamp
#         print(f'{seconds:>7.3f}', end='')
#         fork_lift.run_angle(speed, -angle)
#         new_stamp = sw.time()
#         seconds = (new_stamp - stamp) / 1000
#         stamp = new_stamp
#         print(f'{seconds:>7.3f}')


# fork_lift.run_angle(180, 100)
# lift.lift_units(n=12, speed=180)
# wait(1000)
# lift.lift_units(n=-12, speed=180)
# wait(1000)
# ring.reset_angle(0)
# ring.twist(speed=180, rotation_angle=360)
# print(ring.ring_angle())
# wait(5000)
# ring.twist(speed=180, rotation_angle=-360)
# print(ring.ring_angle())

# lift.lift_units(speed=180, rotation_angle=8)
# wait(1000)
# lift.lift_units(speed=180, rotation_angle=-8)



left_wheel.reset_angle(0)
rite_wheel.reset_angle(0)

drive_base.reset()


for _ in range(10):
    drive_base.turn(36)

    print(
        (rd:=rite_wheel.angle()*56*pi/360),
        (ld:=left_wheel.angle()*56*pi/360),
        (yaw:=hub.imu.heading()),
        (rd-ld)*180/yaw/pi

    )

