from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task
from umath import pi

_CONFIG = {
    'wheel_diameter': 54.2,       # The diameter of the wheels is 56mm.  However,
                                  # we ran a number of trials where we compared
                                  # the angle of rotation for each motor and the
                                  # measured distance traveled by the robot.  The
                                  # test resulted in an estimated 54.2mm wheel dia.
    'wheel_base': 145.3,          # The distance from the center of one wheel to
                                  # the center of the other wheel is 18 peg
                                  # lengths.  With each peg length equal to 8mm
                                  # that's a total of 144mm.  However, we ran
                                  # a number of trials where we compared the
                                  # distance traveled by each motor under the
                                  # assumption that the wheels have a diameter
                                  # of 56mm and the heading angle.  The test
                                  # resulted in an estimated 145.3mm wheel base.
    'ring_drive_teeth': 36,
    'ring_outer_teeth': 140,      # 35 per quarter ring * 4
    'lift_deg_teeth_ratio': 4.5,  # 45 degree turn on the lift motor results in
                                  # 10 teeth on the lift rack which is one full
                                  # length four piece.  The entire rack is six
                                  # pieces or 60 teeth in which the range only
                                  # goes to approximately 55 teeth. That trans-
                                  # lates into approximately 225 degrees of
                                  # motion on the drive motor.
                                  # This is intended for speed but will run into
                                  # issues if we need more torque/lift power.
    'hub_class': InventorHub,
    'top_face_direction': Axis.Z,
    'front_face_direction': Axis.X,
}

class Lift(Motor):
    teeth_per_unit = 2.5

    def ratio(self):
        return self.teeth_per_unit * _CONFIG['lift_deg_teeth_ratio']

    def liftby(self, rotation_angle, speed=90, **kwargs):
        rotation_angle *= self.ratio()
        return self.run_angle(speed, rotation_angle, **kwargs)

class Ring(Motor):
    def ratio(self):
        return _CONFIG['ring_outer_teeth'] / _CONFIG['ring_drive_teeth']

    def twist(self, rotation_angle, trn_rt=90, trn_acc=90, **kwargs):
        trn_rt *= self.ratio()
        trn_acc *= self.ratio()
        rotation_angle *= self.ratio()
        speed, acceleration, torque = self.control.limits()
        self.control.limits(trn_rt, trn_acc, torque)
        result = self.run_angle(trn_rt, rotation_angle, **kwargs)
        self.control.limits(speed, acceleration, torque)
        return result
    
    def ring_angle(self):
        return self.angle() / self.ratio()

class Drive(DriveBase):
    def tern(self, angle, trn_rt=90, trn_acc=90, *args, **kwargs):
        s_spd, s_acc, t_rate, t_acc = self.settings()
        self.settings(s_spd, s_acc, trn_rt, trn_acc)
        result = self.turn(angle, *args, **kwargs)
        self.settings(s_spd, s_acc, t_rate, t_acc)
        return result

class Bot:
    def __init__(
            self,
            left_motor: Port,
            right_motor: Port,
            ring_motor: Port,
            lift_motor: Port,
            left_eye: Port,
            right_eye: Port,
            hub_type: type = _CONFIG['hub_class']
        ):
        self.hub = hub_type(
            top_side=_CONFIG['top_face_direction'],
            front_side=_CONFIG['front_face_direction']
        )
        self.left_motor = Motor(left_motor, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_motor, Direction.CLOCKWISE)
        self.ring = Ring(ring_motor, Direction.CLOCKWISE)
        self.ring.control.target_tolerances(10, 5)
        self.lift = Lift(lift_motor, Direction.COUNTERCLOCKWISE)
        self.left_eye = ColorSensor(left_eye)
        self.right_eye = ColorSensor(right_eye)
        self.drive = Drive(
            self.left_motor,
            self.right_motor,
            _CONFIG['wheel_diameter'],
            _CONFIG['wheel_base']
        )
        self.drive.heading_control.target_tolerances(10, 5)

    def twist(self, *args, **kwargs):
        return self.ring.twist(*args, **kwargs)

    def liftby(self, *args, **kwargs):
        return self.lift.liftby(*args, **kwargs)

    def turn(self, *args, **kwargs):
        return self.drive.tern(*args, **kwargs)
    
    def straight(self, *args, **kwargs):
        return self.drive.straight(*args, **kwargs)

    def twist_turn(self, angle, trn_rt=120, trn_acc=120):
        async def tt():
            await multitask(
                self.turn(angle, trn_rt=trn_rt, trn_acc=trn_acc),
                self.twist(angle, trn_rt=trn_rt, trn_acc=trn_acc)
            )
        return run_task(tt())
    
