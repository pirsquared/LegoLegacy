from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task
from umath import pi

_CONFIG = {
    'wheel_diameter': 56,
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
}

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

class DBase(DriveBase):
    def __init__(
        self, left_motor, right_motor,
        wheel_diameter, axle_track
    ):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_diamter = wheel_diameter
        self.axle_track = axle_track
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)
