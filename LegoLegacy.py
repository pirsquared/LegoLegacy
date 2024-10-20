from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task
from umath import pi, sin, cos, asin, acos

def clear_console():
    print("\x1b[H\x1b[2J", end="")


_CONFIG = {
    'wheel_diameter': 54.2,       # The diameter of the wheels is 56mm.  However,
                                  # we ran a number of trials where we compared
                                  # the angle of rotation for each motor and the
                                  # measured distance traveled by the robot.  The
                                  # test resulted in an estimated 54.2mm wheel dia.
    'wheel_base': 142.3,          # The distance from the center of one wheel to
                                  # the center of the other wheel is 18 peg
                                  # lengths.  With each peg length equal to 8mm
                                  # that's a total of 144mm.  However, we ran
                                  # a number of trials where we compared the
                                  # distance traveled by each motor under the
                                  # assumption that the wheels have a diameter
                                  # of 56mm and the heading angle.  The test
                                  # resulted in an estimated 145.3mm wheel base.
                                  # New estimate with measure turn of 7160 degrees
                                  # and left/right motors at 18_794/18_795 respectively
                                  # using 54.2mm wheel_diameter is 142.27

    'ring_drive_teeth': 36,
    'ring_outer_teeth': 140,      # 35 per quarter ring * 4
    'lift_deg_teeth_ratio': 7.5,  # 75 degree turn on the lift motor results in
                                  # 10 teeth on the lift rack which is one full
                                  # length four piece.  The entire rack is sev
                                  # pieces or 70 teeth in which the range only
                                  # goes to approximately 55 teeth. That trans-
                                  # lates into approximately 225 degrees of
                                  # motion on the drive motor.
                                  # This is intended for speed but will run into
                                  # issues if we need more torque/lift power.
    'hub_class': InventorHub,
    'top_face_direction': Axis.Z,
    'front_face_direction': Axis.X,
}


def twist_params(x):
    w_g = 110
    l_g = 202
    r_g = ((110/2)**2 + 202**2)**0.5
    try:
        theta = acos(x/r_g)
    except ValueError as e:
        print(x, r_g)
        raise ValueError from e

    y = r_g * sin(theta)
    return y, theta * 180 / pi


class Motor(Motor):
    def parameterize(self, equation):
        current_angle = self.angle()
        def new_equation(time):
            target_angle = equation(time) + current_angle
            self.track_target(target_angle)
        return new_equation
    
    def parametric_ratio(self, mm=1):
        return mm * 360 / _CONFIG['wheel_diameter'] / pi
    
    def get_wave_parametric(self, amplitude, start, end):
        amplitude = self.parametric_ratio(amplitude)
        start = start * pi / 180
        end = end * pi / 180
        shift = amplitude * sin(start)
        def parametric(t):
            theta = start + (end-start) * t
            return amplitude * sin(theta) - shift
        return parametric

class Lift(Motor):
    teeth_per_unit = 2.5
    teeth_per_mm = 2.5 / 8

    def ratio(self):
        return self.teeth_per_unit * _CONFIG['lift_deg_teeth_ratio']
    
    def mm2deg(self, mm=1):
        return mm * self.teeth_per_mm * _CONFIG['lift_deg_teeth_ratio']
    
    def deg2mm(self, deg=1):
        return deg / self.mm2deg()
    
    def parametric_ratio(self, unit):
        return self.mm2deg(unit)

    def unit_to_deg(self, unit):
        return unit * self.ratio()
    
    def deg_to_unit(self, deg):
        return deg / self.ratio()

    def unit(self):
        return self.deg_to_unit(self.angle())
    
    def liftto(self, unit, speed=90, **kwargs):
        return self.run_target(speed, self.unit_to_deg(unit), **kwargs)
        
    def liftby(self, unit, speed=90, **kwargs):
        start = self.deg_to_unit(self.angle())
        target = start + unit
        return self.liftto(target, speed, **kwargs)

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
    
    def parametric_ratio(self, deg=1):
        return deg * self.ratio()

    def ring_track(self, angle):
        self.track_target(self.parametric_ratio(angle))

class Drive(DriveBase):
    def tern(self, angle, trn_rt=90, trn_acc=90, *args, **kwargs):
        s_spd, s_acc, t_rate, t_acc = self.settings()
        self.settings(s_spd, s_acc, trn_rt, trn_acc)
        result = self.turn(angle, *args, **kwargs)
        self.settings(s_spd, s_acc, t_rate, t_acc)
        return result

    def strait(self, distance, speed=200, acceleration=500, *args, **kwargs):
        s_spd, s_acc, t_rate, t_acc = self.settings()
        self.settings(speed, acceleration, t_rate, t_acc)
        result = self.straight(distance, *args, **kwargs)
        self.settings(s_spd, s_acc, t_rate, t_acc)
        return result
    
    def distance_to_angle(self, distance):
        return distance * 360 / (pi * _CONFIG['wheel_diameter'])

    
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
        self.wheel_diameter = _CONFIG['wheel_diameter']
        self.wheel_base = _CONFIG['wheel_base']
        self.hub = hub_type(
            top_side=_CONFIG['top_face_direction'],
            front_side=_CONFIG['front_face_direction']
        )
        self.left_motor = Motor(left_motor, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_motor, Direction.CLOCKWISE)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.ring = Ring(ring_motor, Direction.CLOCKWISE)
        self.ring.reset_angle(0)
        self.ring.control.target_tolerances(10, 5)
        self.lift = Lift(lift_motor, Direction.COUNTERCLOCKWISE)
        self.lift.reset_angle(0)
        self.left_eye = ColorSensor(left_eye)
        self.right_eye = ColorSensor(right_eye)
        self.drive = Drive(
            self.left_motor,
            self.right_motor,
            self.wheel_diameter,
            self.wheel_base
        )
        self.drive.use_gyro(True)
        self.drive.heading_control.target_tolerances(10, 5)


    def twist(self, *args, **kwargs):
        return self.ring.twist(*args, **kwargs)

    def liftby(self, *args, **kwargs):
        return self.lift.liftby(*args, **kwargs)

    def liftto(self, *args, **kwargs):
        return self.lift.liftto(*args, **kwargs)

    def turn(self, *args, **kwargs):
        return self.drive.tern(*args, **kwargs)
    
    def straight(self, *args, **kwargs):
        return self.drive.straight(*args, **kwargs)

    def strait(self, *args, **kwargs):
        return self.drive.strait(*args, **kwargs)

    def left_angle(self):
        return self.left_motor.angle()

    def right_angle(self):
        return self.right_motor.angle()

    def wheel_circ(self):
        return self.wheel_diameter * pi

    def wheel_ratio(self):
        return self.wheel_diameter / self.wheel_base

    def drive_mean(self):
        return (self.left_angle() + self.right_angle()) / 2

    def drive_diff(self):
        return (self.left_angle() - self.right_angle()) / 2

    def dead_head(self):
        return self.drive_diff() * self.wheel_ratio()

    def dead_reck(self):        
        return self.drive_mean() * self.wheel_circ() / 360

    def track_parametric(
        self, duration, left=None, right=None, lift=None, ring=None,
    ):
        equations = []
        if left is not None:
            equations.append(self.left_motor.parameterize(left))
        if right is not None:
            equations.append(self.right_motor.parameterize(right))
        if lift is not None:
            equations.append(self.lift.parameterize(lift))
        if ring is not None:
            equations.append(self.ring.parameterize(ring))

        async def track():
            watch = StopWatch()
            start = watch.time()
            time = 0
            while time < duration:
                time = watch.time() - start
                t = time / duration
                for equation in equations:
                    equation(t)
        
        async def multitask_track():
            await multitask(track(), wait(duration*1.1))

        return run_task(multitask_track())

    def accu_turn(self, angle, trn_rt=90, tolerance=0.25):
        start_angle = self.dead_head()
        current_angle = start_angle
        togo = angle
        while abs(togo) > tolerance:
            turn_rate = max(min(trn_rt, abs(togo)*5), 50)
            self.turn(togo, trn_rt=turn_rate)
            current_angle = self.dead_head()
            togo = angle - (current_angle-start_angle)
        return self.drive.stop()

    def accu_straight(self, distance, speed=100, tolerance=0.25):
        start_position = self.dead_reck()
        current_position = start_position
        togo = distance 
        while abs(togo) > tolerance:
            modified_speed = max(min(speed, abs(togo)*5), 45)
            self.strait(togo, speed=modified_speed)
            current_position = self.dead_reck()
            togo = distance - (current_position-start_position)
        return self.drive.stop()

    def twist_turn(self, angle, trn_rt=120, trn_acc=120):
        async def tt():
            await multitask(
                self.turn(angle, trn_rt=trn_rt, trn_acc=trn_acc),
                self.twist(angle, trn_rt=trn_rt, trn_acc=trn_acc)
            )
        return run_task(tt())

    def move_lift(self, duration, drive_kwargs, lift_kwargs):
        lift = self.lift.get_wave_parametric(**lift_kwargs)
        left = self.left_motor.get_wave_parametric(**drive_kwargs)
        right = self.right_motor.get_wave_parametric(**drive_kwargs)
        return self.track_parametric(duration, lift=lift, right=right, left=left)

    def ring_turn(self, duration, angle):
        drive_angle = angle / self.wheel_ratio()
        ring_angle = -self.ring.parametric_ratio(angle)

        def ring(t):
            return ring_angle * t

        def right(t):
            return drive_angle * t

        def left(t):
            return -right(t)

        return self.track_parametric(duration, ring=ring, left=left, right=right)

    async def grab_callback(self, bot_pos, waypoints):
        """this assumes we are going straight at a given x coord
        and we need to swing the gatherer back and forth to collect
        objects off the central path.

        We'll take each target and calculate the distance required to position the
        gatherer sweet spot on top of the target.

        This function will swing the gather to the correct angle a buffered amount
        prior to the target
        """
        bot_x, bot_y = bot_pos
        waypoints = sorted(waypoints, key=lambda way: way[1])
        enhanced_waypoints = []
        for waypoint in waypoints:
            way_x, way_y = waypoint
            x = bot_x - way_x
            delta, theta = twist_params(x)
            enhanced_waypoints.append({
                'way_x': way_x,
                'way_y': way_y,
                'delta': delta,
                'theta': theta,
                'x': x,
                'thresh': waypoint[1] - bot_y - delta
            })
            
        right_buffer = 10
        start_pos = self.dead_reck()
        dist = 0
        if not waypoints:
            return
        last = max(enhanced_waypoints, key=lambda way: way['thresh'])
        recent = None
        while dist < last['thresh'] + right_buffer:
            curr = max(enhanced_waypoints, key=lambda way: dist < way['thresh'])
            self.ring.ring_track(curr['theta'])
            await wait(0)
            dist = self.dead_reck() - start_pos

    def grab_on_the_go(self, distance, speed, bot_pos, waypoints):
        async def task():
            await multitask(
                self.straight_with_heading_callback(distance, 0, speed=speed),
                self.grab_callback(bot_pos, waypoints),
                race=True
            )
        return run_task(task())

    async def pivot_callback(self, hyp, distance):
        start = self.dead_reck()
        pos = start
        theta_0 = self.ring.ring_angle()
        base = sin(theta_0 * pi / 180)
        ratio = (pos - start) / hyp
        opp = max(min(base - ratio, 1), -1)
        target_angle = asin(opp) * 180 / pi
        recent = None
        while abs(pos - start - distance) > 1:
            pos = self.dead_reck()
            ratio = (pos - start) / hyp
            opp = max(min(base - ratio, 1), -1)
            target_angle = asin(opp) * 180 / pi
            self.ring.ring_track(target_angle)
            await wait(0)

    def pivot_and_go(self, distance, speed, hyp, timeout=5000):
        async def task():
            await multitask(
                self.strait(distance, speed=speed),
                self.pivot_callback(hyp, distance),
                wait(timeout),
                race=True
            )
        return run_task(task())

    async def straight_with_heading_callback(self, distance, heading, speed, gain=2):
        speed = abs(speed)
        if distance < 0:
            speed *= -1

        start = self.dead_reck()
        travel = 0

        while abs(travel - distance) > 1:
            correction = heading - self.drive.angle()
            self.drive.drive(speed, correction * gain)
            await wait(10)
            travel = self.dead_reck() - start

        self.drive.stop()

    def straight_with_heading(self, *args, **kwargs):
        return run_task(self.straight_with_heading_callback(*args, **kwargs))
