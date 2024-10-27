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
    'gatherer_length': 110,
    'gatherer_width': 202,
}


def twist_params(adjacent):
    """Given the horizontal displacement from the center of the bot to the target,
    calculate the angle and vertical displacement required to position the gatherer."""
    width = _CONFIG['gatherer_width']
    length = _CONFIG['gatherer_length']
    hypotenuse = ((width/2)**2 + length**2)**0.5
    try:
        theta = acos(adjacent/hypotenuse)
    except ValueError as e:
        print(adjacent, hypotenuse)
        raise ValueError from e

    y = hypotenuse * sin(theta)
    return y, theta * 180 / pi


class Motor(Motor):
    def make_para_callback(self, equation):
        current_angle = self.angle()
        def new_equation(time):
            target_angle = equation(time) + current_angle
            self.track_target(target_angle)
        return new_equation
    
    def parametric_ratio(self, mm=1):
        """Converts a distance in mm to an angle in degrees.
        Intended to be overridden by subclasses as needed."""
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
    ratio = teeth_per_unit * _CONFIG['lift_deg_teeth_ratio']
    
    def mm2deg(self, mm=1):
        return mm * self.teeth_per_mm * _CONFIG['lift_deg_teeth_ratio']
    
    def deg2mm(self, deg=1):
        return deg / self.mm2deg()
    
    def parametric_ratio(self, unit):
        return self.mm2deg(unit)

    def peg2deg(self, unit):
        return unit * self.ratio()
    
    def deg2peg(self, deg):
        return deg / self.ratio()

    def peg(self):
        return self.deg2peg(self.angle())
    
    def lift_target(self, pegs, speed=90, **kwargs):
        return self.run_target(speed, self.peg2deg(pegs), **kwargs)
        
    def liftby(self, pegs, speed=90, **kwargs):
        start = self.deg2peg(self.angle())
        target = start + pegs
        return self.lift_target(target, speed, **kwargs)

class Ring(Motor):
    ratio = _CONFIG['ring_outer_teeth'] / _CONFIG['ring_drive_teeth']

    def twist_target(self, angle, speed=1000, **kwargs):
        return self.run_target(speed, self.parametric_ratio(angle), **kwargs)

    def ring_angle(self):
        return self.angle() / self.ratio
    
    def parametric_ratio(self, deg=1):
        return deg * self.ratio

class Drive(DriveBase):
    wheel_diameter = _CONFIG['wheel_diameter']
    wheel_base = _CONFIG['wheel_base']
    wheel_ratio = wheel_diameter / wheel_base
    
    def mm2deg(self, mm=1):
        return mm * 360 / self.wheel_diameter / pi
    
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
        self.wheel_circumference = self.wheel_diameter * pi
        self.wheel_base = _CONFIG['wheel_base']
        self.wheel_ratio = self.wheel_diameter / self.wheel_base
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
        self.ring.control.target_tolerances(10, 1)
        # default: self.ring.control.pid(42484, 21242, 5310, 8, 15)
        self.ring.control.pid(42484*10, 21242*1, 5310*1, 8, 1000)
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
        self.drive.heading_control.target_tolerances(10, 1)
        # default: self.drive.heading_control.pid(7558, 0, 1889, 3, 5)
        # self.drive.heading_control.pid(7558*25, 0, 1889, 3, 5)

    def heading(self):
        return self.hub.imu.heading()

    def ring_angle(self):
        return self.ring.ring_angle()

    def left_angle(self):
        return self.left_motor.angle()

    def right_angle(self):
        return self.right_motor.angle()

    def drive_mean_angle(self):
        return (self.left_angle() + self.right_angle()) / 2

    def drive_diff_angle(self):
        return (self.left_angle() - self.right_angle()) / 2

    def dead_head(self):
        return self.drive_diff_angle() * self.wheel_ratio

    def dead_reck(self):        
        return self.drive_mean_angle() * self.wheel_circumference / 360

    async def grab(self, bot_pos, waypoints):
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
        while dist < last['thresh'] + right_buffer:
            curr = max(enhanced_waypoints, key=lambda way: dist < way['thresh'])
            await self.ring.twist_target(curr['theta'])
            dist = self.dead_reck() - start_pos

    async def straight_at(self, distance, heading, speed, gain=2):
        speed = abs(speed)
        direction = 1
        if distance < 0:
            direction = -1
            speed *= -1

        start = self.dead_reck()
        travel = 0

        while (distance - travel) * direction > 1:
            correction = heading - self.drive.angle()
            self.drive.drive(speed, correction * gain)
            await wait(0)
            travel = self.dead_reck() - start

        self.drive.stop()

    async def straight_at_and_grab(self, distance, heading, speed, bot_pos, waypoints):
        await multitask(
            self.straight_at(distance, heading, speed=speed),
            self.grab(bot_pos, waypoints)
        )

    async def pivot(self, hypotenuse, distance_target):
        start = self.dead_reck()
        theta0 = self.ring.ring_angle()
        sin0 = sin(theta0 * pi / 180)
        opp0 = sin0 * hypotenuse
        target_y = start + opp0
        distance_travelled = 0
        while abs(distance_travelled - distance_target) > 1:
            position = self.dead_reck()
            delta_y = target_y - position
            # clip the ratio to between -1 and 1
            # when the delta_y exceeds the hypotenuse
            # we still want to navigate to the extreme
            sin_now = max(min(delta_y / hypotenuse, 1), -1)
            target_angle = asin(sin_now) * 180 / pi
            await self.ring.twist_target(target_angle)
            distance_travelled = position - start
            


    async def pivot_and_go(self, distance, heading, speed, hypotenuse):
        # pivot isn't quite working and I don't have time to debug
        # just going to use twist_target
        angle = heading if distance < 0 else heading - 180
        turn_rate = self.ring.parametric_ratio(angle) * speed / distance
        await multitask(
            self.straight_at(distance, heading, speed),
            self.ring.twist_target(-90, speed=turn_rate)
        )

    async def twist_turn(self, angle, base_turn_rate):
        base_turn_rate = abs(base_turn_rate)
        head0 = self.heading()
        ring0 = self.ring_angle()

        head_target = head0 + angle
        ring_target = ring0 + angle

        change = self.heading() - head0

        head_flag = abs(head_target - self.heading()) > 1
        twist_flag = abs(ring_target - self.ring_angle()) > 1

        while head_flag or twist_flag:
            heading = self.heading()

            sign = 1 if head_target > heading else -1
            change = heading - head0
            ring_target_now = ring0 + change

            factor = abs(head_target - heading) / 20

            turn_rate = min(base_turn_rate, base_turn_rate * factor)
            
            self.drive.drive(0, turn_rate * sign)
            self.ring.track_target(self.ring.parametric_ratio(ring_target_now))

            await wait(0)
            head_flag = abs(head_target - self.heading()) > 1
            twist_flag = abs(ring_target - self.ring_angle()) > 1

        self.drive.stop()

    async def twist_at(self, angle):
        self.ring.track_target(self.ring.parametric_ratio(angle))
        await wait(0)




