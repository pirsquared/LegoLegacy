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
    'wheel_base': 128,            # The distance from the center of one wheel to
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

    'ring_drive_teeth': 24,
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
    'gatherer_length': 200,       # Measured @ 202 but 200 puts focus more in 
    'gatherer_width': 110,        # middle of gatherer.  Might want to put back
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
        return -mm * self.teeth_per_mm * _CONFIG['lift_deg_teeth_ratio']
    
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

    def twist_target(self, angle, speed=1000, kpf=10, **kwargs):
        spd_tol, pos_tol = self.control.target_tolerances()
        self.control.target_tolerances(spd_tol, 1)
        kp, *pid_params = self.control.pid()
        self.control.pid(kp * kpf, *pid_params)
        result = self.run_target(speed, self.parametric_ratio(angle), **kwargs)
        self.control.target_tolerances(spd_tol, pos_tol)
        self.control.pid(kp, *pid_params)
        return result

    async def twist_target_async(self, angle, speed=1000, **kwargs):
        await self.run_target(speed, self.parametric_ratio(angle), **kwargs)

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
        self.ring = Ring(ring_motor, Direction.CLOCKWISE)
        self.ring.control.target_tolerances(10, 3)
        # default: self.ring.control.pid(42484, 21242, 5310, 8, 15)
        # self.ring.control.pid(42484*10, 21242*1, 5310*1, 8, 1000)
        self.lift = Lift(lift_motor, Direction.COUNTERCLOCKWISE)

        self.drive = Drive(
            self.left_motor,
            self.right_motor,
            self.wheel_diameter,
            self.wheel_base
        )
        self.drive.use_gyro(True)
        self.drive.heading_control.target_tolerances(10, 3)
        self.stopwatch = StopWatch()
        # default: self.drive.heading_control.pid(7558, 0, 1889, 3, 5)
        # self.drive.heading_control.pid(7558*25, 0, 1889, 3, 5)

        self.reset()

    def reset(self):
        self.lift.reset_angle(0)
        self.ring.reset_angle(0)
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.hub.imu.reset_heading(0)
        self.drive.reset()

    def ring_kp(self, kp=None):
        if kp is None:
            return self.ring.control.pid()[0]
        *pid_params, = self.ring.control.pid()
        pid_params[0] = kp
        self.ring.control.pid(*pid_params)

    def ring_tol(self, *args, **kwargs):
        return self.ring.control.target_tolerances(*args, **kwargs)

    async def goturn_async(self, distance, heading, speed, angle):
        await multitask(
            self.straight_at(distance, heading, speed),
            self.ring.twist_target(angle)
        )

    def goturn(self, *args, **kwargs):
        run_task(self.goturn_async(*args, **kwargs))


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

    def liftup(self, mm=None, *args, **kwargs):
        if mm is None:
            return self.lift.run_until_stalled(300, duty_limit=50)
        angle = self.lift.angle() + self.lift.mm2deg(mm)
        return self.lift.run_target(300, angle, *args, **kwargs)

    def liftdown(self, mm=None, *args, **kwargs):
        if mm is None:
            return self.lift.run_until_stalled(-300, duty_limit=50)
        angle = self.lift.angle() - self.lift.mm2deg(mm)
        return self.lift.run_target(300, angle, *args, **kwargs)

    async def grab(self, bot_pos, waypoints, wait_time=5, verbose=False):
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

        current_wp, *remaining_wp = enhanced_waypoints
        while True:
            while remaining_wp and current_wp['thresh'] < dist:
                current_wp, *remaining_wp = remaining_wp
        
            if verbose:
                print(f"   ↪ Current wp ({current_wp['way_x']:>4}, {current_wp['way_y']:>4})       ts: {self.stopwatch.time():>4}")

            angle = self.ring.parametric_ratio(current_wp['theta'])
            self.ring.track_target(angle)
            await wait(wait_time)
            dist = self.dead_reck() - start_pos

    async def straight_at(self, distance, heading, speed, gain=2, wait_time=5, verbose=False):
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
            if verbose:
                print(f"Current travel {travel:>10.2f}          ts: {self.stopwatch.time():>4}")
            await wait(wait_time)
            travel = self.dead_reck() - start

        self.drive.stop()

    async def straight_at_and_grab(self, distance, heading, speed, bot_pos, waypoints, wait_time=5):
        await multitask(
            self.grab(bot_pos, waypoints, wait_time),
            self.straight_at(distance, heading, speed=speed, wait_time=wait_time),
            race=True
        )

    async def pivot(self, hypotenuse, distance_target, wait_time=5):
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
            angle = self.ring.parametric_ratio(target_angle)
            self.ring.track_target(angle)
            await wait(wait_time)
            distance_travelled = position - start

    async def pivot_and_go(self, distance, heading, speed, hypotenuse, wait_time=5):
        await multitask(
            self.straight_at(distance, heading, speed, wait_time=wait_time),
            self.pivot(hypotenuse, distance, wait_time=wait_time)
        )

    def duty_twist(self, angle, direction=1, speed=400, timeout=2000):
        ring0 = self.ring_angle()
        ring_diff = (angle - ring0) % 360
        if direction > 0:
            ring_diff -= 360
        target_angle = ring0 + ring_diff
        parametric_angle = self.ring.parametric_ratio(target_angle)
        
        time0 = self.stopwatch.time()
        while abs(target_angle - self.ring_angle()) > 1:
            self.ring.run_target(speed=speed, target_angle=parametric_angle, wait=False)
            if self.ring.stalled() or self.stopwatch.time() - time0 > timeout:
                self.ring.stop()
                break

    def sync_twist_turn(self, heading, base_turn_rate, twist=True, steps=100, wait_time=10, lead=1):
        direction = 1 if base_turn_rate > 0 else -1
        base_turn_rate = abs(base_turn_rate)
        head0 = self.heading()
        ring0 = self.ring_angle()
        head_diff = (heading - head0) % 360
        if direction < 0:
            head_diff -= 360

        target_heading = head0 + head_diff

        left_start = self.left_angle()
        right_start = self.right_angle()
        wheel_diff = head_diff * self.wheel_base / self.wheel_diameter
        left_target = left_start + wheel_diff
        right_target = right_start - wheel_diff

        for i in range(steps+1):
            wheel_delta = i * wheel_diff / steps
            angle_delta = (i+lead) * head_diff / steps

            self.left_motor.track_target(left_start + wheel_delta)
            self.right_motor.track_target(right_start - wheel_delta)
            if twist:
                self.ring.twist_target(ring0-angle_delta, speed=3000, kpf=100, wait=False)
            wait(wait_time)


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

            await wait(1)
            head_flag = abs(head_target - self.heading()) > 1
            twist_flag = abs(ring_target - self.ring_angle()) > 1

        self.drive.stop()

    async def twist_at(self, angle):
        self.ring.track_target(self.ring.parametric_ratio(angle))
        await wait(0)



