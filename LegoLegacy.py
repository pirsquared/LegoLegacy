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
    'front_face_direction': Axis.Y,
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

def curve_get_r_theta_from_x_y(x, y):
    xx = x * x
    yy = y * y
    xy = x * y
    return (xx + yy)/(2*x), asin(2*xy/(xx + yy)) / pi * 180

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

    async def _run_t(self, *args, **kwargs):
        await self.run_target(*args, **kwargs)

    def run_t(self, *args, **kwargs):
        return run_task(self._run_t(*args, **kwargs))

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

    async def _forklift(self, distance, speed, **kwargs):
        await self.run_target(speed, self.mm2deg(distance), **kwargs)

    def forklift(self, *args, **kwargs):
        return run_task(self._forklift(*args, **kwargs))
        
    def liftby(self, pegs, speed=90, **kwargs):
        start = self.deg2peg(self.angle())
        target = start + pegs
        return self.lift_target(target, speed, **kwargs)

    async def _lift(self, speed, distance, duty_limit=80, *args, **kwargs):
        lift0 = self.angle()
        target = lift0 + self.mm2deg(distance)

        kwargs['wait'] = False
        self.run_target(speed, target, *args, **kwargs)

        while abs(self.angle() - target) > self.control.target_tolerances()[1]:
            dl = abs(self.load() * 100 / self.control.limits()[2])
            if dl >= duty_limit:
                print(f'Attempted {distance}')
                print(f'Achieved {self.parametric_ratio(self.angle() - lift0)}')
                print(f'Duty {dl}')
                self.stop()
                break
            await wait(10)

    def lift(self, *args, **kwargs):
        return run_task(self._lift(*args, **kwargs))

    async def _bump(self, speed, distance, *args, **kwargs):
        lift0 = self.angle()
        await self._lift(speed, distance, *args, **kwargs)
        bump_distance = self.deg2mm(lift0 - self.angle())
        await self._lift(speed, bump_distance, *args, **kwargs)

    def bump(self, *args, **kwargs):
        return run_task(self._bump(*args, **kwargs))

class Ring(Motor):
    """"""
    ratio = _CONFIG['ring_outer_teeth'] / _CONFIG['ring_drive_teeth']
    orientation = -1

    async def _twist_target(self, angle, speed=1000, kpf=10, **kwargs):
        spd_tol, pos_tol = self.control.target_tolerances()
        self.control.target_tolerances(spd_tol, 1)
        kp, *pid_params = self.control.pid()
        self.control.pid(kp * kpf, *pid_params)
        adj_angle = self.orientation * self.parametric_ratio(angle)
        result = await self.run_target(speed, adj_angle, **kwargs)
        self.control.target_tolerances(spd_tol, pos_tol)
        self.control.pid(kp, *pid_params)
        return result

    def twist_target(self, *args, **kwargs):
        return run_task(self._twist_target(*args, **kwargs))

    async def twist_target_async(self, angle, speed=1000, **kwargs):
        adj_angle = self.orientation * self.parametric_ratio(angle)
        await self.run_target(speed, adj_angle, **kwargs)

    def twist_track(self, angle):
        adj_angle = self.orientation * self.parametric_ratio(angle)
        return self.track_target(adj_angle)

    def ring_angle(self):
        return self.orientation * self.angle() / self.ratio
    
    def parametric_ratio(self, deg=1):
        return deg * self.ratio

class Drive(DriveBase):
    wheel_diameter = _CONFIG['wheel_diameter']
    wheel_base = _CONFIG['wheel_base']
    wheel_ratio = wheel_diameter / wheel_base
    
    def mm2deg(self, mm=1):
        return mm * 360 / self.wheel_diameter / pi
    
    def turn(self, angle, turn_rate, *args, **kwargs):
        ss, sa, tr, ta = self.settings()
        self.settings(ss, sa, turn_rate, ta)
        result = super().turn(angle, *args, **kwargs)
        self.settings(ss, sa, tr, ta)
        return result

    def curve(self, radius, angle, speed, *args, **kwargs):
        ss, sa, tr, ta = self.settings()
        self.settings(speed, sa, tr, ta)
        result = super().curve(radius, angle, *args, **kwargs)
        self.settings(ss, sa, tr, ta)
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
        self.wheel_diameter = _CONFIG['wheel_diameter']
        self.wheel_circumference = self.wheel_diameter * pi
        self.wheel_base = _CONFIG['wheel_base']
        self.wheel_ratio = self.wheel_diameter / self.wheel_base
        self.hub = hub_type(
            top_side=_CONFIG['top_face_direction'],
            front_side=_CONFIG['front_face_direction']
        )
        self.left_eye = ColorSensor(left_eye)
        self.right_eye = ColorSensor(right_eye)
        self.left_motor = Motor(left_motor, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(right_motor, Direction.CLOCKWISE)
        self.ring = Ring(ring_motor, Direction.CLOCKWISE)
        self.ring.control.target_tolerances(10, 3)
        # default: self.ring.control.pid(42484, 21242, 5310, 8, 15)
        # self.ring.control.pid(42484*10, 21242*1, 5310*1, 8, 1000)
        self.lift = Lift(lift_motor, Direction.COUNTERCLOCKWISE, profile=5)

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

    def get_state(self):
        return (
            self.lift.angle(),
            self.ring.angle(),
            self.left_motor.angle(),
            self.right_motor.angle(),
            self.hub.imu.heading()
        )

    def set_state(self, lift, ring, left, rignt, head):
        self.lift.reset_angle(lift)
        self.ring.reset_angle(ring)
        self.left_motor.reset_angle(left)
        self.right_motor.reset_angle(right)
        self.hub.imu.reset_heading(head)

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
            self.ring.twist_target(current_wp['theta'], wait=False)
            await wait(wait_time)
            dist = self.dead_reck() - start_pos

    async def _straight(self, distance, heading, speed, gain=2, wait_time=5, verbose=False):
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

    def straight(self, *args, **kwargs):
        return run_task(self._straight(*args, **kwargs))

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

    def straight_at_sync(self, *args, **kwargs):
        return run_task(self.straight_at(*args, **kwargs))


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

    async def maintain_ring_orientation(self, angle=None, wait_time=10):
        if angle is None:
            angle = self.ring_angle()

        head0 = self.heading()

        while True:
            self.ring.twist_track(angle + (self.heading() - head0))
            await wait(wait_time)

    async def _turn(self, angle, turn_rate, twist=False, orientation=None, **kwargs):
        if twist:

            async def turn(angle, turn_rate, **kwargs):
                await self.drive.turn(angle, turn_rate, **kwargs)

            await multitask(
                turn(angle, turn_rate, **kwargs),
                self.maintain_ring_orientation(orientation),
                race=True,
            )

        else:
            await self.drive.turn(angle, turn_rate, **kwargs)

    async def _curve(self, radius, angle, speed, twist=False, orientation=None, **kwargs):
        if twist:

            async def curve(radius, angle, speed, **kwargs):
                await self.drive.curve(radius, angle, speed, **kwargs)

            await multitask(
                curve(radius, angle, speed, **kwargs),
                self.maintain_ring_orientation(orientation),
                race=True,
            )

        else:
            await self.drive.curve(radius, angle, speed, **kwargs)

    def turn(self, *args, **kwargs):
        return run_task(self._turn(*args, **kwargs))

    def curve(self, *args, **kwargs):
        return run_task(self._curve(*args, **kwargs))

    def curve_links(self, radius, angles, speed, **kwargs):
        results = []
        orientation = kwargs.get('orientation', self.ring_angle())
        head0 = self.heading()
        expected_final_heading = head0 + sum(angles)
        delta = orientation - head0
        for angle in angles[:-1]:
            results.append(self.curve(
                radius, angle, speed, then=Stop.NONE,
                orientation=delta + self.heading(), **kwargs
            ))
        results.append(
            self.curve(radius, angles[-1], speed, then=Stop.HOLD,
            orientation=delta + self.heading(), **kwargs
        ))
        self.turn(
            expected_final_heading - self.heading(),
            turn_rate=45,
            twist=True,
            orientation=delta + self.heading()
        )
        return results

    async def _twist_target(self, *args, **kwargs):
        await self.ring._twist_target(*args, **kwargs)

    def twist_target(self, *args, **kwargs):
        return run_task(self._twist_target(*args, **kwargs))

    async def _lrlt(self, left, right, lift, wait_time):
        self.left_motor.track_target(left)
        self.right_motor.track_target(right)
        self.lift.track_target(lift)
        await wait(wait_time)

    def lrlt(self, *args, **kwargs):
        return run_task(self._lrlt(*args, **kwargs))

    async def _lrl(self, left, right, lift, speed=1000, then=Stop.HOLD):
        await multitask(
            self.left_motor._run_t(speed, left, then=then),
            self.right_motor._run_t(speed, right, then=then),
            self.lift._run_t(speed, lift, then=then)
        )

    def lrl(self, *args, **kwargs):
        return run_task(self._lrl(*args, **kwargs))

    async def _para_lift_drive(self, lift_f, drive_f, n=100, wait_time=100):

        left0 = self.left_angle()
        right0 = self.right_angle()
        lift0 = self.lift.angle()

        time = 0
        for i in range(n):
            time = (i + 0.5)/n

            drive_deg = self.right_motor.parametric_ratio(drive_f(time))
            lift_deg = self.lift.parametric_ratio(lift_f(time))
            
            await self._lrlt(
                left  = drive_deg + left0,
                right = drive_deg + right0,
                lift  = lift_deg  + lift0,
                wait_time = wait_time
            )

        drive_deg = self.right_motor.parametric_ratio(drive_f(1))
        lift_deg = self.lift.parametric_ratio(lift_f(1))
            
        await self._lrl(
            left  = drive_deg + left0,
            right = drive_deg + right0,
            lift  = lift_deg  + lift0,
        )

    def para_lift_drive(self, *args, **kwargs):
        return run_task(self._para_lift_drive(*args, **kwargs))



        

