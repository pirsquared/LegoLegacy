from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import multitask, run_task, wait, StopWatch, hub_menu
from umath import pi, sin, cos, asin, acos, sqrt, degrees, atan2
from LegoLegacy import Bot, clear_console, curve_get_r_theta_from_x_y


def lift_f(time):
    peg = 8
    if time < .1:
        return time * peg / .1
    if time < .9:
        t = (time - .1) / .8
        return 80 * sin(t * pi) + peg
    return (1 - time) * peg / .1

def drive_f(time):
    if time < .1:
        return 0
    if time < .9:
        t = (time - .1) / .8
        return 80 * (1 - cos(t * pi))
    return 160

def drive_r(time):
    return -drive_f(time)

def scurve(x, y):
    xx = x*x
    yy = y*y
    xx_yy = xx + yy
    xy = x*y
    return (xx_yy)/(4*x), asin(2*xy/(xx_yy)) * 180 / pi

def arc_radius_and_angle(x, y):
    a_dir = 1 if x > 0 else -1
    r_dir = 1 if y > 0 else -1

    x = abs(x)
    y = abs(y)

    if x > y:
        raise ValueError('Keep x <= y')

    xy = x*y
    xx = x*x
    yy = y*y
    xx_yy = xx+yy

    angle = asin(2*xy/xx_yy) * 180 / pi
    radius = xx_yy/2/x

    return r_dir * radius, a_dir * angle



class GameBot(Bot):
    """                                                       North
      ██████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
      █   ███████                 ███████                ███████     ██████               ██████████                           █
      █  ████████                 ███ ███                ███████   █████████             ███      ███                   █████  █
      █  ██  ███                  ███ ████                           █████               ███      ███                   █████  █
      █  ███████                  ███ ████                            ██      ████       ███      ███           ███            █
      █       █████               ████████                                   ████         ██████████           ██████          █
      █        ██                                              █████████████████            █████                ██████        █
      █                                                       ████████████████                                     █████       █
      █                                                                                                                        █
      █                                                                                                                █████████
      █                       ██                                                                       ███             █████████
      █   ██       █          ██                                           ███                                                 █
      █ ██████████ █              █████                                  ██   ██                                               █
      █  ████      █              █████     █                            ██   ██                                  ████         █
      █                  ███                █                ██            ███                              ███     ████       █
    W █   ██                                █  ██████      ██                         ████                            ████     █ E
    e █   ██                                █  ███████    ██████                      ██████                            ████   █ a
    s █                                     █  ██████   ████████                        ████                                   █ s
    t █                                     █           ████████                              ██                               █ t
      █                        ██           █        ███         ███                        ██        ███                      █
      █                                                       ██  ██                                                           █
      ████████                                                  ███                                                     ████████
      █████████████                                            ██                                                   ████████████
      ████████████████                                                                                          ████████████████
      ████████West███████                                                ██                                  █████East██████████
      ████████Launch████████                                            ████                              ████████Launch████████
      ███████████████████████                                           ████                             ███████████████████████
      █████████████████████████                                         ████                           █████████████████████████
      ██████████████████████████                                        ████                          ██████████████████████████
      ███████████████████████████                                       █████                        ███████████████████████████
      ████████████████████████████                                       ████                       ████████████████████████████
      ████████████████████████████                                      █████                       ████████████████████████████
      █████████████████████████████                                     █████                      █████████████████████████████
      ██████████████████████████████  ██████                                               ██████ ██████████████████████████████
      ██████████████████████████████  ██                                                       ██ ██████████████████████████████
      ██████████████████████████████  ██                                                       ██ ██████████████████████████████
      ██████████████████████████████  ██████                                               ██████ ██████████████████████████████
                                                            South
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.head0 = 0

    def set_head0(self, head0):
        self.head0 = head0

    def heading(self):
        return super().heading() + self.head0
    
    def pause(self, msg='', ask=True):
        print(f'{msg:<30} | H: {self.heading():8.2f} | R: {self.ring_angle():8.2f}')
        
        if ask:
            p = hub_menu('G', 'S')
            if p == 'S':
                run_task(multitask(
                    self._turn(-bot.heading(), 50),
                    self._twist_target(0)
                ))
                self.liftdown()
                raise ValueError('Hmm!')
            else:
                self.hub.display.char('@')

    def phase0(self):
        """This is the first phase of the game and will start in the right launch area.
        with the robot facing North with the forklift facing north and the gatherer
        facing west.  The right side of the rear block should be aligned with the
        line near 150mm from the west edge of the field.

        Plan:
        Drive straight 100mm at 200mm/s with a heading of 0 degrees. This will get the
        robot off the wall so that it can turn into the "Unexpected Encounter" mission.

        In order to nab the krill, the robot will need to turn 45 degrees to the left
        but the ring will need to be offset by 10 degrees to the right.  This will allow
        the ring to be in the correct orientation to grab the krill.  After gathering
        the krill, the ring will twist back to aligned with the forward direction
        (-90 degrees) in order to use the pusher at the front of the gatherer to push
        the "Unexpected Encounter" model and catch the squid in the gatherer basket.

        We back off the "Unexpected Encounter" model and then curve to the right to
        gather the coral and prepare for the "Change Shipping Lanes" mission.  As we are
        about to align with the "Change Shipping Lanes" mission, we will twist the ring
        to secure the coral in the gatherer.

        In order to perform the "Change Shipping Lanes" mission, we will need to align
        the forklift, move forward, turn 90 degrees to the right while maintaining the
        orientation of the ring.  Then we have to utilize parametric lift and drive
        functions in order to trace a semi-circle with the forklift.  This enables the
        smooth movement of the mission.

        We then turn north and gather another krill and swipe right to grab the other
        krill.

        Then turn west and drive to the submersible.  Get within range to perform the
        squaring code.  By this time, the robot has accumulated a bunch of errors and
        squaring as well as subsequent line following will we orient the robot so that
        we know where we are at on the board.  So we approach the angled line in front
        of the "Submersible Rescue" mission and square up to it.  Move forward to lift
        the submersible and then lower it back down.  Turn left and follow the line with
        the right eye to the "Angler Fish" mission.

        Move south to the "Angler Fish" mission and turn to face the back of the robot
        to the mission.  Turn the robot to engage the mission and then move forward to
        get clearance and twist the ring to align the forklift for "Sea Floor Sample".

        Appraoch the "Sea Floor Sample" mission and lift the sample.  Then back off and
        turn to prepare for last pass throught the left side of the field and back to
        the left launch area.  We need a series of curves and turns to gather the water
        sample, two coral, and one more krill.

        """

        self.straight(100, 0, 200)  # get off wall
        ue_angle = -45  # unexpected encounter angle

        # turn to get krill and "Unexpected Encounter"
        self.turn(ue_angle, turn_rate=180, twist=True, orientation=-(90+ue_angle+10))
        self.straight(160, ue_angle, 200)
        run_task(multitask(
            self._straight(160, ue_angle, 120),
            self._twist_target(-90)
        ))

        # Back off and curve to the right
        self.straight(-76, ue_angle, 200)
        self.curve(90, 90, 200, twist=True)  # puts me right in front of "Change Shipping Lanes"
        # self.twist_target(-60)  # secure coral
        # self.twist_target(0)

        self.straight(72, 45, 50, gain=4)  # place forklift under "Change Shipping Lanes" model
        self.turn(-45 - self.heading(), turn_rate=90, twist=True)  # turn to right
        self.para_lift_drive(lift_f, drive_r, n=50, wait_time=20)  # trace semi-circle
        wait(200)  # make sure para_lift_drive is done
        self.turn(45 - self.heading(), turn_rate=90, twist=True)  # turn back to perpendicular

        #  Begin edit
        self.straight(-120, 45, 200)  # back off

        self.liftup(64, wait=False)  # lift up to get out of the way of things on the ground
        self.turn(0 - self.heading(), turn_rate=200, twist=True, orientation=-45)  # turn north
        self.straight(164, 0, 400)  # charge forward to get krill

        self.pause('How to get sample?', False)
        self.twist_target(-145, speed=1000)
        self.twist_target(-100, speed=200)

        self.pause('nab krill', False)
        self.straight(55, 0, 400)
        run_task(multitask(
            self._curve(radius=-20, angle=90, speed=300),
            self._twist_target(-90)
        ))

        self.turn(-90-self.heading(), turn_rate=50)
        self.pause('make sure I get to right spot', False)

        radius, theta = curve_get_r_theta_from_x_y(145, 505)
        self.curve(radius=radius, angle=theta, speed=300, twist=True)

        self.pause('Close enuf 2 sub?', False)

        run_task(multitask(
            self._maintain_ring_orientation(self.heading()+45),
            self._square(100, heading_target=-45, creep_factor=0.25),
            race=True
        ))
        self.twist_target(0)

        self.pause('did I square', False)
        self.straight(distance=20, heading=-45, speed=100)
        self.lift.bump(speed=500, distance=70)
        run_task(multitask(
            self._maintain_ring_orientation(self.heading()),
            self._square(-100, heading_target=-45, creep_factor=0.25),
            race=True
        ))
        self.pause('did I square again', False)
        self.turn(-30, turn_rate=90, twist=True, orientation=self.heading())
        self.twist_target(self.heading(), speed=400)
        self.pause('Should be facing East', False)
        run_task(self._stop_at_edge(eye='right', angle=-90,
            turn_rate=30, twist=True, orientation=self.heading()))
        self.pause('rdy for follow?', False)
        run_task(multitask(
            self._maintain_ring_orientation(),
            self._lags(
                light_target=50, heading_target=-90, distance_target=300, speed=50,
                eye_side='right', line_orientation='wb',
                light_kp=1, light_ki=0.01, light_kd=0.05, light_integral_range=100,
                heading_kp=0.2, heading_ki=0.01, heading_kd=0.01, heading_integral_range=100,
                stop_at_end_of_line=True
            ),
            race=True
        ))
        self.what_do_i_see()
        self.pause('did I follow', False)
        self.twist_target(-90)
        self.turn(-45, turn_rate=90, twist=True)
        self.straight(distance=80, heading=-135, speed=200)
        self.pause('destroy?', False)
        run_task(multitask(
            self._turn(angle=0-self.heading(), turn_rate=120),
            self._twist_target(-45)
        ))
        self.pause('after angler', False)
        self.straight(distance=10, heading=0, speed=200)
        self.twist_target(0)
        self.straight(distance=64, heading=0, speed=300)
        self.pause('At Bed Sample?', False)
        self.lift.lift(speed=500, distance=1000)
        self.straight(-10, self.heading(), 200)

        self.turn(-90, 90, twist=True)
        self.twist_target(-75)

        self.curve(radius=900, angle=-10, speed=300, twist=True, orientation=-75, then=Stop.NONE)
        self.curve(radius=750, angle=-20, speed=300, then=Stop.NONE)
        self.curve(radius=150, angle=-60, speed=300, then=Stop.NONE)
        self.curve(radius=350, angle=80, speed=300)


        self.pause('reset me')

    def phase1(self):
        """
        1. Coral pieces
        2. Coral Tree
        3. Raise the Mast
        4. Kraken's Treasure
        5. Coral Nursery bed"""
        def lift_c(time):
            """Control the height of the lift"""
            radius = 192
            return radius * sin(time * pi / 2)

        def drive_c(time):
            """Control the distance of the bot"""
            radius = 172
            return radius * (1 - cos(time * pi / 2))

        speed = 200
        self.straight(distance=130, heading=0, speed=speed)             # approach tree
        self.para_lift_drive(lift_c, drive_c, n=100, wait_time=15)      # trace semi-circle for tree
        self.straight(distance=8, heading=0, speed=speed)               # forward a smidge to correct distance
        self.lift.lift(distance=-42, speed=speed)                       # drop lift to place tree
        self.straight(distance=-28, heading=0, speed=speed)             # back off of tree
        self.turn(angle=90-self.heading(), turn_rate=90, twist=True)    # Face East(right)
        self.straight(distance=176, heading=90, speed=speed)            # Go partial distance towards Kraken 1 of 3
        self.turn(angle=45-self.heading(), turn_rate=90, twist=True)    # Face NE to cover vertical and horizontal distance 2 of 3
        self.straight(distance=140, heading=45, speed=speed)            # Go partial distance towards Kraken 2 of 3
        self.turn(angle=90-self.heading(), turn_rate=90, twist=True)    # Face East(right) 3 of 3
        run_task(multitask(                                             # Got final distance towards Kraken 3 of 3
            self._twist_target(88),
            self._straight(distance=128, heading=90, speed=50),
        ))
        wait(200)
        self.straight(distance=-120, heading=90, speed=speed)           # Back off Kraken
        self.turn(angle=0-self.heading(), turn_rate=90, twist=True)     # Face North
        self.straight(distance=116, heading=0, speed=speed)             # Travel North
        self.turn(angle=-90-self.heading(), turn_rate=90, twist=True)   # Face West
        self.straight(distance=90, heading=-90, speed=speed)            # Move to coral nursery
        self.straight(distance=-40, heading=-90, speed=speed)           # Back off of Nursery
        self.turn(angle=0-self.heading(), turn_rate=90, twist=True)     # Head Home
        self.curve(radius=-300, angle=-60, speed=300, then=Stop.NONE)
        self.curve(radius=-300, angle=60, speed=300)
        
        self.pause('end')

    def phase2(self):
        """
        1. Coral Reef bed
        2. Shark
        3. Pick Diver up
        4. Drop Diver off"""

        dy = 752
        dx = 88

        fast_speed = 200
        slow_speed = 100
        run_task(multitask(
            self._straight(distance=200, heading=0, speed=fast_speed),
            self.lift._lift(distance=72, speed=500)
        ))
        sum_y = self.dead_reck()
        self.turn(angle=30-self.heading(), turn_rate=90)
        self.straight(distance=dx*2, heading=30, speed=fast_speed)
        sum_y += (self.dead_reck() - sum_y) * sqrt(3) / 2
        self.turn(angle=0-self.heading(), turn_rate=90)
        self.straight(distance=dy-sum_y, heading=0, speed=fast_speed)
        self.pause('@reef1', False)
        self.lift.bump(distance=-32, speed=500)

        self.curve(radius=-self.wheel_base/2, angle=90, speed=fast_speed)

        self.straight(distance=120, heading=-90, speed=fast_speed),
        self.pause('@shark', False)

        self.lift.bump(distance=-90, speed=500)

        self.lift.lift(distance=65, speed=500)

        self.turn(angle=-100-self.heading(), turn_rate=90)

        self.pause('@diver', False)

        self.straight(distance=20, heading=-95, speed=fast_speed)

        self.lift.lift(distance=24, speed=500)

        self.curve(radius=-self.wheel_base/2, angle=-42+self.heading(), speed=fast_speed)
        self.pause('@@reef', False)
        self.straight(distance=104, heading=45, speed=fast_speed)

        self.pause('@reef', False)

        self.lift.lift(distance=-32, speed=500)
        wait(200)

        self.straight(distance=-50, heading=45, speed=fast_speed)
        self.straight(distance=-700, heading=0, speed=fast_speed)

        self.pause('end')

    def phase3(self):
        self.straight(distance=-100, heading=0, speed=300)
        wait(500)
        self.straight(distance=50, heading=0, speed=300)

    def phase4(self):
        shark_angle = 67
        self.curve(radius=self.wheel_base/2, angle=shark_angle, speed=300)
        self.straight(distance=500, heading=shark_angle, speed=300)
        self.lift.lift(distance=52, speed=500)
        self.straight(distance=-120, heading=shark_angle, speed=300)
        self.turn(angle=-40, turn_rate=90)
        self.lift.lift(distance=-52, speed=500)
        self.turn(angle=100-self.heading(), turn_rate=90)
        self.straight(distance=1000, heading=95, speed=500)
        self.pause('end')

    def phase5(self):
        self.straight(distance=100, heading=0, speed=300)
        run_task(multitask(
            self._straight(distance=516, heading=0, speed=300),
            self._twist_target(angle=45)
        ))
        self.turn(angle=45, turn_rate=90)
        self.straight(distance=80, heading=45, speed=300)
        self.straight(distance=100, heading=45, speed=50)
        wait(500)
        self.straight(distance=-192, heading=45, speed=300)
        self.turn(angle=-45, turn_rate=90)
        self.straight(distance=-500, heading=0, speed=200)
        run_task(multitask(
            self._straight(distance=-375, heading=0, speed=300),
            self._twist_target(angle=0)
        ))

        self.pause('end')

    def old_phase4(self):
        self.curve(radius=110, angle=90, speed=100)
        self.straight(distance=400, heading=90, speed=100)
        self.turn(angle=45-self.heading(), turn_rate=45, twist=True)
        self.straight(distance=150, heading=45, speed=200)
        self.turn(angle=0-self.heading(), turn_rate=90, twist=True)
        self.straight(distance=-110, heading=0, speed=100)
        self.turn(angle=90-self.heading(), turn_rate=90, twist=True)
        self.pause('boat')
        kp, ki, kd, int_d, int_r = self.drive.heading_control.pid()
        self.drive.heading_control.pid(kp, 100, kd, int_d, int_r)
        self.straight(distance=950, heading=90, speed=500)
        self.drive.heading_control.pid(kp, ki, kd, int_d, int_r)
        self.pause('end')




if __name__ == "__main__":

    clear_console()
    bot = GameBot(
        left_motor=Port.A,
        right_motor=Port.E,
        ring_motor=Port.C,
        lift_motor=Port.D,
        left_eye=Port.B,
        right_eye=Port.F,
        hub_type=PrimeHub
    )

    programs = {}
    i = 0
    while hasattr(bot, f'phase{i}'):
        programs[str(i)] = getattr(bot, f'phase{i}')
        i += 1

    options = sorted(programs)
    selected = hub_menu(*options, 'X')
    try:
        programs[selected]()
    except (SystemExit, TypeError, AttributeError) as e:
        bot.drive.stop()
        bot.ring.stop()
        bot.lift.stop()
        print()
        print(e)
        print()
        bot.pause('U interupted!')




