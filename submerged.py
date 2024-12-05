from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import multitask, run_task, wait, StopWatch, hub_menu
from umath import pi, sin, cos, asin, acos, sqrt
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
        self.straight(180, ue_angle, 200)
        run_task(multitask(
            self._straight(160, ue_angle, 120),
            self._twist_target(-90)
        ))

        # Back off and curve to the right
        self.straight(-76, ue_angle, 200)
        self.curve(90, 90, 200, twist=True)  # puts me right in front of "Change Shipping Lanes"
        self.twist_target(-70)  # secure coral
        self.twist_target(0)

        self.straight(68, 45, 50, gain=4)  # place forklift under "Change Shipping Lanes" model
        self.turn(-45 - self.heading(), turn_rate=45, twist=True)  # turn to right
        self.para_lift_drive(lift_f, drive_r, n=50, wait_time=20)  # trace semi-circle
        wait(200)  # make sure para_lift_drive is done
        self.turn(45 - self.heading(), turn_rate=45, twist=True)  # turn back to perpendicular
        self.straight(-210, 45, 200)  # back off

        self.liftup(64, wait=False)  # lift up to get out of the way of things on the ground
        self.turn(0 - self.heading(), turn_rate=200, twist=True, orientation=-45)  # turn north
        self.straight(330, 0, 400)  # charge forward to get krill
        temp_angle = self.ring_angle()
        self.twist_target(temp_angle-65)  # twist to get krill
        self.twist_target(temp_angle)


        self.turn(-90 - self.heading(), turn_rate=45)  # turn west
        self.straight(300, -90, 200)
        self.pause('make sure I get to right spot')
        # I need to get the bot in the aproximate position to get to the submersible


        run_task(multitask(
            self._maintain_ring_orientation(-45),
            self._square(100, heading_target=45, creep_factor=0.25),
            race=True
        ))
        self.twist_target(0)
        self.straight(distance=20, heading=45, speed=100)
        self.lift.bump(speed=500, distance=70)
        self.square(-100)
        self.twist_target(-45, speed=400)
        run_task(self._stop_at_edge(eye='right', angle=-45, turn_rate=45, twist=True))
        run_task(multitask(
            self._maintain_ring_orientation(),
            self._lags(
                light_target=50, heading_target=0, distance_target=300, speed=50,
                eye_side='right', line_orientation='wb',
                light_kp=1, light_ki=0.01, light_kd=0.05, light_integral_range=100,
                heading_kp=0.2, heading_ki=0.01, heading_kd=0.01, heading_integral_range=100,
                stop_at_end_of_line=True
            ),
            race=True
        ))
        # raise SystemExit
        self.twist_target(-90)
        self.turn(-45, turn_rate=45, twist=True)
        self.straight(distance=85, heading=self.drive.angle(), speed=100)
        self.turn(angle=135, turn_rate=90, twist=True)
        self.twist_target(-60)

        self.straight(distance=10, heading=self.heading(), speed=200)
        self.twist_target(0)
        self.straight(distance=110, heading=self.drive.angle(), speed=200)
        self.lift.lift(speed=500, distance=1000)
        self.straight(-10, self.heading(), 100)

        self.turn(-90, 45, twist=True)
        self.twist_target(-75)


        self.curve(radius=1000, angle=-10, speed=100, twist=True, orientation=-75, then=Stop.NONE)
        self.curve(radius=750, angle=-20, speed=100, then=Stop.NONE)
        self.curve(radius=150, angle=-60, speed=100, then=Stop.NONE)
        self.curve(radius=350, angle=90, speed=100)
        self.pause('you good')

        ## Phase0 ##
        ## End    ##


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
    while True:
        selected = hub_menu(*options)
        bot.reset()
        programs[selected]()
        bot.drive.stop()
        bot.ring.stop()
        bot.lift.stop()
        i = options.index(selected)
        options = [str((int(j) + i + 1 % len(options))) for j in options]





