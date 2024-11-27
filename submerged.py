from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import multitask, run_task, wait, StopWatch, hub_menu
from umath import pi, sin, cos, asin, acos, sqrt
from LegoLegacy import Bot, clear_console, twist_params


class GameBot(Bot):
    """
    """

    def supp_tt(self, angle, btr=100):
        self.sync_twist_turn(angle, btr)
        self.drive.turn(angle-self.heading())
        self.ring.twist_target(angle)

    def phase0(self):
        """Align left wheel with second line from the left"""
        self.reset()
        self.liftup()
        self.lift.run_angle(-300, bot.lift.parametric_ratio(56))

        print('grabbing')
        bot_pos = [347, 108]
        run_task(self.straight_at_and_grab(
            distance=700, heading=0, speed=200, bot_pos=bot_pos,
            waypoints=[(395, 553), (300, 711), (385, 827), (216, 907)],
            wait_time=1
        ))

        print('pivot')
        run_task(self.pivot_and_go(
            distance=100, heading=0, speed=200, hypotenuse=110, wait_time=1
        ))

        print('twist')
        self.ring.twist_target(-90)


        # get squid
        run_task(self.straight_at(-310, 5, 200))
        self.duty_twist(angle=-180, speed=350)  # smack squid

        self.drive.curve(-106, 45)  # get onto line towards lane change
        self.drive.turn(angle=-90)  # point wheels tords lane change
        self.ring.twist_target(angle=-180, speed=500)  # point lift towards lane change

        self.liftdown()

        run_task(self.straight_at(distance=-72, heading=-135, speed=100))  # move towards lane change

        self.sync_twist_turn(-225, -100, steps=20, wait_time=40, lead=2)


        self.liftup(10)
        radius = 80
        left_start = self.left_angle()
        right_start = self.right_angle()
        lift_start = self.lift.angle()


        for i in range(101):
            t = i * pi / 100
            x = radius - cos(t) * radius
            y = sin(t) * radius

            drive_angle = self.drive.mm2deg(x)
            lift_angle = self.lift.mm2deg(y)

            self.left_motor.track_target(drive_angle+left_start)
            self.right_motor.track_target(drive_angle+right_start)
            self.lift.track_target(lift_angle+lift_start)
            wait(20)

        self.sync_twist_turn(-315, -100, steps=20, wait_time=40, lead=2)

        heading = self.heading()
        run_task(self.straight_at(-50, heading, speed=100))
        self.drive.curve(-100, 45)
        heading = self.heading()
        self.lift.run_until_stalled(-305, duty_limit=50)  # lower lift
        run_task(self.straight_at(-200, heading, 100))

        self.sync_twist_turn(0, 1, steps=10, wait_time=40, lead=2)

    def phase1(self):
        """Align left wheel with second line from the left

        Krill in basket
        Gatherer on top
        
        """
        self.reset()
        self.liftdown()
        s, a, t = self.drive.distance_control.limits()
        self.drive.distance_control.limits(500, a, t)
        self.drive.curve(240, -90, then=Stop.NONE)
        self.drive.curve(240, 90)
        self.liftup(80)
        self.drive.curve(-240, 90)
        run_task(self.straight_at(550, -90, 400))
        self.drive.curve(40, 90)
        run_task(self.straight_at(10, self.heading(), 100))
        self.liftdown()
        self.drive.curve(45, -95)
        self.drive.curve(700, -45)
        self.liftup(80)
        self.drive.distance_control.limits(s, a, t)

    def phase2(self):
        """right side of set up block is 4 squares from right side of left
        launch area
        
        Gatherer at 9 o'clock
        """
        self.reset()
        self.liftup()
        run_task(self.straight_at(50, 0, 300))
        self.ring.twist_target(-70, wait=False)
        run_task(self.straight_at(340, 0, 300))
        self.ring.twist_target(-105, wait=False)
        self.drive.curve(-200, 45, then=Stop.NONE)
        self.drive.curve(-200, -45)
        self.ring.twist_target(-90, wait=False)
        run_task(self.straight_at(430, 0, 300))
        self.ring.twist_target(90, speed=400, wait=False)
        run_task(self.straight_at(50, 0, 100))
        self.drive.curve(-700, -50)
        self.ring.twist_target(0)

    def phase3(self):
        """left side of set up block is 1 squares from left side of left
        launch area"""

        def ttgo(angle, direction, distance, speed):
            self.supp_tt(angle, direction*100)
            run_task(self.straight_at(distance, angle, speed))

        speed = 200
        self.reset()
        self.liftdown()
        run_task(self.straight_at(280, 0, speed))

        radius_horizontal = 184
        radius_vertical = 180
        left_start = self.left_angle()
        right_start = self.right_angle()
        lift_start = self.lift.angle()

        # Lift coral reef stick
        for i in range(101):
            t = i * pi / 200
            y = sin(t) * radius_vertical
            x = (1 - cos(t)) * radius_horizontal

            drive_angle = self.drive.mm2deg(x)
            lift_angle = self.lift.mm2deg(y)

            self.left_motor.track_target(drive_angle+left_start)
            self.right_motor.track_target(drive_angle+right_start)
            self.lift.track_target(lift_angle+lift_start)
            wait(25)

        self.liftdown(40)
        run_task(self.straight_at(-68, 0, 100))

        ttgo(60, 1, 210, speed)
        ttgo(90, 1, 180, 100)
        run_task(self.straight_at(-115, 90, 100))
        ttgo(150, 1, -160, 100)
        ttgo(180, 1, 160, speed)
        ttgo(210, 1, 450, speed)
        ttgo(360, 1, -50, 100)

    def phase4(self):
        """Setup block 4 from right"""
        self.reset()
        self.liftdown()
        self.liftup(50, wait=False)
        self.drive.curve(447.85, 30, then=Stop.NONE)
        self.drive.curve(447.85, -30, then=Stop.NONE)
        run_task(self.straight_at(distance=300, heading=0, speed=150))
        self.liftdown()
        self.liftup(50)
        self.drive.turn(-75)
        run_task(self.straight_at(100, -75, 100))
        self.liftdown()
        self.liftup(128)
        self.drive.turn(-103-self.heading())
        run_task(self.straight_at(85, -103, 100))
        self.liftup(40)
        self.drive.curve(-240, self.heading())
        run_task(self.straight_at(180, 0, 100))
        self.liftdown(50)
        run_task(self.straight_at(-190, 0, 100))
        self.drive.curve(-400, -90)

    def phase5(self):
        """Setup block 90 deg 7 from left"""
        self.liftup()
        run_task(self.straight_at(120, 0, 100))
        self.ring.twist_target(-15, wait=False)
        self.liftdown()
        self.liftup(50)
        self.ring.twist_target(0, wait=False)
        run_task(self.straight_at(-120, 0, 100))
        self.liftdown()

    def phase6(self):
        """Setup block 90 deg 7 from left"""
        self.liftdown()
        run_task(self.straight_at(60, 0, 50))
        run_task(self.straight_at(-60, 0, 100))

    def phase7(self):
        """setup block right side is 5 squares from right"""
        self.liftup()
        self.liftdown(144, wait=False)
        self.drive.curve(self.wheel_base/2 + 40, 90)
        self.ring.twist_target(-90, wait=False)
        run_task(self.straight_at(660, 90, 200))
        run_task(self.straight_at(-150, 90, 200))

        self.drive.curve(-150, 90)

        self.sync_twist_turn(90, 100)
        self.liftdown()
        run_task(self.straight_at(325, 90, 200))
        run_task(self.straight_at(-125, 90, 200))

        self.liftup()

        self.sync_twist_turn(0, -100)

        self.drive.curve(self.wheel_base/2+160, 90)

        run_task(self.straight_at(760, 90, 300))

        # prep for next phase
        self.liftup()


    def phase8(self):
        """Setup block 8 from right
        BUT!!!!! The front center is aligned with grid.
        
        Back of Bot facing bottom of board
        Lift facing 3 o'clock clockwise

        krill attachment at 12 o'clock
        kelp sample lift attachment (on lift)
        """
        self.liftup()
        theta = 30
        radius = 70 * (2 + sqrt(3))


        s, a, t = self.drive.distance_control.limits()
        self.drive.distance_control.limits(500, a, t)
        self.drive.curve(radius, -theta, then=Stop.NONE)
        self.drive.curve(radius, theta, then=Stop.NONE)

        y = 628 - radius

        run_task(self.straight_at(y, 0, 300))

        # Using another mehtod that turns but won't stall out after its tried
        # to get to the requested angle.  We are intentionally aiming for 53
        # degrees to get to 45 degrees.  We noticed that when we went for 45
        # directly, this method undershot by 8 degrees... hence 45 + 8 = 53.
        # This is very specific to this setup with the particular heavy
        # attachments we have.
        self.sync_twist_turn(
            heading=53, base_turn_rate=100, twist=False, steps=10, wait_time=100
        )
        print(self.heading())
        run_task(self.straight_at(180, 45, 200))
        self.liftdown()
        run_task(self.straight_at(-20, 45, 300))
        run_task(self.straight_at(20, 45, 300))
        run_task(self.straight_at(-175, 45, 500))
        self.drive.curve(-self.wheel_base/2, 45)
        self.drive.curve(-500, 90)
        self.drive.distance_control.limits(s, a, t)

    def phase9(self):
        """setup block right side is 5 squares from right"""
        self.liftup()
        self.drive.curve(self.wheel_base/2 + 140, 90)
        run_task(self.straight_at(1440, 90, 300))


if __name__ == "__main__":

    clear_console()
    bot = GameBot(
        left_motor=Port.A,
        right_motor=Port.E,
        ring_motor=Port.C,
        lift_motor=Port.D,
        hub_type=PrimeHub
    )


    # bot.ring.twist_target(30)
    bot.curve_links(50, [30, -60, 60, -30], 100, twist=True)
    bot.curve_links(-50, [-30, 60, -60, 30], 100, twist=True)
    # bot.ring.twist_target(0)
    # bot.curve(50, 60, 50, twist=True, orientation=None)
    # print(bot.heading(), bot.ring_angle())
    # bot.curve(50, -60, 50, twist=True, orientation=bot.heading())
    # print(bot.heading(), bot.ring_angle())

    # bot_pos = [347, 108]
    # run_task(bot.straight_at_and_grab(
    #     distance=700, heading=0, speed=200, bot_pos=bot_pos,
    #     waypoints=[(395, 553), (300, 711), (385, 827), (216, 907)],
    #     wait_time=1
    # ))



    raise SystemExit

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




