from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import multitask, run_task, wait, StopWatch, hub_menu
from umath import pi, sin, cos, asin, acos, sqrt
from LegoLegacy import Bot, clear_console, twist_params


class GameBot(Bot):
    """
    """

    def phase0(self):
        self.reset()
        self.liftup()
        self.lift.run_angle(-300, bot.lift.parametric_ratio(56))

        bot_pos = [347, 108]
        run_task(self.straight_at_and_grab(
            distance=700, heading=0, speed=200, bot_pos=bot_pos,
            waypoints=[(395, 553), (300, 711), (385, 827), (216, 907)],
            wait_time=1
        ))

        run_task(self.pivot_and_go(
            distance=100, heading=0, speed=200, hypotenuse=110, wait_time=1
        ))

        self.ring.twist_target(-90)


        # get squid
        run_task(self.straight_at(-310, 5, 200))
        self.duty_twist(angle=-180, speed=350)  # smack squid

        self.drive.curve(-106, 45)  # get onto line towards lane change
        self.drive.turn(angle=-90)  # point wheels tords lane change
        self.ring.twist_target(angle=-180, speed=500)  # point lift towards lane change

        self.liftdown()

        run_task(self.straight_at(distance=-75, heading=-135, speed=100))  # move towards lane change

        self.sync_twist_turn(-225, -100, steps=20, wait_time=40, lead=2)


        self.lift.run_target(50, self.lift.mm2deg(10))
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
        self.reset()
        self.drive.curve(240, -90)
        self.drive.curve(240, 90)
        self.liftup(80)
        self.drive.curve(-240, 90)
        run_task(self.straight_at(550, -90, 100))
        self.drive.curve(45, 90)
        self.liftdown()
        turn_speed, turn_acc, turn_torque = self.drive.heading_control.limits()
        self.drive.heading_control.limits(100, turn_acc, turn_torque)
        self.drive.curve(60, -90)
        self.drive.heading_control.limits(turn_speed, turn_acc, turn_torque)
        self.drive.curve(485, -60)

    def phase2(self):
        """right side of set up block is 4 squares from right side of left
        launch area"""
        self.reset()
        self.liftup()
        run_task(self.straight_at(100, 0, 100))
        self.ring.twist_target(-70)
        run_task(self.straight_at(300, 0, 100))
        self.ring.twist_target(-105)
        self.drive.curve(-200, 45)
        self.drive.curve(-200, -45)
        run_task(self.straight_at(450, 0, 100))
        self.ring.twist_target(90, speed=400, wait=False)
        run_task(self.straight_at(50, 0, 100))
        self.drive.curve(-600, -50)
        wait(2000)

    def phase3(self):
        """left side of set up block is 1 squares from left side of left
        launch area"""

        self.reset()
        self.liftdown()
        run_task(self.straight_at(300, 0, 100))

        radius_horizontal = 184
        radius_vertical = 184
        left_start = self.left_angle()
        right_start = self.right_angle()
        lift_start = self.lift.angle()


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

        self.lift.run_angle(100, -self.lift.mm2deg(40))
        run_task(self.straight_at(-88, 0, 100))

        self.sync_twist_turn(90, 100, steps=20, wait_time=40, lead=2)

        x = 108
        y = 200
        r = (y**2 + x**2)/(2*x)
        theta = asin(y/r) * 180 / pi
        r1 = 170
        r2 = r - r1

        self.drive.curve(r1, -theta)
        self.drive.curve(r2, theta)

        self.sync_twist_turn(90, 100, twist=False)
        self.drive.turn(90-self.heading())
        print(self.ring_angle(), self.heading())
        self.ring.twist_target(90, speed=100, kpf=3)
        run_task(self.straight_at(182, heading=90, speed=100))
        run_task(self.straight_at(-130, heading=90, speed=100))

        self.drive.turn(90)
        run_task(self.straight_at(-124, heading=180, speed=100))
        self.drive.turn(-90)
        run_task(self.straight_at(-85, heading=90, speed=100))
        self.drive.curve(100, 90)
        self.drive.curve(380, 80)

    def phase4(self):
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
        self.drive.turn(-105-self.heading())
        run_task(self.straight_at(80, -105, 100))
        self.liftup(40)
        self.drive.curve(-232, self.heading())
        run_task(self.straight_at(175, 0, 100))
        self.liftdown(50)
        run_task(self.straight_at(-190, 0, 100))
        self.drive.curve(-400, -90)
        # raise SystemExit


        # self.liftup()
        # run_task(self.straight_at(distance=735, heading=0, speed=100))
        # self.drive.turn(-60)
        # self.liftdown()
        # self.liftup(120)
        # self.drive.turn(-30)
        # run_task(self.straight_at(distance=50, heading=-90, speed=100))
        # self.liftup(110)
        # self.hub.imu.reset_heading(-90)
        # run_task(self.straight_at(distance=-300, heading=-90, speed=100))
        # self.drive.turn(90)
        # run_task(self.straight_at(distance=-60, heading=0, speed=100))
        # self.liftdown(60)
        # run_task(self.straight_at(distance=50, heading=0, speed=100))
        # self.liftdown(40)
        # run_task(self.straight_at(distance=-50, heading=0, speed=100))
        

        

        # self.liftdown()
        # self.liftup(50)

    def phase5(self):
        self.liftup()
        run_task(self.straight_at(120, 0, 100))
        self.ring.twist_target(-15, wait=False)
        self.liftdown()
        self.liftup(50)
        self.ring.twist_target(0, wait=False)
        run_task(self.straight_at(-120, 0, 100))
        self.liftdown()

    def phase6(self):
        self.liftdown()
        run_task(self.straight_at(60, 0, 50))
        run_task(self.straight_at(-60, 0, 100))
        pass

    def phase7(self):
        """setup block right side is 5 squares from right"""
        self.liftup()
        self.drive.curve(self.wheel_base/2 + 40, 90)
        self.ring.twist_target(-90, wait=False)
        run_task(self.straight_at(100, 90, 100))
        self.liftdown(144, wait=False)
        run_task(self.straight_at(560, 90, 100))
        run_task(self.straight_at(-150, 90, 100))

        self.drive.curve(-150, 90)

        self.sync_twist_turn(90, 100)
        self.liftdown()
        run_task(self.straight_at(325, 90, 100))
        run_task(self.straight_at(-125, 90, 100))

        self.liftup()

        self.sync_twist_turn(0, -100)

        self.drive.curve(self.wheel_base/2+160, 90)

        run_task(self.straight_at(760, 90, 300))


    def phase8(self):
        self.liftup()
        theta = 30
        radius = 70 * (2 + sqrt(3))


        s, a, t = self.drive.distance_control.limits()
        self.drive.distance_control.limits(200, a, t)
        self.drive.curve(radius, -theta, then=Stop.NONE)
        self.drive.curve(radius, theta, then=Stop.NONE)
        self.drive.distance_control.limits(s, a, t)

        y = 628 - radius

        run_task(self.straight_at(y, 0, 200))
        self.drive.turn(45)
        run_task(self.straight_at(180, 45, 150))
        self.liftdown()
        run_task(self.straight_at(-20, 45, 200))
        run_task(self.straight_at(20, 45, 200))
        run_task(self.straight_at(-175, 45, 100))
        self.drive.curve(-self.wheel_base/2, 45)
        self.drive.curve(-450, 90)




if __name__ == "__main__":

    clear_console()
    bot = GameBot(
        left_motor=Port.A,
        right_motor=Port.F,
        ring_motor=Port.C,
        lift_motor=Port.B,
        left_eye=Port.E,
        right_eye=Port.D,
        hub_type=PrimeHub
    )

    bot.phase8()
    raise SystemExit

    programs = {}
    i = 0
    while hasattr(bot, f'phase{i}'):
        programs[str(i)] = getattr(bot, f'phase{i}')
        i += 1

    selected = hub_menu(*sorted(programs))
    bot.reset()
    programs[selected]()
    raise SystemExit




