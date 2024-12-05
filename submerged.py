from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop, Color, Button
from pybricks.tools import multitask, run_task, wait, StopWatch, hub_menu
from umath import pi, sin, cos, asin, acos, sqrt
from LegoLegacy import Bot, clear_console, curve_get_r_theta_from_x_y


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
        left_eye=Port.B,
        right_eye=Port.F,
        hub_type=PrimeHub
    )

    # bot.lift.forklift(-80, 1000)
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

    
    def pause(msg='', ask=True):
        print(f'{msg:<30} | H: {bot.heading():8.2f} | R: {bot.ring_angle():8.2f}')
        
        if ask:
            p = hub_menu('G', 'S')
            if p == 'S':
                run_task(multitask(
                    bot._turn(-bot.heading(), 50),
                    bot._twist_target(0)
                ))
                bot.liftdown()
                raise ValueError('Hmm!')
            else:
                bot.hub.display.char('@')

    

    ##  Beginnings of phase0  ##
    ##  Please Save           ##

    bot.straight(100, 0, 200)
    ue_angle = -45
    bot.turn(ue_angle, turn_rate=180, twist=True, orientation=-(90+ue_angle+10))
    bot.straight(180, ue_angle, 200)
    run_task(multitask(
        bot._straight(160, ue_angle, 120),
        bot._twist_target(-90)
    ))
    bot.straight(-76, ue_angle, 200)
    bot.curve(90, 90, 200, twist=True)
    bot.twist_target(-70)
    bot.twist_target(0)

    pause('heading to change lanes', False)


    pause('check alignment', False)
    bot.straight(68, 45, 50, gain=4)
    pause('check again', False)

    bot.turn(-45 - bot.heading(), turn_rate=45, twist=True)

    bot.para_lift_drive(lift_f, drive_r, n=50, wait_time=20)
    wait(200)
    bot.turn(45 - bot.heading(), turn_rate=45, twist=True)

    bot.straight(-210, 45, 200)

    bot.liftup(64, wait=False)
    bot.turn(0 - bot.heading(), turn_rate=200, twist=True, orientation=-45)

    pause('just after change lanes', False)


    bot.straight(330, 0, 400)

    pause('Can I nab krill', False)
    temp_angle = bot.ring_angle()
    bot.twist_target(temp_angle-65)
    bot.twist_target(temp_angle)

    bot.turn(-90 - bot.heading(), turn_rate=200)

    pause('before submersible', False)


    bot.straight(200, -90, 200)
    orientation = -90
    radius, theta = curve_get_r_theta_from_x_y(100, 430)
    bot.curve(radius=radius, angle=theta, speed=200, twist=True, orientation=orientation)

    bot.liftup(64)
    bot.liftdown(64)

    pause('Did I submerge?', True)


    bot.straight(-40, bot.heading(), 100)
    bot.turn(-135 - bot.heading(), turn_rate=100, twist=True)

    pause('check distances', False)
    radius, theta = curve_get_r_theta_from_x_y(60, 152)
    bot.curve(radius=radius, angle=theta, speed=200, twist=True)

    pause('debating turn vs twist.')

    bot.turn(0 - bot.heading(), turn_rate=100)
    bot.twist_target(0)

    pause('at angler?', True)

    bot.turn(0 - bot.heading(), turn_rate=100, twist=True)
    bot.straight(150, 0, 100)
    bot.liftup()
    bot.straight(-50, 0, 100)

    pause('did I get sample?', True)

    bot.turn(-90 - bot.heading(), turn_rate=100, twist=True)
    bot.straight(50, -90, 100)
    radius, theta = curve_get_r_theta_from_x_y(-100, 350)
    bot.curve(radius=-radius, angle=theta, speed=100, twist=False)
    
    pause('Look at nursery', True)

    bot.turn(0 - bot.heading(), turn_rate=100, twist=True, orientation=45)

    pause('am I preppared for nursery?', True)

    ## Phase0 ##
    ## End    ##


    # run_task(multitask(
    #     bot._straight(100, 0, 100),
    #     bot._twist_target(-90)
    # ))
    # run_task(multitask(
    #     bot._straight(-100, 0, 50),
    #     bot._twist_target(0, 500)
    # ))
    # bot.curve_links(50, [30, -60, 60, -30], 100, twist=True)
    # bot.curve_links(-50, [-30, 60, -60, 30], 100, twist=True)

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



#### From other module

from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait, StopWatch, hub_menu
from umath import pi, sin, cos, asin, acos
from LegoLegacy import Bot, clear_console, _CONFIG, curve_get_r_theta_from_x_y
from LegoLegacy import Motor, ColorSensor

clear_console()
bot = Bot(
    left_motor=Port.A,
    right_motor=Port.E,
    ring_motor=Port.C,
    lift_motor=Port.D,
    left_eye=Port.B,
    right_eye=Port.F,
    hub_type=PrimeHub
)

clear_console()

def pause(msg='', ask=True):
    print(f'{msg:<30} | H: {bot.heading():8.2f} | R: {bot.ring_angle():8.2f}')
    bot.what_do_i_see()
    if ask:
        p = hub_menu('G', 'S')
        if p == 'S':
            run_task(multitask(
                bot._turn(-bot.heading(), 50),
                bot._twist_target(-90)
            ))
            bot.lift.run_target(100, 0)
            raise ValueError('Hmm!')
        else:
            bot.hub.display.char('@')



# bot.lags(
#     light_target=50, heading_target=0, distance_target=500, speed=50,
#     eye_side='right', line_orientation='wb',
#     light_kp=1, light_ki=0.01, light_kd=0.5, light_integral_range=100,
#     heading_kp=0.1, heading_ki=0.01, heading_kd=0.1, heading_integral_range=100,
# )

# while True:
#     clear_console()
#     run_task(bot._what_do_i_see())
#     run_task(bot.is_black_or_white('right'))
#     wait(100)
# run_task(bot._get_off_edge('rignt', -45))
# run_task(bot._stop_at_edge('right', -180))

# raise SystemExit
bot.ring.reset_angle(bot.ring.parametric_ratio(90))
print(bot.ring_angle())
# raise SystemExit
run_task(multitask(
    bot._maintain_ring_orientation(-45),
    bot._square(100, heading_target=45, creep_factor=0.25),
    race=True
))
bot.twist_target(0)
bot.straight(distance=20, heading=45, speed=100)
bot.lift.bump(speed=500, distance=70)
bot.square(-100)
bot.twist_target(-45, speed=400)
run_task(bot._stop_at_edge(eye='right', angle=-45, turn_rate=45, twist=True))
# pause('am I on the line?')
run_task(multitask(
    bot._maintain_ring_orientation(),
    bot._lags(
        light_target=50, heading_target=0, distance_target=300, speed=50,
        eye_side='right', line_orientation='wb',
        light_kp=1, light_ki=0.01, light_kd=0.05, light_integral_range=100,
        heading_kp=0.2, heading_ki=0.01, heading_kd=0.01, heading_integral_range=100,
        stop_at_end_of_line=True
    ),
    race=True
))
# raise SystemExit
bot.twist_target(-90)
bot.turn(-45, turn_rate=45, twist=True)
bot.straight(distance=85, heading=bot.drive.angle(), speed=100)
bot.turn(angle=135, turn_rate=90, twist=True)
bot.twist_target(-60)

bot.straight(distance=10, heading=bot.heading(), speed=200)
bot.twist_target(0)
bot.straight(distance=110, heading=bot.drive.angle(), speed=200)
bot.lift.lift(speed=500, distance=1000)
bot.straight(-10, bot.heading(), 100)

bot.turn(-90, 45, twist=True)
bot.twist_target(-75)

# pause('check b4 sweep')

bot.curve(radius=1000, angle=-10, speed=100, twist=True, orientation=-75, then=Stop.NONE)
# pause('you good')
bot.curve(radius=750, angle=-20, speed=100, then=Stop.NONE)
# pause('you good')
bot.curve(radius=150, angle=-60, speed=100, then=Stop.NONE)
bot.curve(radius=350, angle=90, speed=100)
pause('you good')
