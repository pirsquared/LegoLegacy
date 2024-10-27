from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait, StopWatch
from umath import pi, sin, cos, asin, acos
from LegoLegacy import Bot, clear_console


class GameBot(Bot):
    """
    method mnumonics
    rs3k1ccl: right side, 3 krill, 1 coral, change lanes
    rs3k1clsws1k2c: right side, 3 krill, 1 coral,
                    left side, water sample, 1 krill, 2 coral
    """

    async def tri_hard(self, a, b, d, turn_rate=100):
        await multitask(
            self.target_heading_callback(a, turn_rate, twist=True),
            self.lift.run_until_stalled(d*300, duty_limit=50)
        )
        await self.ring.twist_callback(b)


    def rs3k1ccl(self):

        # get lift out of the way
        self.lift.run_until_stalled(300, duty_limit=50)

        # assuming the bot's center starts at 355.6 mm from the right and 108 mm
        # from the bottom, this traverses a straight line maintaining a heading
        # of 0 degrees while twisting its ring to get its gather attachement
        # in place to snag elements at each way point.
        #
        # There may be occasion to add a phantom way point in order to get the
        # bot to move a certain way for some purpose.
        self.grab_on_the_go(
            distance=700, speed=200, bot_pos=(355.6, 108),
            waypoints=[(396, 540), (299, 700), (400, 825), (215, 900)]
        )

        # pivot_and_go is a maneuver that moves the bot straight while twisting
        # the ring in a way as to keep a specified point in the gatherer at a
        # fixed lattitude, distance from the bottom.
        self.pivot_and_go(200, 200, 80)

        # Move backwards and approach position to set up for change lanes
        self.straight_with_heading(-340, 0, 300)

        # turn heading (and wheels) to the 45 degree mark, align and lower arms
        run_task(self.tri_hard(45, 0, -1))

        # move forward and change heading by 90 degrees while keeping arms in
        # place
        self.straight_with_heading(80, 45, 50)
        self.target_heading(-45, 50, twist=True, tol=2)

        # perform parametric coordination of lift arms and bot movement in order
        # to get lift arms to trace a semi-circle in the x-z plane
        self.move_lift(
            2000,
            dict(amplitude=80, start=90, end=-90),
            dict(amplitude=80, start=-10, end=190)
        )

        # readjust heading, back out, head home
        self.target_heading(45, 50, twist=True, tol=2)
        self.straight_with_heading(-40, 45, 100)
        self.target_heading(0, 100, twist=True)
        self.straight_with_heading(-200, 0, 200)

    def rs3k1clsws1k2c(self):

        # get lift out of the way
        self.lift.run_until_stalled(300, duty_limit=50)

        # assuming the bot's center starts at 355.6 mm from the right and 108 mm
        # from the bottom, this traverses a straight line maintaining a heading
        # of 0 degrees while twisting its ring to get its gather attachement
        # in place to snag elements at each way point.
        #
        # There may be occasion to add a phantom way point in order to get the
        # bot to move a certain way for some purpose.
        run_task(self.straight_at_and_grab(
            distance=700, heading=0, speed=200, bot_pos=(355.6, 108),
            waypoints=[(396, 540), (299, 700), (400, 825), (215, 900)]
        ))

        # pivot_and_go is a maneuver that moves the bot straight while twisting
        # the ring in a way as to keep a specified point in the gatherer at a
        # fixed lattitude, distance from the bottom.
        run_task(self.pivot_and_go(200, 0, 200, 80))
        self.ring.twist_target(-90)
        self.drive.curve(-150, -90),
        # self.drive.curve(-150, -90)
        # run_task(self.ring.twist_callback(-90))
        run_task(self.straight_at(-900, 90, 100))

    def test(self, twist, distance):
        async def task():
            await multitask(
                self.ring.twist_callback(twist, tol=2),
                self.straight_with_heading_callback(distance, 0, 100),
            )
        return run_task(task())

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

bot.rs3k1clsws1k2c()

# default: bot.drive.settings(189, 709, 126, 571)
# bot.drive.settings(189, 709, 126*10//4, 1000)
# print(bot.ring.control.limits())
# bot.ring.control.limits(2000, 2000, 1000)
# print(bot.ring.control.limits())
# run_task(bot.twist_turn(120, 50))
# run_task(bot.twist_turn(-120, 50))
# bot.ring.twist_target(360)
# bot.ring.twist_target(0)
# run_task(bot.straight_at(50, 0, 100))
# run_task(bot.straight_at(-50, 0, 50))

# init_buffer = 10
# run_task(bot.straight_at(init_buffer, 0, 50))
# run_task(bot.straight_at_and_grab(
#     distance=200-init_buffer, heading=0, speed=50, bot_pos=(355.6, 108-init_buffer),
#     waypoints=[(355.6, 300), (396, 320), (299, 340)]
# ))
# bot.ring.twist_target(180)
# bot.ring.twist_target(90)
# bot.ring.twist_target(0)
# bot.ring.twist_target(-90)
# bot.ring.twist_target(0)
# run_task(bot.straight_at(-200, 0, 100))

# bot.ring.twist_target(45)
# bot.ring.twist_target(135)
# bot.ring.twist_target(90)
# bot.ring.twist_target(180)
# bot.ring.twist_target(135)
# bot.ring.twist_target(270)
# bot.ring.twist_target(180)
# bot.ring.twist_target(315)
# bot.ring.twist_target(270)
# bot.ring.twist_target(360)
# bot.ring.twist_target(0)

# bot.ring.twist_target(-90)
# run_task(bot.pivot_and_go(distance=-200, heading=0, speed=200, hypotenuse=200))
# run_task(bot.straight_at(-200, 0, 100))
