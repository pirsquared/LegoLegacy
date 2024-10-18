from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait, StopWatch
from umath import pi, sin, cos, asin, acos
from LegoLegacy import Bot, clear_console, _CONFIG


class GameBot(Bot):

    def _13(self, reverse=False):
        distance = -157 if reverse else 157
        # self.liftby(2)
        # self.strait(100, speed=400)
        self.twist_turn(90, trn_rt=200, trn_acc=200)
        self.move_lift_sine(distance, 720, 12, 4000)
        self.twist_turn(-90, trn_rt=200, trn_acc=200)
        # self.liftby(-1.5)
        # return self.strait(-100, speed=400)


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

bot.straight_with_heading(0, 0, 50)
bot.grab_on_the_go(
    distance=700, speed=300, bot_pos=(355.6, 108),
    waypoints=[(395, 538), (300, 696), (390, 812), (216, 892)]
)
bot.pivot_and_go(200, 100, 80)
bot.straight_with_heading(-300, 0, 300)
bot.ring.ring_angle(-45)

async def tri_hard(a, b):
    await multitask(
        bot.turn(a),
        bot.twist(b),
        bot.lift.run_until_stalled(-300, duty_limit=50)
    )

run_task(tri_hard(45, 90))
# bot.straight_with_heading(70, 45, 50)
# bot.twist_turn(90)

# # change lanes (semi-circle)
# bot.move_lift(
#     2000,
#     dict(amplitude=80, start=-90, end=90),
#     dict(amplitude=80, start=-10, end=190)
# )
# bot.twist_turn(-90)
# bot.strait(-70)
# run_task(tri_hard(-45, -90))

# bot.liftby(20, 300)


# bot.strait(-500, speed=400)
# bot.drive.curve(-bot.wheel_base, 90)
# bot.strait(-100, speed=400)
# bot.ring.run_target(100, 0)
# wait(2000)

