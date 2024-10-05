from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait, StopWatch
from umath import pi, sin, cos, acos
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

# for i in range(4):
#     sign = (i%2*2-1)
#     bot.move_lift(
#         3000,
#         drive_kwargs=dict(amplitude=80, start=sign*90, end=-sign*90),
#         lift_kwargs=dict(amplitude=80, start=-10, end=190)
#     )

def target_parameters(bot_pos, target):
    # `p` is unit abbr for peg length
    # 10p is 10 peg lengths
    # each peg length is 8mm
    # Bot diameter is 21p. From center of bot to center of outer peg
    # is 10p.  The sweet spot is between the ends of the straight
    # portions of the lift arms.  The sweet spot is 12p past the last
    # part of the bot diameter.
    # We'll use the sweet spot radius as the hypotenuse for calculating
    # the desired angle to align the sweet spot with the target x coordinate
    bot_x, bot_y = bot_pos
    tgt_x, tgt_y = target
    hyp = 176  # (10p + 12p) * 8mm/p
    adj = bot_x - tgt_x
    theta = acos(adj/hyp)

    opp = sin(theta) * hyp

    return tgt_y - (bot_y + opp), angle * 180 / pi

async def ring_callback(bot_pos, targets):

    targets = sorted([target_parameters(bot_pos, target) for target in targets])
    buffer = 20
    start_pos = bot.dead_reck()
    dist = 0
    if not targets:
        return
    while dist < positions[-1][0] + y_buffer:
        tgt, angle = max(positions, key=lambda x: x[0] > dist)
        if dist > tgt - buffer:
            if dist > tgt + buffer:
                bot.ring.track_target(bot.ring.parametric_ratio(90))
            else:
                bot.ring.track_target(bot.ring.parametric_ratio(angle))
        dist = bot.dead_reck() - start_pos


async def main():
    await multitask(
        bot.strait(700, speed=50),
        ring_callback(
            (355.6, 124),
            []
        ),
        race=True
    )

run_task(main())
# bot.ring.track_target(bot.ring.parametric_ratio(105))
# wait(2000)
# bot.ring.track_target(bot.ring.parametric_ratio(80))
# wait(2000)
# bot.ring.track_target(bot.ring.parametric_ratio(105))
# wait(2000)
# bot.ring.track_target(bot.ring.parametric_ratio(70))
# wait(5000)
bot.ring.run_target(bot.ring.parametric_ratio(0))
bot.strait(-680)
