from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait, StopWatch
from umath import pi, sin
from LegoLegacy import Bot, clear_console


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
i = 1
# bot._13(reverse=i==0)
# bot.twist_turn(90, trn_rt=200, trn_acc=200)
# bot.straight(-40)
# bot.liftby(7)
# bot.straight(20)
# bot.liftby(3)
# bot.straight(56)
# bot.liftby(-28, speed=3000)
sw = StopWatch()
bot.lift_sine_unit(3000, 14, 1000)
print(sw.time())
# bot.lift.reset_angle(0)
# bot.lift.run_angle(720, 360)
# print(bot.lift.angle())
# bot.straight(56)
# bot.lift.reset_angle(0)
# bot.liftto(2)
# bot.liftto(-2)
# bot.lift_sine_unit(180, 13, 2000)
# bot.lift_sine_unit(180, 13, 2000)
# bot.lift_sine_unit(180, 13, 2000)
# bot.move_lift_sine(-50, 180, 13, 2000)
# bot.lift.run_target(360, 270, Stop.COAST, wait=True)
# time = watch.time() / 1000
# print(bot.lift.angle() / time, time)
# bot.lift.run_target(360, 0, Stop.COAST, wait=True)
