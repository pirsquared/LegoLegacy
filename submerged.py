from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait
from LegoLegacy import Bot, clear_console


class GameBot(Bot):
    def _13(self, reverse=False):
        distance = -80 if reverse else 80
        self.strait(100, speed=400)
        self.twist_turn(90, trn_rt=200, trn_acc=200)
        self.liftby(2)
        self.move_lift(distance, 10, move_speed=200, lift_speed=400)
        self.move_lift(distance, -10, move_speed=200, lift_speed=400)
        self.liftby(-2)
        self.twist_turn(-90, trn_rt=200, trn_acc=200)
        return self.strait(-100, speed=400)

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

bot._13()
