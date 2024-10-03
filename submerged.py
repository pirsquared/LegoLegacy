from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait, StopWatch
from umath import pi, sin, cos
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

def get_sweep_right_motor_callback(angle):
    drive_angle = angle / bot.wheel_ratio()
    def right(t):
        return drive_angle * t
    return right

def get_sweep_left_motor_callback(angle):
    drive_angle = angle / bot.wheel_ratio()
    def left(t):
        return drive_angle * t
    return left

def get_ring_motor_callback(angle):
    start_position = bot.dead_reck()
    def ring(t):
        distance = bot.dead_reck() - start_position
        if distance < 100:
            return 90


bot.ring_turn(8000, -360)
