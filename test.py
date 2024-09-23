from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.tools import multitask, run_task, wait
from LegoLegacy import Bot

bot = Bot(
    left_motor=Port.A,
    right_motor=Port.F,
    ring_motor=Port.C,
    lift_motor=Port.B,
    left_eye=Port.E,
    right_eye=Port.D,
    hub_type=PrimeHub
)

# bot.twist(-60)
# bot.twist(60)

# bot.turn(60)
# bot.turn(-60)

# bot.straight(25.4)
# bot.straight(-25.4)

# bot.lift(4)
# bot.lift(-4)
# bot.twist_turn(120)
# bot.twist_turn(-120)

bot.left_motor.reset_angle(0)
bot.right_motor.reset_angle(0)
# bot.drive.use_gyro(True)
# bot.accu_turn(360)
bot.move_lift(50, 10, move_speed=50, lift_speed=100)
bot.move_lift(-50, -10, move_speed=50, lift_speed=100)
