from pybricks.hubs import PrimeHub
from pybricks.parameters import Port
from pybricks.tools import multitask, run_task, 
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
bot.twist_turn(120)
bot.twist_turn(-120)
