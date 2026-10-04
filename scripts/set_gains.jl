# Set the servo position-loop gains (P, D, I) on all joints, until the next power cycle.
#
#   julia --project=. scripts/set_gains.jl tuned|default
#
# tuned = MyCobot.TUNED_GAINS (integral action on J1–J3), default = MyCobot.DEFAULT_GAINS.
# Does not move the robot. Don't run it while the ATOM plays a plan (one bus master at a time).

import MyCobot

which = isempty(ARGS) ? "tuned" : ARGS[1]
gains = which == "tuned" ? MyCobot.TUNED_GAINS : which == "default" ? MyCobot.DEFAULT_GAINS : error("tuned or default")
sp = MyCobot.open_bus()
try
    println("gains (P, D, I) now: ", MyCobot.set_servo_gains!(sp, gains))
finally
    close(sp)
end
