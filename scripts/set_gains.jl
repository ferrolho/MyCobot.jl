# Set the servo position-loop gains (P, D, I) on all joints, until the next power cycle.
#
#   julia --project=. scripts/set_gains.jl [ours|stored|stock] [--atom=IP]
#
# ours = MyCobot.GAINS (also written by the controller firmware at power-up), stored =
# MyCobot.SERVO_STORED_GAINS, stock = MyCobot.STOCK_FIRMWARE_GAINS. With --atom=IP through the
# ATOM (firmware ≥ 3), otherwise through the FT232R. Does not move the robot.

import MyCobot

pos = filter(a -> !startswith(a, "--"), ARGS)
which = isempty(pos) ? "ours" : pos[1]
gains = Dict("ours" => MyCobot.GAINS, "stored" => MyCobot.SERVO_STORED_GAINS, "stock" => MyCobot.STOCK_FIRMWARE_GAINS)[which]
atom = findfirst(startswith("--atom="), ARGS)
if atom === nothing
    sp = MyCobot.open_bus()
    try
        println("gains (P, D, I) now: ", MyCobot.set_servo_gains!(sp, gains))
    finally
        close(sp)
    end
else
    link = MyCobot.AtomLink(split(ARGS[atom], "=")[2])
    try
        println("gains (P, D, I) now: ", MyCobot.atom_set_gains!(link, gains))
    finally
        close(link)
    end
end
