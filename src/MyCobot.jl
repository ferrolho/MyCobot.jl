module MyCobot

import LibSerialPort

greet() = print("Hello World!")

include("ProtocolCode.jl")

include("atom_io_control.jl")
include("gripper_control.jl")
include("mdi_mode.jl")
include("robot_status.jl")
include("servo_control.jl")
include("system_status.jl")

include("utils.jl")

end # module MyCobot
