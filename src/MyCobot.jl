module MyCobot

import LibSerialPort

greet() = print("Hello World!")

include("ProtocolCode.jl")

function send_command(sp::LibSerialPort.SerialPort, command::ProtocolCodeEnum, data::Vector{UInt8}=UInt8[])
    # Construct the command frame
    frame = UInt8[]
    push!(frame, UInt8(ProtocolCode.HEADER))
    push!(frame, UInt8(ProtocolCode.HEADER))

    # Command length: 2 (header) + 1 (length byte) + 1 (command) + length(data) + 1 (footer)
    length_byte = UInt8(2 + 1 + 1 + length(data) + 1)
    push!(frame, length_byte)

    # Command ID
    push!(frame, UInt8(command))

    # Command content (if any)
    append!(frame, data)

    # Footer
    push!(frame, UInt8(ProtocolCode.FOOTER))

    # Send the frame
    write(sp, frame)
end

end # module MyCobot
