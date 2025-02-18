function get_angles(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = MyCobot.prepare_frame(MyCobot.ProtocolCode.GET_ANGLES)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.GET_ANGLES; verbose)

    # Parse the angles from the response frame
    angles = MyCobot.parse_response_frame_get_angles(response_frame)
    verbose && println("Angles: ", angles)

    return angles
end

function parse_response_frame_get_angles(frame::Vector{UInt8})
    # Initialize an array to store the angles
    angles = zeros(Float32, 6)

    # Extract the high and low bytes of the angles
    angles_hi = frame[range(5, step=2, length=6)]
    angles_lo = frame[range(6, step=2, length=6)]

    # Define a function to parse the angle value. The formula to calculate
    # the angle value is the one provided in the MyCobot documentation.
    function parse_angle(angle_hi::UInt8, angle_lo::UInt8)
        temp = angle_lo + angle_hi * 256
        (temp > 33000 ? (temp - 65536) : temp) / 100
    end

    # Iterate over the angles and parse each one of them
    for (i, (angle_hi, angle_lo)) in enumerate(zip(angles_hi, angles_lo))
        # Parse the angle value and store it in the array
        angles[i] = parse_angle(angle_hi, angle_lo)
    end

    return angles
end

"""
    send_angles(sp::LibSerialPort.SerialPort, angles::Vector{Float32}, speed::UInt8; verbose::Bool=false)

Send all joint angles to the robot arm.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `angles::Vector{Float32}`: A vector of 6 joint angles (in degrees).
- `speed::UInt8`: The movement speed (0-100, where 100 is maximum speed).
- `verbose::Bool`: If `true`, print debugging information.
"""
function send_angles(sp::LibSerialPort.SerialPort, angles::Vector{Float32}, speed::UInt8; verbose::Bool=false)
    # Validate the input
    if length(angles) != 6
        error("The angles vector must contain exactly 6 values.")
    end
    if speed > 100
        error("The speed value must be between 0 and 100.")
    end

    # Convert angles to high and low bytes
    angle_bytes = UInt8[]
    for angle in angles
        # Multiply the angle by 100 and convert to an integer
        angle_int = round(Int16, angle * 100)

        # Extract the high and low bytes
        angle_high = UInt8((angle_int >> 8) & 0xFF)
        angle_low = UInt8(angle_int & 0xFF)

        push!(angle_bytes, angle_high)
        push!(angle_bytes, angle_low)
    end

    # Prepare the data for the frame
    data = vcat(angle_bytes, speed)

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SEND_ANGLES, data)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end
