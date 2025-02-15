module MyCobot

import LibSerialPort

greet() = print("Hello World!")

include("ProtocolCode.jl")

"""
    power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Power on the robot arm.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare the request frame
    request_frame = prepare_frame(ProtocolCode.POWER_ON)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame
    LibSerialPort.write(sp, request_frame)
end

"""
    power_off(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Power off the robot arm and disconnect.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function power_off(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare the request frame
    request_frame = prepare_frame(ProtocolCode.POWER_OFF)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame
    LibSerialPort.write(sp, request_frame)
end

"""
    is_power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Query the power state of the Atom (main controller). Returns `true` if the Atom is powered on, `false` if it is powered off.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function is_power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare the request frame
    request_frame = prepare_frame(ProtocolCode.IS_POWER_ON)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame
    LibSerialPort.write(sp, request_frame)

    response_buffer = UInt8[]
    matching_frames = Vector{UInt8}[]

    while isempty(matching_frames)
        # Read the response and append it to the buffer
        response = LibSerialPort.read(sp)
        isempty(response) && continue
        append!(response_buffer, response)
        verbose && println("Response buffer: ", response_buffer)

        # Extract all frames from the response buffer
        frames = extract_all_frames(response_buffer)
        if verbose && !isempty(frames)
            println("All frames:")
            for frame in frames
                println("  ", frame)
            end
        end

        # Filter frames by command ID
        matching_frames = filter(frame -> frame[4] == UInt8(ProtocolCode.IS_POWER_ON), frames)

        if isempty(matching_frames)
            verbose && println("No valid response frame found for IS_POWER_ON command. Retrying...")
        end
    end

    # Use the most recent frame (last in the list)
    response_frame = matching_frames[end]
    verbose && println("Selected response frame: ", response_frame)

    # Parse the power state
    power_state_byte = response_frame[5]
    if power_state_byte == 0x01
        return true  # Atom is powered on
    elseif power_state_byte == 0x00
        return false  # Atom is powered off
    else
        error("Invalid power state byte in response: ", power_state_byte)
    end
end

"""
    release_all_servos(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Power off the robot arm only (without disconnecting).

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function release_all_servos(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare the request frame
    request_frame = prepare_frame(ProtocolCode.RELEASE_ALL_SERVOS)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame
    LibSerialPort.write(sp, request_frame)
end

"""
    is_controller_connected(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Check if the robot system is functioning normally.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.

# Returns
- `true`: The robot system is functioning normally.
- `false`: The robot system is not functioning normally.
"""
function is_controller_connected(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare the request frame
    request_frame = prepare_frame(ProtocolCode.IS_CONTROLLER_CONNECTED)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame
    LibSerialPort.write(sp, request_frame)

    response_buffer = UInt8[]
    matching_frames = Vector{UInt8}[]

    while isempty(matching_frames)
        # Read the response and append it to the buffer
        response = LibSerialPort.read(sp)
        isempty(response) && continue
        append!(response_buffer, response)
        verbose && println("Response buffer: ", response_buffer)

        # Extract all frames from the response buffer
        frames = extract_all_frames(response_buffer)
        if verbose && !isempty(frames)
            println("All frames:")
            for frame in frames
                println("  ", frame)
            end
        end

        # Filter frames by command ID
        matching_frames = filter(frame -> frame[4] == UInt8(ProtocolCode.IS_CONTROLLER_CONNECTED), frames)

        if isempty(matching_frames)
            verbose && println("No valid response frame found for IS_CONTROLLER_CONNECTED command. Retrying...")
        end
    end

    # Use the most recent frame (last in the list)
    response_frame = matching_frames[end]
    verbose && println("Selected response frame: ", response_frame)

    # Parse the system status
    status_byte = response_frame[5]
    if status_byte == 0x01
        return true  # System is functioning normally
    elseif status_byte == 0x00
        return false  # System is not functioning normally
    else
        error("Invalid status byte in response: ", status_byte)
    end
end

function get_angles(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare the request frame
    request_frame = MyCobot.prepare_frame(MyCobot.ProtocolCode.GET_ANGLES)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame
    LibSerialPort.write(sp, request_frame)

    response_buffer = UInt8[]
    matching_frames = Vector{UInt8}[]

    while isempty(matching_frames)
        # Read the response and append it to the buffer
        response = LibSerialPort.read(sp)
        isempty(response) && continue
        append!(response_buffer, response)
        verbose && println("Response buffer: ", response_buffer)

        # Extract all frames from the response buffer
        frames = extract_all_frames(response_buffer)
        if verbose && !isempty(frames)
            println("All frames:")
            for frame in frames
                println("  ", frame)
            end
        end

        # Filter frames by command ID
        matching_frames = filter(frame -> frame[4] == UInt8(ProtocolCode.GET_ANGLES), frames)

        if isempty(matching_frames)
            verbose && println("No valid response frame found for GET_ANGLES command. Retrying...")
        end
    end

    # Use the most recent frame (last in the list)
    response_frame = matching_frames[end]
    verbose && println("Selected response frame: ", response_frame)

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

# Example
```julia
angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # All angles set to zero
speed = 30  # 30% speed
send_angles(sp, angles, speed, verbose=true)
```
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
        angle_int = Int16(angle * 100)

        # Extract the high and low bytes
        angle_high = UInt8((angle_int >> 8) & 0xFF)
        angle_low = UInt8(angle_int & 0xFF)

        push!(angle_bytes, angle_high)
        push!(angle_bytes, angle_low)
    end

    # Prepare the data for the frame
    data = vcat(angle_bytes, speed)

    # Prepare the request frame
    request_frame = prepare_frame(ProtocolCode.SEND_ANGLES, data)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame
    LibSerialPort.write(sp, request_frame)
end

end # module MyCobot
