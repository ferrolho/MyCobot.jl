module MyCobot

import LibSerialPort

greet() = print("Hello World!")

include("ProtocolCode.jl")
include("utils.jl")

"""
    power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Power on the robot arm.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.POWER_ON)
    verbose && println("Request frame: ", request_frame)
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
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.POWER_OFF)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    is_power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Query the power state of the ATOM (main controller). Returns `true` if the ATOM is powered on, `false` if it is powered off.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function is_power_on(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.IS_POWER_ON)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.IS_POWER_ON; verbose)

    # Parse the power state
    power_state_byte = response_frame[5]
    if power_state_byte == 0x01
        return true  # ATOM is powered on
    elseif power_state_byte == 0x00
        return false  # ATOM is powered off
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
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.RELEASE_ALL_SERVOS)
    verbose && println("Request frame: ", request_frame)
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
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.IS_CONTROLLER_CONNECTED)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.IS_CONTROLLER_CONNECTED; verbose)

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

"""
    set_fresh_mode(sp::LibSerialPort.SerialPort, mode::Bool; verbose::Bool=false)

Set the motion mode of the robot. Note that this mode seems to persist across power cycles.

- `mode::Bool`: The motion mode to set.
  - `true`: Always execute the latest command first.
  - `false`: Execute instructions sequentially in the form of a queue.
"""
function set_fresh_mode(sp::LibSerialPort.SerialPort, mode::Bool; verbose::Bool=false)
    # Convert the mode to a byte value
    mode_byte = mode ? 0x01 : 0x00

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_FRESH_MODE, [mode_byte])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    get_fresh_mode(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Get the current motion mode of the robot.
"""
function get_fresh_mode(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.GET_FRESH_MODE)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.GET_FRESH_MODE; verbose)

    # Parse the mode byte
    mode_byte = response_frame[5]
    if mode_byte == 0x01
        return true
    elseif mode_byte == 0x00
        return false
    else
        error("Invalid mode byte in response: ", mode_byte)
    end
end

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

"""
    set_color(sp::LibSerialPort.SerialPort, r::Int, g::Int, b::Int; verbose::Bool=false)

Set the color of the ATOM's RGB LED. Each RGB value is in the range [0, 255].
"""
function set_color(sp::LibSerialPort.SerialPort, r::Int, g::Int, b::Int; verbose::Bool=false)
    # Validate the input values
    if !(0 <= r <= 255 && 0 <= g <= 255 && 0 <= b <= 255)
        error("RGB values must be between 0 and 255.")
    end

    # Prepare the data bytes
    data = UInt8[r, g, b]

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_COLOR, data)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    set_digital_output(sp::LibSerialPort.SerialPort, pin_no::Int, pin_signal::Int; verbose::Bool=false)

Set the digital output state of the specified pin in the ATOM.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the ATOM.
- `pin_no::Int`: The pin number to set.
- `pin_signal::Int`: The signal to set (0 - low, 1 - high).
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_digital_output(sp::LibSerialPort.SerialPort, pin_no::Int, pin_signal::Int; verbose::Bool=false)
    # Validate input
    if !(0 ≤ pin_signal ≤ 1)
        error("Invalid pin signal. Must be 0 (low) or 1 (high).")
    end
    if !(0 ≤ pin_no ≤ 255)
        error("Invalid pin number. Must be between 0 and 255.")
    end

    # Prepare the data bytes
    data = UInt8[pin_no, pin_signal]

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_DIGITAL_OUTPUT, data)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    get_digital_input(sp::LibSerialPort.SerialPort, pin_no::Int; verbose::Bool=false)

Get the digital input state of the specified pin in the ATOM.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the ATOM.
- `pin_no::Int`: The pin number to read.
- `verbose::Bool`: If `true`, print debugging information.

# Returns
- `Int`: The digital input signal (0 - low, 1 - high).
"""
function get_digital_input(sp::LibSerialPort.SerialPort, pin_no::Int; verbose::Bool=false)
    # Validate input
    if !(0 ≤ pin_no ≤ 255)
        error("Invalid pin number. Must be between 0 and 255.")
    end

    # Prepare the data bytes
    data = UInt8[pin_no]

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.GET_DIGITAL_INPUT, data)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame
    response_frame = wait_for_command_response(sp, ProtocolCode.GET_DIGITAL_INPUT; verbose)

    # Parse the pin and signal bytes
    pin_byte = response_frame[5]
    signal_byte = response_frame[6]

    # Validate the pin number
    if pin_byte != pin_no
        error("Invalid pin number in response: ", pin_byte)
    end

    # Validate the signal byte
    if signal_byte != 0x00 && signal_byte != 0x01
        error("Invalid signal byte in response: ", signal_byte)
    end

    # Return the signal value
    return Int(signal_byte)
end

"""
    set_pin_mode(sp::LibSerialPort.SerialPort, pin_no::Int, pin_mode::Int; verbose::Bool=false)

Set the mode of the specified pin in the ATOM.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the ATOM.
- `pin_no::Int`: The pin number to configure (0-255).
- `pin_mode::Int`: The mode to set (0 - input, 1 - output, 2 - input_pullup).
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_pin_mode(sp::LibSerialPort.SerialPort, pin_no::Int, pin_mode::Int; verbose::Bool=false)
    # Validate input
    if !(0 ≤ pin_no ≤ 255)
        error("Invalid pin number. Must be between 0 and 255.")
    end
    if !(0 ≤ pin_mode ≤ 2)
        error("Invalid pin mode. Must be 0 (input), 1 (output), or 2 (input_pullup).")
    end

    # Prepare the data bytes
    data = UInt8[pin_no, pin_mode]

    # Prepare the request frame
    request_frame = prepare_frame(ProtocolCode.SET_PIN_MODE, data)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    get_gripper_value(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Get the current gripper position.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.

# Returns
- `value::Int`: Gripper position (0-100).
"""
function get_gripper_value(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.GET_GRIPPER_VALUE)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.GET_GRIPPER_VALUE; verbose)

    # Parse the gripper value
    value_byte = response_frame[5]
    return Int(value_byte)
end

"""
    set_gripper_state(sp::LibSerialPort.SerialPort, flag::Int, speed::Int; verbose::Bool=false)

Set the gripper state (open, close, or release) at a specified speed.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `flag::Int`: 0 (open), 1 (close), or 254 (release).
- `speed::Int`: Speed (1-100).
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_gripper_state(sp::LibSerialPort.SerialPort, flag::Int, speed::Int; verbose::Bool=false)
    # Validate the input values
    if flag ∉ (0, 1, 254)
        error("Invalid flag value. Must be 0 (open), 1 (close), or 254 (release).")
    end
    if !(1 <= speed <= 100)
        error("Speed must be between 1 and 100.")
    end

    # Prepare the data bytes
    data = UInt8[flag, speed]

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_GRIPPER_STATE, data)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    set_gripper_value(sp::LibSerialPort.SerialPort, gripper_value::Int, speed::Int; verbose::Bool=false)

Set the gripper to a specific position at a specified speed.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `gripper_value::Int`: Gripper position (0-100).
- `speed::Int`: Speed (1-100).
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_gripper_value(sp::LibSerialPort.SerialPort, gripper_value::Int, speed::Int; verbose::Bool=false)
    # Validate the input values
    if !(0 <= gripper_value <= 100)
        error("Gripper value must be between 0 and 100.")
    end
    if !(1 <= speed <= 100)
        error("Speed must be between 1 and 100.")
    end

    # Prepare the data bytes
    data = UInt8[gripper_value, speed]

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_GRIPPER_VALUE, data)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    set_gripper_calibration(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Set the current gripper position as the 100% open position.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_gripper_calibration(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_GRIPPER_CALIBRATION)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    is_gripper_moving(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Check if the gripper is currently moving.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.

# Returns
- `0`: Gripper is not moving.
- `1`: Gripper is moving.
- `-1`: Error.
"""
function is_gripper_moving(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.IS_GRIPPER_MOVING)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.IS_GRIPPER_MOVING; verbose)

    # Parse the movement status
    status_byte = response_frame[5]
    if status_byte == 0x00
        return 0  # Gripper is not moving
    elseif status_byte == 0x01
        return 1  # Gripper is moving
    else
        return -1  # Error
    end
end

function get_robot_version(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.ROBOT_VERSION)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.ROBOT_VERSION; verbose)

    # Parse the version byte
    version_byte = response_frame[5]
    return Int(version_byte)
end

function get_software_version(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SOFTWARE_VERSION)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.SOFTWARE_VERSION; verbose)

    # Parse the version byte
    version_byte = response_frame[5]
    return Float64(version_byte / 10)
end

function get_robot_id(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.GET_ROBOT_ID)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.GET_ROBOT_ID; verbose)

    # Parse the ID byte
    id_byte = response_frame[5]
    return Int(id_byte)
end

end # module MyCobot
