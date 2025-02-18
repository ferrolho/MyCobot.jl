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
