"""
    is_servo_powered(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)

Check if a specific servo is powered. Returns `true` if the servo is powered, `false` otherwise.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `joint_number::Int`: The servo joint number (1-6).
- `verbose::Bool`: If `true`, print debugging information.
"""
function is_servo_powered(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)
    # Validate the joint number
    if !(1 <= joint_number <= 6)
        error("Joint number must be between 1 and 6.")
    end

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.IS_SERVO_POWERED, UInt8[joint_number])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.IS_SERVO_POWERED; verbose)

    # Parse the connection status
    status_byte = response_frame[6]
    return status_byte == 0x01
end

"""
    are_all_servos_powered(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Check if all servos are powered. Returns `true` if all servos are powered, `false` otherwise.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function are_all_servos_powered(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.ARE_ALL_SERVOS_POWERED)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.ARE_ALL_SERVOS_POWERED; verbose)

    # Parse the power status
    status_byte = response_frame[5]
    return status_byte == 0x01
end

"""
    set_servo_parameter(sp::LibSerialPort.SerialPort, joint_number::Int, data_id::Int, value::Int; verbose::Bool=false)

Set a specific servo parameter.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `joint_number::Int`: The servo joint number (1-6).
- `data_id::Int`: The parameter ID (e.g., 20 for LED alarm, 21 for position P).
- `value::Int`: The parameter value.
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_servo_parameter(sp::LibSerialPort.SerialPort, joint_number::Int, data_id::Int, value::Int; verbose::Bool=false)
    # Validate the joint number, data ID, and value
    if !(1 <= joint_number <= 6)
        error("Joint number must be between 1 and 6.")
    end

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_SERVO_PARAMETER, UInt8[joint_number, data_id, value])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    read_servo_parameter(sp::LibSerialPort.SerialPort, joint_number::Int, data_id::Int; verbose::Bool=false)

Read a specific servo parameter.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `joint_number::Int`: The servo joint number (1-6).
- `data_id::Int`: The parameter ID (e.g., 20 for LED alarm, 21 for position P).
- `verbose::Bool`: If `true`, print debugging information.

# Returns
- `value::Int`: The parameter value.
"""
function read_servo_parameter(sp::LibSerialPort.SerialPort, joint_number::Int, data_id::Int; verbose::Bool=false)
    # Validate the joint number and data ID
    if !(1 <= joint_number <= 6)
        error("Joint number must be between 1 and 6.")
    end

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.READ_SERVO_PARAMETER, UInt8[joint_number, data_id])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.READ_SERVO_PARAMETER; verbose)

    # Parse the parameter value
    value_byte = response_frame[5]
    return Int(value_byte)
end

"""
    set_servo_zero(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)

Set the zero position for a specific servo.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `joint_number::Int`: The servo joint number (1-6).
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_servo_zero(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)
    # Validate the joint number
    if !(1 <= joint_number <= 6)
        error("Joint number must be between 1 and 6.")
    end

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_SERVO_ZERO, UInt8[joint_number])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    brake_servo(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)

Brake a specific servo.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `joint_number::Int`: The servo joint number (1-6).
- `verbose::Bool`: If `true`, print debugging information.
"""
function brake_servo(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)
    # Validate the joint number
    if !(1 <= joint_number <= 6)
        error("Joint number must be between 1 and 6.")
    end

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.BRAKE_SERVO, UInt8[joint_number])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    power_off_servo(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)

Power off a specific servo.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `joint_number::Int`: The servo joint number (1-6).
- `verbose::Bool`: If `true`, print debugging information.
"""
function power_off_servo(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)
    # Validate the joint number
    if !(1 <= joint_number <= 6)
        error("Joint number must be between 1 and 6.")
    end

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.POWER_OFF_SERVO, UInt8[joint_number])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    power_on_servo(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)

Power on a specific servo.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `joint_number::Int`: The servo joint number (1-6).
- `verbose::Bool`: If `true`, print debugging information.
"""
function power_on_servo(sp::LibSerialPort.SerialPort, joint_number::Int; verbose::Bool=false)
    # Validate the joint number
    if !(1 <= joint_number <= 6)
        error("Joint number must be between 1 and 6.")
    end

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.POWER_ON_SERVO, UInt8[joint_number])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end
