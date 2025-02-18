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

"""
    set_free_mode(sp::LibSerialPort.SerialPort, enable::Bool; verbose::Bool=false)

Enable or disable free movement mode.

The LED matrix of the ATOM will turn yellow when free movement mode is enabled. While
in free mode, pressing the ATOM button (which is the LED display itself) will disengage
the servos, therefore allowing the configuration of the robot to be changed by hand.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `enable::Bool`: `true` to enable free movement mode, `false` to disable.
- `verbose::Bool`: If `true`, print debugging information.
"""
function set_free_mode(sp::LibSerialPort.SerialPort, enable::Bool; verbose::Bool=false)
    # Prepare the data byte (0x01 for enable, 0x00 for disable)
    enable_byte = enable ? 0x01 : 0x00

    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.SET_FREE_MODE, [enable_byte])
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)
end

"""
    is_free_mode(sp::LibSerialPort.SerialPort; verbose::Bool=false)

Check whether the robot is in free movement mode.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `verbose::Bool`: If `true`, print debugging information.
"""
function is_free_mode(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare and send the request frame
    request_frame = prepare_frame(ProtocolCode.IS_FREE_MODE)
    verbose && println("Request frame: ", request_frame)
    LibSerialPort.write(sp, request_frame)

    # Wait for a response frame with the expected command ID
    response_frame = wait_for_command_response(sp, ProtocolCode.IS_FREE_MODE; verbose)

    # Parse the free mode status
    status_byte = response_frame[5]
    return status_byte == 0x01
end
