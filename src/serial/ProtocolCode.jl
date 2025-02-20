import LibSerialPort

baremodule ProtocolCode

using Base: @enum

@enum ProtocolCodeEnum begin

    # Serial data framing
    FRAME_HEADER = 0xFE  # Frame start flag (2 bytes)
    FRAME_FOOTER = 0xFA  # Frame end flag (1 byte)

    # System status
    ROBOT_VERSION = 0x01
    SOFTWARE_VERSION = 0x02
    GET_ROBOT_ID = 0x03

    # Robot status
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWER_ON = 0x12
    RELEASE_ALL_SERVOS = 0x13
    IS_CONTROLLER_CONNECTED = 0x14
    SET_FRESH_MODE = 0x16
    GET_FRESH_MODE = 0x17
    SET_FREE_MODE = 0x1A
    IS_FREE_MODE = 0x1B

    # Manual Data Input (MDI) mode
    GET_ANGLES = 0x20
    SEND_ANGLES = 0x22

    # Servo control
    IS_SERVO_POWERED = 0x50
    ARE_ALL_SERVOS_POWERED = 0x51
    SET_SERVO_PARAMETER = 0x52
    READ_SERVO_PARAMETER = 0x53
    SET_SERVO_ZERO = 0x54
    BRAKE_SERVO = 0x55
    POWER_OFF_SERVO = 0x56
    POWER_ON_SERVO = 0x57

    # ATOM IO control
    SET_PIN_MODE = 0x60
    SET_DIGITAL_OUTPUT = 0x61
    GET_DIGITAL_INPUT = 0x62
    SET_COLOR = 0x6A

    # Gripper control
    GET_GRIPPER_VALUE = 0x65
    SET_GRIPPER_STATE = 0x66
    SET_GRIPPER_VALUE = 0x67
    SET_GRIPPER_CALIBRATION = 0x68
    IS_GRIPPER_MOVING = 0x69

end

end

import .ProtocolCode: ProtocolCodeEnum


"""
    prepare_frame(command::ProtocolCodeEnum, data::Vector{UInt8}=UInt8[])

Prepare a frame to be sent over the serial port.

The frame consists of the following parts:

    - Header byte (0xFE)
    - Header byte (0xFE)
    - Data length byte (excluding the header bytes and the footer byte)
    - Command byte (e.g., ProtocolCode.GET_ANGLES)
    - Data bytes (if any)
    - Footer byte (0xFA)

The `command` argument specifies the command to be sent to the robot, and the `data` argument is an optional vector of data bytes to be included in the frame.
"""
function prepare_frame(command::ProtocolCodeEnum, data::Vector{UInt8}=UInt8[])
    # Ensure the data length is within the protocol limits
    if length(data) > 16
        error("Data length exceeds maximum allowed size (16 bytes)")
    end

    # 1 length byte + 1 command byte + the actual data
    data_length = 2 + length(data)

    frame = UInt8[
        UInt(ProtocolCode.FRAME_HEADER)
        UInt(ProtocolCode.FRAME_HEADER)
        UInt(data_length)
        UInt(command)
        data...
        UInt(ProtocolCode.FRAME_FOOTER)
    ]

    # Ensure the frame length is correct
    # 2 header bytes + `data_length` bytes + 1 footer byte
    @assert length(frame) == 3 + data_length

    return frame
end

"""
    extract_all_frames(response::Vector{UInt8})::Vector{Vector{UInt8}}

Extract all complete frames from a response buffer. Returns a vector of frame vectors.
"""
function extract_all_frames(response::Vector{UInt8})::Vector{Vector{UInt8}}
    frames = Vector{UInt8}[]
    i = 1

    while i <= length(response) - 4  # Minimum frame length is 5 bytes
        # Check for two consecutive header bytes
        if response[i] == response[i+1] == UInt8(ProtocolCode.FRAME_HEADER)
            # Extract the data length byte
            data_length = response[i+2]

            # Calculate the frame length
            frame_length = 3 + data_length

            # Check if the frame is complete
            if i + frame_length - 1 <= length(response)
                # Extract the frame
                frame = response[i:i+frame_length-1]

                # Verify the footer byte and then save the frame
                if frame[end] == UInt8(ProtocolCode.FRAME_FOOTER)
                    push!(frames, frame)
                end
            end

            # Move to the next potential frame
            i += frame_length
        else
            # Move to the next byte
            i += 1
        end
    end

    return frames
end

"""
    wait_for_command_response(sp::LibSerialPort.SerialPort, expected_command::ProtocolCodeEnum; verbose::Bool=false, timeout::Real=5.0)

Wait for a response frame with the expected command ID.

# Arguments
- `sp::LibSerialPort.SerialPort`: The serial port connection to the robot.
- `expected_command::ProtocolCodeEnum`: The expected command ID in the response frame.
- `verbose::Bool`: If `true`, print debugging information.
- `timeout::Real`: Maximum time (in seconds) to wait for a response.

# Returns
- `response_frame::Vector{UInt8}`: The most recent response frame matching the expected command.

# Throws
- `TimeoutError`: If no valid response is received within the timeout period.
"""
function wait_for_command_response(sp::LibSerialPort.SerialPort, expected_command::ProtocolCodeEnum; verbose::Bool=false, timeout::Real=0.1)
    response_buffer = UInt8[]
    matching_frames = Vector{UInt8}[]
    start_time = time()

    while isempty(matching_frames)
        # Check for timeout
        if time() - start_time > timeout
            verbose && println("Response buffer: ", response_buffer)
            error("Timeout while waiting for response to command $expected_command")
        end

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
        matching_frames = filter(frame -> frame[4] == UInt8(expected_command), frames)

        if isempty(matching_frames)
            verbose && println("No valid response frame found for command $expected_command. Retrying...")
        end
    end

    # Use the most recent frame (last in the list)
    response_frame = matching_frames[end]
    verbose && println("Selected response frame: ", response_frame)

    return response_frame
end

include("atom_io_control.jl")
include("gripper_control.jl")
include("mdi_mode.jl")
include("robot_status.jl")
include("servo_control.jl")
include("system_status.jl")
include("utils.jl")
