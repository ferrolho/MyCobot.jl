baremodule ProtocolCode

using Base: @enum

@enum ProtocolCodeEnum begin

    # BASIC
    HEADER = 0xFE
    FOOTER = 0xFA

    # Overall status
    POWER_ON = 0x10
    POWER_OFF = 0x11
    IS_POWER_ON = 0x12
    RELEASE_ALL_SERVOS = 0x13
    IS_CONTROLLER_CONNECTED = 0x14

    # MDI MODE AND OPERATION
    GET_ANGLES = 0x20
    SEND_ANGLES = 0x22

    # ATOM IO
    GET_DIGITAL_INPUT = 0x62

    # Basic
    GET_BASIC_INPUT = 0xA1

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
        UInt(ProtocolCode.HEADER)
        UInt(ProtocolCode.HEADER)
        UInt(data_length)
        UInt(command)
        data...
        UInt(ProtocolCode.FOOTER)
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
        if response[i] == response[i+1] == UInt8(ProtocolCode.HEADER)
            # Extract the data length byte
            data_length = response[i+2]

            # Calculate the frame length
            frame_length = 3 + data_length

            # Check if the frame is complete
            if i + frame_length - 1 <= length(response)
                # Extract the frame
                frame = response[i:i+frame_length-1]

                # Verify the footer byte and then save the frame
                if frame[end] == UInt8(ProtocolCode.FOOTER)
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
