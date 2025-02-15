baremodule ProtocolCode

using Base: @enum

@enum ProtocolCodeEnum begin

    # BASIC
    HEADER = 0xFE
    FOOTER = 0xFA

    # MDI MODE AND OPERATION
    GET_ANGLES = 0x20

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

    data_length = 2 + length(data)

    frame = UInt8[
        UInt(ProtocolCode.HEADER)
        UInt(ProtocolCode.HEADER)
        UInt(data_length)
        UInt(command)
        data...
        UInt(ProtocolCode.FOOTER)
    ]

    return frame
end

"""
    extract_frame(response::Vector{UInt8})

Extract a single frame from the response byte vector. The frame is identified by the two consecutive header bytes (0xFE) and the footer byte (0xFA).

The frame has the following structure:

    1st byte: Header byte (0xFE)
    2nd byte: Header byte (0xFE)
    3rd byte: Data length byte (excluding the header bytes and the footer byte)
    4th byte: Command byte (e.g., ProtocolCode.GET_ANGLES)
    5th byte: First data byte (if any)
    ...
    Last byte: Footer byte (0xFA)
"""
function extract_frame(response::Vector{UInt8})
    # Initialise the index of the frame start
    # (i.e., the index of the first header byte)
    idx_frame_start = 0

    # Find the first pair of consecutive header bytes
    for i in 1:length(response)-1
        # Check if the current byte and the next byte are both header bytes
        if response[i] == response[i+1] == UInt8(ProtocolCode.HEADER)
            # Found the first pair of consecutive header bytes
            idx_frame_start = i
            # Exit the loop
            break
        end
    end

    if idx_frame_start == 0
        # If the frame start index is still 0, it means that no
        # valid frame header was found and so we throw an error
        error("Could not find a frame to extract")
    end

    # Extract the data length byte
    # Note: the data length byte includes the command byte and the data bytes
    idx_data_length = idx_frame_start + 2
    data_length = response[idx_data_length]

    # Extract the command byte
    idx_command = idx_frame_start + 3
    command = response[idx_command]

    # Calculate the index of the footer byte (end of the frame)
    idx_footer = idx_data_length + data_length
    footer = response[idx_footer]

    # If the footer byte is not valid, throw an error
    if footer != UInt8(ProtocolCode.FOOTER)
        error("Invalid footer byte")
    end

    # Extract the frame
    frame = response[idx_frame_start:idx_footer]

    return frame
end
