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
