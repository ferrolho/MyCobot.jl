module MyCobot

import LibSerialPort

greet() = print("Hello World!")

include("ProtocolCode.jl")

function get_angles(sp::LibSerialPort.SerialPort; verbose::Bool=false)
    # Prepare the request frame
    request_frame = MyCobot.prepare_frame(MyCobot.ProtocolCode.GET_ANGLES)
    verbose && println("Request frame: ", request_frame)

    # Send the request frame and receive the response
    LibSerialPort.write(sp, request_frame)
    response = LibSerialPort.read(sp)

    # Extract the response frame from the response
    response_frame = MyCobot.extract_frame(response)
    verbose && println("Response frame: ", response_frame)

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

end # module MyCobot
