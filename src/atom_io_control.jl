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
