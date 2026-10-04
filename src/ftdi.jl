# FT232R USB latency timer. The chip holds short replies for up to `latency` ms before
# sending them to the host; the default of 16 ms was most of the "20 ms get_angles"
# problem. macOS's AppleUSBFTDI driver has no setting for it, but the chip accepts the
# FTDI vendor control request on endpoint 0 while the serial port is open. The value
# resets to 16 ms when the adapter is unplugged or the robot is power-cycled.

import libusb_jll

const FTDI_VID = 0x0403
const FT232R_PID = 0x6001
const SIO_SET_LATENCY_TIMER = 0x09
const SIO_GET_LATENCY_TIMER = 0x0A
const FTDI_INTERFACE_A = 0x0001

function with_ftdi_device(f; vid=FTDI_VID, pid=FT232R_PID)
    ctx = Ref{Ptr{Cvoid}}(C_NULL)
    rc = ccall((:libusb_init, libusb_jll.libusb), Cint, (Ptr{Ptr{Cvoid}},), ctx)
    rc == 0 || error("libusb_init failed ($rc)")
    try
        handle = ccall((:libusb_open_device_with_vid_pid, libusb_jll.libusb), Ptr{Cvoid},
                       (Ptr{Cvoid}, UInt16, UInt16), ctx[], vid, pid)
        handle == C_NULL && error("FTDI device $(string(vid, base=16)):$(string(pid, base=16)) not found or not accessible")
        try
            return f(handle)
        finally
            ccall((:libusb_close, libusb_jll.libusb), Cvoid, (Ptr{Cvoid},), handle)
        end
    finally
        ccall((:libusb_exit, libusb_jll.libusb), Cvoid, (Ptr{Cvoid},), ctx[])
    end
end

function control_transfer(handle, request_type, request, value, index, buffer::Vector{UInt8})
    ccall((:libusb_control_transfer, libusb_jll.libusb), Cint,
          (Ptr{Cvoid}, UInt8, UInt8, UInt16, UInt16, Ptr{UInt8}, UInt16, Cuint),
          handle, request_type, request, value, index, buffer, length(buffer), 1000)
end

"""
    get_latency_timer()

Read the FT232R latency timer in milliseconds.
"""
function get_latency_timer()
    with_ftdi_device() do handle
        buf = zeros(UInt8, 1)
        rc = control_transfer(handle, 0xC0, SIO_GET_LATENCY_TIMER, 0, FTDI_INTERFACE_A, buf)
        rc == 1 || error("reading the latency timer failed ($rc)")
        return Int(buf[1])
    end
end

"""
    set_latency_timer(ms=1)

Set the FT232R latency timer (1–255 ms). Use 1 ms for fast replies.
"""
function set_latency_timer(ms::Integer=1)
    1 <= ms <= 255 || throw(ArgumentError("latency must be 1–255 ms"))
    with_ftdi_device() do handle
        rc = control_transfer(handle, 0x40, SIO_SET_LATENCY_TIMER, ms, FTDI_INTERFACE_A, UInt8[])
        rc == 0 || error("setting the latency timer failed ($rc)")
    end
    return get_latency_timer()
end
