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
# The robot's FT232R. The ATOM's USB chip also reports FTDI's 0403:6001, so pick by serial.
const FT232_SERIAL = "B00033ZX"

# struct libusb_device_descriptor (18 bytes)
struct DeviceDescriptor
    bLength::UInt8; bDescriptorType::UInt8; bcdUSB::UInt16; bDeviceClass::UInt8
    bDeviceSubClass::UInt8; bDeviceProtocol::UInt8; bMaxPacketSize0::UInt8
    idVendor::UInt16; idProduct::UInt16; bcdDevice::UInt16
    iManufacturer::UInt8; iProduct::UInt8; iSerialNumber::UInt8; bNumConfigurations::UInt8
end

function with_ftdi_device(f; vid=FTDI_VID, pid=FT232R_PID, serial::AbstractString=FT232_SERIAL)
    ctx = Ref{Ptr{Cvoid}}(C_NULL)
    rc = ccall((:libusb_init, libusb_jll.libusb), Cint, (Ptr{Ptr{Cvoid}},), ctx)
    rc == 0 || error("libusb_init failed ($rc)")
    list = Ref{Ptr{Ptr{Cvoid}}}(C_NULL)
    try
        n = ccall((:libusb_get_device_list, libusb_jll.libusb), Cssize_t, (Ptr{Cvoid}, Ptr{Ptr{Ptr{Cvoid}}}), ctx[], list)
        n < 0 && error("libusb_get_device_list failed ($n)")
        found = String[]
        for i in 1:n
            dev = unsafe_load(list[], i)
            desc = Ref{DeviceDescriptor}()
            ccall((:libusb_get_device_descriptor, libusb_jll.libusb), Cint, (Ptr{Cvoid}, Ref{DeviceDescriptor}), dev, desc) == 0 || continue
            (desc[].idVendor == vid && desc[].idProduct == pid) || continue
            handle = Ref{Ptr{Cvoid}}(C_NULL)
            ccall((:libusb_open, libusb_jll.libusb), Cint, (Ptr{Cvoid}, Ref{Ptr{Cvoid}}), dev, handle) == 0 || continue
            buf = zeros(UInt8, 64)
            len = ccall((:libusb_get_string_descriptor_ascii, libusb_jll.libusb), Cint, (Ptr{Cvoid}, UInt8, Ptr{UInt8}, Cint),
                        handle[], desc[].iSerialNumber, buf, length(buf))
            sn = len > 0 ? String(buf[1:len]) : ""
            push!(found, sn)
            if sn == serial
                try
                    return f(handle[])
                finally
                    ccall((:libusb_close, libusb_jll.libusb), Cvoid, (Ptr{Cvoid},), handle[])
                end
            end
            ccall((:libusb_close, libusb_jll.libusb), Cvoid, (Ptr{Cvoid},), handle[])
        end
        error("FTDI device with serial $serial not found (found: $(isempty(found) ? "none" : join(found, ", ")))")
    finally
        list[] == C_NULL || ccall((:libusb_free_device_list, libusb_jll.libusb), Cvoid, (Ptr{Ptr{Cvoid}}, Cint), list[], 1)
        ccall((:libusb_exit, libusb_jll.libusb), Cvoid, (Ptr{Cvoid},), ctx[])
    end
end

function control_transfer(handle, request_type, request, value, index, buffer::Vector{UInt8})
    ccall((:libusb_control_transfer, libusb_jll.libusb), Cint,
          (Ptr{Cvoid}, UInt8, UInt8, UInt16, UInt16, Ptr{UInt8}, UInt16, Cuint),
          handle, request_type, request, value, index, buffer, length(buffer), 1000)
end

"""
    default_port(; serial=FT232_SERIAL)

The FT232R's serial port: `/dev/tty.usbserial-<serial>` on macOS and
`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_<serial>-if00-port0` on Linux.
"""
default_port(; serial::AbstractString=FT232_SERIAL) =
    Sys.islinux() ? "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_$(serial)-if00-port0" : "/dev/tty.usbserial-$serial"

# On Linux the ftdi_sio driver has the latency timer in sysfs. Writing it needs root, so a udev rule
# sets it to 1 ms when the adapter is plugged in:
#   /etc/udev/rules.d/99-ftdi-latency.rules:
#   ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
function sysfs_latency_path(serial::AbstractString)
    tty = basename(realpath(default_port(; serial=serial)))
    return "/sys/bus/usb-serial/devices/$tty/latency_timer"
end

"""
    get_latency_timer(; serial=FT232_SERIAL)

Read the FT232R latency timer in milliseconds.
"""
function get_latency_timer(; serial::AbstractString=FT232_SERIAL)
    Sys.islinux() && return parse(Int, strip(read(sysfs_latency_path(serial), String)))
    with_ftdi_device(; serial=serial) do handle
        buf = zeros(UInt8, 1)
        rc = control_transfer(handle, 0xC0, SIO_GET_LATENCY_TIMER, 0, FTDI_INTERFACE_A, buf)
        rc == 1 || error("reading the latency timer failed ($rc)")
        return Int(buf[1])
    end
end

"""
    set_latency_timer(ms=1; serial=FT232_SERIAL)

Set the FT232R latency timer (1–255 ms). Use 1 ms for fast replies. On Linux this needs
write access to sysfs; with the udev rule above the value is already 1 ms.
"""
function set_latency_timer(ms::Integer=1; serial::AbstractString=FT232_SERIAL)
    1 <= ms <= 255 || throw(ArgumentError("latency must be 1–255 ms"))
    if Sys.islinux()
        path = sysfs_latency_path(serial)
        get_latency_timer(; serial=serial) == ms && return ms
        try
            write(path, string(ms))
        catch
            error("cannot write $path (needs root). Add the udev rule in src/ftdi.jl, or run: echo $ms | sudo tee $path")
        end
        return get_latency_timer(; serial=serial)
    end
    with_ftdi_device(; serial=serial) do handle
        rc = control_transfer(handle, 0x40, SIO_SET_LATENCY_TIMER, ms, FTDI_INTERFACE_A, UInt8[])
        rc == 0 || error("setting the latency timer failed ($rc)")
    end
    return get_latency_timer(; serial=serial)
end
