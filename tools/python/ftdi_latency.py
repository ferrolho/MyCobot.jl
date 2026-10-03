"""
Read or set the FT232R USB latency timer.

The FTDI chip holds short replies for up to `latency` ms before sending them to the
host. The default of 16 ms was most of the "20 ms get_angles" problem. macOS's
built-in AppleUSBFTDI driver has no setting for it, but the chip accepts a vendor
control request on endpoint 0, which works while the serial port is open.

The value is NOT stored on the chip: it resets to 16 ms when the adapter is unplugged.

Usage:
    python ftdi_latency.py        # print current value
    python ftdi_latency.py 1      # set to 1 ms (valid range 1-255)

Requires `pyusb` and libusb (`brew install libusb`).
"""

import sys

import usb.backend.libusb1
import usb.core

FTDI_VID, FT232R_PID = 0x0403, 0x6001
SIO_SET_LATENCY_TIMER, SIO_GET_LATENCY_TIMER = 0x09, 0x0A
INTERFACE_A = 1


def find_device():
    backend = usb.backend.libusb1.get_backend(
        find_library=lambda _: "/opt/homebrew/lib/libusb-1.0.dylib")
    dev = usb.core.find(idVendor=FTDI_VID, idProduct=FT232R_PID, backend=backend)
    if dev is None:
        raise SystemExit("FT232R not found")
    return dev


def get_latency(dev):
    return dev.ctrl_transfer(0xC0, SIO_GET_LATENCY_TIMER, 0, INTERFACE_A, 1)[0]


def set_latency(dev, ms):
    dev.ctrl_transfer(0x40, SIO_SET_LATENCY_TIMER, ms, INTERFACE_A, None)


if __name__ == "__main__":
    dev = find_device()
    if len(sys.argv) > 1:
        set_latency(dev, int(sys.argv[1]))
    print(f"FT232R latency timer: {get_latency(dev)} ms")
