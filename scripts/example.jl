using Revise

import LibSerialPort
import MyCobot

portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

sp = LibSerialPort.open(portname, baudrate)

MyCobot.get_angles(sp, verbose=true)

# Set all joint angles to zero and move at 30% speed
angles = Float32[90, 0, 0, 0, 0, 0]  # in degrees
speed = UInt8(30)  # in percentage
MyCobot.send_angles(sp, angles, speed, verbose=true)

LibSerialPort.close(sp)
