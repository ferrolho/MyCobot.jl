using Revise

import LibSerialPort
import MyCobot

portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

sp = LibSerialPort.open(portname, baudrate)

MyCobot.get_angles(sp, verbose=true)

LibSerialPort.close(sp)
