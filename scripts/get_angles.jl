using Revise

import LibSerialPort
import MyCobot

portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 115200

sp = LibSerialPort.open(portname, baudrate)

MyCobot.send_command(sp, MyCobot.ProtocolCode.GET_ANGLES)

LibSerialPort.close(sp)
