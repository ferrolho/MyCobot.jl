using Revise

import LibSerialPort
import MyCobot

portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

sp = LibSerialPort.open(portname, baudrate)

# Query the Atom power state
atom_state = MyCobot.is_power_on(sp)
println("Atom is powered on: ", atom_state)

# Power on the robot arm
MyCobot.power_on(sp)

# Check the robot system status
controller_connected = MyCobot.is_controller_connected(sp, verbose=true)
println("Connected to Atom: ", controller_connected)

# Get the current joint angles
MyCobot.get_angles(sp, verbose=true)

# Set all joint angles to zero and move at 30% speed
angles = Float32[0, 0, 0, 0, 0, 0]  # in degrees
speed = UInt8(30)  # in percentage
MyCobot.send_angles(sp, angles, speed)

# Release all servos
MyCobot.release_all_servos(sp)

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

LibSerialPort.close(sp)
