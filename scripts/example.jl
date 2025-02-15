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

# Set the motion mode to refresh mode
MyCobot.set_fresh_mode(sp, true)

# Get the current motion mode
mode = MyCobot.get_fresh_mode(sp)
println("Current motion mode: ", mode ? "Latest" : "Queue")

# Get the current joint angles
MyCobot.get_angles(sp)

# Set all joint angles to zero and move at 30% speed
angles = Float32[0, 0, 0, 0, 0, 0]  # in degrees
speed = UInt8(30)  # in percentage
MyCobot.send_angles(sp, angles, speed)

angles_sleep = Float32[0, -135, 140, 65, 90, 25]
MyCobot.send_angles(sp, angles_sleep, UInt8(50))

# For 5 seconds, move the first joint according to a sine wave
start_time = time()
while time() - start_time < 5
    amplitude = 90  # in degrees
    frequency = 0.25  # in Hz
    t = time() - start_time
    angle = amplitude * sin(2 * π * frequency * t)
    angles = Float32[angle, -45, 0, 0, 0, 0]  # in degrees
    speed = UInt8(100)  # in percentage
    MyCobot.send_angles(sp, angles, speed)
    sleep(0.010)
end

# Release all servos
MyCobot.release_all_servos(sp)

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

LibSerialPort.close(sp)
