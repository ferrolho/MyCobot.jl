using Revise

import LibSerialPort
import MyCobot

portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

sp = LibSerialPort.open(portname, baudrate)

# Query the ATOM power state
atom_state = MyCobot.is_power_on(sp)
println("ATOM is powered on: ", atom_state)

# Power on the robot arm
MyCobot.power_on(sp)

# Check the robot system status
controller_connected = MyCobot.is_controller_connected(sp, verbose=true)
println("Connected to ATOM: ", controller_connected)

# Set the ATOM's RGB LED to blue
MyCobot.set_color(sp, 0, 0, 255, verbose=true)

# Set the ATOM's RGB LED to red
MyCobot.set_color(sp, 255, 0, 0, verbose=true)

# Set the ATOM's RGB LED to green
MyCobot.set_color(sp, 0, 255, 0, verbose=true)

# Set pin 39 mode to 'input'
MyCobot.set_pin_mode(sp, 39, 0, verbose=true)

# Get the value of pin 39
pin_value = MyCobot.get_digital_input(sp, 39, verbose=true)

MyCobot.run_for_duration(5) do
    # Get ATOM button state
    pin_value = MyCobot.get_digital_input(sp, 39)
    print("\rPin 39 value: ", pin_value)
    sleep(0.010)
end

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

# Release all servos
MyCobot.release_all_servos(sp)

MyCobot.run_for_duration(10) do
    # Get the current joint angles
    angles = MyCobot.get_angles(sp)
    print("\rCurrent joint angles: ", angles)
    sleep(0.010)
end

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

LibSerialPort.close(sp)
