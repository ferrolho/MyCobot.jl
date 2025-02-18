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
controller_connected = MyCobot.is_controller_connected(sp)
println("Connected to Atom: ", controller_connected)

# Set gripper state to open at speed 50
MyCobot.set_gripper_state(sp, 0, 50)

# Disengage the gripper servo
MyCobot.set_gripper_state(sp, 254, 50)

# Set gripper calibration
MyCobot.set_gripper_calibration(sp)

MyCobot.run_for_duration(10) do
    # Check if the gripper is moving
    moving_status = MyCobot.is_gripper_moving(sp)
    print("\rGripper moving status: ", moving_status)
end

MyCobot.run_for_duration(10) do
    # Get the current gripper value
    gripper_value = MyCobot.get_gripper_value(sp)
    print("\rGripper value: ", gripper_value, "    ")
end

# Set gripper to 50% open at speed 20
MyCobot.set_gripper_value(sp, 50, 20)

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

LibSerialPort.close(sp)
