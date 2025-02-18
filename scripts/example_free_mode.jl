import LibSerialPort
import MyCobot

portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

sp = LibSerialPort.open(portname, baudrate)

# Power on the robot arm
MyCobot.power_on(sp)

# Query the ATOM power state
atom_state = MyCobot.is_power_on(sp)
println("ATOM is powered on: ", atom_state)

# Check the robot system status
controller_connected = MyCobot.is_controller_connected(sp)
println("Connected to ATOM: ", controller_connected)

# Enable free movement mode
MyCobot.set_free_mode(sp, true)

# Check if the robot is in free movement mode
free_mode_status = MyCobot.is_free_mode(sp)
println("Free movement mode: ", free_mode_status)

# Disable free movement mode
MyCobot.set_free_mode(sp, false)

# Check if the robot is in free movement mode
free_mode_status = MyCobot.is_free_mode(sp)
println("Free movement mode: ", free_mode_status)

# Release all servos
MyCobot.release_all_servos(sp)

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

LibSerialPort.close(sp)
