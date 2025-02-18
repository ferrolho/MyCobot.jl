import LibSerialPort
import MyCobot

# Set the serial port connection parameters
portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

# Open the serial port connection
sp = LibSerialPort.open(portname, baudrate)

# Query the ATOM power state
atom_state = MyCobot.is_power_on(sp)
println("ATOM is powered on: ", atom_state)

# Power on the robot arm
MyCobot.power_on(sp)

# Check the robot system status
controller_connected = MyCobot.is_controller_connected(sp)
println("Connected to ATOM: ", controller_connected)

# Select the servo joint number
joint_number = 1

# Check if the servo is powered
servo_powered = MyCobot.is_servo_powered(sp, joint_number)
println("Servo $(joint_number) powered: ", servo_powered)

# Check if all servos are powered
all_servos_powered = MyCobot.are_all_servos_powered(sp)
println("All servos powered: ", all_servos_powered)

# Read the position P parameter for the servo
position_p = MyCobot.read_servo_parameter(sp, joint_number, 21)
println("Servo $(joint_number) position P: ", position_p)

MyCobot.brake_servo(sp, joint_number)      # Brake the servo
MyCobot.power_off_servo(sp, joint_number)  # Power off the servo
MyCobot.power_on_servo(sp, joint_number)   # Power on the servo

# Release all servos
MyCobot.release_all_servos(sp)

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

# Close the serial port connection
LibSerialPort.close(sp)
