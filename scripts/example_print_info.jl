import LibSerialPort
import MyCobot

portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

sp = LibSerialPort.open(portname, baudrate)

# Power on the robot arm
MyCobot.power_on(sp)

# Wait a bit for the robot to power on
sleep(0.1)  # 100 ms

# Get the robot's hardware version
robot_version = MyCobot.get_robot_version(sp)
println("Robot version: ", robot_version)

# Get the robot's software version (ATOM's firmware version)
software_version = MyCobot.get_software_version(sp)
println("Software version: ", software_version)

# Get the robot's ID
robot_id = MyCobot.get_robot_id(sp)
println("Robot ID: ", robot_id)

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

LibSerialPort.close(sp)
