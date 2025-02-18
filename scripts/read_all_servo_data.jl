import LibSerialPort
import MyCobot

# Set the serial port connection parameters
portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000

# Open the serial port connection
sp = LibSerialPort.open(portname, baudrate)

# Power on the robot arm
MyCobot.power_on(sp)

# Print the servo data for all servos
MyCobot.print_servo_data(sp)

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

# Close the serial port connection
LibSerialPort.close(sp)
