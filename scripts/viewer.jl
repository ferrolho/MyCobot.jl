using Revise

import LibSerialPort
import MeshCat
import MeshCatMechanisms
import RigidBodyDynamics as RBD

import MyCobot

# Open the 3D visualiser
vis = MeshCat.Visualizer()
MeshCat.open(vis)

# Set the camera and lighting
MeshCat.setprop!(vis["/Cameras/default/rotated/<object>"], "fov", 40)
MeshCat.setprop!(vis["/Lights/FillLight/<object>"], "intensity", 1.1)
MeshCat.setprop!(vis["/Lights/FillLight/<object>"], "position", [-2, 1, 2])

# Load the robot model from the URDF file
robot_name = "mycobot_280_arduino"
package_path = joinpath(@__DIR__, "..")
filename = joinpath(package_path, "mycobot_description", "urdf", robot_name, "$(robot_name).urdf")
mechanism = RBD.parse_urdf(filename, remove_fixed_tree_joints=false)

# Delete any existing robot and display the new robot model in the visualiser
MeshCat.delete!(vis[robot_name])  # clear any existing robot
urdfvisuals = MeshCatMechanisms.URDFVisuals(filename, package_path=[package_path])
mvis = MeshCatMechanisms.MechanismVisualizer(mechanism, urdfvisuals, vis[robot_name])

# Open the serial port connection to the robot
portname = "/dev/tty.usbserial-B00033ZX"
baudrate = 1000000
sp = LibSerialPort.open(portname, baudrate)

# Power on the robot arm
MyCobot.power_on(sp)

# Get the current joint angles
angles = MyCobot.get_angles(sp)
# Convert the angles to radians
angles_in_radians = deg2rad.(angles)
# Update the robot model with the current joint angles
RBD.set_configuration!(mvis, angles_in_radians)

# Release all servos
# WARNING: This will cause the robot arm to fall freely
MyCobot.release_all_servos(sp)

# For a few seconds, get the joint angles and update the robot model in the visualiser
start_time = time()
while time() - start_time < 20
    try
        angles = MyCobot.get_angles(sp)
        angles_in_radians = deg2rad.(angles)
        RBD.set_configuration!(mvis, angles_in_radians)
    catch e
        @warn e  # Catch any errors and print a warning, but continue
    end
    sleep(0.010) # Pause for a short time to avoid overloading the serial port
end

# Power off the robot (WARNING: All motors will be turned off)
MyCobot.power_off(sp)

# Close the serial port connection
LibSerialPort.close(sp)
