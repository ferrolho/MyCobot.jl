using Revise

import LibSerialPort
import MeshCat
import MeshCatMechanisms
import RigidBodyDynamics as RBD

import MyCobot

# Open the 3D visualiser
vis = MeshCat.Visualizer()
MeshCat.open(vis)

robot_name = "mycobot_280_arduino"
package_path = joinpath(@__DIR__, "..")
filename = joinpath(package_path, "mycobot_description", "urdf", robot_name, "$(robot_name).urdf")
mechanism = RBD.parse_urdf(filename, remove_fixed_tree_joints=false)

MeshCat.delete!(vis[robot_name])  # clear any existing robot
urdfvisuals = MeshCatMechanisms.URDFVisuals(filename, package_path=[package_path])
mvis = MeshCatMechanisms.MechanismVisualizer(mechanism, urdfvisuals, vis[robot_name])
