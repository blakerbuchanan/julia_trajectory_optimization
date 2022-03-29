using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using MeshCat
using RigidBodyDynamics
using MeshCatMechanisms

vis = Visualizer()
# open(vis)  # open the visualizer in a separate tab/window
render(vis) # render the visualizer here inside the jupyter notebook

# urdf = "my_first.urdf"
# robot = parse_urdf(urdf)
# delete!(vis)
# mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
# set_configuration!(mvis, [0.0, 0.0])