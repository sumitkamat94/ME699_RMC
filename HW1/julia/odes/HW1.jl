import Pkg;
Pkg.activate(@__DIR__);
using RigidBodyDynamics, RigidBodySim, DifferentialEquations
using LinearAlgebra
using StaticArrays
using MeshCat, MeshCatMechanisms, Blink
vis = Visualizer();open(vis,Window())

function display_urdf(urdfPath,vis)
    # Displays mechanism at config all zeros
    # urdfPath must be a string
    #urdfPath = "planar3R.urdf"

    mechanism = parse_urdf(Float64,urdfPath)

    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    return mechanism
end

urdfPath = "./file.urdf"
robotmech = parse_urdf(Float64,urdfPath)

state = MechanismState(robotmech)
zero_configuration!(state)

mvis = MechanicalVisualizer(robotmech,URDFVisuals(urdfPath),vis)

b = bodies(robotmech)


