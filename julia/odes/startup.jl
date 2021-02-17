import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics, RigidBodySim
using LinearAlgebra
using StaticArrays
using MeshCat, MeshCatMechanisms
#using GeometryTypes, CoordinateTransformations, ColorTypes
using Blink

#Create visualizer using meshcat
vis = Visualizer(); 
open(vis,Window())

square(n) = n*n

function display_urdf(urdfPath,vis)

    mechanism = parse_urdf(Float64,urdfPath) 
    #creates mechanism by parsing the urdf file 
    # under RigidBodyDynamics

    state = MechanismState(mechanism) 
    #stores state information for an entire Mechanism 

    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
   
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    return mvis, mechanism
end





function do_ik!(state::MechanismState,
                desired::Point3D,
                path;
                a=0.1)

    lambda = 0.5
    dof = 10

    mechanism = state.mechanism;
    world = root_frame(mechanism);

    q = copy(configuration(state));
    
    dq=10;

    while norm(dq)>0.0001
        J = Array(geometric_jacobian(state, p))
        # remove orientation part of Jacobian
        J = J[1:size(J, 1) .<=3 , :]

        # Compute an update in joint coordinates using the jacobian transpose
        dq = a * (inv(J'*J + square(lambda)*Matrix(1.0I,dof,dof))*J' *(transform(state, desired, world) - desired).v)
        println(dq)

        # Apply the update
        q .= configuration(state) .+ dq

        set_configuration!(state, q)
        set_configuration!(mvis,configuration(state))
    end
    
    return state
end



 
 

mvis, mechanism = display_urdf("10dofRobot.urdf",vis)
state=MechanismState(mechanism)

b = bodies(mechanism)

left_eef_body = b[end]
right_eef_body = b[end-1]

base_body = b[1]

left_eef_path = path(mechanism,base_body,left_eef_body)
right_eef_path = path(mechanism,base_body,right_eef_body)

point = Point3D(root_frame(mechanism),-2,1,0.5)
do_ik!(state,point,left_eef_path)


transform(state, point, root_frame(mechanism))
