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
                path,
                body,
                constraint;
                a=0.01)

    lambda = 0.5
    dof = 10

    mechanism = state.mechanism;
    world = root_frame(mechanism);

    point_in_world = get_eef_loc(state,body)
    Jp = point_jacobian(state, path, transform(state, point_in_world, world))

    q = copy(configuration(state));
    
    dq=10;

    while norm(dq)>0.0001
        point_in_world = get_eef_loc(state,body)

        Jp = point_jacobian!(Jp, state, path, point_in_world)
        J = Array(Jp)

        # remove orientation part of Jacobian
        J = J[1:size(J, 1) .<=3 , :]

        # zero out the constrained joints
        if constraint
            J[1] = 0.0
            J[2] = 0.0
            J[3] = 0.0
            J[4] = 0.0
            J[5] = 0.0
            J[6] = 0.0
        end

        # Compute an update in joint coordinates using the jacobian transpose
        dq = a * (inv((J'*J) + (square(lambda)*Matrix(1.0I,dof,dof)))*J' *(transform(state,desired,world) - point_in_world).v)
        #println(dq)
        # Apply the update
        q .= configuration(state) .+ dq

        set_configuration!(state, q)
        set_configuration!(mvis,configuration(state))
    end
    
    return state
end


function get_eef_loc(state, body)
    t = transform_to_root(state,body)
    return Point3D(root_frame(state.mechanism),t.mat[13],t.mat[14],t.mat[15])
end
 

mvis, mechanism = display_urdf("10dofRobot.urdf",vis)
state=MechanismState(mechanism)

b = bodies(mechanism)

left_eef_body = b[end]
right_eef_body = b[end-1]

base_body = b[1]

left_eef_path = path(mechanism,base_body,left_eef_body)
right_eef_path = path(mechanism,base_body,right_eef_body)

constrained = false

while true

    global constrained
   
    println("input desired x:")
    x = parse(Float64,readline(stdin))
    println("input desired y:")
    y = parse(Float64,readline(stdin))
    println("input desired z:")
    z = parse(Float64,readline(stdin))

    goal_point = Point3D(root_frame(mechanism),x,y,z)

    eef_l_loc = get_eef_loc(state,left_eef_body)
    eef_r_loc = get_eef_loc(state,right_eef_body)

    if norm((goal_point-eef_l_loc).v) < norm((goal_point-eef_r_loc).v)
        do_ik!(state,goal_point,left_eef_path,left_eef_body,constrained)
        constrained = true
    else
        do_ik!(state,goal_point,right_eef_path,right_eef_body,constrained)
        constrained = true
    end

    println("continue?[y/n]")
    res = readline(stdin)

    if res == "y"
        continue
    elseif res == "n"
        break
    else
        println("unrecognized response, continuing")
    end
end
