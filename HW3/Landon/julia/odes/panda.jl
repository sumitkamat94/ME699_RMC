motion_time = 10
sim_time = 20

q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
qd = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]


function traj(t_in)
    t = t_in/motion_time

    a0 = q0
    a1 = 0.0
    a2 = (3.0/2.0)*(qd.-q0)
    a3 = (1/2)*(q0.-qd)

    q_t = a0.+a1*t.+a2*(t^2).+a3*(t^3)
    qdot_t = a1.+2*a2*t.+3*a3*(t^2)
    qddot_t = 2*a2.+6*a3*t

    return q_t, qdot_t, qddot_t

end



function control_PD!(τ, t, state)
    if t >= motion_time
        t = motion_time
    end

    Kp = diagm([10000, 20000, 10000, 20000, 10000, 10000, 10000, 10000, 10000])
    Kd = diagm([10, 10, 10, 10, 10, 10, 10, 10, 10])

    q_t, qdot_t, qddot_t = traj(t)

    # Do some PD
    τ .=  -Kp*(configuration(state) - q_t) - Kd*(velocity(state) - qdot_t)
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end


function control_CTC!(τ, t, state)
    if t >= motion_time
        t = motion_time
    end

    Kp = diagm([5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000])
    Kd = diagm([5, 5, 5, 5, 5, 5, 5, 5, 5])

    q_t, qdot_t, qddot_t = traj(t)

    e_t = -Kp*(configuration(state) - q_t)
    edot_t = -Kd*(velocity(state) - qdot_t)

    # Do some CTC
    τ .= mass_matrix(state)*(e_t+edot_t+qddot_t) + dynamics_bias(state)
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end



function plot_sol(p,sol,colorarg,saveflag,savename)
    qsol = vcat(sol[:]'...)
    for i=1:9
        push!(p,layer(x=sol.t,y=qsol[:,i],Geom.line,color=colorarg))
    end
    p
    if saveflag
        p |> PDF(savename)
    end
end


function do_control(control_func)
    # Refresh visualization
    delete!(vis)
    # Load mechanism info
    urdfPath = PandaRobot.urdfpath()
    mvis, mechanism = display_urdf(urdfPath,vis)
    # Create state and set initial config and velocity
    state = MechanismState(mechanism)
    set_configuration!(state,q0)
    zero_velocity!(state)
    # Update mechanism visual
    set_configuration!(mvis, configuration(state))

    # Define ODE Problem, which defines closed loop using  control!
    problem = ODEProblem(Dynamics(mechanism,control_func), state, (0., sim_time));
    # Solve ODE problem using Tsit5 scheme, and given numerical tolerances
    sol = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
    # Animate solution
    setanimation!(mvis, sol; realtime_rate = 1.0);
    # Plot joint angles vs time using Gadfly
    p = plot()
    plot_sol(p,sol,[colorant"red"],false,nothing)

    qa = zeros(9)

    for i in 1:9
        qa[i] = sol[end][i]
        println("final joint angle $i : $(sol[end][i])")
    end

    println(norm(qa-qd))
    # Show plot in browser
    # p
end


do_control(control_PD!)
sleep(sim_time)
do_control(control_CTC!)
