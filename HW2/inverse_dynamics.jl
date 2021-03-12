import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using LinearAlgebra
using StaticArrays



i_c1 = [1 0 0;
       0 0.083 0
       0 0 1];

i_c2 = [1 0 0;
       0 0.083 0
       0 0 1];

i_c3 = [1 0 0;
       0 0.33 0
       0 0 1];


m1 = 1
m2 = 1
m3 = 1


l_l1 = 0.4
l_l2 = 1.1
l_l3 = 1.1


r_l1 = 0.2
r_l2 = 0.05
r_l3 = 0.05



B_damping = [1 0 0;
             0 1 0;
             0 0 1]


T_w_c1(q) = [cos(q) -sin(q) 0 0;
             sin(q) cos(q) 0 0;
             0 0 1 (l_l1/2.0);
             0 0 0 1]

T_w_l2(q1, q2) = T_w_c1(q1) *  [cos(q2) -sin(q2) 0 0;
                                0 0 1 0;
                                -sin(q2) -cos(q2) 0 (l_l1/2.0)-0.05;
                                0 0 0 1]

T_w_c2(q1, q2) = T_w_l2(q1, q2) * [1 0 0 0;
                                   0 1 0 -(l_l2/2.0);
                                   0 0 1 0
                                   0 0 0 1]

T_w_l3(q1, q2, q3) = T_w_c2(q1, q2) * [cos(q3) -sin(q3) 0 0;
                                       sin(q3) cos(q3) 0 -(l_l2/2.0)+0.05;
                                       0 0 1 0;
                                       0 0 0 1]

T_w_c3(q1, q2, q3) = T_w_l3(q1, q2, q3) * [1 0 0 0;
                                           0 1 0 -(l_l3/2.0)-0.05;
                                           0 0 1 (r_l2+r_l3);
                                           0 0 0 1]

T_w_eef(q1, q2, q3) = T_w_c3(q1, q2, q3) * [1 0 0 0;
                                            0 1 0 -(l_l3/2.0);
                                            0 0 1 0;
                                            0 0 0 1]


function rot_c1(q)
    t = T_w_c1(q)
    return [t[1:3] t[5:7] t[9:11]]
end

function rot_c2(q1, q2)
    t = T_w_c2(q1,q2)
    return [t[1:3] t[5:7] t[9:11]]
end

function rot_c3(q1, q2, q3)
    t = T_w_c3(q1,q2,q3)
    return [t[1:3] t[5:7] t[9:11]]
end



function I_c1(q)
    r = rot_c1(q)
    return r*i_c1*r'
end

function I_c2(q1, q2)
    r = rot_c2(q1, q2)
    return r*i_c2*r'
end

function I_c3(q1, q2, q3)
    r = rot_c3(q1, q2, q3)
    return r*i_c3*r'
end



function Jacobian(q1, q2, q3)
    c1 = T_w_c1(q1)
    c2 = T_w_c2(q1,q2)
    c3 = T_w_c3(q1,q2,q3)

    eef = T_w_eef(q1,q2,q3)[13:15]

    z1 = c1[9:11]
    z2 = c2[9:11]
    z3 = c3[9:11]

    O_1 = c1[13:15]
    O_2 = c2[13:15]
    O_3 = c3[13:15]

    v1 = cross(z1,(eef-O_1))
    v2 = cross(z2,(eef-O_2))
    v3 = cross(z3,(eef-O_3))

    return [z1 z2 z3;
            v1 v2 v3]

end


function J_w(J)
    return [J[1:3] J[7:9] J[13:15]]
end

function J_v(J)
    return [J[4:6] J[10:12] J[16:18]]
end


function D(q)

    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    J = Jacobian(q1,q2,q3)
    j_v = J_v(J)
    j_w = J_w(J)

    D = (m1*j_v[1:3]'*j_v[1:3] + j_w[1:3]'*I_c1(q1)*j_w[1:3]) +
        (m2*j_v[4:6]'*j_v[4:6] + j_w[4:6]'*I_c2(q1,q2)*j_w[4:6]) +
        (m3*j_v[7:9]'*j_v[7:9] + j_w[7:9]'*I_c3(q1,q2,q3)*j_w[7:9])

    return D

end




function custom_inversedynamics(qd_dot, q_dot, q)

    # placeholder for future torques
    torques = [0 0 0];






    return torques

end
