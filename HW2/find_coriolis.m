% trot_ and transl functions from robotics toolbox by Peter Corke

clear all

syms q1;
syms q2;
syms q3;

syms q1_dot;
syms q2_dot;
syms q3_dot;

q = [q1 q2 q3].';
q_dot = [q1_dot q2_dot q3_dot].';


T_0_c1 = trotz(q1)*transl(0,0,0.2);

T_0_l1 = T_0_c1*transl(0,0,0.15)*trotx(-pi/2)*trotz(q2);

T_0_c2 = T_0_l1*transl(0,-0.55,0);

T_0_l2 = T_0_c2*transl(0,-0.5,0)*trotz(q3);

T_0_c3 = T_0_l2*transl(0,-0.55,0.1);



J_w1 = [T_0_c1(1:3,3) [0 0 0]' [0 0 0]' ];

J_w2 = [T_0_c1(1:3,3) T_0_c2(1:3,3) [0 0 0]'];

J_w3 = [T_0_c1(1:3,3) T_0_c2(1:3,3) T_0_c3(1:3,3)];


J_v1 = zeros(3,3);

J_v2 = [cross([0 0 1]',T_0_c2(1:3,4)-[0 0 0]') cross(T_0_l1(1:3,3),T_0_c2(1:3,4)-T_0_l1(1:3,4)) [0 0 0]'];

J_v3 = [cross([0 0 1]',T_0_c3(1:3,4)-[0 0 0]') cross(T_0_l1(1:3,3),T_0_c3(1:3,4)-T_0_l1(1:3,4)) cross(T_0_l2(1:3,3),T_0_c3(1:3,4)-T_0_l2(1:3,4))];



m1 = 1;
m2 = 1;
m3 = 1;


I_c1 = T_0_c1(1:3,1:3)*[1 0 0; 0 0.083 0; 0 0 1]*T_0_c1(1:3,1:3).';

I_c2 = T_0_c2(1:3,1:3)*[1 0 0; 0 0.083 0; 0 0 1]*T_0_c2(1:3,1:3).';

I_c3 = T_0_c3(1:3,1:3)*[1 0 0; 0 0.33 0; 0 0 1]*T_0_c3(1:3,1:3).';



D = (m1*J_v1.'*J_v1+J_w1.'*I_c1*J_w1) + (m2*J_v2.'*J_v2+J_w2.'*I_c2*J_w2) + (m3*J_v3.'*J_v3+J_w3.'*I_c3*J_w3);


C = cell(3,3);





for k = 1:3
    for j = 1:3
        sum = 0;
        for i = 1:3
            
            p_dkj = diff(D(k,j),q(i));
            p_dki = diff(D(k,i),q(j));
            p_dij = diff(D(i,j),q(k));
            
            sum = sum + ((0.5*(p_dkj + p_dki - p_dij))*q_dot(i));
        end
        C{k,j} = simplify(sum);
    end
end



