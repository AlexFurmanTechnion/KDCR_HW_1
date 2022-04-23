function [J_dot, JL_dot]=jacobian_mat_dot(qi,q_doti,params) 
% J_dot – the jacobian's time derivative. 
% q_dot – the joint parameters' time derivative. 
% joint values q: [theta1, theta2,  d4]
% params:[H,l2,l3]
l2 = params(2);
l3 = params(3);

c1 = cos(qi(1));
s1 = sin(qi(1));
c2 = cos(qi(2));
s2 = sin(qi(2));
d4 = qi(3);
J_dot = zeros(6,3);
dJdq = zeros(6,3,3);

dJdq(:,:,1)=[c1*s1*(d4+l3)-c1*c2*l2, s1*(s2*l2+c2*(d4+l3)), s1*s2;
            -s1*c2*l2+s2*s2*(d4+l3), -c1*(s2*l2+c2*(d4+l3)), -c1*s2;
            0                     , -s1*(c1*c2*l2-c1*s2*(d4+l3))+c1*(-s1*c2*l2+s2*s2*(d4+l3))+c1*(c2*s1*l2-s1*s2*(d4+l3))+s1*(c1*c2*l2-c1*s2*(d4+l3))  ,0;
            0                     , c1                        ,0;
            0                     , s1                        ,0;
            0                     ,  0                        ,0
    ];
dJdq(:,:,2)=[ s1*c2*(d4+l3)+s2*s1*l2, -c1*(c2*l2-s2*(d4+l3)), -c1*c2;
             -c1*s2*l2+c1*c2*(d4+l3), -s1*(c2*l2-s2*(d4+l3)), -s1*c2;
             0                      ,c1*(-c1*s2*l2+c1*c2*(d4+l3))-s1*(s1*c2*(d4+l3)+s2*s1*l2), -s2;
             0                      ,0                      ,0;
             0                      ,0                      ,0;
             0                      ,0                      ,0
    ];
dJdq(:,:,3)=[s1*s2, -c1*c2, 0;
             -c1*s2,-s1*c2, 0;
             0,-c1*c1*s2-s1*s1*s2,0;
             0,0,0;
             0,0,0;
             0,0,0
    ];

for i =1:6
    for j=1:3
        for k = 1:3
            J_dot(i,j) = J_dot(i,j) + dJdq(i,j,k)*q_doti(k);
        end
    end
end

JL_dot = J_dot(1:3,:);

end
