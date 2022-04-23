function [q]=inverse_kin(xi,elbows, params) 
% x - position vector of the tool (3x1)
% elbows – decision values vector for the different solutions, (matrix of 1x2, first value for joint 2 and second for d4)
% 1 for elbow up, -1 for elbow down.
% T - matrix of 4x4, transformation matrix.
%   joint values q: [theta1, theta2,  d4]

H = params(1);
l2 = params(2);
l3 = params(3);

q = zeros(3,1);

q(1)=atan2(xi(2),xi(1));
c1=cos(q(1));
s1=sin(q(1));

if elbows(2)==1
    q(3)=sqrt(xi(1)^2+xi(2)^2+(xi(3)-H)^2-l2^2)-l3;

 if elbows(1)==1
     % positive d3 and elbow up for theta 2
    c2=((xi(1)/c1)*l2-(q(3)+l3)*(H-xi(3)))/(l2^2+(q(3)+l3)^2);
    s2=(xi(3)-H-q(3)*c2-l3*c2)/l2;
    q(2)=atan2(s2,c2);
 else
      % positive d3 and elbow down for theta 2
    c2=((xi(1)/c1)*l2-(q(3)+l3)*(H-xi(3)))/(l2^2+(q(3)+l3)^2);
    s2=(xi(3)-H-q(3)*c2-l3*c2)/l2;;
    q(2)=atan2(s2,c2);
 end
end

if elbows(2)==-1
   if elbows(1)==1
     % negative d3 and elbow up for theta 2
    c2=((xi(1)/c1)*l2-(q(3)+l3)*(H-xi(3)))/(l2^2+(q(3)+l3)^2);
    s2=(xi(3)-H-q(3)*c2-l3*c2)/l2;;
    q(2)=atan2(s2,c2);
   else
      % negative d3 and elbow down for theta 2
    c2=((xi(1)/c1)*l2-(q(3)+l3)*(H-xi(3)))/(l2^2+(q(3)+l3)^2);
    s2=(xi(3)-H-q(3)*c2-l3*c2)/l2;;
    q(2)=atan2(s2,c2);
   end
end

end
