function [ output ] = LM_escape( curr,over,obstacle )
%Compute function 

k_att=10; % gain factor of attractive force
repu_LM=0;  % initilize the repulsion 
k_rep=100; % gain factor of the repulsive force 
Q_star=1;  % the obstacle avidance reaction range, if the distance between obstacle and robot is smaller than the range, then repulsion affcet its movement, otherwise repulsion is 0

% the new traction function to escape the local minima

rot_angle=2*pi/3; % the rotation angle is 120 degrees (depends on the reality)
rot_matrix= [cos(rot_angle) -sin(rot_angle);
            sin(rot_angle)  cos(rot_angle)];

A= curr-over;
B= transpose(A);
C= B*rot_matrix;
attr_LM=(1/2*k_att*(norm(C))^2);

% attr_LM=(1/2*k_att*(norm(curr-over))^2);

% Pa=0.06; % the distance to identify if the robot is in the desirable location
% s=0.1;
% dis_LM=norm(curr-over); % calculate ????
% if dis_LM > Pa
%   Uadd=s*(dis_LM^2);
%   else
%       Uadd=0;
% end

% the repulsion function 
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star
      %repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2; % original method --- compute the repulsion function     
      repu_LM=repu_LM+1/2*k_rep*((1/norm(curr-obstacle(:,i))-1/Q_star)^2)*(norm(curr-over))^3; % improved method --- solution to unreachable target

    else
        repu_LM=repu_LM+0;
    end
end

output=attr_LM+repu_LM;
%output=attr_LM+repu_LM+Uadd;

end