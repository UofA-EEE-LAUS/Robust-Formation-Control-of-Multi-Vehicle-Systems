function [ output ] = ob_Algrith( curr,over,obstacle )
%Compute function 

k_att=10; % gain factor of attractive force
repu=0;  % initilize the repulsion 
k_rep=100; % gain factor of the repulsive force 
detection=0.4;  % the obstacle avoidance reaction range, if the distance between obstacle and robot is smaller than the range, then repulsion affcet its movement, otherwise repulsion is 0

% the traction function 
attr=1/2*k_att*(norm(curr-over))^2; % compute the traction potential 
% r=1/attr= (-1)*k_att*(norm(curr-over)); % compute the traction force 

% the repulsion function 
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=detection
        
      %repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/detection)^2; % original method --- compute the repulsion potential      
      repu=repu+1/2*k_rep*((1/norm(curr-obstacle(:,i))-1/detection)^2)*(norm(curr-over))^6; % improved method --- solution to unreachable target

    else
        repu=repu+0;
    end
end

output=attr+repu;

end