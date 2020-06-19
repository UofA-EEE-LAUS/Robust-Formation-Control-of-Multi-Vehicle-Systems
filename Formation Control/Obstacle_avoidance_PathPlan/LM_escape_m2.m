function [ output ] = LM_escape_m2( test_point,over,obstacle )
%Compute function 

k_att=10; % gain factor of attractive force
repu=0;  % initilize the repulsion 
k_rep=100; % gain factor of the repulsive force 
detection=0.4;  % the obstacle avoidance reaction range, if the distance between obstacle and robot is smaller than the range, then repulsion affcet its movement, otherwise repulsion is 0
Ke=1;

%the extra potential from the virtual obstacle 
% if norm(ori_curr-LM_curr)<=detection
%    ext_P=(-1/2)*(Ke/detection)*(norm(ori_curr-LM_curr))^2;
%    
% else
%     ext_P=(-Ke)*(norm(ori_curr-LM_curr)-detection/2);
%     
% end 

% the traction function 
attr=1/2*k_att*(norm(test_point-over))^2; % compute the traction potential 
% r=1/attr= (-1)*k_att*(norm(curr-over)); % compute the traction force 

% the repulsion function 
for i=1:size(obstacle,2)
    if norm(test_point-obstacle(:,i))<=detection
        
      %repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/detection)^2; % original method --- compute the repulsion potential      
      repu=repu+1/2*k_rep*((1/norm(test_point-obstacle(:,i))-1/detection)^2)*(norm(test_point-over))^3; % improved method --- solution to unreachable target

    else
        repu=repu+0;
    end
end

output=attr+repu;

end