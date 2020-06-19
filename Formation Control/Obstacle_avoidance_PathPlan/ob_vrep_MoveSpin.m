function [] = ob_vrep_MoveSpin (x,y)



%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);


%get handles 
[returnCode,rover]=vrep.simxGetObjectHandle(clientID,'rover',vrep.simx_opmode_blocking);
[returnCode,laser_sensor]=vrep.simxGetObjectHandle(clientID,'laser_sensor',vrep.simx_opmode_oneshot_wait);

%get positions 
[returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
[returnCode] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_streaming);
[returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);

%positions setting
position_x=position(:,1); %rover's position 
position_y=position(:,2);

if (orientations(1) >= 0)
    theta_sample = orientations(2);
else
    if(orientations(1) < 0)      %ÄÇ¸ö½Ç¶È£¿
    	theta_sample = (pi/2-orientations(2))+pi/2;
    elseif(orientations(2) < 0)
    	theta_sample = (pi/2-orientations(2))-pi/2;
    end
end

orien_0 = theta_sample;

rover_radius = 15;
wheel_radius = 5.22;
dphi = 0/ 180 * pi; % 0£¿
% dphi=0; %stop spinning
phi = orien_0;
% dphi = phi;
dist_x = zeros(500); %£¿
dist_y = zeros(500);
dist_x(1:500) = x-position_x;
dist_y(1:500) = y-position_y;
i = 1;
elapsedTime = 1;
threshold = 0.1;

%control
while abs(position_x-x) >= threshold || abs(position_y-y) >= threshold
    
    tic
    %get object position for derivative
    [returnCode, position_d]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
    [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
    position_dx=position_d(:,1); 
    position_dy=position_d(:,2);
    
    const_speed = 3; % the speed of the rover
    Kp = 0.75; %£¿
    Kd = 2.25;
    
    dx = (x - position_x); % distance between destination and the current position?
    dy = (y - position_y);

    if (orientations(1) >= 0)
        theta_sample = orientations(2);
    else
        if(orientations(1) < 0)
            theta_sample = (pi/2-orientations(2))+pi/2;
        elseif(orientations(2) < 0)
            theta_sample = (pi/2-orientations(2))-pi/2;
        end
    end
    
    phi = theta_sample + (pi + 1.5);
    disp(phi);
%     dphi = 2 / 180 * pi; % control the spin
    dphi = 0;%stop spinning 
    theta = abs(atan(dy/dx));
    
    v_xs = (position_dx - position_x) / elapsedTime;
    v_ys = (position_dy - position_y) / elapsedTime;
    
    v_x = Kp * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) + Kd * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) / elapsedTime;
    v_y = Kp * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) + Kd * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) / elapsedTime;
    w = dphi;
    
    v   = ( v_x * cos(phi) + v_y * sin(phi) ) / 7.5;
    v_n = (-v_x * sin(phi) + v_y * cos(phi) ) / 7.5;
    
    v0 = -v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
    v1 =                - v_n             + w * rover_radius;
    v2 =  v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
    
    %defining motor handles
    %return code functions as a debug tool/error message
    [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,'motor_0',vrep.simx_opmode_blocking);
    [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,'motor_1',vrep.simx_opmode_blocking);
    [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,'motor_2',vrep.simx_opmode_blocking);
    
    %setting motor speeds for straight line
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,v0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,v1,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,v2,vrep.simx_opmode_blocking);
    
    %get object position
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
    position_x=position(:,1);
    position_y=position(:,2);
    
    %record position
    dist_x(i) = dist_x(i) - (x - position_x);
    dist_y(i) = dist_y(i) - (y - position_y);
    i = i + 1;
    
    elapsedTime = toc;

end


%without control
% while abs(position_x-x) >= 0.01 || abs(position_y-y) >= 0.01 && i <= 500
% 
%     %get object position for derivative
%     [returnCode,position_d]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
%     [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
%     position_dx=position_d(:,1);
%     position_dy=position_d(:,2);
%     
%     const_speed = 20;
% 
%     
%     dx = (x - position_x);
%     dy = (y - position_y); 
%     dphi = 1;
%     phi = orientations(3)-orien_0;
%     theta = abs(atan(dy/dx));
% 
%     
%     v_x = const_speed * (dx/abs(dx)) * abs(cos(theta));
%     v_y = const_speed * (dy/abs(dy)) * abs(sin(theta));
% %     v_x = 0;
% %     v_y = 0;
%     w = dphi;
%     
%     v   = ( v_x * cos(phi) + v_y * sin(phi) ) / 7.5;
%     v_n = (-v_x * sin(phi) + v_y * cos(phi) ) / 7.5;
%     
%     v0 = -v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
%     v1 =                - v_n             + w * rover_radius;
%     v2 =  v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
%     
%     %defining motor handles
%     %return code functions as a debug tool/error message
%     [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,'motor_0',vrep.simx_opmode_blocking);
%     [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,'motor_1',vrep.simx_opmode_blocking);
%     [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,'motor_2',vrep.simx_opmode_blocking);
%     
%     %setting motor speeds for straight line
%     [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,v0,vrep.simx_opmode_blocking);
%     [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,v1,vrep.simx_opmode_blocking);
%     [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,v2,vrep.simx_opmode_blocking);
%     
%     %get object position
%     [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
%     position_x=position(:,1);
%     position_y=position(:,2);
%     
%     dist_x(i) = dist_x(i) - (x - position_x);
%     dist_y(i) = dist_y(i) - (y - position_y);
%     i = i + 1;
%     
% %     phi = phi + 0.1;
% 
% end
%end without control

%%plot distance
% figure(1)
% plot(dist_x)
% figure(2)
% plot(dist_y)

% vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); %simx_opmode_oneshot
% vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot);
% vrep.simxFinish(-1);
end
