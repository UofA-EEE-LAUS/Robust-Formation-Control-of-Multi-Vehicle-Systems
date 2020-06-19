%MATLAB script for controlling the 3 omni-wheel rover platform

%---------------------------------SCRIPT----------------------------------%
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    elapsedTime = 1;
    
    %------------------------------CODE HERE------------------------------%
    
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    
    K1 = kron(eye(2),[-3, -1.5]);
    K2 = kron(eye(2),[0.43, 1]);
    B1 = kron(eye(2),[1,0]');
    B2 = kron(eye(2),[0,1]');
    
    A = [0,1,0,1;
        1,0,1,0;
        0,1,0,1;
        1,0,1,0];
    
    L = indegree(A) + A;
    R1 = kron(eye(4), (B2*K1+B1*B2'));
    R2 = kron(L,B2*K2);
    R3 = kron(eye(4), B2*K1);
    R4 = kron(L,B2*K2);
    R5 = kron(eye(4),B2);
    dt = 0.05;
    
    %Get 5 rovers' position
    [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover_0'),vrep.simx_opmode_blocking);
    [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_1]=vrep.simxGetObjectHandle(clientID,strcat('rover_1'),vrep.simx_opmode_blocking);
    [returnCode,position_1]=vrep.simxGetObjectPosition(clientID,rover_1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_2]=vrep.simxGetObjectHandle(clientID,strcat('rover_2'),vrep.simx_opmode_blocking);
    [returnCode,position_2]=vrep.simxGetObjectPosition(clientID,rover_2,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_3]=vrep.simxGetObjectHandle(clientID,strcat('rover_3'),vrep.simx_opmode_blocking);
    [returnCode,position_3]=vrep.simxGetObjectPosition(clientID,rover_3,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_4]=vrep.simxGetObjectHandle(clientID,strcat('rover_4'),vrep.simx_opmode_blocking);
    [returnCode,position_4]=vrep.simxGetObjectPosition(clientID,rover_4,-1,vrep.simx_opmode_blocking);
    
    position = [position_0; position_1; position_2; position_3; position_4];
    
    h = [0,0,2,0, ...
        0,0,0,0, ...
        2,0,0,0, ...
        2,0,2,0]';
    
    Theta = [0,0,0,0, ...
        0,0,0,0, ...
        0,0,0,0, ...
        0,0,0,0]';
    
    Theta(1:2:16) = [position_0(1:2); position_1(1:2); position_2(1:2); position_3(1:2)]';
    
    Theta_N = zeros(16,1);
    
    
    for t = 0:dt:5
        %Get 5 rovers' position
%         [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover_0'),vrep.simx_opmode_blocking);
%         [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_1]=vrep.simxGetObjectHandle(clientID,strcat('rover_1'),vrep.simx_opmode_blocking);
%         [returnCode,position_1]=vrep.simxGetObjectPosition(clientID,rover_1,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_2]=vrep.simxGetObjectHandle(clientID,strcat('rover_2'),vrep.simx_opmode_blocking);
%         [returnCode,position_2]=vrep.simxGetObjectPosition(clientID,rover_2,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_3]=vrep.simxGetObjectHandle(clientID,strcat('rover_3'),vrep.simx_opmode_blocking);
%         [returnCode,position_3]=vrep.simxGetObjectPosition(clientID,rover_3,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_4]=vrep.simxGetObjectHandle(clientID,strcat('rover_4'),vrep.simx_opmode_blocking);
%         [returnCode,position_4]=vrep.simxGetObjectPosition(clientID,rover_4,-1,vrep.simx_opmode_blocking);
        
%         position = [position_0; position_1; position_2; position_3; position_4];
%         Theta(1:2:20) = [position_0(1:2); position_1(1:2); position_2(1:2); position_3(1:2); position_4(1:2)]';
        
%         h1=[1*cos(t)-2;-1*sin(t);1*sin(t)+2;1*cos(t)];
%         h2=[1*cos(t)-2;-1*sin(t);1*sin(t)-2;1*cos(t)];
%         h3=[1*cos(t);-1*sin(t);1*sin(t);1*cos(t)];
%         h3=[1*cos(t)+2;-1*sin(t);1*sin(t)-2;1*cos(t)];
%         h4=[1*cos(t)+2;-1*sin(t);1*sin(t)+2;1*cos(t)];
%         h=[h1;h2;h3;h4];

%         h1 = [t-2;1;4;0];
%         h2 = [t-2;1;3;0];
%         h3 = [t-2;1;2;0];
%         h4 = [t-2;1;1;0];
%         h=[h1;h2;h3;h4];
        
        Theta_N = (R1-R2) * Theta - (R3-R4) * h;
        Theta = Theta + Theta_N * dt;
        X = Theta(1:2:16);
        
        %Running rovers
        a=[X(1),X(2),0, ...
            X(3),X(4),0, ...
            X(5),X(6),0, ...
            X(7),X(8),0, ...
            5,0,0];
        packedData=vrep.simxPackFloats(a);
        [returnCode]=vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot);
        
        pause(5*dt)
        
    end
    
    A = [0,1,0,0,1;
        1,0,1,0,0;
        0,1,0,1,0;
        0,0,1,0,1;
        1,0,0,1,0];
    
    L = indegree(A) + A;
    R1 = kron(eye(5), (B2*K1+B1*B2'));
    R2 = kron(L,B2*K2);
    R3 = kron(eye(5), B2*K1);
    R4 = kron(L,B2*K2);
    R5 = kron(eye(5),B2);
    
    [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover_0'),vrep.simx_opmode_blocking);
    [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_1]=vrep.simxGetObjectHandle(clientID,strcat('rover_1'),vrep.simx_opmode_blocking);
    [returnCode,position_1]=vrep.simxGetObjectPosition(clientID,rover_1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_2]=vrep.simxGetObjectHandle(clientID,strcat('rover_2'),vrep.simx_opmode_blocking);
    [returnCode,position_2]=vrep.simxGetObjectPosition(clientID,rover_2,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_3]=vrep.simxGetObjectHandle(clientID,strcat('rover_3'),vrep.simx_opmode_blocking);
    [returnCode,position_3]=vrep.simxGetObjectPosition(clientID,rover_3,-1,vrep.simx_opmode_blocking);
    [returnCode,rover_4]=vrep.simxGetObjectHandle(clientID,strcat('rover_4'),vrep.simx_opmode_blocking);
    [returnCode,position_4]=vrep.simxGetObjectPosition(clientID,rover_4,-1,vrep.simx_opmode_blocking);
    
    position = [position_0; position_1; position_2; position_3; position_4];
    
    Theta = zeros(20,1);
    Theta(1:2:20) = [position_0(1:2); position_1(1:2); position_2(1:2); position_3(1:2);position_4(1:2)]';
    
    Theta_N = zeros(20,1);
    
    h = [0,0,2,0, ...
        0,0,0,0, ...
        1,0,1,0, ...
        2,0,2,0, ...
        2,0,0,0]';
    
    for t = 5:dt:10
        %Get 5 rovers' position
%         [returnCode,rover_0]=vrep.simxGetObjectHandle(clientID,strcat('rover_0'),vrep.simx_opmode_blocking);
%         [returnCode,position_0]=vrep.simxGetObjectPosition(clientID,rover_0,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_1]=vrep.simxGetObjectHandle(clientID,strcat('rover_1'),vrep.simx_opmode_blocking);
%         [returnCode,position_1]=vrep.simxGetObjectPosition(clientID,rover_1,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_2]=vrep.simxGetObjectHandle(clientID,strcat('rover_2'),vrep.simx_opmode_blocking);
%         [returnCode,position_2]=vrep.simxGetObjectPosition(clientID,rover_2,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_3]=vrep.simxGetObjectHandle(clientID,strcat('rover_3'),vrep.simx_opmode_blocking);
%         [returnCode,position_3]=vrep.simxGetObjectPosition(clientID,rover_3,-1,vrep.simx_opmode_blocking);
%         [returnCode,rover_4]=vrep.simxGetObjectHandle(clientID,strcat('rover_4'),vrep.simx_opmode_blocking);
%         [returnCode,position_4]=vrep.simxGetObjectPosition(clientID,rover_4,-1,vrep.simx_opmode_blocking);
        
%         position = [position_0; position_1; position_2; position_3; position_4];
%         Theta(1:2:20) = [position_0(1:2); position_1(1:2); position_2(1:2); position_3(1:2); position_4(1:2)]';
        
%         h1=[1*cos(t)-2;-1*sin(t);1*sin(t)+2;1*cos(t)];
%         h2=[1*cos(t)-2;-1*sin(t);1*sin(t)-2;1*cos(t)];
%         h4=[1*cos(t);-1*sin(t);1*sin(t);1*cos(t)];
%         h3=[1*cos(t)+2;-1*sin(t);1*sin(t)-2;1*cos(t)];
%         h5=[1*cos(t)+2;-1*sin(t);1*sin(t)+2;1*cos(t)];
%         h=[h1;h2;h3;h4;h5];

%         h1 = [t-2;1;4;0];
%         h2 = [t-2;1;3;0];
%         h3 = [t-2;1;2;0];
%         h4 = [t-2;1;1;0];
%         h5 = [t-2;1;0;0];
%         h=[h1;h2;h3;h4;h5];
        
        Theta_N = (R1-R2) * Theta - (R3-R4) * h;
        Theta = Theta + Theta_N * dt;
        X = Theta(1:2:20);
        
        %Running rovers
        a=[X(1),X(2),0, ...
            X(3),X(4),0, ...
            X(5),X(6),0, ...
            X(7),X(8),0, ...
            X(9),X(10),0];
        packedData=vrep.simxPackFloats(a);
        [returnCode]=vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot);
        
        pause(3*dt)
        
    end
    
else
    disp('Failed connecting to remote API server');
end

vrep.delete()