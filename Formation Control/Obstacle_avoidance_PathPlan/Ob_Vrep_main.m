%MATLAB script for controlling the 3 omni-wheel rover platform

%------------------------------INSTRUCTIONS--%-----------------------------%
%{
1.  Ensure the files inside of
    C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab
    C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\lib\lib\Windows\64Bit
    are copied into the current MATLAB workspace

2.  In v-rep, ensure that the line:
    simRemoteApi.start(19999)
    is inside a non-threaded child script, under sysCall_init()
    (this runs once and starts the internal server for MATLAB to connect to)

3.  For more information on API functions, please see
    http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
%}

%---------------------------------SCRIPT----------------------------------%
tic
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    
    %------------------------------CODE HERE------------------------------%
    
         %------------simulation of obstacle avoidance----------------% 
    
    %get handles
    [~,ob1]=vrep.simxGetObjectHandle(clientID,'Ob1',vrep.simx_opmode_blocking);
    [~,ob2]=vrep.simxGetObjectHandle(clientID,'Ob2',vrep.simx_opmode_blocking);
    [~,ob3]=vrep.simxGetObjectHandle(clientID,'Ob3',vrep.simx_opmode_blocking);
    [~,ob4]=vrep.simxGetObjectHandle(clientID,'Ob4',vrep.simx_opmode_blocking);
    [~,ob5]=vrep.simxGetObjectHandle(clientID,'Ob5',vrep.simx_opmode_blocking);
    [~,ini_rover]=vrep.simxGetObjectHandle(clientID,'rover',vrep.simx_opmode_blocking);
    [returnCode,tar]=vrep.simxGetObjectHandle(clientID,'Plane',vrep.simx_opmode_blocking);
    
    %get positions 
    [returnCode,ob1_pos]=vrep.simxGetObjectPosition(clientID,ob1,-1,vrep.simx_opmode_blocking);
    [returnCode,ob2_pos]=vrep.simxGetObjectPosition(clientID,ob2,-1,vrep.simx_opmode_blocking);
    [returnCode,ob3_pos]=vrep.simxGetObjectPosition(clientID,ob3,-1,vrep.simx_opmode_blocking);
    [returnCode,ob4_pos]=vrep.simxGetObjectPosition(clientID,ob4,-1,vrep.simx_opmode_blocking);
    [returnCode,ob5_pos]=vrep.simxGetObjectPosition(clientID,ob5,-1,vrep.simx_opmode_blocking);
    [returnCode,ini_rover_pos]=vrep.simxGetObjectPosition(clientID,ini_rover,-1,vrep.simx_opmode_blocking);
    [returnCode,tar_pos]=vrep.simxGetObjectPosition(clientID,tar,-1,vrep.simx_opmode_blocking);
    
    %positions setting 
    ob1_x=ob1_pos(:,1);
    ob1_y=ob1_pos(:,2);
    ob2_x=ob2_pos(:,1);
    ob2_y=ob2_pos(:,2);
    ob3_x=ob3_pos(:,1);
    ob3_y=ob3_pos(:,2);
    ob4_x=ob4_pos(:,1);
    ob4_y=ob4_pos(:,2);
    ob5_x=ob5_pos(:,1);
    ob5_y=ob5_pos(:,2);
    ini_rover_pos_x=ini_rover_pos(:,1);
    ini_rover_pos_y=ini_rover_pos(:,2);
    tar_pos_x=tar_pos(:,1);
    tar_pos_y=tar_pos(:,2);
    
    %all obstacles 
    obstacles=[ ob1_x ob2_x ob3_x  ob4_x ob5_x;
                ob1_y ob2_y ob3_y  ob4_y ob5_y];
            
    %iniial position
    begin=[ini_rover_pos_x;
           ini_rover_pos_y];
    
    %destination 
    over=[tar_pos_x;
          tar_pos_y];
   
    
        %---------plotting the simulation results in Matlab--------%
    
    figure(1);
    axis([-10 10 -10 10]);
    hold on;
    plot(begin(1),begin(2),'+k','MarkerSize',10);
    plot(over(1),over(2),'*b','MarkerSize',10);
    plot(obstacles(1,:),obstacles(2,:),'.r','Markersize',30);

    xlabel('X');
    ylabel('Y');
    title('Simulations of APF on single robot');
    text(10,10,'+ the initial position','Color','k','FontSize',10);
    text(10,9,'* the goal','Color','b','FontSize',10);
    text(10,8,'O obstacles','Color','r','FontSize',10);
    
    %the path stored in here 
    point= ob_Path(begin,over,obstacles);
    
    %move the rover based on the points in Path
    num=size(point,2);%get the number of points in the Path
    
    for i=1:num
        tem=point(:,i);
        x=tem(1);
        y=tem(2);
        ob_vrep_MoveSpin(x,y);
        
    end
    
%     MoveSpin(1,3);

    %set the motor speed as 0 so that it can stop moving 
    %handles 
    [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,'motor_0',vrep.simx_opmode_blocking);
    [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,'motor_1',vrep.simx_opmode_blocking);
    [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,'motor_2',vrep.simx_opmode_blocking);
    
    %setting motor speed as 0
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,0,vrep.simx_opmode_blocking);
    
    %destroy connection to v-rep simulation
    vrep.simxFinish(-1);
else
    disp('Failed connecting to remote API server');
end

vrep.delete()

total_time=toc