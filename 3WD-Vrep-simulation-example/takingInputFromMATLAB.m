clear();
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%%
if (clientID>-1)
    disp('Connected')
    %vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%start Vrep simulation
 
    [returnCode,rover]=vrep.simxGetObjectHandle(clientID,'rover',vrep.simx_opmode_blocking);
 
%% 
    while 1
         inputCoordinates=input('rover positions = ');%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
         %if there is only one rover, it takes [x1 y1 angle1].
         %if there are three rovers, it takes [x1 y1 angle1 x2 y2 angle2 x3 y3 angle3]
         
         if(inputCoordinates == 's')
             %this program terminate if user inputs 's'
             break;
             	
            
         end


         packedData   = vrep.simxPackFloats(inputCoordinates);%covert into floats data pack
         [returnCode] = vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot); %write the String to the handle

    end
%%
    vrep.simxFinish(-1);
end
%%
vrep.delete();

