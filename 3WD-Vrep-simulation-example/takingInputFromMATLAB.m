



clear();
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);


if (clientID>-1)
    disp('Connected')
    %vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%start Vrep simulation
 
    [returnCode,rover]=vrep.simxGetObjectHandle(clientID,'rover',vrep.simx_opmode_blocking);
 
 
    while 1
         inputCoordinates=input('rover positions = ');%Taking inputs 
         
         if(inputCoordinates == 's')
             
             break;
             	
            
         end


         packedData   = vrep.simxPackFloats(inputCoordinates);%covert into floats data pack
         [returnCode] = vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot); %write the String to the handle


    end



    vrep.simxFinish(-1);
end


vrep.delete();

