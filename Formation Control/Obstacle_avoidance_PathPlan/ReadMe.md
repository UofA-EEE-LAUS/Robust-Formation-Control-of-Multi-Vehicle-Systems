#Introduction 

The ob_mainAPF is the main function for running the simulation on MATLAB.
If you try to run simulation on V-REP with connecting to MATLAB, make sure you have files (including remApi.m, remoteApi.dll, remoteApiProto.m and simpleTest.m) before you do the simulation. 
This is an improved APF, which can deal with the unreachble target issue and local minima issue. And the idea to solve the local minima issue is from an article [1], which introduced a "virtual obstacle" method. 




