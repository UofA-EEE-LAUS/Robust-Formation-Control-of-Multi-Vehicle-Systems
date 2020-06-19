# Introduction 

The ob_mainAPF is the main function for running the simulation on MATLAB. \
If you try to run simulation on V-REP with connecting to MATLAB, make sure you have files (including remApi.m, remoteApi.dll, remoteApiProto.m and simpleTest.m) before you do the simulation. \
This is an improved APF, which can deal with the unreachble target issue and local minima issue. And the idea to solve the local minima issue is from an article [1], which introduced a "virtual obstacle" method. 

# Reference 
[1] P. Min & L. M.  Cheol. (2003). " A new technique to escape local minimum in artificial potential field based path planning ". Journal of Mechanical Science and Technology. 17. 1876-1885. 10.1007/BF02982426.  


