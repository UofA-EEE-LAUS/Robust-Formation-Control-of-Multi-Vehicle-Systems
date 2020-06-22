The university of Adelaide 
Kamalpreet Singh
2019
The use of the function will be with following files in same folder:
1. remApi.m
2. remoteApiProto.m
3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)
4. simpleTest.m (or any other example program)
5. motion_part.m
6. motion_part.ttt
-------------------------------------------------------------------

STEPS:

1. the file motion_part.m(matlab) and motion_part.ttt(VREP) should be run simultaneously.
2. the vrep simulation scene should be running.
3. run the matlab file.
4. input the desired point in the form of
       [x1 y1 angle1 x2 y2 angle2 x3 y3 angle3 x4 y4 angle4 x5 y5 angle5]
the x and y will be in meters and angle is in degrees.
5. any input can be given even which the running simulation.
6. for a linear movement fix the angle.
7. for pure roational movement fix the x and y points.
8. the detailed information of every step can be read from the model scripts.
9. desired points are transffered from matlab to Vrep cuboidal model and then sent to the respective rover.
10. tunning of Kp and Kd is done to smoothen and fasten the response.
