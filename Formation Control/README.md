# Instruction
## MATLAB Scripts
roverFormation3.m is for the simulation of 3 rovers. \
roverFormation5.m is for the simulation of 5 rovers. \
roverFormation5LUA.m is for the simulation of 5 rovers movement function in LUA. \
fc_apf.m is for the simulation of introducting APF obstacle avoidance based on roverFormation5LUA.m. 

## V-REP Scenes
3-rover.ttt is for roverFormation3.m. \
5-rover.ttt is for roverFormation5.m. \
5-rover-w:KP.ttt is for roverFormation5LUA.m. \
fc_apf.ttt is for fc_apf.m. 

# Logs
## 03/04/2020
Update the MATLAB scirpt fc_apf.m and Path.m to fix the matrix exceeding issues. \
Rename folders.

## 02/04/2020
Update obstacle avoidance function. \
New function is included in the fc_apf.m and corresponding V-REP scene. \
Bug: Matrix exceeds when nearly reaching the destination

## 30/03/2020
Create repository. \
Update formation control algorithm and corresponding V-REP scene.


