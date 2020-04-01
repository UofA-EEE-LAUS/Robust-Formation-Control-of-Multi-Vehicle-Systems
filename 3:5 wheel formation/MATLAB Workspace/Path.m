function [ point ] = Path(begin,over,obstacle)
%PATH_PLAN 

iters=1; % iterations
curr=begin;
testR=0.4;   % radius of the circle, in which the current point is the centre. The threshold of MoveSpin Function is 0.1, the step length here should be bigger than the threshold
sp_len=testR/2; % the step length

while (norm(curr-over)>0.2) &&  (iters<=500) % the error is 0.2
    

% Original method 

% at first, find out the coordinates of these 8 points
    for i=1:8  % the potential direction of robot is 360 degrees (2pi), then dividing the potential direction into 8 points (45 degrees between each point)
        testPoint(:,i)=[testR*sin((i-1)*pi/4)+curr(1);testR*cos((i-1)*pi/4)+curr(2)]; % the coordinate of these 8 points
        testOut(:,i)=Algrith(testPoint(:,i),over,obstacle); % the output is the resultant
        
    end
    [temp num]=min(testOut); % find out the smallest one, since in the APF the robot always moves to the point with the smallest potential energy  
    
    %the step length of iteration is sp_len
    curr=(curr+testPoint(:,num))/2; % step length half of the circle radius is 0.1, thus the coordinate has to reduce to the half of the result
    

    %-------- strategy to escape local minima -------------%
        
       LM_test(:,iters)=curr; 
    
    if iters > 4  %identify if the robot is trapped in local mimima (LM)--- identify this every 4 points (1&5,2&6,3&7.....)  
       tem_a=LM_test(:,iters); % this point is trapped in the LM 
       tem_b=LM_test(:,iters-4);
       
       if (norm(tem_a-tem_b)) < (3*sp_len)  % if robot is in LM, then the distance must be smaller or equal to double step length (if not in LM, then it is 4*step length), so 3 times to step length is chosen
          LM_curr=tem_a;
          
     %---------- method 1 -------------%
     
     ww = rand(1);
     curr1=[testR*sin(pi*ww);testR*cos(pi*ww)]/2;
     curr=(curr1+curr);
%      
%      %---------- end of method 1 -------------%
%      
%      %---------- method 2 -------------%
% %           for j=1:8  
% %               LM_testPoint(:,j)=[testR*sin((j-1)*pi/4)+LM_curr(1);testR*cos((j-1)*pi/4)+LM_curr(2)]; 
% %               LM_testOut(:,j)=LM_escape(LM_testPoint(:,j),over,obstacle); 
% %     
% %           end
% %            
% %               [temp num]=min(LM_testOut); 
% %     
% %               curr=(LM_curr+LM_testPoint(:,num))/2; 
% 
%       %----------- end of method 2 -----------------%
       else 

       end 
       
    else 
        
    end 
%---------------end of strategy of LM----------------%


    point(:,iters)=curr;
    iters=iters+1; 
  
 
end
end