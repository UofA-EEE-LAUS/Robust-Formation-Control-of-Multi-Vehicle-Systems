function [ point ] = ob_Path(begin,over,obstacle)
%PATH_PLAN 

iters=1; % iterations
curr=begin;
testR=0.5;   % radius of the circle, in which the current point is the centre. The threshold of MoveSpin Function is 0.1, the step length here should be bigger than the threshold
sp_len=testR/2; % the step length
iters_num=500;

while (norm(curr-over)>0.2) &&  (iters<=iters_num) % the error is 0.2
    
    ori_curr=curr; % stored the original position value

    % at first, find out the coordinates of 8 points around the robot
    for i=1:8  % the potential moving direction of robot is 360 degrees (2pi), then dividing the potential direction into 8 points (45 degrees between each point)
        testPoint(:,i)=[testR*sin((i-1)*pi/4)+ori_curr(1);testR*cos((i-1)*pi/4)+ori_curr(2)]; % the coordinate of these 8 points
        testOut(:,i)=ob_Algrith(testPoint(:,i),over,obstacle); % the output is the resultant
        
    end
    [~, num]=min(testOut); % find out the smallest one, since in the APF, the robot always moves to the point with the smallest potential energy  
    
    %the step length of iteration is sp_len
    curr=(ori_curr+testPoint(:,num))/2; % step length half of the circle radius is 0.2, thus the coordinate has to reduce to the half of the result
    

    %-------- strategy to escape local minima -------------%
        
       LM_test(:,iters)=curr; %tempory value for testing if the point is in LM 
    
    if iters > 2 %identify if the robot is trapped in local mimima (LM)--- identify this every 4 points (1&5,2&6,3&7.....)  
       tem_a=LM_test(:,iters); 
       tem_b=LM_test(:,iters-2);
       
       if (norm(tem_a-tem_b)) < 0.01  % if robot is in LM, then the distance must be equal
          LM_point=tem_a; % this point is the LM point 
    
%      %---------- Method 2: virtual obstacle  -------------%
          n=4; % the number of potential point around the current position 
          for j=1:n  
              testPoint_m2(:,j)=[testR*sin((j-1)*(2*pi/n))+ori_curr(1);testR*cos((j-1)*(2*pi/n))+ori_curr(2)];
              
              %crate a new matrix to store the new obstacles
              new_ob=obstacle;
              ob_size=size(new_ob,2);
              new_ob=[new_ob(:,1:ob_size) LM_point]; 
              
              testOut_m2(:,j)=ob_Algrith(testPoint_m2(:,j),over,new_ob); 
    
          end
           
              [~, num]=min(testOut_m2); 
    
              curr=(ori_curr+testPoint_m2(:,num))/2; 
% 
%       %----------- end of method 2 -----------------%
       else 

       end 
       
    else 
        
    end 
%--------------- end of strategy of escaping LM----------------%

    
    plot(curr(1),curr(2),'og');
    point(:,iters)=curr;
%     pause(0.01);
    iters=iters+1; 
    
end

 if iters==1
    point(:,iters)=curr;
    
 end

 end