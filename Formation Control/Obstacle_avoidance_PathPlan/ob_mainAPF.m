tic
figure(1);
axis([-10 10 -10 10]);

%--------- begin point and over point setting ---------------%

%for the original  
% begin=[3;3];
% over=[14;14]; 

%for the unreachable target 
% begin=[6;8];
% over=[9;8];

%for the local minimum  
begin=[1;1.4];
over=[4;1.4];



hold on;
plot(begin(1),begin(2),'+k','MarkerSize',10);
plot(over(1),over(2),'*b','MarkerSize',10);

%---------------------------------- Obstacles setting -----------------------------------%

%for the original
% obstacle=[5 8 10 7 10 5 ;
%           5 6 9 9 13 10];

%for unreachable target issue
% obstacle= [9.2;
%             8];  

%for local minimum issue

obstacle= [ 2.5   2.5  2.5  2.5  2.5  2.5 2.5 2.5 2.5 2.5 2.5 2.5 2.5  ;
            1   1.3   1.5  1.8   0.8  0.5 2.0 0.8 0.5 0.2 2.8 2.2 0];


%---------------------------------- Plot -----------------------------------%

plot(obstacle(1,:),obstacle(2,:),'.r','Markersize',20);

xlabel('X');
ylabel('Y');
title('Simulations of APF on single robot');
text(6,2,'+ the initial position','Color','k','FontSize',10);
text(6,3,'* the goal','Color','b','FontSize',10);
text(6,4,'O obstacles','Color','r','FontSize',10);

% store the robot path
point= ob_Path(begin,over,obstacle);

T=toc