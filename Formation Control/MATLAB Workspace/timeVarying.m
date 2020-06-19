clear
K1 = kron(eye(2),[-3, -1.5]);
K2 = kron(eye(2),[0.43, 1]);
B1 = kron(eye(2),[1,0]');
B2 = kron(eye(2),[0,1]');

A = [0,1,0,0,1;
    1,0,1,0,0;
    0,1,0,1,0;
    0,0,1,0,1;
    1,0,0,1,0];

L = indegree(A) + A;
R1 = kron(eye(5), (B2*K1+B1*B2'));
R2 = kron(L,B2*K2);
R3 = kron(eye(5), B2*K1);
R4 = kron(L,B2*K2);
R5 = kron(eye(5),B2);

dt = 0.01;

% h = [1,0,1,0, ...
%     2,0,2,0, ...
%     3,0,3,0, ...
%     4,0,4,0, ...
%     5,0,5,0]';



Theta = [0,0,0,0, ...
    1,0,0,0, ...
    2,0,0,0, ...
    3,0,0,0, ...
    4,0,0,0]';


Theta_N = zeros(20,1);
recTheta = zeros(20,1000);
recH = zeros(20,1000);
runTime = 0;

for t = 0:dt:10
    
    runTime = runTime+1;
    
        % time-varying formation
        h1=[1*cos(t)-2;-1*sin(t);1*sin(t)+2;1*cos(t)];
        h2=[1*cos(t)-2;-1*sin(t);1*sin(t)-2;1*cos(t)];
        h3=[1*cos(t);-1*sin(t);1*sin(t);1*cos(t)];
        h4=[1*cos(t)+2;-1*sin(t);1*sin(t)-2;1*cos(t)];
        h5=[1*cos(t)+2;-1*sin(t);1*sin(t)+2;1*cos(t)];
        h=[h1;h2;h3;h4;h5];
        
    recH(:,runTime) = h;
    
    
    U = (R1-R2) * Theta - (R3-R4) * h;
    
    
    Theta = Theta + U * dt;
    
    recTheta(:,runTime) = Theta;
    X = Theta(1:2:20);
    
    %     figure(1)
    %     plot(X(1),X(2),'.','Color','g','Markersize',10);
    %     hold on
    %     plot(X(3),X(4),'.','Color','g','Markersize',10);
    %     plot(X(5),X(6),'.','Color','g','Markersize',10);
    %     plot(X(7),X(8),'.','Color','g','Markersize',10);
    %     plot(X(9),X(10),'.','Color','g','Markersize',10);
    %     xlim([-10,10])
    %     ylim([-10,1])
    %     hold off
    
    
    
end

figure(1)
%plot1
subplot(2,2,1)
plot(recTheta(1,:),recTheta(3,:),'color','black')
hold on
plot(recTheta(5,:),recTheta(7,:),'color','black')
plot(recTheta(9,:),recTheta(11,:),'color','black')
plot(recTheta(13,:),recTheta(15,:),'color','black')
plot(recTheta(17,:),recTheta(19,:),'color','black')

% plot(recTheta(1,1),recTheta(3,1),'x','color','magenta')
% plot(recTheta(1,1001),recTheta(3,1001),'o','color','r')
% plot(recTheta(5,1),recTheta(7,1),'x','color','magenta')
% plot(recTheta(5,1001),recTheta(7,1001),'o','color','r')
% plot(recTheta(9,1),recTheta(11,1),'x','color','magenta')
% plot(recTheta(9,1001),recTheta(11,1001),'o','color','r')
% plot(recTheta(13,1),recTheta(15,1),'x','color','magenta')
% plot(recTheta(13,1001),recTheta(15,1001),'o','color','r')
% plot(recTheta(17,1),recTheta(19,1),'x','color','magenta')
% plot(recTheta(17,1001),recTheta(19,1001),'o','color','r')

plot(recTheta(1,1),recTheta(3,1),'x','color','magenta')
plot(recTheta(1,1001),recTheta(3,1001),'o','color','r')
plot(recTheta(5,1),recTheta(7,1),'x','color','magenta')
plot(recTheta(5,1001),recTheta(7,1001),'o','color','r')
plot(recTheta(9,1),recTheta(11,1),'x','color','magenta')
plot(recTheta(9,1001),recTheta(11,1001),'o','color','r')
plot(recTheta(13,1),recTheta(15,1),'x','color','magenta')
plot(recTheta(13,1001),recTheta(15,1001),'o','color','r')
plot(recTheta(17,1),recTheta(19,1),'x','color','magenta')
plot(recTheta(17,1001),recTheta(19,1001),'o','color','r')
title('Agent trajectories')
xlim([-5,5])
ylim([-5,5])

%plot2
subplot(2,2,2)
plot(0:0.01:10,recTheta(2,:),'color','b')
hold on
plot(0:0.01:10,recTheta(4,:),'color','r')
plot(0:0.01:10,recH(2,:),'color','g')
plot(0:0.01:10,recH(4,:),'color','k')
% plot(0:0.01:10,recTheta(10,:),'color','r')
% plot(0:0.01:10,recTheta(12,:),'color','r')
% plot(0:0.01:10,recTheta(14,:),'color','k')
% plot(0:0.01:10,recTheta(16,:),'color','k')
% plot(0:0.01:10,recTheta(18,:),'color','m')
% plot(0:0.01:10,recTheta(20,:),'color','m')
% legend('x1','y1','x2','y2','x3','y3','x4','y4','x5','y5')
legend('V_1X','V_1Y','HV_1X','HV_1Y')
grid on
title('Agent 1 Velocities')
xlabel('Time/s')

%plot3
subplot(2,1,2)
recDistance = [sqrt((recTheta(1,:)-h(1)).^2 + (recTheta(3,:)-h(3)).^2); ...
    sqrt((recTheta(5,:)-h(5)).^2 + (recTheta(7,:)-h(7)).^2); ...
    sqrt((recTheta(9,:)-h(9)).^2 + (recTheta(11,:)-h(11)).^2); ...
    sqrt((recTheta(13,:)-h(13)).^2 + (recTheta(15,:)-h(15)).^2); ...
    sqrt((recTheta(17,:)-h(17)).^2 + (recTheta(19,:)-h(19)).^2); ...
    ];

plot(0:0.01:10,recTheta(1,:),'color','b')
hold on
plot(0:0.01:10,recTheta(3,:),'color','g')
plot(0:0.01:10,recH(1,:),'color','r')
plot(0:0.01:10,recH(3,:),'color','k')
% plot(0:0.01:10,recDistance(5,:),'color','m')
legend('X_1X','X_1Y','HX_1X','HX_1Y')
% legend('Agent 1','Agent 2','Agent 3','Agent 4','Agent 5')
grid on
title('Agent 1 Movement')
xlabel('Time/s')

%Robustness
% Theta([17,19]) = [2,3];
%
% for t = 0:dt:10
%     Theta_N = (R1-R2) * Theta - (R3-R4) * h;
%
%
%
%     Theta = Theta + Theta_N * dt;
%     X = Theta(1:2:20);
%
%     figure(1)
%     plot(X(1),X(2),'.','Color','g','Markersize',10);
%     hold on
%     plot(X(3),X(4),'.','Color','g','Markersize',10);
%     plot(X(5),X(6),'.','Color','g','Markersize',10);
%     plot(X(7),X(8),'.','Color','g','Markersize',10);
%     plot(X(9),X(10),'.','Color','g','Markersize',10);
%     xlim([-10,10])
%     ylim([-10,10])
%     hold off
%
%     pause(0.1)
% end

