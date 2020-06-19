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
    


    
end

runTime=0;

figure(1)




for t=0:dt:10
    runTime = runTime+1;
    figure(1)
    %plot1
    plot(recTheta(1,runTime),recTheta(3,runTime),'.','color','black','markersize',20)
    hold on
    plot(recTheta(5,runTime),recTheta(7,runTime),'.','color','green','markersize',20)
    plot(recTheta(9,runTime),recTheta(11,runTime),'.','color','magenta','markersize',20)
    plot(recTheta(13,runTime),recTheta(15,runTime),'.','color','red','markersize',20)
    plot(recTheta(17,runTime),recTheta(19,runTime),'.','color','blue','markersize',20)
    hold off
%     legend('Agent 1','Agent 2','Agent 3','Agent 4','Agent 5')
title('Agent trajectories')
xlim([-5,5])
ylim([-5,5])
    
    %plot2
    subplot(2,2,2)
    plot([0:dt:t],recTheta(2,1:runTime),'color','b')
    hold on
    plot([0:dt:t],recTheta(4,1:runTime),'color','r')
    plot([0:dt:t],recH(2,1:runTime),'color','g')
    plot([0:dt:t],recH(4,1:runTime),'color','k')
%     plot([0:dt:t],recTheta(10,1:runTime),'color','r')
%     plot([0:dt:t],recTheta(12,1:runTime),'color','r')
%     plot([0:dt:t],recTheta(14,1:runTime),'color','k')
%     plot([0:dt:t],recTheta(16,1:runTime),'color','k')
%     plot([0:dt:t],recTheta(18,1:runTime),'color','m')
%     plot([0:dt:t],recTheta(20,1:runTime),'color','m')
    hold off
%     legend('V_(1X)','V_(1Y)','V_(2X)','V_(2Y)','V_(3X)','V_(3Y)','V_(4X)','V_(4Y)','V_(5X)','V_(5Y)')
    legend('V_(1X)','V_(1Y)','HV_(1X)','HV_(1Y)')
    grid on
    xlim([0,10])
    ylim([-4,4])
    title('Agent 1 Velocities')
    xlabel('Time/s')
    
    %     
    %plot3
    subplot(2,1,2)
%     recDistance = [sqrt((recTheta(1,:)-h(1)).^2 + (recTheta(3,:)-h(3)).^2); ...
%         sqrt((recTheta(5,:)-h(5)).^2 + (recTheta(7,:)-h(7)).^2); ...
%         sqrt((recTheta(9,:)-h(9)).^2 + (recTheta(11,:)-h(11)).^2); ...
%         sqrt((recTheta(13,:)-h(13)).^2 + (recTheta(15,:)-h(15)).^2); ...
%         sqrt((recTheta(17,:)-h(17)).^2 + (recTheta(19,:)-h(19)).^2); ...
%         ];
    
    plot([0:dt:t],recTheta(1,1:runTime),'color','b')
    hold on
    plot([0:dt:t],recTheta(3,1:runTime),'color','g')
    plot([0:dt:t],recH(1,1:runTime),'color','r')
    plot([0:dt:t],recH(3,1:runTime),'color','k')
    % plot(0:0.01:10,recDistance(5,:),'color','m')
    hold off
    legend('X_(1X)','X_(1Y)','HX_(1X)','HX_(1Y)')
    % legend('Agent 1','Agent 2','Agent 3','Agent 4','Agent 5')
    grid on
    xlim([0,10])
    ylim([-5,5])
    title('Agent 1 Position')
    xlabel('Time/s')

end



