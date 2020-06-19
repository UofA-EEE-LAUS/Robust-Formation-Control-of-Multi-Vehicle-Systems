%plot3
figure(2)



plot(0:0.01:10,recTheta(1,:),'color','b')
hold on
plot(0:0.01:10,recTheta(3,:),'color','g')
plot(0:0.01:10,recH(1,:),'color','r')
plot(0:0.01:10,recH(3,:),'color','k')
% plot(0:0.01:10,recDistance(5,:),'color','m')
legend('X_1X','X_1Y','HX_1X','HX_1Y')
% legend('Agent 1','Agent 2','Agent 3','Agent 4','Agent 5')
grid on
title('Agent 1 Position')
xlabel('Time/s')