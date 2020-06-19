A = [0,1,0,0,1;
    1,0,1,0,0;
    0,1,0,1,0;
    0,0,1,0,1;
    1,0,0,1,0];
L = indegree(A) - A;
L = kron(eye(4),L);
dt = 0.05;
 

h = [0,0,0,0,0, ...
    1,2,3,4,5, ...
    1,2,3,4,5, ...
    0,0,0,0,0]';

Theta = [1,2,3,4,5, ...
    0,0,0,0,0, ...
    0.5,0.5,0.5,0.5,0.5, ...
    0,0,0,0,0]';

Theta_N = zeros(20,1);

U = zeros(20,1);

for t = 0:dt:30
    

    U = 0.5 * - L * (Theta - h);
    
    Theta = Theta + U * dt;
    
    Theta(1:10) = Theta(1:10) + Theta(11:20)*dt;
    
    X = Theta(1:10);
    
    figure(1)
    plot(X(1),X(6),'.','Color','g','Markersize',10);
    hold on
    plot(X(2),X(7),'.','Color','g','Markersize',10);
    plot(X(3),X(8),'.','Color','g','Markersize',10);
    plot(X(4),X(9),'.','Color','g','Markersize',10);
    plot(X(5),X(10),'.','Color','g','Markersize',10);
    xlim([-50,50])
    ylim([-50,50])
    hold off

    
end