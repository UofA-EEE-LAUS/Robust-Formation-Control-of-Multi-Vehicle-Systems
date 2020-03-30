function [position,roverID] = formation(x0,y0,u,v)

%Initialisation
alpha0 = [0;0;0];
beta0 = [0;0;0];
angle = 360;
radius = 100;

%Munkres
xs=x0;
ys=y0;
n=length(xs);

xss=repmat(xs, 1, n);
yss=repmat(ys, 1, n);
uss=repmat(u, 1, n);
vss=repmat(v, 1, n);
diffu=xss'-uss;
diffv=yss'-vss;
dists_uv = sqrt(diffu.*diffu + diffv.*diffv);

[assignment,cost] = munkres(dists_uv);

%Assignment
x1 = zeros(n,1);
y1 = zeros(n,1);
alpha1 = zeros(n,1);
beta1 = zeros(n,1);
roverID = zeros(n,1);
for i = 1:n
    for j = 1:n
        if assignment(i,j)==1
            x1(i)=x0(j);
            y1(i)=y0(j);
            roverID(i) = j-1; 
            alpha1(i)=alpha0(j);
            beta1(i)=beta0(j);
        end
    end
end

%Initialisation
xs=x1;
ys=y1;
alphas=alpha1;
betas=beta1;

%Graph parameters
%A=adjacency(xs, ys, alphas, betas, radius);
A = [0,1,1;1,0,1;1,1,0];
L=laplacian(A);

% Simulate
sys=system_formation(L);
[X,Y] = simulate(sys, xs, ys, u, v);

position = [X(1001, :)',Y(1001, :)'];
end