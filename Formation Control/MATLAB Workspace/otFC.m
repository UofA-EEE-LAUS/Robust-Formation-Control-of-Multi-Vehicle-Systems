clear

%incidence matrix
E = [1,1,1,0,0,0;
    -1,0,0,1,0,-1;
    0,-1,0,-1,1,0;
    0,0,-1,0,-1,1];

Er = [1,1,1;
    -1,0,0;
    0,-1,0;
    0,0,-1];

Ec = [0,0,0;
    1,0,-1;
    -1,1,0;
    0,-1,1];

T = inv(Er'*Er) * Er' * Ec;
I = eye(3);
R = [I, T];

%gc
Ecr = [0,0;
    0,-1;
    1,0;
    -1,1];
Ecc = [0;
    1;
    -1;
    0];

Tc = inv(Ecr'*Ecr) * Ecr' * Ecc;
Ic = eye(2);
Rc = [Ic, Tc];

%Laplacian
Ln = E*E'; %graph laplacian
Le = E'*E; %edge laplacian

% Lgc = Ecr*Rc*Rc'*Ecr';
a = -blkdiag(Ln,Ln);
b = blkdiag(E,E);
c = eye(8);
d = zeros(size(b));

sys=ss(a, b, c, d);

%simulation


T=0:0.1:10;


X0=[1;2;3;4;1;2;3;4];

U = [1;0;1;1;0;1;0;1;1;1;1;0];
% U=[1;1;sqrt(2);sqrt(2);1;1];

U=repmat(U', length(T), 1);
[tmpy,tmpt,tmpX]=lsim(sys, U, T, X0);

half = size(tmpX,2)/2;
X=tmpX(:, 1:half);
Y=tmpX(:, half+1:end);
