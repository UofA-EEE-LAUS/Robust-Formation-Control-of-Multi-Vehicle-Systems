function [ sys ] = system( L )

  a = -blkdiag(L,L);
  b = -a;
  c = eye(length(L)*2);
  d=b;
  
  sys=ss(a, b, c, d);
  
end

