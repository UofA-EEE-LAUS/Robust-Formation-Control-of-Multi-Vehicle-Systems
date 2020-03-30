function [ A ] = adjacency( xs, ys, alphas, betas, radius )

    % Repeat the matrix to calculate differences between agent coordinates
    n=length(xs);
    xss=repmat(xs, 1, n);
    yss=repmat(ys, 1, n);
    diffx=(xss'-xss);
    diffy=(yss'-yss);

    % Calculate agent distances
    dists = sqrt(diffx.*diffx + diffy.*diffy);

    % Calculate the angles between aents
    angles = atan2(diffy, diffx);
    angles(angles<0)=angles(angles<0)+2*pi;

    % Threshold distance based on "visibility" radius
    A = (dists>0) & (dists<radius) & ...
        ( ((angles>=repmat(alphas,1,n)) & (angles<=repmat(betas,1,n))) | ...
          (((angles-2*pi)>=repmat(alphas,1,n)) & ((angles-2*pi)<=repmat(betas,1,n))) | ...
          (((angles+2*pi)>=repmat(alphas,1,n)) & ((angles+2*pi)<=repmat(betas,1,n))) );
end 

