function [ X, Y ] = simulate( sys, x0, y0, u, v )

    T=0:0.1:0.5;

    % Concatenate X and Y coordinates
    X0=[x0;y0];

    % Do we have input?
    if nargin>3
        % Simulate for initial conditions and input
        U=[u(:)' v(:)'];
        U=repmat(U, length(T), 1);
        [tmpy,tmpt,tmpX]=lsim(sys, U, T, X0);   
    else
        % Simulate only for initial conditions
        [tmpy,tmpt,tmpX]=initial(sys, X0, T);
    end

    % Extract X and Y trajectories
    half = size(tmpX,2)/2;
    X=tmpX(:, 1:half);
    Y=tmpX(:, half+1:end);

end

