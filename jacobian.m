function [J] = jacobian(S, theta)
n = size(S);
n = n(2);
if n == 1
    J = S;
else
    e = zeros(4,4,n);
    J = zeros(6,n);
    a(:,:,1) = eye(4);
    
    for i = 1:n
        e(:,:,i) = expm(bracket(S(:,i))*theta(i));
    end

    for i = 1:n
        a(:,:,i+1) = a(:,:,i)*e(:,:,i);
        J(:,i) = adjoint(a(:,:,i))*S(:,i);
    end
end
