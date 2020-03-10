function [ askew ] = askew( A )
% Find the corresponding vector for the skew-symmetric matrix
askew = [A(3,2); A(1,3); A(2,1)];

end

