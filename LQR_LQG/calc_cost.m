function [ cost ] = calc_cost(err, u)
%CALC_COST Summary of this function goes here
%   Detailed explanation goes here

% Compute the values of the cost function
Q = eye(3);
F = eye(3);
R = eye(2);

% Calculate error
term1 = err{end-1}'*F*err{end-1};
term2 = zeros(1,size(u,2));
term3 = term2;
for n = 1:size(u,2)
    term2(n) = err{n}'*Q*err{n};
    term3(n) = u{n}'*R*u{n};
end

% Add terms together for overall cost
cost = term1 + sum(term2) + sum(term3);


end

