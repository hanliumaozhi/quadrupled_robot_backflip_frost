function [ q_error_fun ] = q_error( nlp )
%torque cost for torque

% Compute function for torque cost
x = nlp.Plant.States.x;
init_q = [-0.032 1.1256 -2.088 -0.032 1.1256 -2.088 -0.032 1.1256 -2.088 -0.032 1.1256 -2.088];
cost = 0;
for i=7:length(x)
    cost = cost + (x(i) - init_q(i-6))*(x(i) - init_q(i-6));
end
q_error_fun = SymFunction('q_error', cost, {x});

end