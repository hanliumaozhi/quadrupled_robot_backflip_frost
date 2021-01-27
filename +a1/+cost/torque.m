function [ torque_fun ] = torque( nlp )
%torque cost for torque

% Compute function for torque cost
u = nlp.Plant.Inputs.Control.u;
cost = sum((u).^2);
torque_fun = SymFunction('torque', cost, {u});

end

