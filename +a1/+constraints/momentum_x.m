function [momentum_x_fun] = momentum_x(nlp)
momentum = getMomentum(nlp.Plant);
expr = momentum(1);
momentum_x_fun = SymFunction(['momentum_x_', nlp.Plant.Name], expr, {nlp.Plant.States.x, nlp.Plant.States.dx});
end