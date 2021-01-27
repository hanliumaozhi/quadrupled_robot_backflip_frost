function [momentum_z_fun] = momentum_z(nlp)
momentum = getMomentum(nlp.Plant);
expr = momentum(3);
momentum_z_fun = SymFunction(['momentum_z_', nlp.Plant.Name], expr, {nlp.Plant.States.x, nlp.Plant.States.dx});
end