function [ptich_fun] = vel_dx(nlp)
expr = nlp.Plant.States.dx(1);
ptich_fun = SymFunction(['vel_dx_', nlp.Plant.Name], expr, {nlp.Plant.States.dx});
end