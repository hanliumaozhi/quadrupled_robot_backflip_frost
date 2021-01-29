function [ptich_fun] = vel_x(nlp)
expr = nlp.Plant.States.dx(1);
ptich_fun = SymFunction(['vel_x_', nlp.Plant.Name], expr, {nlp.Plant.States.dx});
end