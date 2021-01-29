function [ptich_fun] = pitch(nlp)
expr = nlp.Plant.States.x(5);
ptich_fun = SymFunction(['pitch_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end