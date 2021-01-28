function [fr_knee_z_fun] = fr_knee_z(nlp)
p_fr_knee = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.fr_leg);
expr = p_fr_knee(3);
fr_knee_z_fun = SymFunction(['fr_knee_z_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end