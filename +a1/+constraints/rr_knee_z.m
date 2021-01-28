function [rr_knee_z_fun] = rr_knee_z(nlp)
p_rr_knee = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.rr_leg);
expr = p_rr_knee(3);
rr_knee_z_fun = SymFunction(['rr_knee_z_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end