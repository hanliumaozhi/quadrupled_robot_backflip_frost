function [rl_knee_z_fun] = rl_knee_z(nlp)
p_rl_knee = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.rl_leg);
expr = p_rl_knee(3);
rl_knee_z_fun = SymFunction(['rl_knee_z_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end