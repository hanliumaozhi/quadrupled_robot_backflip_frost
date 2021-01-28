function [fl_knee_z_fun] = fl_knee_z(nlp)
p_fl_knee = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.fl_leg);
expr = p_fl_knee(3);
fl_knee_z_fun = SymFunction(['fl_knee_z_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end