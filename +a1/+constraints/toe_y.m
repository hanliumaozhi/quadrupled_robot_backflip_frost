function [toe_y_fun] = toe_y(nlp)
p_fl_toe = getCartesianPosition(nlp.Plant, nlp.Plant.ContactPoints.front_left_toe);
p_fr_toe = getCartesianPosition(nlp.Plant, nlp.Plant.ContactPoints.front_right_toe);
p_rl_toe = getCartesianPosition(nlp.Plant, nlp.Plant.ContactPoints.rear_left_toe);
p_rr_toe = getCartesianPosition(nlp.Plant, nlp.Plant.ContactPoints.rear_right_toe);


p_fl_thigh = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.fl_thigh);
p_fr_thigh = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.fr_thigh);
p_rl_thigh = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.rl_thigh);
p_rr_thigh = getCartesianPosition(nlp.Plant, nlp.Plant.OtherPoints.rr_thigh);


expr = (p_fl_toe(2) - p_fl_thigh(2))*(p_fl_toe(2) - p_fl_thigh(2)) + (p_fr_toe(2)-p_fr_thigh(2))*(p_fr_toe(2)-p_fr_thigh(2));
expr = expr + (p_rl_toe(2)- p_rl_thigh(2))*(p_rl_toe(2)- p_rl_thigh(2)) + (p_rr_toe(2) - p_rr_thigh(2))*(p_rr_toe(2) - p_rr_thigh(2));
toe_y_fun = SymFunction(['toe_y_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end