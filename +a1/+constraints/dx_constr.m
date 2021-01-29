function [dx_constr_fun] = dx_constr(nlp, dx_index)
expr = nlp.Plant.States.dx(dx_index);
dx_constr_fun = SymFunction(['dx_constr_', num2str(dx_index) , nlp.Plant.Name], expr, {nlp.Plant.States.dx});
end